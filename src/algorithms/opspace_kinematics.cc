/**
 * opspace_kinematics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 5, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/opspace_kinematics.h"

#include <limits>  // std::numeric_limits

#include "structs/articulated_body_cache.h"

namespace spatial_dyn {
namespace opspace {

Eigen::Vector3d OrientationError(const Eigen::Quaterniond &quat, const Eigen::Quaterniond &quat_des) {
  Eigen::Quaterniond quat_err = quat * quat_des.inverse();
  Eigen::AngleAxisd aa_err(quat_err);  // Angle will always be between [0, pi]
  double angle = (quat_err.w() < 0) ? aa_err.angle() - 2 * M_PI : aa_err.angle();
  return angle * aa_err.axis();
}

Eigen::Vector3d LookatError(const Eigen::Vector3d &vec, const Eigen::Vector3d &vec_des) {
  Eigen::Quaterniond quat_err = Eigen::Quaterniond::FromTwoVectors(vec_des, vec);
  Eigen::AngleAxisd aa_err(quat_err);
  double angle = (quat_err.w() < 0) ? aa_err.angle() - 2 * M_PI : aa_err.angle();
  return angle * aa_err.axis();
}

Eigen::Quaterniond NearQuaternion(const Eigen::Quaterniond& quat,
                                  const Eigen::Quaterniond& quat_reference) {
  Eigen::Quaterniond result = quat;
  if (quat.dot(quat_reference) < 0) result.coeffs() *= -1;
  return result;
}

Eigen::Quaterniond NearQuaternion(Eigen::Ref<const Eigen::Matrix3d> ori,
                                  const Eigen::Quaterniond& quat_reference) {
  Eigen::Quaterniond result(ori);
  if (result.dot(quat_reference) < 0) result.coeffs() *= -1;
  return result;
}

Eigen::Quaterniond FarQuaternion(const Eigen::Quaterniond& quat,
                                 const Eigen::Quaterniond& quat_reference) {
  Eigen::Quaterniond result = quat;
  if (quat.dot(quat_reference) > 0) result.coeffs() *= -1;
  return result;
}

Eigen::Matrix<double,4,3> AngularVelocityToQuaternionMap(const Eigen::Quaterniond& quat) {
  Eigen::Matrix<double,4,3> E;
  E <<  quat.w(),  quat.z(), -quat.y(),
       -quat.z(),  quat.w(),  quat.x(),
        quat.y(), -quat.x(),  quat.w(),
       -quat.x(), -quat.y(), -quat.z();
  return 0.5 * E;
}

Eigen::Matrix<double,4,3> AngularVelocityToAngleAxisMap(const Eigen::AngleAxisd& aa) {
  Eigen::Matrix<double,4,3> E;
  E << aa.axis().transpose(),
       -0.5 * (std::sin(aa.angle()) / (1 - std::cos(aa.angle())) *
               ctrl_utils::Eigen::DoubleCrossMatrix(aa.axis()) +
               ctrl_utils::Eigen::CrossMatrix(aa.axis()));
  return E;
}

Eigen::Matrix3d ExpCoordsJacobianImpl(const Eigen::Matrix3d& R, const Eigen::AngleAxisd& aa,
                                      const Eigen::Vector3d& p) {
  const double theta = aa.angle();  // theta is always positive
  if (theta == 0.) {
    return Eigen::Matrix3d::Zero();
  } else if (theta < std::numeric_limits<double>::epsilon()) {
    return -ctrl_utils::Eigen::CrossMatrix(p);
  }
  const Eigen::Vector3d w = theta * aa.axis();

  return R * ctrl_utils::Eigen::CrossMatrix(p / -(theta * theta)) *
         (w * w.transpose() + (R.transpose() - Eigen::Matrix3d::Identity()) *
          ctrl_utils::Eigen::CrossMatrix(w));
}

Eigen::Matrix3d ExpCoordsJacobian(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
  return ExpCoordsJacobianImpl(R, Eigen::AngleAxisd(R), p);
}
Eigen::Matrix3d ExpCoordsJacobian(const Eigen::AngleAxisd& aa, const Eigen::Vector3d& p) {
  return ExpCoordsJacobianImpl(aa.toRotationMatrix(), aa, p);
}

Eigen::Matrix<double,9,3> ExpCoordsJacobianImpl(const Eigen::Matrix3d& R, const Eigen::AngleAxisd& aa) {
  Eigen::Matrix<double,9,3> dR_dw;

  const double theta = aa.angle();  // theta is always positive
  if (theta == 0.) {
    dR_dw.setZero();
    return dR_dw;
  } else if (theta < std::numeric_limits<double>::epsilon()) {
    for (size_t i = 0; i < 3; i++) {
      Eigen::Map<Eigen::Matrix3d> dR_dwi(dR_dw.col(i).data());
      dR_dwi = ctrl_utils::Eigen::CrossMatrix(Eigen::Vector3d::Unit(i));
    }
    return dR_dw;
  }

  const Eigen::Vector3d w = theta * aa.axis();
  const Eigen::Matrix3d w_cross = ctrl_utils::Eigen::CrossMatrix(w);
  const Eigen::Matrix3d R_hat = R / (theta * theta);
  for (size_t i = 0; i < 3; i++) {
    Eigen::Map<Eigen::Matrix3d> dR_dwi(dR_dw.col(i).data());
    dR_dwi = (w(i) * w_cross +
              ctrl_utils::Eigen::CrossMatrix(w.cross(Eigen::Vector3d::Unit(i) - R.col(i)))) * R_hat;
  }
  return dR_dw;
}

Eigen::Matrix<double,9,3> ExpCoordsJacobian(Eigen::Ref<const Eigen::Matrix3d> R) {
  return ExpCoordsJacobianImpl(R, Eigen::AngleAxisd(R));
}

Eigen::Matrix<double,9,3> ExpCoordsJacobian(const Eigen::AngleAxisd& aa) {
  return ExpCoordsJacobianImpl(aa.toRotationMatrix(), aa);
}

/**
 * Phi = f(R)
 * w = log(R)
 * delta = [Phi_32 - Phi_23; Phi_13 - Phi_31; Phi_21 - Phi_12] = cross_basis(Phi - Phi^T)
 * w = acos((Tr(Phi) - 1) / 2) / sqrt(4 - (Tr(Phi) - 1)^2) delta
 * dlog(f(R))/dw = acos((Tr(Phi) - 1) / 2) / sqrt(4 - (Tr(Phi) - 1)^2) * ddelta/dw -
 *                 1 / (4 - (Tr(Phi) - 1)^2) * delta * dTrPhi/dw
 * @return dlog(f(R))/dw
 */
Eigen::Matrix3d LogExpCoordsJacobian(const Eigen::Matrix3d& Phi,
                                     const Eigen::Matrix<double,9,3> dR_dw,
                                     const Eigen::Matrix3d& ddelta_dw,
                                     const Eigen::Matrix3d& dtrPhi_dR) {
  const double trPhi = Phi.diagonal().sum();
  if (3. - trPhi < 1e-5) {
    return Eigen::Matrix3d::Zero(); // TODO: Derive from taylor approx
  }
  const double theta = std::acos((trPhi - 1.) / 2.);
  const double det = 4. - (trPhi - 1.) * (trPhi - 1.);
  Eigen::Vector3d dtrPhi_dw;
  for (size_t i = 0; i < 3; i++) {
    const Eigen::Map<const Eigen::Matrix3d> dR_dwi(dR_dw.col(i).data());
    dtrPhi_dw(i) = (dtrPhi_dR.array() * dR_dwi.array()).sum();  // trace(dtrfR_dR.T * dR_dw)
  }
  const Eigen::Vector3d delta(Phi(2, 1) - Phi(1, 2), Phi(0, 2) - Phi(2, 0), Phi(1, 0) - Phi(0, 1));
  std::cout << "th: " << theta << " " << 2 * std::sin(theta) << " " << std::sqrt(det) << std::endl;
  std::cout << "ddelta_dw: " << std::endl << ddelta_dw << std::endl;
  std::cout << "w_hat: " << delta.transpose() / std::sqrt(det) << std::endl;
  std::cout << "dtrPhi_dw: " << dtrPhi_dw.transpose() << std::endl;
  std::cout << "tr_ddelta: " << theta / std::sqrt(det) << std::endl << (theta / std::sqrt(det)) * ddelta_dw << std::endl;
  std::cout << "delta_dTr: " << std::endl << delta / det * dtrPhi_dw.transpose() << std::endl;
  return (theta / std::sqrt(det)) * ddelta_dw - delta / det * dtrPhi_dw.transpose();
}

/**
 * Phi = f(R)
 * w = log(R)
 * delta = [Phi_32 - Phi_23; Phi_13 - Phi_31; Phi_21 - Phi_12] = cross_basis(Phi - Phi^T)
 * w = 1 / sqrt(4 - (Tr(Phi) - 1)^2) delta
 * dlog(f(R))/dw = 1/sqrt(4 - (Tr(Phi) - 1)^2) ddelta/dw +
 *                 delta * sqrt(4 - (Tr(Phi) - 1)^2)^(-3) dTrPhi/dw
 * @return dlog(f(R))/dw
 */
// Eigen::Matrix3d AxisLogExpCoordsJacobian(const Eigen::Matrix3d& Phi,
//                                      const Eigen::Matrix<double,9,3> dR_dw,
//                                      const Eigen::Matrix3d& ddelta_dw,
//                                      const Eigen::Matrix3d& dTrPhi_dR) {
//   const double trR_1 = Phi.diagonal().sum() - 1.;
//   if (2. - trR_1 < 1e-5) {
//     return Eigen::Matrix3d::Zero(); // TODO: Derive from taylor approx
//   }
//   const double det = std::sqrt(4. - trR_1 * trR_1);
//   Eigen::Vector3d dTrPhi_dw;
//   for (size_t i = 0; i < 3; i++) {
//     const Eigen::Map<const Eigen::Matrix3d> dR_dwi(dR_dw.col(i).data());
//     dTrPhi_dw(i) = (dTrPhi_dR.array() * dR_dwi.array()).sum();  // trace(dtrfR_dR.T * dR_dw)
//   }
//   const Eigen::Vector3d delta(Phi(2, 1) - Phi(1, 2), Phi(0, 2) - Phi(2, 0), Phi(1, 0) - Phi(0, 1));
//   return (ddelta_dw / det) - delta * (trR_1 / (det * det * det) * dTrPhi_dw.transpose());
// }

Eigen::Matrix3d LogExpCoordsJacobianInvLeft(const Eigen::Matrix3d& R, const Eigen::Matrix3d& B) {
  Eigen::Matrix3d Phi = R.transpose() * B;

  Eigen::Matrix<double,9,3> dR_dw = ExpCoordsJacobian(R);

  Eigen::Matrix3d ddelta_dw;
  ddelta_dw.row(0) = B.col(1).transpose() * dR_dw.block<3,3>(2 * 3, 0) -
                     B.col(2).transpose() * dR_dw.block<3,3>(1 * 3, 0);
  ddelta_dw.row(1) = B.col(2).transpose() * dR_dw.block<3,3>(0 * 3, 0) -
                     B.col(0).transpose() * dR_dw.block<3,3>(2 * 3, 0);
  ddelta_dw.row(2) = B.col(0).transpose() * dR_dw.block<3,3>(1 * 3, 0) -
                     B.col(1).transpose() * dR_dw.block<3,3>(0 * 3, 0);

  Eigen::Matrix3d dTrPhi_dR = -1. * (R * B.transpose() * R);

  return LogExpCoordsJacobian(Phi, dR_dw, ddelta_dw, dTrPhi_dR);
}

Eigen::Matrix3d LogExpCoordsJacobian(const Eigen::Matrix3d& A, const Eigen::Matrix3d& R) {
  const Eigen::Matrix3d Phi = A * R;

  const Eigen::Matrix<double,9,3> dR_dw = ExpCoordsJacobian(R);
  std::cout << "A: " << std::endl << A << std::endl;
  std::cout << "dR_dw:" << std::endl << dR_dw << std::endl << std::endl;
  std::cout << dR_dw.block<3,3>(0, 0) << std::endl << std::endl;
  std::cout << dR_dw.block<3,3>(3, 0) << std::endl << std::endl;
  std::cout << dR_dw.block<3,3>(6, 0) << std::endl << std::endl;

  Eigen::Matrix3d ddelta_dw;
  ddelta_dw.row(0) = A.row(2) * dR_dw.block<3,3>(1 * 3, 0) -
                     A.row(1) * dR_dw.block<3,3>(2 * 3, 0);
  ddelta_dw.row(1) = A.row(0) * dR_dw.block<3,3>(2 * 3, 0) -
                     A.row(2) * dR_dw.block<3,3>(0 * 3, 0);
  ddelta_dw.row(2) = A.row(1) * dR_dw.block<3,3>(0 * 3, 0) -
                     A.row(0) * dR_dw.block<3,3>(1 * 3, 0);

  const Eigen::Matrix3d dTrPhi_dR = A.transpose();

  return LogExpCoordsJacobian(Phi, dR_dw, ddelta_dw, dTrPhi_dR);
}

Eigen::Vector3d NormLogExpCoordsGradient(const Eigen::Matrix3d& Phi,
                                         const Eigen::Matrix<double,9,3>& dR_dw,
                                         const Eigen::Matrix3d dTrPhi_dR) {
  Eigen::Vector3d g;

  const double trPhi = Phi.diagonal().sum();
  if (3. - trPhi < std::numeric_limits<double>::epsilon()) {
    g.setZero();
    return g;
  }
  const double theta = std::acos((trPhi - 1.) / 2.);
  const double det = 4. - (trPhi - 1.) * (trPhi - 1.);
  const double a = -theta / std::sqrt(det);

  for (size_t i = 0; i < 3; i++) {
    const Eigen::Map<const Eigen::Matrix3d> dR_dwi(dR_dw.col(i).data());
    g(i) = a * (dTrPhi_dR.array() * dR_dwi.array()).sum();
  }
  return g;
}

Eigen::Vector3d NormLogExpCoordsGradient(const Eigen::Matrix3d& A, const Eigen::Matrix3d& R) {
  return NormLogExpCoordsGradient(A * R, ExpCoordsJacobian(R), A.transpose());
}

// Eigen::Matrix6d LogExpCoordsJacobian(const Eigen::Isometry3d& A, const Eigen::Isometry3d& T) {
//   Eigen::Matrix6d J;
//   J.topLeftCorner<3,3>() = LogExpCoordsJacobian(A.linear(), T.linear());
//   J.topRightCorner<3,3>().setZero();
//   const Eigen::Isometry3d AT = A * T;
//   const Eigen::Vector3d R = AT.linear();
//   const Eigen::Vector3d p = AT.translation();
//   const Eigen::AngleAxisd aa(R);
//   const double theta = aa.angle();
//   const Eigen::Vector3d w = theta * aa.axis();
//   const double a = 1 - theta / (2 * std::tan(theta / 2));

//   J.bottomLeftCorner<3,3>() = (-ctrl_utils::Eigen::CrossMatrix(p / 2) + dw_x_w_x_p) * J.topLeftCorner<3,3>();
//   J.bottomRightCorner<3,3>() = A.linear().transpose() +
//                                ctrl_utils::Eigen::CrossMatrix(w / 2) * A.linear().transpose() +
//                                a * ctrl_utils::Eigen::DoubleCrossMatrix(w).transpose() * A.linear().transpose();
// }

Eigen::Vector3d Log(const Eigen::Matrix3d& R) {
  const Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

SpatialMotiond Log(const Eigen::Isometry3d& T) {
  const Eigen::AngleAxisd aa(T.linear());
  const double theta = aa.angle();
  const Eigen::Vector3d w = theta * aa.axis();
  const Eigen::Vector3d& p = T.translation();
  const Eigen::Vector3d w_x_p = w.cross(p);
  const double a = 1 - theta / (2 * std::tan(theta / 2));
  // const double a = (1 - theta * std::sin(theta) / (2 - 2 * std::cos(theta))) / (theta * theta);

  Eigen::Vector6d v;
  v.head<3>() = w;
  v.tail<3>() = p - w_x_p / 2 + a * w.cross(w_x_p);
  return v;
}

Eigen::Matrix3d Exp(const Eigen::Vector3d& w) {
  return Eigen::AngleAxisd(w.norm(), w.normalized()).toRotationMatrix();
}

}  // namespace opspace
}  // namespace spatial_dyn
