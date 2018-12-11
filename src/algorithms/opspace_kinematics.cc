/**
 * opspace_kinematics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 5, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/opspace_kinematics.h"

#include "structs/articulated_body_cache.h"

namespace SpatialDyn {
namespace Opspace {

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

Eigen::Quaterniond FarQuaternion(const Eigen::Quaterniond& quat,
                                 const Eigen::Quaterniond& quat_reference) {
  Eigen::Quaterniond result = quat;
  if (quat.dot(quat_reference) > 0) result.coeffs() *= -1;
  return result;
}

Eigen::Matrix<double,4,3> QuaternionJacobian(const Eigen::Quaterniond& quat) {
  Eigen::Matrix<double,4,3> E;
  E <<  quat.w(),  quat.z(), -quat.y(),
       -quat.z(),  quat.w(),  quat.x(),
        quat.y(), -quat.x(),  quat.w(),
       -quat.x(), -quat.y(), -quat.z();
  return 0.5 * E;
}

Eigen::Matrix<double,4,3> AngleAxisJacobian(const Eigen::AngleAxisd& aa) {
  Eigen::Matrix<double,4,3> E;
  E << aa.axis().transpose(),
       -0.5 * (std::sin(aa.angle()) / (1 - std::cos(aa.angle())) * aa.axis().doubleCrossMatrix() + aa.axis().crossMatrix());
  return E;
}

}  // namespace Opspace
}  // namespace SpatialDyn
