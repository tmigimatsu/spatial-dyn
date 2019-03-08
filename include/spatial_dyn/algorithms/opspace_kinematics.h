/**
 * opspace_kinematics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 5, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_OPSPACE_KINEMATICS_H_
#define SPATIAL_DYN_ALGORITHMS_OPSPACE_KINEMATICS_H_

#include "spatial_dyn/eigen/spatial_math.h"

namespace spatial_dyn {
namespace opspace {

/**
 * Compute the quaternion error between the given orientations.
 *
 * This quaternion error is equivalent to the angular velocity required to go
 * from `quat_des` to `quat` in one second. Following this angular velocity also
 * produces the SLERP interpolation between the two orientations.
 *
 * For orientation control, a control law might look like the following:
 *
 * ```{.cc}
 * Eigen::Matrix3d J_w = AngularJacobian(ab);
 * Eigen::Vector3d w_err = opspace::OrientationError(Orientation(ab), quat_des);
 * Eigen::Vector3d dw = -kp_ori * w_err - kv_ori * J_w * ab.dq;
 * Eigen::VectorXd tau = opspace::InverseDynamics(ab, J_w, dw);
 * ```
 *
 * @param quat Current orientation.
 * @param quat_des Desired orientation.
 * @return Quaternion error.
 * @see Python: spatialdyn.opspace.orientation_error()
 */
Eigen::Vector3d OrientationError(const Eigen::Quaterniond &quat, const Eigen::Quaterniond &quat_des);

/**
 * Compute the orientation error between two lookat vectors.
 *
 * Unlike opspace::OrientationError(), this error ignores rotations about the
 * lookat vectors, producing the shortest rotation between the two.
 *
 * @param quat Current orientation.
 * @param quat_des Desired orientation.
 * @return Quaternion error.
 * @see Python: spatialdyn.opspace.orientation_error()
 */
Eigen::Vector3d LookatError(const Eigen::Vector3d &vec, const Eigen::Vector3d &vec_des);

Eigen::Quaterniond NearQuaternion(const Eigen::Quaterniond& quat,
                                  const Eigen::Quaterniond& quat_reference);

Eigen::Quaterniond NearQuaternion(Eigen::Ref<const Eigen::Matrix3d> ori,
                                  const Eigen::Quaterniond& quat_reference);

Eigen::Quaterniond FarQuaternion(const Eigen::Quaterniond& quat,
                                 const Eigen::Quaterniond& quat_reference);

/**
 * Compute the Jacobian relating angular velocity to a quaternion rotation.
 *
 * @return E_quat where quat' = E_quat(quat) w.
 */
Eigen::Matrix<double,4,3> AngularVelocityToQuaternionMap(const Eigen::Quaterniond& quat);

/**
 * Compute the Jacobian relating angular velocity to an angle axis rotation.
 *
 * @return E_aa where ktheta' = E_aa(ktheta) w.
 */
Eigen::Matrix<double,4,3> AngularVelocityToAngleAxisMap(const Eigen::AngleAxisd& aa);

/**
 * Compute the Jacobian of a rotated vector R * p with respect to the rotation's
 * exponential coordinates (angle axis representation).
 *
 * @return d(Rp)/dw.
 */
Eigen::Matrix3d ExpCoordsJacobian(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);
Eigen::Matrix3d ExpCoordsJacobian(const Eigen::AngleAxisd& R, const Eigen::Vector3d& p);

/**
 * Compute the Jacobian of the rotation matrix with respect to its exponential
 * coordinates (angle axis representation).
 *
 * @return Jacobian tensor stacked into a matrix [dR/dw_x; dR/dw_y; dR/dw_z].
 */
Eigen::Matrix<double,9,3> ExpCoordsJacobian(Eigen::Ref<const Eigen::Matrix3d> R);
Eigen::Matrix<double,9,3> ExpCoordsJacobian(const Eigen::AngleAxisd& aa);

/**
 * Compute the Jacobian of the log map of a function of a rotation matrix R with
 * respect to R's exponential coordinates.
 *
 * Phi = f(R)
 * w = log(R)
 * delta = [Phi_32 - Phi_23; Phi_13 - Phi_31; Phi_21 - Phi_12] = cross_basis(Phi - Phi^T)
 * w = 1 / sqrt(4 - (Tr(Phi) - 1)^2) delta
 * dlog(f(R))/dw = 1/sqrt(4 - (Tr(Phi) - 1)^2) ddelta/dw +
 *                 delta * sqrt(4 - (Tr(Phi) - 1)^2)^(-3) dTrPhi/dw
 *
 * @param Phi f(R).
 * @param dR_dw ExponentialCoordinatesJacobian(R).
 * @param ddelta_dw Jacobian of delta with respect to R's exponential coordinates.
 * @param dTrPhi_dR Jacobian of the trace of Phi with respect to R.
 * @return dlog(Phi)/dw.
 */
Eigen::Matrix3d LogExpCoordsJacobian(const Eigen::Matrix3d& Phi,
                                     const Eigen::Matrix<double,9,3> dR_dw,
                                     const Eigen::Matrix3d& ddelta_dw,
                                     const Eigen::Matrix3d& dTrPhi_dR);

/**
 * Compute the Jacobian of the log map of R^T B for some rotation matrices R and
 * B with respect to R's exponential coordinates.
 *
 * @param R Rotation matrix to which the derivative is taken.
 * @param B Rotation matrix.
 * @return dlog(R^T B)/dw.
 */
Eigen::Matrix3d LogExpCoordsJacobianInvLeft(const Eigen::Matrix3d& R, const Eigen::Matrix3d& B);

/**
 * Compute the Jacobian of the log map of A R for some rotation matrices R and A
 * with respect to R's exponential coordinates.
 *
 * @param A Rotation matrix.
 * @param R Rotation matrix to which the derivative is taken.
 * @return dlog(AR)/dw.
 */
Eigen::Matrix3d LogExpCoordsJacobian(const Eigen::Matrix3d& A, const Eigen::Matrix3d& R);

Eigen::Vector3d NormLogExpCoordsGradient(const Eigen::Matrix3d& A, const Eigen::Matrix3d& R);

Eigen::Matrix6d LogExpCoordsJacobian(const Eigen::Isometry3d& A, const Eigen::Isometry3d& T);

Eigen::Vector3d NormLogExpCoordsGradient(const Eigen::Matrix3d& Phi,
                                         const Eigen::Matrix<double,9,3>& dR_dw,
                                         const Eigen::Matrix3d dTrPhi_dR);

Eigen::Vector3d Log(const Eigen::Matrix3d& R);

SpatialMotiond Log(const Eigen::Isometry3d& T);

Eigen::Matrix3d Exp(const Eigen::Vector3d& w);

}  // namespace opspace
}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_ALGORITHMS_OPSPACE_KINEMATICS_H_
