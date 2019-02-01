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

#include "spatial_dyn/utils/spatial_math.h"

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

Eigen::Matrix<double,4,3> QuaternionJacobian(const Eigen::Quaterniond& quat);

Eigen::Matrix<double,4,3> AngleAxisJacobian(const Eigen::AngleAxisd& aa);

}  // namespace opspace
}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_ALGORITHMS_OPSPACE_KINEMATICS_H_
