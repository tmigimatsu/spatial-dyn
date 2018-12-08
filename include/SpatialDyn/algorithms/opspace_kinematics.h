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

#include "SpatialDyn/utils/spatial_math.h"

namespace SpatialDyn {
namespace Opspace {

Eigen::Vector3d OrientationError(const Eigen::Quaterniond &quat, const Eigen::Quaterniond &quat_des);

Eigen::Vector3d LookatError(const Eigen::Vector3d &vec, const Eigen::Vector3d &vec_des);

Eigen::Quaterniond NearQuaternion(const Eigen::Quaterniond& quat,
                                  const Eigen::Quaterniond& quat_reference);

Eigen::Quaterniond FarQuaternion(const Eigen::Quaterniond& quat,
                                 const Eigen::Quaterniond& quat_reference);

Eigen::Matrix<double,4,3> QuaternionJacobian(const Eigen::Quaterniond& quat);

Eigen::Matrix<double,4,3> AngleAxisJacobian(const Eigen::AngleAxisd& aa);

}  // namespace Opspace
}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_ALGORITHMS_OPSPACE_KINEMATICS_H_
