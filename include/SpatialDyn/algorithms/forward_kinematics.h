/**
 * forward_kinematics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 8, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_FORWARD_KINEMATICS_H_
#define SPATIAL_DYN_ALGORITHMS_FORWARD_KINEMATICS_H_

#include "SpatialDyn/structs/articulated_body.h"
#include "SpatialDyn/utils/spatial_math.h"

namespace SpatialDyn {

Eigen::Vector3d Position(const ArticulatedBody& ab, int link = -1,
                         const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());
Eigen::Vector3d Position(const ArticulatedBody& ab,
                         Eigen::Ref<const Eigen::VectorXd> q, int link = -1,
                         const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

Eigen::Quaterniond Orientation(const ArticulatedBody& ab, int link = -1);
Eigen::Quaterniond Orientation(const ArticulatedBody& ab,
                               Eigen::Ref<const Eigen::VectorXd> q, int link = -1);

Eigen::Quaterniond NearQuaternion(const Eigen::Quaterniond& quat,
                                  const Eigen::Quaterniond& quat_reference);
Eigen::Quaterniond FarQuaternion(const Eigen::Quaterniond& quat,
                                 const Eigen::Quaterniond& quat_reference);

// Eigen::Quaterniond Orientation(const ArticulatedBody& ab, int link = -1,
//                                 const Eigen::Quaterniond& near, bool near = true);

Eigen::Matrix6Xd Jacobian(const ArticulatedBody& ab, int link = -1,
                          const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());
Eigen::Matrix6Xd Jacobian(const ArticulatedBody& ab,
                          Eigen::Ref<const Eigen::VectorXd> q, int link = -1,
                          const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

Eigen::Matrix3Xd LinearJacobian(const ArticulatedBody& ab, int link = -1,
                                const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());
Eigen::Matrix3Xd LinearJacobian(const ArticulatedBody& ab,
                                Eigen::Ref<const Eigen::VectorXd> q, int link = -1,
                                const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

Eigen::Matrix3Xd AngularJacobian(const ArticulatedBody& ab, int link = -1);
Eigen::Matrix3Xd AngularJacobian(const ArticulatedBody& ab,
                                 Eigen::Ref<const Eigen::VectorXd> q, int link = -1);

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_ALGORITHMS_FORWARD_KINEMATICS_H_
