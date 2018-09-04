/**
 * forward_kinematics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 8, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_FORWARD_KINEMATICS_H_
#define SPATIAL_DYN_FORWARD_KINEMATICS_H_

#include "articulated_body.h"
#include "spatial_math.h"

namespace SpatialDyn {

Eigen::Vector3d Position(const ArticulatedBody& ab, int link = -1,
                         const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

Eigen::Quaterniond Orientation(const ArticulatedBody& ab, int link = -1);

// Eigen::Quaterniond Orientation(const ArticulatedBody& ab, int link = -1,
//                                 const Eigen::Quaterniond& near, bool near = true);

Eigen::Matrix6Xd Jacobian(const ArticulatedBody& ab, int link = -1,
                          const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_FORWARD_KINEMATICS_H_
