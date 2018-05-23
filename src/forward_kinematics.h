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

#include <vector>  // std::vector

namespace SpatialDyn {

SpatialMotionXd SpatialJacobian(const ArticulatedBody& ab, int link = -1) {
  SpatialMotionXd J = SpatialMotionXd::Zero(ab.dof_);
  if (link < 0) link += ab.dof_;
  for (const int i : ab.ancestors_[link]) {
    J.col(i) = ab.T_to_world_[i] * ab.rigid_bodies_[i].joint().subspace();
  }
  return J;
}

Eigen::Matrix6Xd Jacobian(const ArticulatedBody& ab, int link = -1,
                          const Eigen::Vector3d& offset = Eigen::Vector3d::Zero()) {
  Eigen::Matrix6Xd J = Eigen::Matrix6Xd::Zero(6, ab.dof_);
  if (link < 0) link += ab.dof_;
  const auto& p_0n = ab.T_to_world_[link].translation() +
                     ab.T_to_world_[link].rotation() * offset;
  for (const int i : ab.ancestors_[link]) {
    Eigen::Affine3d T_to_link = Eigen::Translation3d(p_0n - ab.T_to_world_[i].translation()) *
                                ab.T_to_world_[i].rotation();
    J.col(i) = (T_to_link * ab.rigid_bodies_[i].joint().subspace()).matrix();
  }
  return J;
}

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_FORWARD_KINEMATICS_H_
