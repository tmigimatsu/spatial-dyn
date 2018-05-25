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

SpatialMotionXd SpatialJacobian(const ArticulatedBody& ab, int link = -1) {
  SpatialMotionXd J = SpatialMotionXd::Zero(ab.dof());
  if (link < 0) link += ab.dof();
  for (const int i : ab.ancestors_[link]) {
    J.col(i) = ab.T_to_world(i) * ab.rigid_bodies(i).joint().subspace();
  }
  return J;
}

Eigen::Matrix6Xd Jacobian(const ArticulatedBody& ab, int link = -1,
                          const Eigen::Vector3d& offset = Eigen::Vector3d::Zero()) {
  Eigen::Matrix6Xd J = Eigen::Matrix6Xd::Zero(6, ab.dof());
  if (link < 0) link += ab.dof();
  const auto& p_0n = ab.T_to_world(link).translation() +
                     ab.T_to_world(link).linear() * offset;
  for (const int i : ab.ancestors_[link]) {
    Eigen::Affine3d T_i_to_point = Eigen::Translation3d(p_0n - ab.T_to_world(i).translation()) *
                                   ab.T_to_world(i).linear();
    J.col(i) = (T_i_to_point * ab.rigid_bodies(i).joint().subspace()).matrix();
  }
  return J;
}

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_FORWARD_KINEMATICS_H_
