/**
 * forward_kinematics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "forward_kinematics.h"

namespace SpatialDyn {

Eigen::Matrix6Xd Jacobian(const ArticulatedBody& ab, int link,
                          const Eigen::Vector3d& offset) {
  Eigen::Matrix6Xd J = Eigen::Matrix6Xd::Zero(6, ab.dof());
  if (link < 0) link += ab.dof();
  const auto& p_0n = ab.T_to_world(link) * offset;
  for (const int i : ab.ancestors(link)) {
    Eigen::Affine3d T_i_to_point = Eigen::Translation3d(ab.T_to_world(i).translation() - p_0n) *
                                   ab.T_to_world(i).linear();
    J.col(i) = (T_i_to_point * ab.rigid_bodies(i).joint().subspace()).matrix();
  }
  return J;
}

}  // namespace SpatialDyn
