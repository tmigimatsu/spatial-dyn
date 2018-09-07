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

Eigen::Vector3d Position(const ArticulatedBody& ab, int link,
                         const Eigen::Vector3d& offset) {
  if (link < 0) link += ab.dof();
  return ab.T_to_world(link) * offset;
}

Eigen::Quaterniond Orientation(const ArticulatedBody& ab, int link) {
  if (link < 0) link += ab.dof();
  return Eigen::Quaterniond(ab.T_to_world(link).linear());
}

// Eigen::Quaterniond Orientation(const ArticulatedBody& ab, int link = -1,
//                                 const Eigen::Quaterniond& near, bool near = true);

Eigen::Matrix6Xd Jacobian(const ArticulatedBody& ab, int link,
                          const Eigen::Vector3d& offset) {
  Eigen::Matrix6Xd J = Eigen::Matrix6Xd::Zero(6, ab.dof());
  if (link < 0) link += ab.dof();
  const Eigen::Vector3d p_0n = ab.T_to_world(link) * offset;
  for (const int i : ab.ancestors(link)) {
    auto T_i_to_point = Eigen::Translation3d(ab.T_to_world(i).translation() - p_0n) *
                        ab.T_to_world(i).linear();
    J.col(i) = (T_i_to_point * ab.rigid_bodies(i).joint().subspace()).matrix();
  }
  return J;
}

Eigen::Matrix3Xd LinearJacobian(const ArticulatedBody& ab, int link,
                                const Eigen::Vector3d& offset) {
  Eigen::Matrix3Xd J = Eigen::Matrix3Xd::Zero(3, ab.dof());
  if (link < 0) link += ab.dof();
  const Eigen::Vector3d p_0n = ab.T_to_world(link) * offset;
  for (const int i : ab.ancestors(link)) {
    auto T_i_to_point = Eigen::Translation3d(ab.T_to_world(i).translation() - p_0n) *
                        ab.T_to_world(i).linear();
    J.col(i) = (T_i_to_point * ab.rigid_bodies(i).joint().subspace()).linear();
  }
  return J;
}

Eigen::Matrix3Xd AngularJacobian(const ArticulatedBody& ab, int link) {
  Eigen::Matrix3Xd J = Eigen::Matrix3Xd::Zero(3, ab.dof());
  if (link < 0) link += ab.dof();
  const Eigen::Vector3d p_0n = ab.T_to_world(link).translation();
  for (const int i : ab.ancestors(link)) {
    auto T_i_to_point = Eigen::Translation3d(ab.T_to_world(i).translation() - p_0n) *
                        ab.T_to_world(i).linear();
    J.col(i) = (T_i_to_point * ab.rigid_bodies(i).joint().subspace()).angular();
  }
  return J;
}

}  // namespace SpatialDyn
