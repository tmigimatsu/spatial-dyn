/**
 * forward_kinematics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/forward_kinematics.h"

namespace SpatialDyn {

Eigen::Vector3d Position(const ArticulatedBody& ab, int link,
                         const Eigen::Vector3d& offset) {
  if (link < 0) link += ab.dof();
  return ab.T_to_world(link) * offset;
}
Eigen::Vector3d Position(const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> q,
                         int link, const Eigen::Vector3d& offset) {
  if (link < 0) link += ab.dof();
  Eigen::Vector3d pos = offset;
  for (int i = link; i != -1; i = ab.rigid_bodies(i).id_parent()) {
    const RigidBody& rb = ab.rigid_bodies(i);
    pos = rb.T_to_parent() * (rb.joint().T_joint(q(i)) * pos);
  }
  return ab.T_base_to_world() * pos;
}

Eigen::Quaterniond Orientation(const ArticulatedBody& ab, int link) {
  if (link < 0) link += ab.dof();
  return Eigen::Quaterniond(ab.T_to_world(link).linear());
}
Eigen::Quaterniond Orientation(const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> q,
                               int link) {
  if (link < 0) link += ab.dof();
  Eigen::Matrix3d ori = Eigen::Matrix3d::Identity();
  for (int i = link; i != -1; i = ab.rigid_bodies(i).id_parent()) {
    const RigidBody& rb = ab.rigid_bodies(i);
    ori = rb.T_to_parent().linear() * rb.joint().T_joint(q(i)).linear() * ori;
  }
  return Eigen::Quaterniond(ab.T_base_to_world().linear() * ori);
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
Eigen::Matrix6Xd Jacobian(const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> q,
                          int link, const Eigen::Vector3d& offset) {
  Eigen::Matrix6Xd J = Eigen::Matrix6Xd::Zero(6, ab.dof());
  if (link < 0) link += ab.dof();
  const std::vector<int>& ancestors = ab.ancestors(link);
  std::vector<Eigen::Isometry3d> T_to_world(ancestors.size());
  for (size_t idx = 0; idx < ancestors.size(); idx++) {
    int i = ancestors[idx];
    const RigidBody& rb = ab.rigid_bodies(i);
    if (idx == 0) {
      T_to_world[idx] = ab.T_base_to_world() * rb.T_to_parent() * rb.joint().T_joint(q(i));
    } else {
      T_to_world[idx] = T_to_world[idx-1] * rb.T_to_parent() * rb.joint().T_joint(q(i));
    }
  }
  const Eigen::Vector3d p_0n = T_to_world.back() * offset;
  for (size_t idx = 0; idx < ancestors.size(); idx++) {
    int i = ancestors[idx];
    auto T_i_to_point = Eigen::Translation3d(T_to_world[idx].translation() - p_0n) *
                        T_to_world[idx].linear();
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
Eigen::Matrix3Xd LinearJacobian(const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> q,
                                int link, const Eigen::Vector3d& offset) {
  Eigen::Matrix3Xd J = Eigen::Matrix3Xd::Zero(6, ab.dof());
  if (link < 0) link += ab.dof();
  const std::vector<int>& ancestors = ab.ancestors(link);
  std::vector<Eigen::Isometry3d> T_to_world(ancestors.size());
  for (size_t idx = 0; idx < ancestors.size(); idx++) {
    int i = ancestors[idx];
    const RigidBody& rb = ab.rigid_bodies(i);
    if (idx == 0) {
      T_to_world[idx] = ab.T_base_to_world() * rb.T_to_parent() * rb.joint().T_joint(q(i));
    } else {
      T_to_world[idx] = T_to_world[idx-1] * rb.T_to_parent() * rb.joint().T_joint(q(i));
    }
  }
  const Eigen::Vector3d p_0n = T_to_world.back() * offset;
  for (size_t idx = 0; idx < ancestors.size(); idx++) {
    int i = ancestors[idx];
    auto T_i_to_point = Eigen::Translation3d(T_to_world[idx].translation() - p_0n) *
                        T_to_world[idx].linear();
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
Eigen::Matrix3Xd AngularJacobian(const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> q,
                                 int link) {
  Eigen::Matrix3Xd J = Eigen::Matrix3Xd::Zero(6, ab.dof());
  if (link < 0) link += ab.dof();
  const std::vector<int>& ancestors = ab.ancestors(link);
  std::vector<Eigen::Isometry3d> T_to_world(ancestors.size());
  for (size_t idx = 0; idx < ancestors.size(); idx++) {
    int i = ancestors[idx];
    const RigidBody& rb = ab.rigid_bodies(i);
    if (idx == 0) {
      T_to_world[idx] = ab.T_base_to_world() * rb.T_to_parent() * rb.joint().T_joint(q(i));
    } else {
      T_to_world[idx] = T_to_world[idx-1] * rb.T_to_parent() * rb.joint().T_joint(q(i));
    }
  }
  const Eigen::Vector3d p_0n = T_to_world.back().translation();
  for (size_t idx = 0; idx < ancestors.size(); idx++) {
    int i = ancestors[idx];
    auto T_i_to_point = Eigen::Translation3d(T_to_world[idx].translation() - p_0n) *
                        T_to_world[idx].linear();
    J.col(i) = (T_i_to_point * ab.rigid_bodies(i).joint().subspace()).angular();
  }
  return J;
}


}  // namespace SpatialDyn
