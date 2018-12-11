/**
 * rigid_body.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "structs/rigid_body.h"

#include <exception>  // std::invalid_argument

namespace SpatialDyn {

void RigidBody::set_T_to_parent(const Eigen::Quaterniond& ori_in_parent,
                     const Eigen::Vector3d& pos_in_parent) {
  T_to_parent_ = Eigen::Translation3d(pos_in_parent) * ori_in_parent;
}
void RigidBody::set_T_to_parent(const Eigen::Isometry3d& T_to_parent) {
  T_to_parent_ = T_to_parent;
}

void RigidBody::set_inertia(double mass,
                            const Eigen::Vector3d& com,
                            const Eigen::Vector6d& I_com_flat) {
  if (mass < 0.) {
    throw std::invalid_argument("RigidBody::set_inertia(): Mass must be non-negative (mass=" + std::to_string(mass) + ").");
  } 
  inertia_ = SpatialInertiad(mass, com, I_com_flat);
}
void RigidBody::set_inertia(SpatialInertiad&& inertia) {
  if (inertia.mass < 0.) {
    throw std::invalid_argument("RigidBody::set_inertia(): Mass must be non-negative (mass=" + std::to_string(inertia.mass) + ").");
  } 
  inertia_ = SpatialInertiad(std::move(inertia));
}
void RigidBody::set_inertia(const SpatialInertiad& inertia) {
  if (inertia.mass < 0.) {
    throw std::invalid_argument("RigidBody::set_inertia(): Mass must be non-negative (mass=" + std::to_string(inertia.mass) + ").");
  } 
  inertia_ = SpatialInertiad(inertia);
}

void RigidBody::set_joint(Joint&& joint) {
  joint_ = joint;
}

void RigidBody::set_joint(const Joint& joint) {
  joint_ = joint;
}

std::ostream& operator<<(std::ostream& os, const RigidBody& rb) {
  os << "RigidBody(name=\"" << rb.name << "\", id=" << rb.id() << ", id_parent=" << rb.id_parent()
     << ", joint=" << std::string(rb.joint()) << ")";
  return os;
}

}  // namespace SpatialDyn
