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

namespace spatial_dyn {

void RigidBody::set_inertia(double mass,
                            const Eigen::Vector3d& com,
                            const Eigen::Vector6d& I_com_flat) {
  if (mass < 0.) {
    throw std::invalid_argument("RigidBody::set_inertia(): Mass must be non-negative (mass=" + std::to_string(mass) + ").");
  } 
  inertia_ = SpatialInertiad(mass, com, I_com_flat);
}
void RigidBody::set_inertia(const SpatialInertiad& inertia) {
  if (inertia.mass < 0.) {
    throw std::invalid_argument("RigidBody::set_inertia(): Mass must be non-negative (mass=" + std::to_string(inertia.mass) + ").");
  } 
  inertia_ = SpatialInertiad(inertia);
}

std::ostream& operator<<(std::ostream& os, const RigidBody& rb) {
  os << "RigidBody(name=\"" << rb.name << "\", id=" << rb.id() << ", id_parent=" << rb.id_parent()
     << ", joint=" << rb.joint().type() << ")";
  return os;
}

}  // namespace spatial_dyn
