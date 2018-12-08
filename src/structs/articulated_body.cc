/**
 * articulated_body.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "structs/articulated_body.h"

#include <exception>  // std::invalid_argument

namespace SpatialDyn {

ArticulatedBody::ArticulatedBody(const std::string& name) : name(name) {}

size_t ArticulatedBody::dof() const {
  return dof_;
}

void ArticulatedBody::set_T_base_to_world(const Eigen::Quaterniond& ori_in_parent,
                                          const Eigen::Vector3d& pos_in_parent) {
  T_base_to_world_ = Eigen::Translation3d(pos_in_parent) * ori_in_parent;
}
void ArticulatedBody::set_T_base_to_world(const Eigen::Isometry3d& T_to_parent) {
  T_base_to_world_ = T_to_parent;
}
const Eigen::Isometry3d& ArticulatedBody::T_base_to_world() const {
  return T_base_to_world_;
}

const std::vector<RigidBody>& ArticulatedBody::rigid_bodies() const {
  return rigid_bodies_;
}
const RigidBody& ArticulatedBody::rigid_bodies(int i) const {
  if (i < 0) i += dof();
  return rigid_bodies_[i];
}
const std::vector<int>& ArticulatedBody::ancestors(int i) const {
  if (i < 0) i += dof();
  return ancestors_[i];
}

const Eigen::VectorXd& ArticulatedBody::q() const {
  return q_;
}
double ArticulatedBody::q(int i) const {
  if (i < 0) i += dof();
  return q_(i);
}
void ArticulatedBody::set_q(Eigen::Ref<const Eigen::VectorXd> q) {
  if (q.size() != static_cast<int>(dof_)) {
    throw std::invalid_argument("ArticulatedBody::set_state(): q must be of size " + std::to_string(dof_));
  }

  // Return early if q has already been set
  if (q_ == q) return;
  q_ = q;

  CalculateTransforms();
  vel_data_.is_computed = false;
  jac_data_.is_computed = false;
  cc_data_.is_computed = false;
  grav_data_.is_computed = false;
  crba_data_.is_computed = false;
  crba_data_.is_inv_computed = false;
  aba_data_.is_computed = false;
  aba_data_.is_A_inv_computed = false;
  opspace_data_.is_lambda_computed = false;
  opspace_data_.is_lambda_inv_computed = false;
  opspace_data_.is_jbar_computed = false;
  opspace_aba_data_.is_lambda_computed = false;
  opspace_aba_data_.is_lambda_inv_computed = false;
}

const Eigen::VectorXd& ArticulatedBody::dq() const {
  return dq_;
}
double ArticulatedBody::dq(int i) const {
  if (i < 0) i += dof();
  return dq_(i);
}
void ArticulatedBody::set_dq(Eigen::Ref<const Eigen::VectorXd> dq) {
  if (dq.size() != static_cast<int>(dof_)) {
    throw std::invalid_argument("ArticulatedBody::set_state(): q must be of size " + std::to_string(dof_));
  }

  // Return early if q has already been set
  if (dq_ == dq) return;
  dq_ = dq;

  vel_data_.is_computed = false;
  grav_data_.is_friction_computed = false;
  cc_data_.is_computed = false;
  aba_data_.is_computed = false;
}

const Eigen::VectorXd& ArticulatedBody::ddq() const {
  return ddq_;
}
double ArticulatedBody::ddq(int i) const {
  if (i < 0) i += dof();
  return ddq_(i);
}
void ArticulatedBody::set_ddq(Eigen::Ref<const Eigen::VectorXd> ddq) {
  ddq_ = ddq;
}

const Eigen::VectorXd& ArticulatedBody::tau() const {
  return tau_;
}
double ArticulatedBody::tau(int i) const {
  if (i < 0) i += dof();
  return tau_(i);
}
void ArticulatedBody::set_tau(Eigen::Ref<const Eigen::VectorXd> tau) {
  tau_ = tau;
}

const SpatialMotiond& ArticulatedBody::g() const {
  return g_;
}
void ArticulatedBody::set_g(const Eigen::Vector3d& g) {
  g_ << g, Eigen::Vector3d::Zero();
  grav_data_.is_computed = false;
  aba_data_.is_computed = false;
}

const Eigen::Isometry3d& ArticulatedBody::T_to_parent(int i) const {
  if (i < 0) i += dof();
  return T_to_parent_[i];
}
const Eigen::Isometry3d& ArticulatedBody::T_from_parent(int i) const {
  if (i < 0) i += dof();
  return T_from_parent_[i];
}
const Eigen::Isometry3d& ArticulatedBody::T_to_world(int i) const {
  if (i < 0) i += dof();
  return T_to_world_[i];
}

const std::vector<int>& ArticulatedBody::subtree(int i) const {
  if (i < 0) i += dof();
  return subtrees_[i];
}

Eigen::VectorXd ArticulatedBody::Map(const std::function<double(const RigidBody& rb)>& rb_function) const {
  Eigen::VectorXd result(dof());
  for (size_t i = 0; i < dof(); i++) {
    result(i) = rb_function(rigid_bodies(i));
  }
  return result;
}

int ArticulatedBody::AddRigidBody(RigidBody&& rb, int id_parent) {
  // TODO: Set default state
  int id = rigid_bodies_.size();
  if ((id_parent < 0 || id_parent >= id) && id != 0) {
    throw std::invalid_argument("ArticulatedBody::AddRigidBody(): Parent rigid body with id " + std::to_string(id_parent) + " does not exist.");
  }
  rb.id_ = id;
  rb.id_parent_ = id_parent;
  rigid_bodies_.push_back(std::move(rb));
  ExpandDof(id, id_parent);
  return id;
}
int ArticulatedBody::AddRigidBody(const RigidBody& rb_in, int id_parent) {
  int id = rigid_bodies_.size();
  if ((id_parent < 0 || id_parent >= id) && id != 0) {
    throw std::invalid_argument("ArticulatedBody::AddRigidBody(): Parent rigid body with id " + std::to_string(id_parent) + " does not exist.");
  }
  RigidBody rb = rb_in;
  rb.id_ = id;
  rb.id_parent_ = id_parent;
  rigid_bodies_.push_back(std::move(rb));
  ExpandDof(id, id_parent);
  return id;
}

void ArticulatedBody::ExpandDof(int id, int id_parent) {
  dof_++;
  T_to_parent_.resize(dof_);
  T_from_parent_.resize(dof_);
  T_to_world_.resize(dof_);

  // Expand state size while leaving old values in place
  if (q_.size() < static_cast<int>(dof_)) q_.conservativeResize(dof_);
  if (dq_.size() < static_cast<int>(dof_)) dq_.conservativeResize(dof_);
  if (ddq_.size() < static_cast<int>(dof_)) ddq_.conservativeResize(dof_);

  // Set default state to zero
  q_(id) = 0.;
  dq_(id) = 0.;
  ddq_(id) = 0.;

  // Compute transforms for added rigid body
  T_to_parent_[id] = rigid_bodies_[id].T_to_parent();
  T_from_parent_[id] = T_to_parent_[id].inverse();
  if (id_parent < 0) {
    T_to_world_[id] = T_base_to_world_ * T_to_parent_[id];
  } else {
    T_to_world_[id] = T_to_world_[id_parent] * T_to_parent_[id];
  }

  // Update ancestors and subtrees
  if (id_parent < 0) {
    ancestors_.push_back({id});
  } else {
    std::vector<int> ancestors = ancestors_[id_parent];
    for (int ancestor : ancestors) {
      subtrees_[ancestor].push_back(id);
    }
    ancestors.push_back(id);
    ancestors_.push_back(std::move(ancestors));
  }
  subtrees_.push_back({id});

  // Resize caches
  vel_data_.is_computed = false;
  vel_data_.v.push_back(SpatialMotiond());

  jac_data_.is_computed = false;
  jac_data_.J.resize(6, dof_);

  cc_data_.is_computed = false;
  cc_data_.c_c.push_back(SpatialMotiond());
  cc_data_.f_c.push_back(SpatialForced());
  cc_data_.C.resize(dof_);

  grav_data_.is_computed = false;
  grav_data_.f_g.push_back(SpatialForced());
  grav_data_.G.resize(dof_);
  grav_data_.is_friction_computed = false;
  grav_data_.F.resize(dof_);

  rnea_data_.a.push_back(SpatialMotiond());
  rnea_data_.f.push_back(SpatialForced());

  crba_data_.is_computed = false;
  crba_data_.I_c.push_back(SpatialInertiad());
  crba_data_.A.resize(dof_, dof_);

  crba_data_.is_inv_computed = false;

  aba_data_.is_computed = false;
  aba_data_.I_a.push_back(SpatialInertiaMatrixd());
  aba_data_.h.push_back(SpatialForced());
  aba_data_.d.push_back(0);

  aba_data_.is_A_inv_computed = false;
  aba_data_.A_inv.resize(dof_, dof_);
  aba_data_.P.push_back(SpatialForceXd());
  aba_data_.A.push_back(SpatialMotionXd());
  for (SpatialForceXd& P_i : aba_data_.P) {
    P_i.resize(6, dof_);
  }
  for (SpatialMotionXd& A_i : aba_data_.A) {
    A_i.resize(6, dof_);
  }

  opspace_data_.is_lambda_computed = false;
  opspace_data_.is_lambda_inv_computed = false;
  opspace_data_.is_jbar_computed = false;

  opspace_aba_data_.is_lambda_computed = false;
  opspace_aba_data_.is_lambda_inv_computed = false;
  opspace_aba_data_.p.push_back(SpatialForce6d());
  opspace_aba_data_.u.push_back(Eigen::Matrix<double,1,6>());
}

void ArticulatedBody::CalculateTransforms() {
  for (size_t i = 0; i < rigid_bodies_.size(); i++) {
    const RigidBody& rb = rigid_bodies_[i];
    T_to_parent_[i] = rb.T_to_parent() * rb.joint().T_joint(q_(i));
    T_from_parent_[i] = T_to_parent_[i].inverse();
    if (rb.id_parent() < 0) {
      T_to_world_[i] = T_base_to_world_ * T_to_parent_[i];
    } else {
      T_to_world_[i] = T_to_world_[rb.id_parent()] * T_to_parent_[i];
    }
  }
}

std::ostream& operator<<(std::ostream& os, const ArticulatedBody& ab) {
  os << "ArticulatedBody(name=\"" << ab.name << "\", dof=" << ab.dof() << ")" << std::endl;
  os << "             q: " << ab.q().transpose() << std::endl;
  os << "            dq: " << ab.dq().transpose() << std::endl;
  os << "           ddq: " << ab.dq().transpose() << std::endl;
  os << "           tau: " << ab.dq().transpose() << std::endl;
  os << "             g: " << ab.g().linear().transpose() << std::endl;
  bool first = true;
  for (const RigidBody& rb : ab.rigid_bodies()) {
    os << (first ? "  rigid_bodies: " : "                ") << rb << std::endl;
    first = false;
  }
  return os;
}

}  // namespace SpatialDyn
