/**
 * articulated_body.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "articulated_body.h"

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
  return rigid_bodies_[i];
}
const std::vector<int>& ArticulatedBody::ancestors(int i) const {
  return ancestors_[i];
}

const Eigen::VectorXd& ArticulatedBody::q() const {
  return q_;
}
const double ArticulatedBody::q(int i) const {
  return q_(i);
}
void ArticulatedBody::set_q(const Eigen::VectorXd& q) {
  if (q.size() != dof_) {
    throw std::invalid_argument("ArticulatedBody::set_state(): q must be of size " + std::to_string(dof_));
  }
  q_ = q;

  CalculateTransforms();
  vel_data_.is_computed = false;
  cc_data_.is_computed = false;
  grav_data_.is_computed = false;
  crba_data_.is_computed = false;
  crba_data_.is_inv_computed = false;
  aba_data_.is_computed = false;
  opspace_data_.is_lambda_computed = false;
  opspace_data_.is_lambda_inv_computed = false;
  opspace_data_.is_jbar_computed = false;
  opspace_aba_data_.is_lambda_computed = false;
  opspace_aba_data_.is_lambda_inv_computed = false;
}
void ArticulatedBody::set_q(Eigen::VectorXd&& q) {
  if (q.size() != dof_) {
    throw std::invalid_argument("ArticulatedBody::set_state(): q must be of size " + std::to_string(dof_));
  }
  q_ = std::move(q);

  CalculateTransforms();
  vel_data_.is_computed = false;
  cc_data_.is_computed = false;
  grav_data_.is_computed = false;
  crba_data_.is_computed = false;
  crba_data_.is_inv_computed = false;
  aba_data_.is_computed = false;
  opspace_data_.is_lambda_computed = false;
  opspace_data_.is_lambda_inv_computed = false;
  opspace_data_.is_jbar_computed = false;
  opspace_aba_data_.is_lambda_computed = false;
  opspace_aba_data_.is_lambda_inv_computed = false;
}

const Eigen::VectorXd& ArticulatedBody::dq() const {
  return dq_;
}
const double ArticulatedBody::dq(int i) const {
  return dq_(i);
}
void ArticulatedBody::set_dq(const Eigen::VectorXd& dq) {
  if (dq.size() != dof_) {
    throw std::invalid_argument("ArticulatedBody::set_state(): q must be of size " + std::to_string(dof_));
  }
  dq_ = dq;

  vel_data_.is_computed = false;
  cc_data_.is_computed = false;
}
void ArticulatedBody::set_dq(Eigen::VectorXd&& dq) {
  if (dq.size() != dof_) {
    throw std::invalid_argument("ArticulatedBody::set_state(): q must be of size " + std::to_string(dof_));
  }
  dq_ = std::move(dq);

  vel_data_.is_computed = false;
  cc_data_.is_computed = false;
}

const Eigen::VectorXd& ArticulatedBody::ddq() const {
  return ddq_;
}
const double ArticulatedBody::ddq(int i) const {
  return ddq_(i);
}
void ArticulatedBody::set_ddq(const Eigen::VectorXd& ddq) {
  ddq_ = ddq;
}
void ArticulatedBody::set_ddq(Eigen::VectorXd&& ddq) {
  ddq_ = std::move(ddq);
}

const Eigen::VectorXd& ArticulatedBody::tau() const {
  return tau_;
}
const double ArticulatedBody::tau(int i) const {
  return tau_(i);
}
void ArticulatedBody::set_tau(const Eigen::VectorXd& tau) {
  tau_ = tau;
}

const SpatialMotiond& ArticulatedBody::g() const {
  return g_;
}
void ArticulatedBody::set_g(const Eigen::Vector3d& g) {
  g_ << g, Eigen::Vector3d::Zero();
  grav_data_.is_computed = false;
}

const Eigen::Isometry3d& ArticulatedBody::T_to_parent(int i) const {
  return T_to_parent_[i];
}
const Eigen::Isometry3d& ArticulatedBody::T_from_parent(int i) const {
  return T_from_parent_[i];
}
const Eigen::Isometry3d& ArticulatedBody::T_to_world(int i) const {
  return T_to_world_[i];
}

const std::vector<int>& ArticulatedBody::subtree(int i) const {
  return subtrees_[i];
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
  // TODO: Set default state
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
  if (q_.size() < dof_) q_.resize(dof_); dq_.setZero();
  if (dq_.size() < dof_) dq_.resize(dof_); dq_.setZero();
  if (ddq_.size() < dof_) ddq_.resize(dof_); dq_.setZero();
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

  vel_data_.is_computed = false;
  vel_data_.v.push_back(SpatialMotiond());

  cc_data_.is_computed = false;
  cc_data_.c_c.push_back(SpatialMotiond());
  cc_data_.f_c.push_back(SpatialForced());
  cc_data_.C.resize(dof_);

  grav_data_.is_computed = false;
  grav_data_.f_g.push_back(SpatialForced());
  grav_data_.G.resize(dof_);

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

  aba_data_.p.push_back(SpatialForced());
  aba_data_.u.push_back(0);
  aba_data_.a.push_back(SpatialMotiond());

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
    T_to_world_[i] = (i > 0) ? T_to_world_[i-1] * T_to_parent_[i] : T_to_parent_[i];
  }
}

}  // namespace SpatialDyn