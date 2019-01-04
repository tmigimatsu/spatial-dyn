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

#include "structs/articulated_body_cache.h"

namespace SpatialDyn {

ArticulatedBody::ArticulatedBody() : cache_(new Cache()) {}

ArticulatedBody::ArticulatedBody(const std::string& name) : name(name), cache_(new Cache()) {}

ArticulatedBody::ArticulatedBody(const ArticulatedBody& ab)
    : name(ab.name),
      graphics(ab.graphics),
      cache_(new Cache(*ab.cache_)),
      dof_(ab.dof_),
      rigid_bodies_(ab.rigid_bodies_),
      q_(ab.q_),
      dq_(ab.dq_),
      g_(ab.g_),
      T_base_to_world_(ab.T_base_to_world_),
      ancestors_(ab.ancestors_),
      subtrees_(ab.subtrees_) {}

ArticulatedBody::~ArticulatedBody() {}

ArticulatedBody& ArticulatedBody::operator=(const ArticulatedBody& ab) {
  name = ab.name;
  graphics = ab.graphics;
  cache_.reset(new Cache(*ab.cache_));
  dof_ = ab.dof_;
  rigid_bodies_ = ab.rigid_bodies_;
  q_ = ab.q_;
  dq_ = ab.dq_;
  g_ = ab.g_;
  T_base_to_world_ = ab.T_base_to_world_;
  ancestors_ = ab.ancestors_;
  subtrees_ = ab.subtrees_;
  return *this;
}

const RigidBody& ArticulatedBody::rigid_bodies(int i) const {
  if (i < 0) i += dof();
  return rigid_bodies_[i];
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

  if (!cache_) return;
  cache_->T_data_.is_T_to_parent_computed = false;
  cache_->T_data_.is_T_to_world_computed = false;
  cache_->vel_data_.is_computed = false;
  cache_->jac_data_.is_computed = false;
  cache_->cc_data_.is_computed = false;
  cache_->grav_data_.is_computed = false;
  cache_->crba_data_.is_computed = false;
  cache_->crba_data_.is_inv_computed = false;
  cache_->aba_data_.is_computed = false;
  cache_->aba_data_.is_A_inv_computed = false;
  cache_->opspace_data_.is_lambda_computed = false;
  cache_->opspace_data_.is_lambda_inv_computed = false;
  cache_->opspace_data_.is_jbar_computed = false;
  cache_->opspace_aba_data_.is_lambda_computed = false;
  cache_->opspace_aba_data_.is_lambda_inv_computed = false;
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

  if (!cache_) return;
  cache_->vel_data_.is_computed = false;
  cache_->cc_data_.is_computed = false;
  cache_->aba_data_.is_computed = false;
}

void ArticulatedBody::set_g(const Eigen::Vector3d& g) {
  g_ << g, Eigen::Vector3d::Zero();
  if (!cache_) return;
  cache_->grav_data_.is_computed = false;
  cache_->aba_data_.is_computed = false;
}

void ArticulatedBody::set_T_base_to_world(const Eigen::Quaterniond& ori_in_world,
                                          const Eigen::Vector3d& pos_in_world) {
  T_base_to_world_ = Eigen::Translation3d(pos_in_world) * ori_in_world;
}
void ArticulatedBody::set_T_base_to_world(const Eigen::Isometry3d& T_to_world) {
  T_base_to_world_ = T_to_world;
}

const Eigen::Isometry3d& ArticulatedBody::T_to_parent(int i) const {
  if (i < 0) i += dof();
  auto& T = cache_->T_data_;
  if (!T.is_T_to_parent_computed) {
    for (size_t j = 0; j < rigid_bodies_.size(); j++) {
      const RigidBody& rb = rigid_bodies_[j];
      T.T_to_parent[j] = rb.T_to_parent() * rb.joint().T_joint(q_(j));
    }
    T.is_T_to_parent_computed = true;
  }
  return T.T_to_parent[i];
}
Eigen::Isometry3d ArticulatedBody::T_to_parent(int i, double q) const {
  if (i < 0) i += dof();
  const RigidBody& rb = rigid_bodies_[i];
  return rb.T_to_parent() * rb.joint().T_joint(q);
}

const Eigen::Isometry3d& ArticulatedBody::T_to_world(int i) const {
  if (i < 0) i += dof();
  auto& T = cache_->T_data_;
  if (!T.is_T_to_world_computed) {
    T_to_parent(0);  // Compute parent transforms
    for (size_t j = 0; j < rigid_bodies_.size(); j++) {
      const RigidBody& rb = rigid_bodies_[j];
      if (rb.id_parent() < 0) {
        T.T_to_world[j] = T_base_to_world_ * T.T_to_parent[j];
      } else {
        T.T_to_world[j] = T.T_to_world[rb.id_parent()] * T.T_to_parent[j];
      }
    }
    T.is_T_to_world_computed = true;
  }
  return T.T_to_world[i];
}
Eigen::Isometry3d ArticulatedBody::T_to_world(int i, Eigen::Ref<const Eigen::VectorXd> q) const {
  if (i < 0) i += dof();
  Eigen::Isometry3d T;
  while (i >= 0) {
    const RigidBody& rb = rigid_bodies_[i];
    T = T_to_parent(i, q(i)) * T;
    i = rb.id_parent();
  }
  return T_base_to_world_ * T;
}

const std::vector<int>& ArticulatedBody::ancestors(int i) const {
  if (i < 0) i += dof();
  return ancestors_[i];
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

  // Expand state size while leaving old values in place
  if (q_.size() < static_cast<int>(dof_)) q_.conservativeResize(dof_);
  if (dq_.size() < static_cast<int>(dof_)) dq_.conservativeResize(dof_);

  // Set default state to zero
  q_(id) = 0.;
  dq_(id) = 0.;

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
  if (!cache_) return;
  cache_->T_data_.is_T_to_parent_computed = false;
  cache_->T_data_.T_to_parent.push_back(Eigen::Isometry3d());
  cache_->T_data_.is_T_to_world_computed = false;
  cache_->T_data_.T_to_world.push_back(Eigen::Isometry3d());

  cache_->vel_data_.is_computed = false;
  cache_->vel_data_.v.push_back(SpatialMotiond());

  cache_->jac_data_.is_computed = false;
  cache_->jac_data_.J.resize(6, dof_);

  cache_->cc_data_.is_computed = false;
  cache_->cc_data_.c_c.push_back(SpatialMotiond());
  cache_->cc_data_.f_c.push_back(SpatialForced());
  cache_->cc_data_.C.resize(dof_);

  cache_->grav_data_.is_computed = false;
  cache_->grav_data_.f_g.push_back(SpatialForced());
  cache_->grav_data_.G.resize(dof_);

  cache_->rnea_data_.a.push_back(SpatialMotiond());
  cache_->rnea_data_.f.push_back(SpatialForced());

  cache_->crba_data_.is_computed = false;
  cache_->crba_data_.I_c.push_back(SpatialInertiad());
  cache_->crba_data_.A.resize(dof_, dof_);

  cache_->crba_data_.is_inv_computed = false;

  cache_->aba_data_.is_computed = false;
  cache_->aba_data_.I_a.push_back(SpatialInertiaMatrixd());
  cache_->aba_data_.h.push_back(SpatialForced());
  cache_->aba_data_.d.push_back(0);

  cache_->aba_data_.is_A_inv_computed = false;
  cache_->aba_data_.A_inv.resize(dof_, dof_);
  cache_->aba_data_.P.push_back(SpatialForceXd());
  cache_->aba_data_.A.push_back(SpatialMotionXd());
  for (SpatialForceXd& P_i : cache_->aba_data_.P) {
    P_i.resize(6, dof_);
  }
  for (SpatialMotionXd& A_i : cache_->aba_data_.A) {
    A_i.resize(6, dof_);
  }

  cache_->opspace_data_.is_lambda_computed = false;
  cache_->opspace_data_.is_lambda_inv_computed = false;
  cache_->opspace_data_.is_jbar_computed = false;

  cache_->opspace_aba_data_.is_lambda_computed = false;
  cache_->opspace_aba_data_.is_lambda_inv_computed = false;
  cache_->opspace_aba_data_.p.push_back(SpatialForce6d());
  cache_->opspace_aba_data_.u.push_back(Eigen::Matrix<double,1,6>());
}

std::ostream& operator<<(std::ostream& os, const ArticulatedBody& ab) {
  os << "ArticulatedBody(name=\"" << ab.name << "\", dof=" << ab.dof() << ")" << std::endl;
  os << "             q: " << ab.q().transpose() << std::endl;
  os << "            dq: " << ab.dq().transpose() << std::endl;
  os << "             g: " << ab.g().linear().transpose() << std::endl;
  bool first = true;
  for (const RigidBody& rb : ab.rigid_bodies()) {
    os << (first ? "  rigid_bodies: " : "                ") << rb << std::endl;
    first = false;
  }
  return os;
}

}  // namespace SpatialDyn
