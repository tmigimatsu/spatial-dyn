/**
 * articulated_body.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 8, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ARTICULATED_BODY_H_
#define SPATIAL_DYN_ARTICULATED_BODY_H_

#include "spatial_math.h"

#include <exception>  // std::invalid_argument
#include <string>     // std::string, std::to_string
#include <vector>     // std::vector
#include <iostream> // TODO: Remove

namespace SpatialDyn {

enum class JointType { UNDEFINED, RX, RY, RZ, PX, PY, PZ };

class Joint {

 public:

  Joint() {}
  Joint(JointType type) {
    set_type(type);
  }

  void set_type(JointType type) {
    type_ = type;
    switch (type_) {
      case JointType::RX:
        subspace_ = SpatialMotiond::UnitAngX();
        break;
      case JointType::RY:
        subspace_ = SpatialMotiond::UnitAngY();
        break;
      case JointType::RZ:
        subspace_ = SpatialMotiond::UnitAngZ();
        break;
      case JointType::PX:
        subspace_ = SpatialMotiond::UnitLinX();
        break;
      case JointType::PY:
        subspace_ = SpatialMotiond::UnitLinY();
        break;
      case JointType::PZ:
        subspace_ = SpatialMotiond::UnitLinZ();
        break;
      default:
        subspace_ = SpatialMotiond::Zero();
    }
  }
  JointType type() const {
    return type_;
  }

  // Motion subspace
  const SpatialMotiond& subspace() const {
    return subspace_;
  }

  Eigen::Affine3d T_joint(double q) const {
    switch (type_) {
      case JointType::RX:
        return Eigen::Affine3d(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitX()));
      case JointType::RY:
        return Eigen::Affine3d(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitY()));
      case JointType::RZ:
        return Eigen::Affine3d(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitZ()));
      case JointType::PX:
        return Eigen::Affine3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitX()));
      case JointType::PY:
        return Eigen::Affine3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitY()));
      case JointType::PZ:
        return Eigen::Affine3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitZ()));
      default:
        return Eigen::Affine3d::Identity();
    }
  }

 protected:

  JointType type_ = JointType::UNDEFINED;
  SpatialMotiond subspace_ = SpatialMotiond(0, 0, 0, 0, 0, 0);

};

class RigidBody;
class ArticulatedBody;
namespace Opspace {
  const Eigen::Matrix6d& Inertia(const ArticulatedBody& ab, int idx_link, const Eigen::Vector3d& offset, double);
  Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, int idx_link, const Eigen::Vector3d& offset, double tolerance);
  Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, const Eigen::MatrixXd&, int idx_link, const Eigen::Vector3d& offset, double);
  Eigen::Vector6d Gravity(ArticulatedBody& ab, int idx_link, const Eigen::Vector3d& offset, double);
}

class ArticulatedBody {

 public:

  ArticulatedBody(const std::string& name) : name(name) {}

  std::string name;

  size_t dof() const {
    return dof_;
  }

  void set_T_base_to_world(const Eigen::Quaterniond& ori_in_parent,
                      const Eigen::Vector3d& pos_in_parent) {
    T_base_to_world_ = Eigen::Translation3d(pos_in_parent) * ori_in_parent;
  }
  void set_T_base_to_world(const Eigen::Affine3d& T_to_parent) {
    T_base_to_world_ = T_to_parent;
  }
  const Eigen::Affine3d& T_base_to_world() const {
    return T_base_to_world_;
  }

  int AddRigidBody(RigidBody&& rb, int id_parent = -1);

  const std::vector<RigidBody>& rigid_bodies() const {
    return rigid_bodies_;
  }
  const RigidBody& rigid_bodies(int i) const {
    return rigid_bodies_[i];
  }
  const std::vector<int>& ancestors(int i) const {
    return ancestors_[i];
  }

  void set_q(const Eigen::VectorXd& q) {
    if (q.size() != dof_) {
      throw std::invalid_argument("ArticulatedBody::set_state(): q must be of size " + std::to_string(dof_));
    }
    q_ = q;

    CalculateTransforms();
    vel_data_.is_computed = false;
    cc_data_.is_vel_computed = false;
    cc_data_.is_force_computed = false;
    grav_data_.is_computed = false;
    crba_data_.is_computed = false;
    crba_data_.is_inv_computed = false;
    aba_data_.is_computed = false;
    opspace_data_.is_computed = false;
  }
  void set_dq(const Eigen::VectorXd& dq) {
    if (dq.size() != dof_) {
      throw std::invalid_argument("ArticulatedBody::set_state(): q must be of size " + std::to_string(dof_));
    }
    dq_ = dq;

    vel_data_.is_computed = false;
    cc_data_.is_vel_computed = false;
    cc_data_.is_force_computed = false;
    opspace_data_.is_computed = false;
  }

  const Eigen::VectorXd& q() const {
    return q_;
  }
  const Eigen::VectorXd& dq() const {
    return dq_;
  }
  const double dq(int i) const {
    return dq_(i);
  }
  const SpatialMotiond& g() const {
    return g_;
  }
  void set_g(const Eigen::Vector3d& g) {
    g_ << g, Eigen::Vector3d::Zero();
    grav_data_.is_computed = false;
  }
  const Eigen::Affine3d& T_to_parent(int i) const {
    return T_to_parent_[i];
  }
  const Eigen::Affine3d& T_to_world(int i) const {
    return T_to_world_[i];
  }
  const std::vector<int>& subtree(int i) const {
    return subtrees_[i];
  }

  void CalculateTransforms();

 protected:

  size_t dof_ = 0;
  Eigen::Affine3d T_base_to_world_ = Eigen::Affine3d::Identity();
  std::vector<RigidBody> rigid_bodies_;

  Eigen::VectorXd q_;
  Eigen::VectorXd dq_;
  Eigen::VectorXd ddq_;
  Eigen::VectorXd tau_;
  SpatialMotionXd J_;
  SpatialMotiond g_ = -9.81 * SpatialMotiond::UnitLinZ();
  std::vector<Eigen::Affine3d> T_to_parent_;
  std::vector<Eigen::Affine3d> T_to_world_;
  std::vector<std::vector<int>> ancestors_;  // Ancestors from base to link i
  std::vector<std::vector<int>> subtrees_;


  struct VelocityData {
    bool is_computed = false;  // Reusable with same position, velocity
    std::vector<SpatialMotiond> v;  // Rigid body velocities
  };
  mutable VelocityData vel_data_;

  struct CentrifugalCoriolisData {
    bool is_vel_computed = false;   // Reusable with same position, velocity
    std::vector<SpatialMotiond> c;  // Rigid body centrifugal accelerations
    std::vector<SpatialForced> b;   // Rigid body Coriolis forces

    bool is_force_computed = false;    // Reusable with same position, velocity
    std::vector<SpatialMotiond> c_c;   // Composite centrifugal accelerations
    std::vector<SpatialForced> f_c;    // Rigid body centrifugal and Coriolis forces
    Eigen::VectorXd C;                 // Joint space centrifugal and Coriolis forces
  };
  mutable CentrifugalCoriolisData cc_data_;

  struct GravityData {
    bool is_computed = false;        // Reusable with same position, gravity
    std::vector<SpatialForced> f_g;  // Rigid body gravity force
    Eigen::VectorXd G;               // Joint space gravity
  };
  mutable GravityData grav_data_;

  struct RneaData {
    std::vector<SpatialMotiond> a;  // Rigid body accelerations
    std::vector<SpatialForced> f;   // Rigid body forces
  };
  mutable RneaData rnea_data_;

  struct CrbaData {
    bool is_computed = false;          // Reusable with same position
    std::vector<SpatialInertiad> I_c;  // Composite inertia
    Eigen::MatrixXd A;                 // Joint space inertia

    bool is_inv_computed = false;    // Reusable with same position
    Eigen::LDLT<Eigen::MatrixXd> A_inv;  // Robust Cholesky decomposition of joint space inertia
  };
  mutable CrbaData crba_data_;

  struct AbaData {
    bool is_computed = false;  // Reusable with same position
    std::vector<SpatialInertiaMatrixd> I_a;
    std::vector<SpatialForced> h;
    std::vector<double> d;

    // Not reusable
    std::vector<SpatialForced> p;
    std::vector<double> u;
    std::vector<SpatialMotiond> a;
  };
  mutable AbaData aba_data_;

  struct OpspaceData {
    bool is_computed = false;  // Reusable with same position, velocity
    std::vector<SpatialForce6d> p;
    std::vector<Eigen::Matrix<double,1,6>> u;

    int idx_link;
    Eigen::Vector3d offset;
    double tolerance;
    Eigen::Matrix6d Lambda_inv;
    Eigen::Matrix6d Lambda;
  };
  mutable OpspaceData opspace_data_;

  friend Eigen::VectorXd InverseDynamics(const ArticulatedBody&, const Eigen::MatrixXd&);
  friend const Eigen::VectorXd& CentrifugalCoriolis(const ArticulatedBody&);
  friend const Eigen::VectorXd& Gravity(const ArticulatedBody&);
  friend const Eigen::MatrixXd& Inertia(const ArticulatedBody&);
  friend const Eigen::LDLT<Eigen::MatrixXd>& InertiaInverse(const ArticulatedBody&);
  friend Eigen::VectorXd ForwardDynamics(const ArticulatedBody&, const Eigen::VectorXd&);
  friend Eigen::VectorXd ForwardDynamicsAba(const ArticulatedBody&, const Eigen::VectorXd&);
  friend void AbaPrecomputeVelocityInertia(const ArticulatedBody&);
  friend Eigen::Vector6d Opspace::CentrifugalCoriolis(const ArticulatedBody&, int, const Eigen::Vector3d&, double);
  friend Eigen::Vector6d Opspace::CentrifugalCoriolis(const ArticulatedBody&, const Eigen::MatrixXd& J, int, const Eigen::Vector3d&, double);
  friend const Eigen::Matrix6d& Opspace::Inertia(const ArticulatedBody&, int, const Eigen::Vector3d&, double);
  friend Eigen::Vector6d Opspace::Gravity(ArticulatedBody&, int, const Eigen::Vector3d&, double);

};

class RigidBody {

 public:

  RigidBody(const std::string& name) : name(name) {}

  std::string name;

  int id() const {
    return id_;
  }
  int id_parent() const {
    return id_parent_;
  }

  void set_T_to_parent(const Eigen::Quaterniond& ori_in_parent,
                       const Eigen::Vector3d& pos_in_parent) {
    T_to_parent_ = Eigen::Translation3d(pos_in_parent) * ori_in_parent;
  }
  void set_T_to_parent(const Eigen::Affine3d& T_to_parent) {
    T_to_parent_ = T_to_parent;
  }
  const Eigen::Affine3d& T_to_parent() const {
    return T_to_parent_;
  }

  template<typename ComDerived, typename IComDerived>
  void set_inertia(double mass,
                   const Eigen::MatrixBase<ComDerived>& com,
                   const Eigen::MatrixBase<IComDerived>& I_com_flat) {
    inertia_ = SpatialInertiad(mass, com, I_com_flat);
  }
  const SpatialInertiad& inertia() const {
    return inertia_;
  }

  void set_joint(Joint&& joint) {
    joint_ = joint;
  }
  const Joint& joint() const {
    return joint_;
  }

 protected:

  int id_ = -1;
  int id_parent_ = -1;
  Eigen::Affine3d T_to_parent_ = Eigen::Affine3d::Identity();
  SpatialInertiad inertia_ = SpatialInertiad(1, Eigen::Vector3d::Zero(), Eigen::Vector6d::Zero());
  Joint joint_;

  friend int ArticulatedBody::AddRigidBody(RigidBody &&, int);

};

int ArticulatedBody::AddRigidBody(RigidBody&& rb, int id_parent) {
  // TODO: Set default state
  int id = rigid_bodies_.size();
  if ((id_parent < 0 || id_parent >= id) && id != 0) {
    throw std::invalid_argument("ArticulatedBody::AddRigidBody(): Parent rigid body with id " + std::to_string(id_parent) + " does not exist.");
  }
  rb.id_ = id;
  rb.id_parent_ = id_parent;
  rigid_bodies_.push_back(std::move(rb));

  dof_++;
  T_to_parent_.resize(dof_);
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

  cc_data_.is_vel_computed = false;
  cc_data_.c.push_back(SpatialMotiond());
  cc_data_.b.push_back(SpatialForced());

  cc_data_.is_force_computed = false;
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

  opspace_data_.is_computed = false;
  opspace_data_.p.push_back(SpatialForce6d());
  opspace_data_.u.push_back(Eigen::Matrix<double,1,6>());

  return id;
}

void ArticulatedBody::CalculateTransforms() {
  for (size_t i = 0; i < rigid_bodies_.size(); i++) {
    const RigidBody& rb = rigid_bodies_[i];
    T_to_parent_[i] = rb.T_to_parent() * rb.joint().T_joint(q_(i));
    T_to_world_[i] = (i > 0) ? T_to_world_[i-1] * T_to_parent_[i] : T_to_parent_[i];
  }
}

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_ARTICULATED_BODY_H_