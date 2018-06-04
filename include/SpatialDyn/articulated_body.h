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
#include "joint.h"
#include "rigid_body.h"
#include "opspace_dynamics.h"

#include <string>  // std::string
#include <vector>  // std::vector
#include <iostream> // TODO: Remove

namespace SpatialDyn {

class ArticulatedBody {

 public:

  ArticulatedBody(const std::string& name);

  std::string name;

  size_t dof() const;

  void set_T_base_to_world(const Eigen::Quaterniond& ori_in_parent,
                           const Eigen::Vector3d& pos_in_parent);
  void set_T_base_to_world(const Eigen::Affine3d& T_to_parent);
  const Eigen::Affine3d& T_base_to_world() const;

  int AddRigidBody(RigidBody&& rb, int id_parent = -1);

  const std::vector<RigidBody>& rigid_bodies() const;
  const RigidBody& rigid_bodies(int i) const;
  const std::vector<int>& ancestors(int i) const;


  const Eigen::VectorXd& q() const;
  const double q(int i) const;
  void set_q(const Eigen::VectorXd& q);

  const Eigen::VectorXd& dq() const;
  const double dq(int i) const;
  void set_dq(const Eigen::VectorXd& dq);

  const SpatialMotiond& g() const;
  void set_g(const Eigen::Vector3d& g);

  const Eigen::Affine3d& T_to_parent(int i) const;
  const Eigen::Affine3d& T_to_world(int i) const;

  const std::vector<int>& subtree(int i) const;

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
    Eigen::MatrixXd J;
    double tolerance;

    bool is_lambda_computed = false;
    Eigen::MatrixXd Lambda;

    bool is_lambda_inv_computed = false;
    Eigen::MatrixXd Lambda_inv;

    bool is_jbar_computed = false;
    Eigen::MatrixXd J_bar;
  };
  mutable OpspaceData opspace_data_;

  struct OpspaceAbaData {
    int idx_link;
    Eigen::Vector3d offset;
    double tolerance;

    bool is_lambda_computed = false;  // Reusable with same position
    Eigen::Matrix6d Lambda;

    bool is_lambda_inv_computed = false;  // Reusable with same position
    Eigen::Matrix6d Lambda_inv;

    std::vector<SpatialForce6d> p;
    std::vector<Eigen::Matrix<double,1,6>> u;
  };
  mutable OpspaceAbaData opspace_aba_data_;

  friend Eigen::VectorXd InverseDynamics(const ArticulatedBody&, const Eigen::MatrixXd&);
  friend const Eigen::VectorXd& CentrifugalCoriolis(const ArticulatedBody&);
  friend const Eigen::VectorXd& Gravity(const ArticulatedBody&);
  friend const Eigen::MatrixXd& Inertia(const ArticulatedBody&);
  friend const Eigen::LDLT<Eigen::MatrixXd>& InertiaInverse(const ArticulatedBody&);

  friend Eigen::VectorXd ForwardDynamicsAba(const ArticulatedBody&, const Eigen::VectorXd&);

  friend const Eigen::MatrixXd& Opspace::Inertia(const ArticulatedBody&, const Eigen::MatrixXd& J, double);
  friend const Eigen::MatrixXd& Opspace::InertiaInverse(const ArticulatedBody&, const Eigen::MatrixXd& J);
  friend const Eigen::MatrixXd& Opspace::JacobianDynamicInverse(const ArticulatedBody&, const Eigen::MatrixXd& J, double);
  friend Eigen::Vector6d Opspace::CentrifugalCoriolis(const ArticulatedBody&, const Eigen::MatrixXd& J, int, const Eigen::Vector3d&, double);

  friend const Eigen::Matrix6d& Opspace::InertiaAba(const ArticulatedBody&, int, const Eigen::Vector3d&, double);
  friend const Eigen::Matrix6d& Opspace::InertiaInverseAba(const ArticulatedBody&, int, const Eigen::Vector3d&);
  friend Eigen::Vector6d Opspace::CentrifugalCoriolisAba(const ArticulatedBody&, int, const Eigen::Vector3d&, double);
  friend Eigen::Vector6d Opspace::GravityAba(const ArticulatedBody&, int, const Eigen::Vector3d&, double);

};

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_ARTICULATED_BODY_H_
