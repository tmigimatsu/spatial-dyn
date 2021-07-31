/**
 * articulated_body_cache.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: December 11, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_STRUCTS_ARTICULATED_BODY_CACHE_H_
#define SPATIAL_DYN_STRUCTS_ARTICULATED_BODY_CACHE_H_

#include "spatial_dyn/structs/articulated_body.h"

#include <vector>  // std::vector

namespace spatial_dyn {

/// @cond
struct ArticulatedBody::Cache {

  struct TransformData {
    bool is_T_to_parent_computed = false;  // Reusable with same position
    std::vector<Eigen::Isometry3d> T_to_parent;

    bool is_T_to_world_computed = false;  // Reusable with same position
    std::vector<Eigen::Isometry3d> T_to_world;
  };
  TransformData T_data_;

  struct VelocityData {
    bool is_computed = false;       // Reusable with same position, velocity
    std::vector<SpatialMotiond> v;  // Rigid body velocities
  };
  VelocityData vel_data_;

  struct JacobianData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int link;
    Eigen::Vector3d offset;

    bool is_computed = false;  // Reusable with same position, link, offset
    Eigen::Matrix6Xd J;
  };
  JacobianData jac_data_;

  struct CentrifugalCoriolisData {
    bool is_computed = false;    // Reusable with same position, velocity
    std::vector<SpatialMotiond> c_c;   // Composite centrifugal accelerations
    std::vector<SpatialForced> f_c;    // Rigid body centrifugal and Coriolis forces
    Eigen::VectorXd C;                 // Joint space centrifugal and Coriolis forces
  };
  CentrifugalCoriolisData cc_data_;

  struct GravityData {
    bool is_computed = false;        // Reusable with same position, gravity
    std::vector<SpatialForced> f_g;  // Rigid body gravity force
    Eigen::VectorXd G;               // Joint space gravity
  };
  GravityData grav_data_;

  struct RneaData {
    std::vector<SpatialMotiond> a;  // Rigid body accelerations
    std::vector<SpatialForced> f;   // Rigid body forces
  };
  RneaData rnea_data_;

  struct CrbaData {
    bool is_computed = false;          // Reusable with same position
    std::vector<SpatialInertiad> I_c;  // Composite inertia
    Eigen::MatrixXd A;                 // Joint space inertia

    bool is_inv_computed = false;        // Reusable with same position
    Eigen::LDLT<Eigen::MatrixXd> A_inv;  // Robust Cholesky decomposition of joint space inertia
  };
  CrbaData crba_data_;

  struct AbaData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool is_computed = false;  // Reusable with same position
    std::vector<SpatialInertiaMatrixd> I_a;
    std::vector<SpatialForced> h;
    std::vector<double> d;

    bool is_A_inv_computed = false;  // Reusable with same position
    Eigen::MatrixXd A_inv;           // Inverse inertia computed with ABA
    std::vector<SpatialForceXd> P;
    std::vector<SpatialMotionXd> A;
  };
  AbaData aba_data_;

  struct OpspaceData {
    Eigen::MatrixXd J;
    double svd_epsilon;

    bool is_lambda_computed = false;
    Eigen::MatrixXd Lambda;

    bool is_lambda_inv_computed = false;
    Eigen::MatrixXd Lambda_inv;
    Eigen::MatrixXd A_inv_J_T;
    bool is_singular;

    bool is_jbar_computed = false;
    Eigen::MatrixXd J_bar;
  };
  OpspaceData opspace_data_;

  struct OpspaceAbaData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int idx_link;
    Eigen::Vector3d offset;
    double svd_epsilon;

    bool is_lambda_computed = false;  // Reusable with same position
    Eigen::Matrix6d Lambda;

    bool is_lambda_inv_computed = false;  // Reusable with same position
    Eigen::Matrix6d Lambda_inv;

    std::vector<SpatialForce6d> p;
    std::vector<Eigen::Matrix<double,1,6>> u;
  };
  OpspaceAbaData opspace_aba_data_;

  struct DrneaData {
    Eigen::VectorXd q_prev;
    double dt = 0.;
    bool is_q_prev_valid = false;

    std::vector<Eigen::Isometry3d> T_to_prev;
    std::vector<SpatialForced> p_prev;
    bool is_prev_computed = false;  // Reusable with same q_prev, position

    std::vector<Eigen::Isometry3d> T_from_next;
    std::vector<SpatialForced> p;
    std::vector<SpatialForced> dp;
  };
  DrneaData drnea_data_;

};
/// @endcond

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_STRUCTS_ARTICULATED_BODY_CACHE_H_
