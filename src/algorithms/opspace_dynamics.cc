/**
 * opspace_dynamics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/opspace_dynamics.h"

#include <exception>  // std::invalid_argument

#include "algorithms/forward_dynamics.h"
#include "algorithms/forward_kinematics.h"
#include "algorithms/inverse_dynamics.h"
#include "structs/articulated_body_cache.h"

namespace spatial_dyn {
namespace opspace {

bool IsSingular(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double svd_epsilon) {
  auto& ops = ab.cache_->opspace_data_;

  Inertia(ab, J, svd_epsilon);

  return ops.is_singular;
}

Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                const Eigen::VectorXd& ddx, Eigen::MatrixXd *N,
                                const std::map<size_t, SpatialForced>& f_external,
                                const InverseDynamicsOptions& options) {
  // Project Jacobian in nullspace
  Eigen::MatrixXd JN;
  bool use_nullspace = N != nullptr && N->size() > 0;
  if (use_nullspace) {
    JN = J * (*N);
  }
  const Eigen::MatrixXd& J_x = (use_nullspace) ? JN : J;

  // Compute opspace dynamics
  Eigen::VectorXd F_x = Inertia(ab, J_x, options.svd_epsilon) * ddx;
  if (options.f_acc_max > 0.) {
    if (F_x.norm() > options.f_acc_max) {
      F_x = options.f_acc_max * F_x.normalized();
    }
  }

  if (!f_external.empty()) {
    F_x += ExternalForces(ab, J_x, f_external, options.svd_epsilon);
  }

  if (options.gravity) {
    F_x += Gravity(ab, J_x, options.svd_epsilon);
  }

  // TODO: Only works for 6d pos_ori tasks at the origin of the ee frame
  if (options.centrifugal_coriolis) {
    F_x += CentrifugalCoriolis(ab, J_x, options.svd_epsilon);
  }

  if (options.friction) {
    F_x += Friction(ab, J_x, J_x.transpose() * F_x, options.svd_epsilon, options.stiction_epsilon);
  }

  // Update nullspace
  if (N != nullptr) {
    const Eigen::MatrixXd& J_bar = JacobianDynamicInverse(ab, J_x, options.svd_epsilon);
    if (N->size() == 0) {
      *N = Eigen::MatrixXd::Identity(ab.dof(), ab.dof()) - J_bar * J_x;
    } else {
      *N = (Eigen::MatrixXd::Identity(ab.dof(), ab.dof()) - J_bar * J_x) * (*N);
    }
  }

  return J_x.transpose() * F_x;
}


const Eigen::MatrixXd& Inertia(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                               double svd_epsilon) {
  auto& ops = ab.cache_->opspace_data_;

  if (!ops.is_lambda_computed || ops.J.size() != J.size() || ops.J != J ||
      ops.svd_epsilon != svd_epsilon) {
    ops.Lambda = ctrl_utils::PseudoInverse(InertiaInverse(ab, J), svd_epsilon, &ops.is_singular);
    ops.svd_epsilon = svd_epsilon;
    ops.is_lambda_computed = true;
  }

  return ops.Lambda;
}

const Eigen::MatrixXd& InertiaInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J) {
  auto& ops = ab.cache_->opspace_data_;

  if (!ops.is_lambda_inv_computed || ops.J.size() != J.size() || ops.J != J) {
    ops.A_inv_J_T = spatial_dyn::InertiaInverse(ab).solve(J.transpose());
    ops.Lambda_inv = J * ops.A_inv_J_T;
    ops.J = J;
    ops.is_lambda_inv_computed = true;
    ops.is_lambda_computed = false;
    ops.is_jbar_computed = false;
  }

  return ops.Lambda_inv;
}

const Eigen::MatrixXd& JacobianDynamicInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                              double svd_epsilon) {
  auto& ops = ab.cache_->opspace_data_;

  if (!ops.is_jbar_computed || ops.J.size() != J.size() || ops.J != J ||
      ops.svd_epsilon != svd_epsilon) {
    // TODO: Force recompute
    opspace::Inertia(ab, J, svd_epsilon);
    ops.J_bar = ops.A_inv_J_T * ops.Lambda;
    ops.is_jbar_computed = true;
  }

  return ops.J_bar;
}

// TODO: Fix for compatibility with InverseDynamics (arbitrary Jacobian)
Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                    int idx_link, const Eigen::Vector3d&,
                                    double svd_epsilon) {
  if (J.rows() != 6) {
    throw std::invalid_argument("opspace::CentrifugalCoriolis(): Jacobian must be the basic Jacobian at the origin of the desired link.");
  }

  if (idx_link < 0) idx_link += ab.dof();
  auto& cc = ab.cache_->cc_data_;

  // Compute joint space centrifugal/Coriolis first to cache cc.c
  Eigen::Vector6d mu = JacobianDynamicInverse(ab, J, svd_epsilon).transpose() *
                       spatial_dyn::CentrifugalCoriolis(ab);
  mu -= opspace::Inertia(ab, J) *
        (Eigen::Isometry3d(ab.T_to_world(idx_link).linear()) * cc.c_c[idx_link]).matrix();
  return mu;
}

Eigen::VectorXd Gravity(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                        double svd_epsilon) {
  return JacobianDynamicInverse(ab, J, svd_epsilon).transpose() * Gravity(ab);
}

Eigen::VectorXd ExternalForces(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                               const std::map<size_t, SpatialForced>& f_external,
                               double svd_epsilon) {
  return JacobianDynamicInverse(ab, J, svd_epsilon).transpose() * ExternalTorques(ab, f_external);
}

Eigen::VectorXd Friction(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                         Eigen::Ref<const Eigen::VectorXd> tau, double svd_epsilon,
                         double stiction_epsilon) {
  return JacobianDynamicInverse(ab, J, svd_epsilon).transpose() * Friction(ab, tau, stiction_epsilon);
}

}  // namespace opspace
}  // namespace spatial_dyn
