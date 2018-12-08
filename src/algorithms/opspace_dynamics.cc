/**
 * opspace_dynamics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/opspace_dynamics.h"

#include "algorithms/forward_dynamics.h"
#include "algorithms/forward_kinematics.h"
#include "algorithms/inverse_dynamics.h"

namespace SpatialDyn {
namespace Opspace {

Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                const Eigen::VectorXd& ddx, Eigen::MatrixXd *N,
                                const std::vector<std::pair<int, SpatialForced>>& f_external,
                                bool gravity, bool centrifugal_coriolis, bool friction,
                                double svd_epsilon) {
  // Project Jacobian in nullspace
  Eigen::MatrixXd JN;
  bool use_nullspace = N != nullptr && N->size() > 0;
  if (use_nullspace) {
    JN = J * (*N);
  }
  const Eigen::MatrixXd& J_x = (use_nullspace) ? JN : J;

  // Compute opspace dynamics
  Eigen::VectorXd F_x;
  if (gravity) {  // Compute inline for efficiency
    F_x = Inertia(ab, J_x, svd_epsilon) * ddx + Gravity(ab, J_x, f_external, svd_epsilon);
  } else {
    F_x = Inertia(ab, J_x, svd_epsilon) * ddx;
  }

  // TODO: Only works for 6d pos_ori tasks at the origin of the ee frame
  if (centrifugal_coriolis && F_x.size() == 6) {
    F_x += CentrifugalCoriolis(ab, J_x, svd_epsilon);
  }

  if (friction) {
    F_x += Friction(ab, J_x, svd_epsilon);
  }

  // Update nullspace
  if (N != nullptr) {
    const Eigen::MatrixXd& J_bar = JacobianDynamicInverse(ab, J_x, svd_epsilon);
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
  auto& ops = ab.opspace_data_;

  if (!ops.is_lambda_computed || ops.J.size() != J.size() || ops.J != J ||
      ops.svd_epsilon != svd_epsilon) {
    ops.Lambda = Eigen::PseudoInverse(InertiaInverse(ab, J), svd_epsilon);
    ops.svd_epsilon = svd_epsilon;
    ops.is_lambda_computed = true;
  }

  return ops.Lambda;
}

const Eigen::MatrixXd& InertiaInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J) {
  auto& ops = ab.opspace_data_;

  if (!ops.is_lambda_inv_computed || ops.J.size() != J.size() || ops.J != J) {
    ops.A_inv_J_bar_T = SpatialDyn::InertiaInverse(ab).solve(J.transpose());
    ops.Lambda_inv = J * ops.A_inv_J_bar_T;
    ops.J = J;
    ops.is_lambda_inv_computed = true;
    ops.is_lambda_computed = false;
    ops.is_jbar_computed = false;
  }

  return ops.Lambda_inv;
}

const Eigen::MatrixXd& JacobianDynamicInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                              double svd_epsilon) {
  auto& ops = ab.opspace_data_;

  if (!ops.is_jbar_computed || ops.J != J || ops.svd_epsilon != svd_epsilon) {
    // TODO: Force recompute
    Opspace::Inertia(ab, J, svd_epsilon);
    ops.J_bar = ops.A_inv_J_bar_T * ops.Lambda;
    ops.is_jbar_computed = true;
  }

  return ops.J_bar;
}

// TODO: Fix for compatibility with InverseDynamics (arbitrary Jacobian)
Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                    int idx_link, const Eigen::Vector3d&,
                                    double svd_epsilon) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& cc = ab.cc_data_;

  // Compute joint space centrifugal/Coriolis first to cache cc.c
  Eigen::Vector6d mu = JacobianDynamicInverse(ab, J, svd_epsilon).transpose() *
                       SpatialDyn::CentrifugalCoriolis(ab);
  mu -= Opspace::Inertia(ab, J) *
        (Eigen::Isometry3d(ab.T_to_world(idx_link).linear()) * cc.c_c[idx_link]).matrix();
  return mu;
}

Eigen::VectorXd Gravity(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                        const std::vector<std::pair<int, SpatialForced>>& f_external,
                        double svd_epsilon) {
  return JacobianDynamicInverse(ab, J, svd_epsilon).transpose() * Gravity(ab, f_external);
}

Eigen::VectorXd Friction(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                         double svd_epsilon) {
  return JacobianDynamicInverse(ab, J, svd_epsilon).transpose() * Friction(ab);
}

}  // namespace Opspace
}  // namespace SpatialDyn
