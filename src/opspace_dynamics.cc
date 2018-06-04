/**
 * opspace_dynamics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "opspace_dynamics.h"
#include "articulated_body.h"
#include "forward_kinematics.h"
#include "inverse_dynamics.h"

namespace SpatialDyn {
namespace Opspace {

const Eigen::MatrixXd& Inertia(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                               double tolerance) {
  auto& ops = ab.opspace_data_;

  if (!ops.is_lambda_computed || ops.J != J || ops.tolerance != tolerance) {
    ops.Lambda = Eigen::PseudoInverse(InertiaInverse(ab, J), tolerance);
    ops.tolerance = tolerance;
    ops.is_lambda_computed = true;
  }

  return ops.Lambda;
}

const Eigen::MatrixXd& InertiaInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J) {
  auto& ops = ab.opspace_data_;

  if (!ops.is_lambda_inv_computed || ops.J != J) {
    ops.Lambda_inv = J * SpatialDyn::InertiaInverse(ab).solve(J.transpose());
    ops.J = J;
    ops.is_lambda_inv_computed = true;
    ops.is_lambda_computed = false;
    ops.is_jbar_computed = false;
  }

  return ops.Lambda_inv;
}

const Eigen::MatrixXd& JacobianDynamicInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                              double tolerance) {
  auto& ops = ab.opspace_data_;

  if (!ops.is_jbar_computed || ops.J != J) {
    ops.J_bar = InertiaInverse(ab).solve(J.transpose() * Opspace::Inertia(ab, J, tolerance));
    ops.is_jbar_computed = true;
  }

  return ops.J_bar;
}

Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                    int idx_link, const Eigen::Vector3d& offset,
                                    double tolerance) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& cc = ab.cc_data_;

  // Compute joint space centrifugal/Coriolis first to cache cc.c
  Eigen::Vector6d mu = JacobianDynamicInverse(ab, J, tolerance).transpose() *
                       SpatialDyn::CentrifugalCoriolis(ab);
  mu -= Opspace::Inertia(ab, J) *
        (Eigen::Affine3d(ab.T_to_world(idx_link).linear()) * cc.c_c[idx_link]).matrix();
  return mu;
}

Eigen::VectorXd Gravity(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                        double tolerance) {
  return JacobianDynamicInverse(ab, J, tolerance).transpose() * Gravity(ab);
}

}  // namespace Opspace
}  // namespace SpatialDyn
