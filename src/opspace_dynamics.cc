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

Eigen::MatrixXd InertiaInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance) {
  return J * SpatialDyn::InertiaInverse(ab).solve(J.transpose());
}

Eigen::MatrixXd Inertia(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance) {
  return Eigen::PseudoInverse(Opspace::InertiaInverse(ab, J, tolerance));
}

Eigen::MatrixXd JacobianDynamicInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance) {
  return InertiaInverse(ab).solve(J.transpose() * Opspace::Inertia(ab, J, tolerance));
}

Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, const Eigen::MatrixXd& J, int idx_link, const Eigen::Vector3d& offset, double tolerance) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& cc = ab.cc_data_;

  // Compute joint space centrifugal/Coriolis first to cache cc.c
  Eigen::Vector6d mu = JacobianDynamicInverse(ab, J, tolerance).transpose() *
                       SpatialDyn::CentrifugalCoriolis(ab);
  mu -= Opspace::Inertia(ab, J) *
        (Eigen::Affine3d(ab.T_to_world(idx_link).linear()) * cc.c_c[idx_link]).matrix();
  return mu;
}

Eigen::VectorXd Gravity(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance) {
  return JacobianDynamicInverse(ab, J, tolerance).transpose() * Gravity(ab);
}

}  // namespace Opspace
}  // namespace SpatialDyn
