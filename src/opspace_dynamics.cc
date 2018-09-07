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

Eigen::Vector3d OrientationError(const Eigen::Quaterniond &quat, const Eigen::Quaterniond &quat_des) {
  Eigen::Quaterniond quat_err = quat * quat_des.inverse();
  Eigen::AngleAxisd aa_err(quat_err);  // Angle will always be between [0, pi]
  double angle = (quat_err.w() < 0) ? aa_err.angle() - 2 * M_PI : aa_err.angle();
  return 2 * angle * aa_err.axis();
}

Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                const Eigen::VectorXd& ddx, Eigen::MatrixXd *N,
                                double svd_epsilon, bool centrifugal_coriolis, bool gravity) {
  // TODO: centrifugal_coriolis
  Eigen::VectorXd tau;

  if (N == nullptr || N->size() == 0) {
    if (!gravity) {
      tau = J.transpose() * (Inertia(ab, J, svd_epsilon) * ddx);
    } else {
      tau = J.transpose() * (Inertia(ab, J, svd_epsilon) * ddx + Gravity(ab, J, svd_epsilon));
    }

    if (N != nullptr) {
      const Eigen::MatrixXd& J_bar = JacobianDynamicInverse(ab, J, svd_epsilon);
      if (N->size() == 0) {
        *N = Eigen::MatrixXd::Identity(ab.dof(), ab.dof()) - J_bar * J;
      } else {
        *N = (Eigen::MatrixXd::Identity(ab.dof(), ab.dof()) - J_bar * J) * (*N);
      }
    }
  } else {
    Eigen::MatrixXd JN = J * (*N);
    if (!gravity) {
      tau = JN.transpose() * (Inertia(ab, JN, svd_epsilon) * ddx);
    } else {
      tau = JN.transpose() * (Inertia(ab, JN, svd_epsilon) * ddx + Gravity(ab, JN, svd_epsilon));
    }

    const Eigen::MatrixXd& JN_bar = JacobianDynamicInverse(ab, JN, svd_epsilon);
    *N = (Eigen::MatrixXd::Identity(ab.dof(), ab.dof()) - JN_bar * JN) * (*N);
  }

  return tau;
}


const Eigen::MatrixXd& Inertia(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                               double svd_epsilon) {
  auto& ops = ab.opspace_data_;

  if (!ops.is_lambda_computed || ops.J != J || ops.svd_epsilon != svd_epsilon) {
    ops.Lambda = Eigen::PseudoInverse(InertiaInverse(ab, J), svd_epsilon);
    ops.svd_epsilon = svd_epsilon;
    ops.is_lambda_computed = true;
  }

  return ops.Lambda;
}

const Eigen::MatrixXd& InertiaInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J) {
  auto& ops = ab.opspace_data_;

  if (!ops.is_lambda_inv_computed || ops.J != J) {
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

Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                    int idx_link, const Eigen::Vector3d& offset,
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
                        double svd_epsilon) {
  return JacobianDynamicInverse(ab, J, svd_epsilon).transpose() * Gravity(ab);
}

}  // namespace Opspace
}  // namespace SpatialDyn
