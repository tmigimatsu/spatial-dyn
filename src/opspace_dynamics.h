/**
 * opspace_dynamics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 31, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_OPSPACE_DYNAMICS_H_
#define SPATIAL_DYN_OPSPACE_DYNAMICS_H_

#include "articulated_body.h"
#include "forward_kinematics.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "spatial_math.h"

namespace SpatialDyn {
namespace Opspace {

Eigen::MatrixXd InertiaInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance = 0) {
  return J * SpatialDyn::InertiaInverse(ab).solve(J.transpose());
}

Eigen::MatrixXd Inertia(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance = 0) {
  return Eigen::PseudoInverse(Opspace::InertiaInverse(ab, J, tolerance));
}

Eigen::MatrixXd JacobianDynamicInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance = 0) {
  return InertiaInverse(ab).solve(J.transpose() * Opspace::Inertia(ab, J, tolerance));
}

Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, const Eigen::MatrixXd& J, int idx_link = -1, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(), double tolerance = 0) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& cc = ab.cc_data_;

  // Compute joint space centrifugal/Coriolis first to cache cc.c
  Eigen::Vector6d mu = JacobianDynamicInverse(ab, J, tolerance).transpose() *
                       SpatialDyn::CentrifugalCoriolis(ab);
  mu -= Opspace::Inertia(ab, J) *
        (Eigen::Affine3d(ab.T_to_world(idx_link).linear()) * cc.c[idx_link]).matrix();
  return mu;
}

Eigen::VectorXd Gravity(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance = 0) {
  return JacobianDynamicInverse(ab, J, tolerance).transpose() * Gravity(ab);
}

// ABA
const Eigen::Matrix6d& Inertia(const ArticulatedBody& ab, int idx_link = -1, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(), double tolerance = 0) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& ops = ab.opspace_data_;
  auto& aba_I = ab.aba_inertia_data_;

  if (ops.is_computed && ops.idx_link == idx_link && ops.offset == offset) {
    if (ops.tolerance != tolerance) {
      ops.Lambda = Eigen::PseudoInverse(ops.Lambda_inv, tolerance);
      ops.tolerance = tolerance;
    }
    return ops.Lambda;
  }
  // TODO: Implement transforms for rotations and translations only
  // TODO: Implement for offset

  // Forward pass
  for (int i = 0; i < ab.dof(); i++) {
    if (!aba_I.is_computed) aba_I.I_a[i] = ab.rigid_bodies(i).inertia();
    if (i == idx_link) {
      ops.p[i] = Eigen::Affine3d(ab.T_to_world(i).linear().transpose()) * -SpatialForce6d::Identity();
    } else {
      ops.p[i].setZero();
    }
  }

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();
    if (!aba_I.is_computed) {
      aba_I.h[i] = aba_I.I_a[i] * s;
      aba_I.d[i] = s.dot(aba_I.h[i]);
      if (parent >= 0) {
        aba_I.I_a[i] -= aba_I.h[i].matrix() / aba_I.d[i] * aba_I.h[i].transpose();
        aba_I.I_a[parent] += ab.T_to_parent(i) * aba_I.I_a[i];
      }
    }

    ops.u[i] = -s.matrix().transpose() * ops.p[i].matrix();

    if (parent >= 0) {
      ops.p[parent] += ab.T_to_parent(i) * (ops.p[i] + aba_I.h[i] / aba_I.d[i] * ops.u[i]);
    }
  }
  aba_I.is_computed = true;

  SpatialMotion6d a = SpatialMotion6d::Zero();
  for (int i : ab.ancestors_[idx_link]) {
    const auto T_from_parent = ab.T_to_parent(i).inverse();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    a = T_from_parent * a;
    auto ddq = (ops.u[i] - aba_I.h[i].transpose() * a.matrix()) / aba_I.d[i];
    a += s * ddq;
  }

  ops.idx_link = idx_link;
  ops.offset = offset;
  ops.Lambda_inv = Eigen::Affine3d(ab.T_to_world(idx_link).linear()) * a;
  ops.Lambda = Eigen::PseudoInverse(ops.Lambda_inv, tolerance);
  ops.is_computed = true;
  return ops.Lambda;
}

Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, int idx_link = -1, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(), double tolerance = 0) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& aba = ab.aba_data_;
  auto& aba_I = ab.aba_inertia_data_;

  AbaPrecomputeVelocityInertia(ab);

  // Forward pass
  for (int i = 0; i < ab.dof(); i++) {
    aba.p[i] = aba.b[i];
  }

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();
    aba.u[i] = -s.dot(aba.p[i]);
    if (parent >= 0) {
      aba.p[parent] += ab.T_to_parent(i) *
          (aba.p[i] + aba_I.I_a[i] * aba.c[i] + aba.u[i] / aba_I.d[i] * aba_I.h[i]);
    }
  }

  SpatialMotiond a = SpatialMotiond::Zero();
  for (int i : ab.ancestors_[idx_link]) {
    const auto T_from_parent = ab.T_to_parent(i).inverse();
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    if (parent >= 0) {
      a = T_from_parent * a + aba.c[i];
    }
    double ddq = (aba.u[i] - aba_I.h[i].dot(a)) / aba_I.d[i];
    a += ddq * s;
  }

  Eigen::Vector6d mu_p = Eigen::Affine3d(ab.T_to_world(idx_link).linear()) * a;
  return Opspace::Inertia(ab, idx_link, offset, tolerance) * -mu_p;
}

Eigen::Vector6d Gravity(ArticulatedBody& ab, int idx_link = -1, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(), double tolerance = 0) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& aba = ab.aba_data_;
  auto& aba_I = ab.aba_inertia_data_;

  AbaPrecomputeVelocityInertia(ab);

  // Forward pass
  for (int i = 0; i < ab.dof(); i++) {
    aba.p[i].setZero();
  }

  // Backward pass
  const Eigen::VectorXd& tau = -SpatialDyn::Gravity(ab);
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();
    aba.u[i] = tau(i) - s.dot(aba.p[i]);
    if (parent >= 0) {
      aba.p[parent] += ab.T_to_parent(i) * (aba.p[i] + aba.u[i] / aba_I.d[i] * aba_I.h[i]);
    }
  }
  aba_I.is_computed = true;

  SpatialMotiond a = SpatialMotiond::Zero();
  for (int i : ab.ancestors_[idx_link]) {
    const auto T_from_parent = ab.T_to_parent(i).inverse();
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    if (parent >= 0) {
      a = T_from_parent * a;
    }
    double ddq = (aba.u[i] - aba_I.h[i].dot(a)) / aba_I.d[i];
    a += ddq * s;
  }

  Eigen::Vector6d p = Eigen::Affine3d(ab.T_to_world(idx_link).linear()) * a;
  return Opspace::Inertia(ab, idx_link, offset, tolerance) * -p;
}

}  // namespace Opspace
}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_OPSPACE_DYNAMICS_H_
