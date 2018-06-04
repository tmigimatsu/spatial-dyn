/**
 * opspace_dynamics_aba.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "opspace_dynamics.h"
#include "articulated_body.h"
#include "inverse_dynamics.h"

namespace SpatialDyn {
namespace Opspace {

const Eigen::Matrix6d& Inertia(const ArticulatedBody& ab, int idx_link, const Eigen::Vector3d& offset, double tolerance) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& ops = ab.opspace_data_;
  auto& aba = ab.aba_data_;

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
    if (!aba.is_computed) aba.I_a[i] = ab.rigid_bodies(i).inertia();
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
    if (!aba.is_computed) {
      aba.h[i] = aba.I_a[i] * s;
      aba.d[i] = s.dot(aba.h[i]);
      if (parent >= 0) {
        aba.I_a[i] -= aba.h[i].matrix() / aba.d[i] * aba.h[i].transpose();
        aba.I_a[parent] += ab.T_to_parent(i) * aba.I_a[i];
      }
    }

    ops.u[i] = -s.matrix().transpose() * ops.p[i].matrix();

    if (parent >= 0) {
      ops.p[parent] += ab.T_to_parent(i) * (ops.p[i] + aba.h[i] / aba.d[i] * ops.u[i]);
    }
  }
  aba.is_computed = true;

  SpatialMotion6d a = SpatialMotion6d::Zero();
  for (int i : ab.ancestors_[idx_link]) {
    const auto T_from_parent = ab.T_to_parent(i).inverse();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    a = T_from_parent * a;
    auto ddq = (ops.u[i] - aba.h[i].transpose() * a.matrix()) / aba.d[i];
    a += s * ddq;
  }

  ops.idx_link = idx_link;
  ops.offset = offset;
  ops.Lambda_inv = Eigen::Affine3d(ab.T_to_world(idx_link).linear()) * a;
  ops.Lambda = Eigen::PseudoInverse(ops.Lambda_inv, tolerance);
  ops.is_computed = true;
  return ops.Lambda;
}

Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, int idx_link, const Eigen::Vector3d& offset, double tolerance) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& aba = ab.aba_data_;
  auto& cc = ab.cc_data_;
  auto& vel = ab.vel_data_;

  // Forward pass
  for (int i = 0; i < ab.dof(); i++) {
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();

    if (!vel.is_computed) {
      if (parent < 0) {
        vel.v[i] = ab.dq(i) * s;
      } else {
        const auto& T_from_parent = ab.T_to_parent(i).inverse();
        vel.v[i] = T_from_parent * vel.v[parent] + ab.dq(i) * s;
      }
    }

    if (!cc.is_vel_computed) {
      if (parent < 0) cc.c[i].setZero();
      else            cc.c[i] = vel.v[i].cross(ab.dq(i) * s);

      cc.b[i] = vel.v[i].cross(I * vel.v[i]);
    }

    if (!aba.is_computed) aba.I_a[i] = I;

    aba.p[i] = cc.b[i];
  }
  vel.is_computed = true;
  cc.is_vel_computed = true;

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();
    if (!aba.is_computed) {
      aba.h[i] = aba.I_a[i] * s;
      aba.d[i] = s.dot(aba.h[i]);
      if (parent >= 0) {
        aba.I_a[i] -= aba.h[i].matrix() / aba.d[i] * aba.h[i].transpose();
        aba.I_a[parent] += ab.T_to_parent(i) * aba.I_a[i];
      }
    }

    aba.u[i] = -s.dot(aba.p[i]);
    if (parent >= 0) {
      aba.p[parent] += ab.T_to_parent(i) *
          (aba.p[i] + aba.I_a[i] * cc.c[i] + aba.u[i] / aba.d[i] * aba.h[i]);
    }
  }
  aba.is_computed = true;

  SpatialMotiond a = SpatialMotiond::Zero();
  for (int i : ab.ancestors_[idx_link]) {
    const auto T_from_parent = ab.T_to_parent(i).inverse();
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    if (parent >= 0) {
      a = T_from_parent * a + cc.c[i];
    }
    double ddq = (aba.u[i] - aba.h[i].dot(a)) / aba.d[i];
    a += ddq * s;
  }

  Eigen::Vector6d mu = Eigen::Affine3d(ab.T_to_world(idx_link).linear()) * a;
  return Opspace::Inertia(ab, idx_link, offset, tolerance) * -mu;
}

Eigen::Vector6d Gravity(ArticulatedBody& ab, int idx_link, const Eigen::Vector3d& offset, double tolerance) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& aba = ab.aba_data_;

  // Forward pass
  for (int i = 0; i < ab.dof(); i++) {
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    if (!aba.is_computed) aba.I_a[i] = I;

    aba.p[i].setZero();
  }

  // Backward pass
  // TODO: Find more efficient way to incorporate gravity
  const Eigen::VectorXd& tau = -SpatialDyn::Gravity(ab);
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();
    if (!aba.is_computed) {
      aba.h[i] = aba.I_a[i] * s;
      aba.d[i] = s.dot(aba.h[i]);
      if (parent >= 0) {
        aba.I_a[i] -= aba.h[i].matrix() / aba.d[i] * aba.h[i].transpose();
        aba.I_a[parent] += ab.T_to_parent(i) * aba.I_a[i];
      }
    }

    aba.u[i] = tau(i) - s.dot(aba.p[i]);
    if (parent >= 0) {
      aba.p[parent] += ab.T_to_parent(i) * (aba.p[i] + aba.u[i] / aba.d[i] * aba.h[i]);
    }
  }
  aba.is_computed = true;

  SpatialMotiond a = SpatialMotiond::Zero();
  for (int i : ab.ancestors_[idx_link]) {
    const auto T_from_parent = ab.T_to_parent(i).inverse();
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    if (parent >= 0) {
      a = T_from_parent * a;
    }
    double ddq = (aba.u[i] - aba.h[i].dot(a)) / aba.d[i];
    a += ddq * s;
  }

  Eigen::Vector6d p = Eigen::Affine3d(ab.T_to_world(idx_link).linear()) * a;
  return Opspace::Inertia(ab, idx_link, offset, tolerance) * -p;
}

}  // namespace Opspace
}  // namespace SpatialDyn
