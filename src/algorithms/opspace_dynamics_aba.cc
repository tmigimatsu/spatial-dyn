/**
 * opspace_dynamics_aba.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/opspace_dynamics.h"

#include "algorithms/forward_dynamics.h"
#include "algorithms/inverse_dynamics.h"
#include "structs/articulated_body_cache.h"

namespace SpatialDyn {
namespace Opspace {

const Eigen::Matrix6d& InertiaAba(const ArticulatedBody& ab, int idx_link, const Eigen::Vector3d& offset, double svd_epsilon) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& ops = ab.cache_->opspace_aba_data_;

  if (!ops.is_lambda_computed || ops.idx_link != idx_link ||
      ops.offset != offset || ops.svd_epsilon != svd_epsilon) {
    ops.Lambda = Eigen::PseudoInverse(InertiaInverseAba(ab, idx_link, offset), svd_epsilon);
    ops.svd_epsilon = svd_epsilon;
    ops.is_lambda_computed = true;
  }

  return ops.Lambda;
}

const Eigen::Matrix6d& InertiaInverseAba(const ArticulatedBody& ab, int idx_link, const Eigen::Vector3d& offset) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& ops = ab.cache_->opspace_aba_data_;
  auto& aba = ab.cache_->aba_data_;

  if (ops.is_lambda_inv_computed && ops.idx_link == idx_link && ops.offset == offset) {
    return ops.Lambda_inv;
  }
  // TODO: Implement transforms for rotations and translations only
  // TODO: Implement for offset

  // Forward pass
  for (size_t i = 0; i < ab.dof(); i++) {
    if (!aba.is_computed) aba.I_a[i] = ab.rigid_bodies(i).inertia();
    if (static_cast<int>(i) == idx_link) {
      ops.p[i] = Eigen::Isometry3d(ab.T_to_world(i).linear().transpose()) * -SpatialForce6d::Identity();
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
  for (int i : ab.ancestors(idx_link)) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    a = ab.T_from_parent(i) * a;
    auto ddq = (ops.u[i] - aba.h[i].transpose() * a.matrix()) / aba.d[i];
    a += s * ddq;
  }

  ops.Lambda_inv = Eigen::Isometry3d(ab.T_to_world(idx_link).linear()) * a;
  ops.idx_link = idx_link;
  ops.offset = offset;
  ops.is_lambda_inv_computed = true;
  ops.is_lambda_computed = false;
  return ops.Lambda_inv;
}

Eigen::Vector6d CentrifugalCoriolisAba(const ArticulatedBody& ab, int idx_link, const Eigen::Vector3d& offset, double svd_epsilon) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& aba = ab.cache_->aba_data_;
  auto& vel = ab.cache_->vel_data_;
  auto& rnea = ab.cache_->rnea_data_;

  // Forward pass
  for (size_t i = 0; i < ab.dof(); i++) {
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();

    if (!vel.is_computed) {
      if (parent < 0) {
        vel.v[i] = ab.dq(i) * s;
      } else {
        vel.v[i] = ab.T_from_parent(i) * vel.v[parent] + ab.dq(i) * s;
      }
    }

    if (parent < 0) {
      rnea.a[i].setZero();
    } else {
      rnea.a[i] = vel.v[i].cross(ab.dq(i) * s);
    }

    if (!aba.is_computed) aba.I_a[i] = I;

    rnea.f[i] = vel.v[i].cross(I * vel.v[i]);
  }
  vel.is_computed = true;

  // Backward pass
  Eigen::VectorXd ddq(ab.dof());
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

    ddq(i) = -s.dot(rnea.f[i]) / aba.d[i];
    if (parent >= 0) {
      rnea.f[parent] += ab.T_to_parent(i) *
                       (rnea.f[i] + aba.I_a[i] * rnea.a[i] + ddq(i) * aba.h[i]);
    }
  }
  aba.is_computed = true;

  SpatialMotiond a = SpatialMotiond::Zero();
  for (int i : ab.ancestors(idx_link)) {
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    if (parent >= 0) {
      a = ab.T_from_parent(i) * a + rnea.a[i];
    }
    ddq(i) -= aba.h[i].dot(a) / aba.d[i];
    a += ddq(i) * s;
  }

  Eigen::Vector6d mu = Eigen::Isometry3d(ab.T_to_world(idx_link).linear()) * a;
  return Opspace::InertiaAba(ab, idx_link, offset, svd_epsilon) * -mu;
}

Eigen::Vector6d GravityAba(const ArticulatedBody& ab, int idx_link, const Eigen::Vector3d& offset,
                           const std::map<int, SpatialForced>& f_external,
                           double svd_epsilon) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& aba = ab.cache_->aba_data_;
  auto& rnea = ab.cache_->rnea_data_;

  // Forward pass
  for (size_t i = 0; i < ab.dof(); i++) {
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    if (!aba.is_computed) aba.I_a[i] = I;

    rnea.f[i] = aba.I_a[i] * (ab.T_to_world(i).inverse() * ab.g());

    // TODO: Use more efficient data structure for sorting through external forces
    if (f_external.find(i) != f_external.end()) {
      rnea.f[i] -= ab.T_from_world(i) * f_external.at(i);
    }
  }

  // Backward pass
  Eigen::VectorXd ddq(ab.dof());
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

    ddq(i) = -s.dot(rnea.f[i]) / aba.d[i];
    if (parent >= 0) {
      rnea.f[parent] += ab.T_to_parent(i) * (rnea.f[i] + ddq(i) * aba.h[i]);
    }
  }
  aba.is_computed = true;

  SpatialMotiond a = SpatialMotiond::Zero();
  for (int i : ab.ancestors(idx_link)) {
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    if (parent >= 0) {
      a = ab.T_from_parent(i) * a;
    }
    ddq(i) -= aba.h[i].dot(a) / aba.d[i];
    a += ddq(i) * s;
  }

  Eigen::Vector6d p = Eigen::Isometry3d(ab.T_to_world(idx_link).linear()) * a;
  return Opspace::InertiaAba(ab, idx_link, offset, svd_epsilon) * p;
}

}  // namespace Opspace
}  // namespace SpatialDyn
