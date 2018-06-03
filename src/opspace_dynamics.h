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
#include "inverse_dynamics.h"
#include "spatial_math.h"

#include <vector>  // std::vector

namespace SpatialDyn {
namespace Opspace {

Eigen::MatrixXd Inertia(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance = 0) {
  Eigen::MatrixXd Lambda_inv(J.rows(), J.rows());
  return Eigen::PseudoInverse(J * InertiaInverse(ab).solve(J.transpose()));
}

Eigen::MatrixXd JacobianDynamicInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance = 0) {
  return InertiaInverse(ab).solve(J.transpose() * Opspace::Inertia(ab, J, tolerance));
}

Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, int idx_link = -1, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(), double tolerance = 0) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& cc = const_cast<ArticulatedBody::CentrifugalCoriolisData&>(ab.cc_data_);

  Eigen::Matrix6Xd J = Jacobian(ab, idx_link, offset);
  const auto JbarT_C = JacobianDynamicInverse(ab, J, tolerance).transpose() *
                       SpatialDyn::CentrifugalCoriolis(ab);
  const auto Lambda_Jdot_dq = Opspace::Inertia(ab, idx_link, offset, tolerance) *
      (Eigen::Affine3d(ab.T_to_world(idx_link).linear()) * cc.c[idx_link]).matrix();
  return JbarT_C - Lambda_Jdot_dq;
}

Eigen::VectorXd Gravity(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance = 0) {
  return JacobianDynamicInverse(ab, J, tolerance).transpose() * Gravity(ab);
}

// ABA
const Eigen::Matrix6d& Inertia(const ArticulatedBody& ab, int idx_link = -1, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(), double tolerance = 0) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& ops = const_cast<ArticulatedBody::OpspaceData&>(ab.opspace_data_);
  auto& aba_I = const_cast<ArticulatedBody::AbaInertiaData&>(ab.aba_inertia_data_);

  // if (ops.is_computed && ops.idx_link == idx_link && ops.offset == offset) {
  //   if (ops.tolerance != tolerance) {
  //     ops.Lambda = Eigen::PseudoInverse(ops.Lambda_inv, tolerance);
  //     ops.tolerance = tolerance;
  //   }
  //   return ops.Lambda;
  // }
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

Eigen::Vector6d CentrifugalCoriolisGravity(ArticulatedBody& ab, int idx_link = -1, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(), double tolerance = 0) {
  if (idx_link < 0) idx_link += ab.dof();
  auto& aba = const_cast<ArticulatedBody::AbaData&>(ab.aba_data_);
  auto& aba_I = const_cast<ArticulatedBody::AbaInertiaData&>(ab.aba_inertia_data_);
  auto& vel = const_cast<ArticulatedBody::VelocityData&>(ab.vel_data_);

  // Forward pass
  for (int i = 0; i < ab.dof(); i++) {
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
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    if (!aba_I.is_computed) aba_I.I_a[i] = I;
    if (!aba.is_computed) aba.c[i] = vel.v[i].cross(ab.dq(i) * s);
    aba.p[i] = vel.v[i].cross(I * vel.v[i]);
  }
  vel.is_computed = true;
  aba.is_computed = true;

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

    aba.u[i] = s.dot(aba.p[i]);
    if (parent >= 0) {
      aba.p[parent] += ab.T_to_parent(i) *
          (aba.p[i] + aba_I.I_a[i] * aba.c[i] + aba.u[i] / aba_I.d[i] * aba_I.h[i]);
    }
  }
  aba_I.is_computed = true;

  SpatialMotiond a;
  for (int i : ab.ancestors_[idx_link]) {
    const auto T_from_parent = ab.T_to_parent(i).inverse();
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    if (parent < 0) {
      a = T_from_parent * -ab.g() + aba.c[i];
    } else {
      a = T_from_parent * a + aba.c[i];
    }
    double ddq = (aba.u[i] - aba_I.h[i].dot(a)) / aba_I.d[i];
    a += ddq * s;
  }

  Eigen::Vector6d mu_p = Eigen::Affine3d(ab.T_to_world(idx_link).linear()) * a;
  return Opspace::Inertia(ab, idx_link, offset, tolerance) * mu_p;
}

}  // namespace Opspace
}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_OPSPACE_DYNAMICS_H_
