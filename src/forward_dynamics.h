/**
 * forward_dynamics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_FORWARD_DYNAMICS_H_
#define SPATIAL_DYN_FORWARD_DYNAMICS_H_

#include "articulated_body.h"
#include "spatial_math.h"

namespace SpatialDyn {

void AbaPrecomputeVelocityInertia(const ArticulatedBody& ab) {
  auto& aba = ab.aba_data_;
  auto& cc = ab.cc_data_;
  auto& vel = ab.vel_data_;
  if (aba.is_computed && cc.is_vel_computed) return;

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
  }
  vel.is_computed = true;
  cc.is_vel_computed = true;

  // Backward pass
  if (!aba.is_computed) {
    for (int i = ab.dof() - 1; i >= 0; i--) {
      const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
      const int parent = ab.rigid_bodies(i).id_parent();
      aba.h[i] = aba.I_a[i] * s;
      aba.d[i] = s.dot(aba.h[i]);
      if (parent >= 0) {
        aba.I_a[i] -= aba.h[i].matrix() / aba.d[i] * aba.h[i].transpose();
        aba.I_a[parent] += ab.T_to_parent(i) * aba.I_a[i];
      }
    }
  }
  aba.is_computed = true;

}

// ABA
Eigen::VectorXd ForwardDynamics(const ArticulatedBody& ab, const Eigen::VectorXd& tau) {
  auto& aba = ab.aba_data_;
  auto& cc = ab.cc_data_;

  AbaPrecomputeVelocityInertia(ab);

  // Forward pass
  for (int i = 0; i < ab.dof(); i++) {
    aba.p[i] = cc.b[i];
  }

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();
    aba.u[i] = tau(i) - s.dot(aba.p[i]);
    if (parent >= 0) {
      aba.p[parent] += ab.T_to_parent(i) *
          (aba.p[i] + aba.I_a[i] * cc.c[i] + aba.u[i] / aba.d[i] * aba.h[i]);
    }
  }

  Eigen::VectorXd ddq(ab.dof());
  for (int i = 0; i < ab.dof(); i++) {
    const auto T_from_parent = ab.T_to_parent(i).inverse();
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    if (parent < 0) {
      aba.a[i] = T_from_parent * -ab.g() + cc.c[i];
    } else {
      aba.a[i] = T_from_parent * aba.a[parent] + cc.c[i];
    }
    ddq(i) = (aba.u[i] - aba.h[i].dot(aba.a[i])) / aba.d[i];
    aba.a[i] += ddq(i) * s;
  }
  return ddq;
}

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_FORWARD_DYNAMICS_H_
