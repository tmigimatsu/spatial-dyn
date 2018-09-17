/**
 * forward_dynamics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/forward_dynamics.h"

namespace SpatialDyn {

Eigen::VectorXd ForwardDynamics(const ArticulatedBody& ab, const Eigen::VectorXd& tau) {
  return InertiaInverse(ab).solve(tau - CentrifugalCoriolis(ab) - Gravity(ab));
}

// ABA
Eigen::VectorXd ForwardDynamicsAba(const ArticulatedBody& ab, const Eigen::VectorXd& tau) {
  auto& aba = ab.aba_data_;
  auto& vel = ab.vel_data_;

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
      aba.a[i].setZero();
    } else {
      aba.a[i] = vel.v[i].cross(ab.dq(i) * s);
    }

    if (!aba.is_computed) aba.I_a[i] = I;

    aba.p[i] = vel.v[i].cross(I * vel.v[i]);
  }
  vel.is_computed = true;

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

    aba.u[i] = tau(i) - s.dot(aba.p[i]);
    if (parent >= 0) {
      aba.p[parent] += ab.T_to_parent(i) *
          (aba.p[i] + aba.I_a[i] * aba.a[i] + aba.u[i] / aba.d[i] * aba.h[i]);
    }
  }
  aba.is_computed = true;

  Eigen::VectorXd ddq(ab.dof());
  for (size_t i = 0; i < ab.dof(); i++) {
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    if (parent < 0) {
      aba.a[i] = ab.T_from_parent(i) * -ab.g();
    } else {
      aba.a[i] += ab.T_from_parent(i) * aba.a[parent];
    }
    ddq(i) = (aba.u[i] - aba.h[i].dot(aba.a[i])) / aba.d[i];
    aba.a[i] += ddq(i) * s;
  }
  return ddq;
}

}  // namespace SpatialDyn
