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

// ABA
Eigen::VectorXd ForwardDynamics(const ArticulatedBody& ab, const Eigen::VectorXd& tau) {
  auto& aba = const_cast<ArticulatedBody::AbaData&>(ab.aba_data_);
  auto& aba_I = const_cast<ArticulatedBody::AbaInertiaData&>(ab.aba_inertia_data_);
  auto& vel = const_cast<ArticulatedBody::VelocityData&>(ab.vel_data_);

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

    if (!aba.is_computed) {
      if (parent < 0) aba.c[i].setZero();
      else            aba.c[i] = vel.v[i].cross(ab.dq(i) * s);

      aba.b[i] = vel.v[i].cross(I * vel.v[i]);
    }
    aba.p[i] = aba.b[i];

    if (!aba_I.is_computed) aba_I.I_a[i] = I;
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

    aba.u[i] = tau(i) - s.dot(aba.p[i]);
    if (parent >= 0) {
      aba.p[parent] += ab.T_to_parent(i) *
          (aba.p[i] + aba_I.I_a[i] * aba.c[i] + aba.u[i] / aba_I.d[i] * aba_I.h[i]);
    }
  }
  aba_I.is_computed = true;

  Eigen::VectorXd ddq(ab.dof());
  for (int i = 0; i < ab.dof(); i++) {
    const auto T_from_parent = ab.T_to_parent(i).inverse();
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    if (parent < 0) {
      aba.a[i] = T_from_parent * -ab.g() + aba.c[i];
    } else {
      aba.a[i] = T_from_parent * aba.a[parent] + aba.c[i];
    }
    ddq(i) = (aba.u[i] - aba_I.h[i].dot(aba.a[i])) / aba_I.d[i];
    aba.a[i] += ddq(i) * s;
  }
  return ddq;
}

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_FORWARD_DYNAMICS_H_
