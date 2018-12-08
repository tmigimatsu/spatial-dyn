/**
 * forward_dynamics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/forward_dynamics.h"

#include "algorithms/inverse_dynamics.h"
#include "utils/math.h"

namespace SpatialDyn {

Eigen::VectorXd ForwardDynamics(const ArticulatedBody& ab,
                                Eigen::Ref<const Eigen::VectorXd> tau,
                                const std::vector<std::pair<int, SpatialForced>>& f_external,
                                bool gravity, bool centrifugal_coriolis, bool friction) {
  return InertiaInverse(ab).solve(tau - InverseDynamics(ab,
      Eigen::VectorXd::Zero(ab.dof()), f_external, gravity, centrifugal_coriolis, friction));
}

// ABA
Eigen::VectorXd ForwardDynamicsAba(const ArticulatedBody& ab,
                                   Eigen::Ref<const Eigen::VectorXd> tau,
                                   const std::vector<std::pair<int, SpatialForced>>& f_external,
                                   bool friction) {
  auto& aba = ab.aba_data_;
  auto& vel = ab.vel_data_;
  auto& rnea = ab.rnea_data_;

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

    // TODO: Use more efficient data structure for sorting through external forces
    for (const std::pair<int, SpatialForced>& link_f : f_external) {
      int idx_link = link_f.first;
      if (idx_link < 0) idx_link += ab.dof();
      if (idx_link != static_cast<int>(i)) continue;
      rnea.f[i] -= ab.T_to_world(i).inverse() * link_f.second;
    }
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

    ddq(i) = (tau(i) - s.dot(rnea.f[i])) / aba.d[i];
    if (friction) {
      const Joint& joint = ab.rigid_bodies(i).joint();
      ddq(i) -= joint.f_coulomb() * Signum(ab.dq(i)) + joint.f_viscous() * ab.dq(i);
    }
    if (parent >= 0) {
      rnea.f[parent] += ab.T_to_parent(i) *
                       (rnea.f[i] + aba.I_a[i] * rnea.a[i] + ddq(i) * aba.h[i]);
    }
  }
  aba.is_computed = true;

  for (size_t i = 0; i < ab.dof(); i++) {
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    if (parent < 0) {
      rnea.a[i] = ab.T_from_parent(i) * -ab.g();
    } else {
      rnea.a[i] += ab.T_from_parent(i) * rnea.a[parent];
    }
    ddq(i) -= aba.h[i].dot(rnea.a[i]) / aba.d[i];
    rnea.a[i] += ddq(i) * s;
  }
  return ddq;
}

const Eigen::LDLT<Eigen::MatrixXd>& InertiaInverse(const ArticulatedBody& ab) {
  auto& crba = ab.crba_data_;
  if (!crba.is_inv_computed) {
    crba.A_inv = Inertia(ab).ldlt();
    crba.is_inv_computed = true;
  }
  return crba.A_inv;
}

const Eigen::MatrixXd& InertiaInverseAba(const ArticulatedBody& ab) {
  auto& aba = ab.aba_data_;
  if (aba.is_A_inv_computed) {
    return aba.A_inv;
  }

  // Forward pass
  for (size_t i = 0; i < ab.dof(); i++) {
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    if (!aba.is_computed) aba.I_a[i] = I;

    aba.A[i].setZero();
    aba.P[i].setZero();
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

    for (int j : ab.subtree(i)) {
      aba.A_inv(i,j) = (static_cast<double>(i == j) - s.dot(aba.P[i].col(j))) / aba.d[i];
      if (parent >= 0) {
        aba.P[parent].col(j) += ab.T_to_parent(i) * (aba.P[i].col(j) + aba.A_inv(i,j) * aba.h[i]);
      }
    }
  }
  aba.is_computed = true;

  for (size_t i = 0; i < ab.dof(); i++) {
    const int parent = ab.rigid_bodies(i).id_parent();
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    for (int j : ab.subtree(i)) {
      if (parent >= 0) {
        aba.A[i].col(j) += ab.T_from_parent(i) * aba.A[parent].col(j);
      }
      aba.A_inv(i,j) -= aba.h[i].dot(aba.A[i].col(j)) / aba.d[i];
      if (static_cast<int>(i) != j) aba.A_inv(j,i) = aba.A_inv(i,j);
      aba.A[i].col(j) += aba.A_inv(i,j) * s;
    }
  }
  aba.is_A_inv_computed = true;
  return aba.A_inv;
}

}  // namespace SpatialDyn
