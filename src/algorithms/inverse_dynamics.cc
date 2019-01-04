/**
 * inverse_dynamics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/inverse_dynamics.h"

#include "structs/articulated_body_cache.h"
#include "utils/math.h"

#define CACHE_INVERSE_DYNAMICS

namespace SpatialDyn {

Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::VectorXd& ddq,
                                const std::map<int, SpatialForced>& f_external,
                                bool gravity, bool centrifugal_coriolis, bool friction) {
  auto& rnea = ab.cache_->rnea_data_;
  auto& vel  = ab.cache_->vel_data_;

  // Forward pass
  for (size_t i = 0; i < ab.dof(); i++) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    const int parent = ab.rigid_bodies(i).id_parent();

    if (centrifugal_coriolis && !vel.is_computed) {
      vel.v[i] = ab.dq(i) * s;
      if (parent >= 0) {
        vel.v[i] += ab.T_from_parent(i) * vel.v[parent];
      }
    }

    if (parent < 0) {
      rnea.a[i] = ddq(i) * s;
      if (gravity) {
        rnea.a[i] -= ab.T_to_world(i).inverse() * ab.g();
      }
    } else {
      rnea.a[i] = ab.T_from_parent(i) * rnea.a[parent] + ddq(i) * s;
      if (centrifugal_coriolis) {
        rnea.a[i] += vel.v[i].cross(ab.dq(i) * s);
      }
    }

    rnea.f[i] = I * rnea.a[i];
    if (centrifugal_coriolis) {
      rnea.f[i] += vel.v[i].cross(I * vel.v[i]);
    }

    if (f_external.find(i) != f_external.end()) {
      rnea.f[i] -= ab.T_to_world(i).inverse() * f_external.at(i);
    }
  }
  if (centrifugal_coriolis) vel.is_computed = true;

  // Backward pass
  Eigen::VectorXd tau(ab.dof());  // Resulting joint torques
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const Joint& joint = ab.rigid_bodies(i).joint();
    const SpatialMotiond& s = joint.subspace();
    const int parent = ab.rigid_bodies(i).id_parent();

    tau(i) = s.dot(rnea.f[i]);
    if (friction) {
      tau(i) += joint.f_coulomb() * Signum(ab.dq(i)) + joint.f_viscous() * ab.dq(i);
    }
    if (parent >= 0) rnea.f[parent] += ab.T_to_parent(i) * rnea.f[i];
  }
  return tau;
}

const Eigen::VectorXd& CentrifugalCoriolis(const ArticulatedBody& ab) {
  auto& vel = ab.cache_->vel_data_;
  auto& cc = ab.cache_->cc_data_;
  if (cc.is_computed) return cc.C;

  // Forward pass
  for (size_t i = 0; i < ab.dof(); i++) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    const int parent = ab.rigid_bodies(i).id_parent();

    if (!vel.is_computed) {
      vel.v[i] = ab.dq(i) * s;
      if (parent >= 0) {
        vel.v[i] += ab.T_from_parent(i) * vel.v[parent];
      }
    }

    if (parent < 0) {
      cc.c_c[i].setZero();
    } else {
      cc.c_c[i] = ab.T_from_parent(i) * cc.c_c[parent] + vel.v[i].cross(ab.dq(i) * s);
    }

    cc.f_c[i] = I * cc.c_c[i] + vel.v[i].cross(I * vel.v[i]);
  }
  vel.is_computed = true;

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    cc.C(i) = s.dot(cc.f_c[i]);

    const int parent = ab.rigid_bodies(i).id_parent();
    if (parent >= 0) {
      cc.f_c[parent] += ab.T_to_parent(i) * cc.f_c[i];
    }
  }
  cc.is_computed = true;
  return cc.C;
}

const Eigen::VectorXd& Gravity(const ArticulatedBody& ab) {
  auto& grav = ab.cache_->grav_data_;
  if (grav.is_computed) return grav.G;

  // Forward pass
  for (size_t i = 0; i < ab.dof(); i++) {
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    grav.f_g[i] = I * (ab.T_from_world(i) * -ab.g());
  }

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    grav.G(i) = s.dot(grav.f_g[i]);

    const int parent = ab.rigid_bodies(i).id_parent();
    if (parent >= 0) {
      grav.f_g[parent] += ab.T_to_parent(i) * grav.f_g[i];
    }
  }
  grav.is_computed = true;
  return grav.G;
}

Eigen::VectorXd ExternalTorques(const ArticulatedBody& ab,
                                const std::map<int, SpatialForced>& f_external) {
  // Use gravity data structure. TODO: Change to dedicated structure?
  auto& grav = ab.cache_->grav_data_;

  // Forward pass
  for (size_t i = 0; i < ab.dof(); i++) {
    if (f_external.find(i) != f_external.end()) {
      grav.f_g[i] = ab.T_from_world(i) * -f_external.at(i);
    } else {
      grav.f_g[i].setZero();
    }
  }

  // Backward pass
  Eigen::VectorXd tau;
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    tau(i) = s.dot(grav.f_g[i]);

    const int parent = ab.rigid_bodies(i).id_parent();
    if (parent >= 0) {
      grav.f_g[parent] += ab.T_to_parent(i) * grav.f_g[i];
    }
  }
  return tau;
}

const Eigen::VectorXd& Friction(const ArticulatedBody& ab) {
  auto& grav = ab.cache_->grav_data_;
  if (grav.is_friction_computed) return grav.F;

  for (size_t i = 0; i < ab.dof(); i++) {
    const Joint& joint = ab.rigid_bodies(i).joint();
    grav.F(i) = joint.f_coulomb() * Signum(ab.dq(i)) + joint.f_viscous() * ab.dq(i);
  }
  grav.is_friction_computed = true;
  return grav.F;
}

// CRBA
const Eigen::MatrixXd& Inertia(const ArticulatedBody& ab) {
  auto& crba = ab.cache_->crba_data_;
  if (crba.is_computed) return crba.A;

  // Initialize composite rigid body inertia
  for (size_t i = 0; i < ab.dof(); i++) {
    crba.I_c[i] = ab.rigid_bodies(i).inertia();
  }

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    // Add inertia to parent
    const int parent = ab.rigid_bodies(i).id_parent();
    if (parent >= 0) {
      crba.I_c[parent] += ab.T_to_parent(i) * crba.I_c[i];
    }

    // Compute kinetic energy
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    SpatialForced f = crba.I_c[i] * s;  // Force that produces acceleration s
    crba.A(i,i) = f.dot(s);  // Component of f along joint i: A_ii = s_i^T I_c s_i
    int child = i;
    for (int j = parent; j >= 0; j = ab.rigid_bodies(j).id_parent()) {
      f = ab.T_to_parent(child) * f;  // Force in frame j
      crba.A(i,j) = f.dot(ab.rigid_bodies(j).joint().subspace());  // Component of f along joint j
      crba.A(j,i) = crba.A(i,j);  // A_ji = A_ij
      child = j;
    }
  }
  crba.is_computed = true;
  return crba.A;
}

}  // namespace SpatialDyn
