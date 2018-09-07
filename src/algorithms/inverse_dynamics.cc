/**
 * inverse_dynamics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/inverse_dynamics.h"

#define CACHE_INVERSE_DYNAMICS

namespace SpatialDyn {

// RNEA
Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::VectorXd& ddq,
                                bool gravity, bool centrifugal_coriolis, bool cache) {
  if (!cache) {

    auto& rnea = ab.rnea_data_;
    auto& vel  = ab.vel_data_;

    // Forward pass
    for (int i = 0; i < ab.dof(); i++) {
      const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
      const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
      const int parent = ab.rigid_bodies(i).id_parent();

      if (centrifugal_coriolis && !vel.is_computed) {
        if (parent < 0) {
          vel.v[i] = ab.dq(i) * s;
        } else {
          vel.v[i] = ab.T_from_parent(i) * vel.v[parent] + ab.dq(i) * s;
        }
      }

      if (parent < 0) {
        if (gravity) {
          rnea.a[i] = ddq(i) * s - ab.T_to_world(i).inverse() * ab.g();
        } else {
          rnea.a[i] = ddq(i) * s;
        }
      } else {
        if (centrifugal_coriolis) {
          rnea.a[i] = ab.T_from_parent(i) * rnea.a[parent] + ddq(i) * s +
                      vel.v[i].cross(ab.dq(i) * s);
        } else {
          rnea.a[i] = ab.T_from_parent(i) * rnea.a[parent] + ddq(i) * s;
        }
      }

      if (centrifugal_coriolis) {
        rnea.f[i] = I * rnea.a[i] + vel.v[i].cross(I * vel.v[i]);
      } else {
        rnea.f[i] = I * rnea.a[i];
      }
    }
    if (centrifugal_coriolis) vel.is_computed = true;

    // Backward pass
    Eigen::VectorXd tau(ab.dof());           // Resulting joint torques
    for (int i = ab.dof() - 1; i >= 0; i--) {
      const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
      const int parent = ab.rigid_bodies(i).id_parent();

      tau(i) = s.dot(rnea.f[i]);
      if (parent >= 0) rnea.f[parent] += ab.T_to_parent(i) * rnea.f[i];
    }
    return tau;

  } else {

    auto& rnea = ab.rnea_data_;
    auto& vel  = ab.vel_data_;
    auto& cc = ab.cc_data_;
    auto& grav = ab.grav_data_;

    // Forward pass
    for (int i = 0; i < ab.dof(); i++) {
      const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
      const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
      const int parent = ab.rigid_bodies(i).id_parent();

      if (centrifugal_coriolis && !vel.is_computed) {
        if (parent < 0) vel.v[i] = ab.dq(i) * s;
        else            vel.v[i] = ab.T_from_parent(i) * vel.v[parent] + ab.dq(i) * s;
      }

      if (centrifugal_coriolis && !cc.is_computed) {
        if (parent < 0) cc.c_c[i].setZero();
        else            cc.c_c[i] = ab.T_from_parent(i) * cc.c_c[parent] + vel.v[i].cross(ab.dq(i) * s);

        cc.f_c[i] = I * cc.c_c[i] + vel.v[i].cross(I * vel.v[i]);
      }

      if (gravity && !grav.is_computed) {
        const auto T_from_world = ab.T_to_world(i).inverse();
        grav.f_g[i] = I * (T_from_world * -ab.g());
      }

      if (parent < 0) rnea.a[i] = ddq(i) * s;
      else            rnea.a[i] = ab.T_from_parent(i) * rnea.a[parent] + ddq(i) * s;

      rnea.f[i] = I * rnea.a[i];
    }
    if (centrifugal_coriolis) vel.is_computed = true;

    // Backward pass
    Eigen::VectorXd tau(ab.dof());           // Resulting joint torques
    for (int i = ab.dof() - 1; i >= 0; i--) {
      const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
      const int parent = ab.rigid_bodies(i).id_parent();

      if (centrifugal_coriolis) {
        rnea.f[i] += cc.f_c[i];
      }
      if (gravity) {
        rnea.f[i] += grav.f_g[i];
      }
      tau(i) = s.dot(rnea.f[i]);
      if (parent >= 0) rnea.f[parent] += ab.T_to_parent(i) * rnea.f[i];
    }
    if (centrifugal_coriolis) cc.is_computed = true;
    if (gravity) grav.is_computed = true;
    return tau;

  }
}

const Eigen::VectorXd& CentrifugalCoriolis(const ArticulatedBody& ab) {
  auto& vel = ab.vel_data_;
  auto& cc = ab.cc_data_;
  if (cc.is_computed) {
    return cc.C;
  }

  // Forward pass
  for (int i = 0; i < ab.dof(); i++) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    const int parent = ab.rigid_bodies(i).id_parent();

    if (!vel.is_computed) {
      if (parent < 0) vel.v[i] = ab.dq(i) * s;
      else            vel.v[i] = ab.T_from_parent(i) * vel.v[parent] + ab.dq(i) * s;
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
  auto& grav = ab.grav_data_;
  if (grav.is_computed) {
    return grav.G;
  }

  // Forward pass
  for (int i = 0; i < ab.dof(); i++) {
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    const auto T_from_world = ab.T_to_world(i).inverse();
    grav.f_g[i] = I * (T_from_world * -ab.g());
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

// CRBA
const Eigen::MatrixXd& Inertia(const ArticulatedBody& ab) {
  auto& crba = ab.crba_data_;
  if (crba.is_computed) {
    return crba.A;
  }

  // Initialize composite rigid body inertia
  for (int i = 0; i < ab.dof(); i++) {
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

const Eigen::LDLT<Eigen::MatrixXd>& InertiaInverse(const ArticulatedBody& ab) {
  auto& crba = ab.crba_data_;
  if (!crba.is_inv_computed) {
    crba.A_inv = Inertia(ab).ldlt();
  }
  return crba.A_inv;
}

}  // namespace SpatialDyn
