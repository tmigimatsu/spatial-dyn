/**
 * inverse_dynamics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "algorithms/inverse_dynamics.h"

#include <algorithm>  // std::min
#include <cmath>      // std::abs

#include <ctrl_utils/math.h>

#include "structs/articulated_body_cache.h"

namespace spatial_dyn {

Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::VectorXd& ddq,
                                const std::map<size_t, SpatialForced>& f_external,
                                const InverseDynamicsOptions& options) {
  auto& rnea = ab.cache_->rnea_data_;
  auto& vel  = ab.cache_->vel_data_;
  auto& crba = ab.cache_->crba_data_;
  auto& cc   = ab.cache_->cc_data_;
  auto& grav = ab.cache_->grav_data_;

  Eigen::VectorXd tau(ab.dof());  // Resulting joint torques

  if (crba.is_computed && (!options.centrifugal_coriolis || cc.is_computed) &&
      (!options.gravity || grav.is_computed)) {
    tau = crba.A * ddq + ExternalTorques(ab, f_external);
    if (options.centrifugal_coriolis) tau += cc.C;
    if (options.gravity) tau += grav.G;
    if (options.friction) tau += Friction(ab, tau, true, options.stiction_epsilon);
    return tau;
  }

  // Forward pass
  SpatialInertiad I_total;
  for (size_t i = 0; i < ab.dof(); i++) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const bool has_load = ab.inertia_load().find(i) != ab.inertia_load().end();
    if (has_load) I_total = ab.rigid_bodies(i).inertia() + ab.inertia_load().at(i);
    const SpatialInertiad& I = has_load ? I_total : ab.rigid_bodies(i).inertia();
    const int parent = ab.rigid_bodies(i).id_parent();

    if (!vel.is_computed && options.centrifugal_coriolis) {
      vel.v[i] = ab.dq(i) * s;
      if (parent >= 0) {
        vel.v[i] += ab.T_from_parent(i) * vel.v[parent];
      }
    }

    rnea.a[i] = ddq(i) * s;
    if (parent < 0) {
      if (options.gravity) {
        rnea.a[i] -= ab.T_from_world(i) * ab.g();
      }
      if (options.centrifugal_coriolis) {
        rnea.a[i] += vel.v[i].cross(ab.dq(i) * s);
      }
    } else if (!options.centrifugal_coriolis) {
      rnea.a[i] += ab.T_from_parent(i) * rnea.a[parent];
    } else {
      rnea.a[i] += ab.T_from_parent(i) * rnea.a[parent] + vel.v[i].cross(ab.dq(i) * s);
    }

    if (!options.centrifugal_coriolis) {
      rnea.f[i] = I * rnea.a[i];
    } else {
      rnea.f[i] = I * rnea.a[i] + vel.v[i].cross(I * vel.v[i]);
    }

    if (f_external.find(i) != f_external.end()) {
      rnea.f[i] -= ab.T_from_world(i) * f_external.at(i);
    }
  }
  vel.is_computed = options.centrifugal_coriolis;

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const Joint& joint = ab.rigid_bodies(i).joint();
    const SpatialMotiond& s = joint.subspace();
    const int parent = ab.rigid_bodies(i).id_parent();

    tau(i) = s.dot(rnea.f[i]);
    if (parent >= 0) rnea.f[parent] += ab.T_to_parent(i) * rnea.f[i];
  }

  // Friction compensation
  if (options.friction) tau += Friction(ab, tau, true, options.stiction_epsilon);

  return tau;
}

const Eigen::VectorXd& CentrifugalCoriolis(const ArticulatedBody& ab) {
  auto& vel = ab.cache_->vel_data_;
  auto& cc = ab.cache_->cc_data_;
  if (cc.is_computed) return cc.C;

  // Forward pass
  SpatialInertiad I_total;
  for (size_t i = 0; i < ab.dof(); i++) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    bool has_load = ab.inertia_load().find(i) != ab.inertia_load().end();
    if (has_load) I_total = ab.rigid_bodies(i).inertia() + ab.inertia_load().at(i);
    const SpatialInertiad& I = has_load ? I_total : ab.rigid_bodies(i).inertia();
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
  SpatialInertiad I_total;
  for (size_t i = 0; i < ab.dof(); i++) {
    bool has_load = ab.inertia_load().find(i) != ab.inertia_load().end();
    if (has_load) I_total = ab.rigid_bodies(i).inertia() + ab.inertia_load().at(i);
    const SpatialInertiad& I = has_load ? I_total : ab.rigid_bodies(i).inertia();
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
                                const std::map<size_t, SpatialForced>& f_external) {
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
  Eigen::VectorXd tau(ab.dof());
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

Eigen::VectorXd Friction(const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> tau,
                         bool compensate, double stiction_epsilon) {
  Eigen::VectorXd F(ab.dof());
  for (size_t i = 0; i < ab.dof(); i++) {
    const Joint& joint = ab.rigid_bodies(i).joint();

    // Kinetic friction
    double f_coulomb = joint.f_coulomb() * ctrl_utils::Signum(ab.dq(i), stiction_epsilon);

    // Static friction
    if (f_coulomb == 0.) {
      double mu = joint.f_coulomb() + joint.f_stiction();
      if (!compensate) {
        mu = std::min(mu, std::abs(tau(i)));
      }
      f_coulomb = mu * ctrl_utils::Signum(tau(i));
    }

    // Add viscous and Coulomb friction
    F(i) = joint.f_viscous() * ab.dq(i) + f_coulomb;
  }
  return F;
}

const Eigen::MatrixXd& Inertia(const ArticulatedBody& ab) {
  auto& crba = ab.cache_->crba_data_;
  if (crba.is_computed) return crba.A;

  // Initialize composite rigid body inertia
  for (size_t i = 0; i < ab.dof(); i++) {
    crba.I_c[i] = ab.rigid_bodies(i).inertia();
    bool has_load = ab.inertia_load().find(i) != ab.inertia_load().end();
    if (has_load) crba.I_c[i] += ab.inertia_load().at(i);
  }

  // Backward pass
  crba.A.setZero();
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

const SpatialInertiad& CompositeInertia(const ArticulatedBody& ab, int link) {
  auto& crba = ab.cache_->crba_data_;

  if (link < 0) link += ab.dof();
  Inertia(ab);
  return crba.I_c[link];
}

}  // namespace spatial_dyn
