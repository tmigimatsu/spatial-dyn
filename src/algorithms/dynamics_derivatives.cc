/**
 * dynamics_derivatives.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 10, 2019
 * Authors: Toki Migimatsu
 */

#include "algorithms/inverse_dynamics.h"

#include "structs/articulated_body_cache.h"

namespace spatial_dyn {

Eigen::MatrixXd InverseDynamicsPositionDerivative(const ArticulatedBody& ab,
                                                  Eigen::Ref<const Eigen::VectorXd> ddq,
                                                  const std::map<size_t, SpatialForced>& f_external,
                                                  const InverseDynamicsOptions& options) {
  auto& rnea = ab.cache_->rnea_data_;
  auto& vel  = ab.cache_->vel_data_;

  Eigen::MatrixXd dtau(ab.dof(), ab.dof());  // Resulting joint torques

  std::vector<SpatialMotionXd> dv(ab.dof(), SpatialMotionXd::Zero(ab.dof()));
  std::vector<SpatialMotionXd> da(ab.dof(), SpatialMotionXd::Zero(ab.dof()));
  std::vector<SpatialForceXd> df(ab.dof(), SpatialForceXd::Zero(ab.dof()));


  // TODO: Watch out for crba cache
  InverseDynamics(ab, ddq, f_external, options);

  // Forward pass
  SpatialInertiad I_total;
  for (size_t i = 0; i < ab.dof(); i++) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const bool has_load = ab.inertia_load().find(i) != ab.inertia_load().end();
    if (has_load) I_total = ab.rigid_bodies(i).inertia() + ab.inertia_load().at(i);
    const SpatialInertiad& I = has_load ? I_total : ab.rigid_bodies(i).inertia();
    const int parent = ab.rigid_bodies(i).id_parent();

    if (parent >= 0 && options.centrifugal_coriolis) {
      for (size_t j = 0; j < i; j++) {
        dv[i].col(j) = ab.T_from_parent(i) * dv[parent].col(j);
      }
      dv[i].col(i) = (ab.T_from_parent(i) * vel.v[parent]).cross(s);
    }

    if (parent < 0) {
      if (options.gravity) {
        da[i].col(i) -= (ab.T_from_world(i) * ab.g()).cross(s);
      }
    } else {
      for (size_t j = 0; j <= i; j++) {
        da[i].col(j) = ab.T_from_parent(i) * da[parent].col(j);
      }
      da[i].col(i) += da[parent] + (ab.T_from_parent(i) * rnea.a[parent]).cross(s);
    }
    if (options.centrifugal_coriolis) {
      for (size_t j = 0; j <= i; j++) {
        da[i].col(j) += dv[i].col(j).cross(ab.dq(i) * s);
      }
    }

    if (!options.centrifugal_coriolis) {
      for (size_t j = 0; j <= i; j++) {
        df[i].col(j) = I * da[i].col(j);
      }
    } else {
      const SpatialForced h_i = I * vel.v[i];
      for (size_t j = 0; j <= i; j++) {
        df[i].col(j) = I * da[i].col(j) + dv[i].col(j).cross(h_i) + vel.v[i].cross(I * dv[i].col(j));
      }
    }

    if (f_external.find(i) != f_external.end()) {
      // TODO: verify
      rnea.f[i] -= s.cross(ab.T_from_world(i) * f_external.at(i));
    }
  }

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();

    for (size_t j = 0; j < ab.dof(); j++) {
      dtau(i, j) = s.dot(df[i].col(j));
    }
    if (parent >= 0) {
      df[parent].col(i) += ab.T_to_parent(i) * s.cross(rnea.f[i]);
      for (size_t j = 0; j < ab.dof(); j++) {
        df[parent].col(j) += ab.T_to_parent(i) * df[i].col(j);
      }
    }
  }

  return dtau;
}

Eigen::MatrixXd InverseDynamicsVelocityDerivative(const ArticulatedBody& ab,
                                                  Eigen::Ref<const Eigen::VectorXd> ddq,
                                                  const std::map<size_t, SpatialForced>& f_external,
                                                  const InverseDynamicsOptions& options) {
  auto& vel  = ab.cache_->vel_data_;

  Eigen::MatrixXd dtau(ab.dof(), ab.dof());  // Resulting joint torques
  if (!options.centrifugal_coriolis) {
    dtau.setZero();
    return dtau;
  }

  std::vector<SpatialMotionXd> dv(ab.dof(), SpatialMotionXd::Zero(ab.dof()));
  std::vector<SpatialMotionXd> da(ab.dof(), SpatialMotionXd::Zero(ab.dof()));
  std::vector<SpatialForceXd> df(ab.dof(), SpatialForceXd::Zero(ab.dof()));

  // TODO: Watch out for crba cache
  InverseDynamics(ab, ddq, f_external, options);

  // Forward pass
  SpatialInertiad I_total;
  for (size_t i = 0; i < ab.dof(); i++) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const bool has_load = ab.inertia_load().find(i) != ab.inertia_load().end();
    if (has_load) I_total = ab.rigid_bodies(i).inertia() + ab.inertia_load().at(i);
    const SpatialInertiad& I = has_load ? I_total : ab.rigid_bodies(i).inertia();
    const int parent = ab.rigid_bodies(i).id_parent();

    if (parent >= 0) {
      for (size_t j = 0; j < i; j++) {
        dv[i].col(j) = ab.T_from_parent(i) * dv[parent].col(j);
      }
    }
    dv[i].col(i) = s;

    for (size_t j = 0; j <= i; j++) {
      da[i].col(j) = dv[i].col(j).cross(ab.dq(i) * s);
    }
    da[i].col(i) += vel.v[i].cross(s);
    if (parent >= 0) {
      for (size_t j = 0; j <= i; j++) {
        da[i].col(j) += ab.T_from_parent(i) * da[parent].col(j);
      }
    }

    const SpatialForced h_i = I * vel.v[i];
    for (size_t j = 0; j <= i; j++) {
      df[i].col(j) = I * da[i].col(j) + dv[i].col(j).cross(h_i) + vel.v[i].cross(I * dv[i].col(j));
    }
  }

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();

    for (size_t j = 0; j < ab.dof(); j++) {
      dtau(i, j) = s.dot(df[i].col(j));
    }
    if (parent >= 0) {
      for (size_t j = 0; j < ab.dof(); j++) {
        df[parent].col(j) += ab.T_to_parent(i) * df[i].col(j);
      }
    }
  }

  return dtau;
}

}  // namespace spatial_dyn
