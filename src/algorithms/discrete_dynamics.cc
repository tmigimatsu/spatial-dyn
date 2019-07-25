/**
 * discrete_dynamics.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: March 5, 2019
 * Authors: Toki Migimatsu
 */

#include "algorithms/discrete_dynamics.h"

#include <utility>  // std::swap

#include <ctrl_utils/euclidian.h>

#include "algorithms/forward_dynamics.h"
#include "algorithms/inverse_dynamics.h"
#include "algorithms/forward_kinematics.h"
#include "structs/articulated_body_cache.h"

namespace spatial_dyn {
namespace discrete {

Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab,
                                Eigen::Ref<const Eigen::VectorXd> q_next,
                                const double dt,
                                const std::map<size_t, SpatialForced>& f_external,
                                const InverseDynamicsOptions& options) {
  auto& drnea = ab.cache_->drnea_data_;

  // Check cache
  if (!drnea.is_q_prev_valid || drnea.dt != dt) {
    drnea.q_prev = ab.q() - dt * ab.dq();
    drnea.dt = dt;
    drnea.is_q_prev_valid = true;
    drnea.is_prev_computed = false;
  }

  // Forward pass for previous timestep
  if (!drnea.is_prev_computed) {
    for (size_t i = 0; i < ab.dof(); i++) {
      // Compute delta transformation from next to curr timestep
      const int parent = ab.rigid_bodies(i).id_parent();
      if (parent < 0) {
        drnea.T_to_prev[i] = ab.T_to_parent(i, drnea.q_prev(i)).inverse() * ab.T_to_parent(i);
      } else {
        drnea.T_to_prev[i] = ab.T_to_parent(i, drnea.q_prev(i)).inverse() *
                             drnea.T_to_prev[parent] * ab.T_to_parent(i);
      }

      // Compute discrete momentum (gradient of kinetic energy) between t_prev and t_curr
      const SpatialMotiond v_prev = ctrl_utils::Log(drnea.T_to_prev[i]) / dt;
      const Eigen::Matrix6d dv_prev = ctrl_utils::LogMapDerivative(drnea.T_to_prev[i]);
      const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
      drnea.p_prev[i] = dv_prev.transpose() * (I * v_prev).matrix();
    }
    drnea.is_prev_computed = true;
  }

  // Forward pass
  for (size_t i = 0; i < ab.dof(); i++) {
    // Compute delta transformation from next to curr timestep
    const int parent = ab.rigid_bodies(i).id_parent();
    if (parent < 0) {
      drnea.T_from_next[i] = ab.T_to_parent(i).inverse() * ab.T_to_parent(i, q_next(i));
    } else {
      drnea.T_from_next[i] = ab.T_to_parent(i).inverse() * drnea.T_from_next[parent] *
                             ab.T_to_parent(i, q_next(i));
    }

    // Compute discrete momentum (gradient of kinetic energy) between t_curr and t_next
    const SpatialMotiond v = ctrl_utils::Log(drnea.T_from_next[i]) / dt;
    const Eigen::Matrix6d dv = ctrl_utils::LogMapDerivative(drnea.T_from_next[i]);
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    drnea.p[i] = dv.transpose() * (I * v).matrix();

    // Compute change in momentum (impulse)
    drnea.dp[i] = drnea.p[i] - drnea.T_to_prev[i].inverse() * drnea.p_prev[i];
    if (f_external.find(i) != f_external.end()) {
      drnea.dp[i] -= ab.T_from_world(i) * f_external.at(i) * dt;
    }
  }

  Eigen::VectorXd tau_dt(ab.dof());  // Resulting joint impulses

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const Joint& joint = ab.rigid_bodies(i).joint();
    const SpatialMotiond& s = joint.subspace();
    const int parent = ab.rigid_bodies(i).id_parent();

    tau_dt(i) = s.dot(drnea.dp[i]);
    if (options.gravity) {
      const Eigen::VectorXd& G = Gravity(ab);
      tau_dt(i) += dt * G(i);
    }
    if (parent >= 0) {
      drnea.dp[parent] += ab.T_to_parent(i) * drnea.dp[i];
    }
  }

  return tau_dt;
}

// TODO: Implement LogMapHessian for SE(3) (after testing for SO(3))
// Eigen::MatrixXd InverseDynamicsDerivative(const ArticulatedBody& ab,
//                                           Eigen::Ref<const Eigen::VectorXd> q_next,
//                                           const double dt,
//                                           const std::map<size_t, SpatialForced>& f_external,
//                                           const InverseDynamicsOptions& options) {
//   auto& drnea = ab.cache_->drnea_data_;

//   SpatialForceXd Dp_prev(ab.dof());
//   SpatialForceXd Dp(ab.dof());
//   SpatialForceXd Ddp(ab.dof());

//   for (size_t i = 0; i < ab.dof(); i++) {
//       // Compute delta transformation from next to curr timestep
//       const int parent = ab.rigid_bodies(i).id_parent();
//       if (parent < 0) {
//         drnea.T_to_prev[i] = ab.T_to_parent(i, drnea.q_prev(i)).inverse() * ab.T_to_parent(i);
//       } else {
//         drnea.T_to_prev[i] = ab.T_to_parent(i, drnea.q_prev(i)).inverse() *
//                              drnea.T_to_prev[parent] * ab.T_to_parent(i);
//       }

//       const SpatialMotiond v_prev = ctrl_utils::Log(drnea.T_to_prev[i]) / dt;
//       const Eigen::Matrix6d dv_prev = ctrl_utils::LogMapDerivative(drnea.T_to_prev[i]);
//       const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
//       // Dp_prev.col(i) = dv_prev.transpose)( * (I )_ )
//   }

//   for (size_t i = 0; i < ab.dof(); i++) {
//     // Compute delta transformation from next to curr timestep
//     const int parent = ab.rigid_bodies(i).id_parent();
//     if (parent < 0) {
//       drnea.T_from_next[i] = ab.T_to_parent(i).inverse() * ab.T_to_parent(i, q_next(i));
//     } else {
//       drnea.T_from_next[i] = ab.T_to_parent(i).inverse() * drnea.T_from_next[parent] *
//                              ab.T_to_parent(i, q_next(i));
//     }

//     // Compute discrete momentum (gradient of kinetic energy) between t_curr and t_next
//     const SpatialMotiond v = ctrl_utils::Log(drnea.T_from_next[i]) / dt;
//     const Eigen::Matrix6d dv = ctrl_utils::LogMapDerivative(drnea.T_from_next[i]);
//     // const Eigen::Matrix6d ddv = ctrl_utils::LogMapHessian(Eigen::AngleAxisd(drnea.T_from_next[i]));
//     const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
//     Dp.col(i) = dv.transpose() * (I * SpatialMotionXd(dv)).matrix() + ddv.transpose() * (I * v).matrix();

//     // Compute change in momentum (impulse)
//     drnea.dp[i] = drnea.p[i] - drnea.T_to_prev[i].inverse() * drnea.p_prev[i];
//     if (f_external.find(i) != f_external.end()) {
//       drnea.dp[i] -= ab.T_from_world(i) * f_external.at(i) * dt;
//     }
//   }
// }

void HandleContact() {
  // Solve n+1 equations for q_c, alpha:
  //     D2_Ld(q_prev, q, dt) + D1_Ld(q, q_c, alpha dt) - (1 + alpha)/2 dt tau_ext = 0
  //     Phi(q_c) = 0
  // Interpretation:
  //     d/dt dL/dq' - dL/dq - tau_ext = 0 (Euler-Lagrange)

  // Solve n+1 equations for q_next, lambda subject to q_next feasible:
  //     D2_Ld(q, q_c, alpha dt) + D1_Ld(q_c, q_next, (1 - alpha) dt) + lambda dPhi(q_c)/dq + 1/2 dt tau_friction = 0
  //     D3_Ld(q, q_c, alpha dt) - D3_Ld(q_c, q_next, (1 - alpha) dt) - dE_contact = 0
  // Interpretation:
  //     (dL/dq'|t_c+ - dL/dq'|t_c-) dq + dp_friction dq = 0, dq tangent to Phi = 0 (Conservation of momentum)
  //     E|t_c+ - E|t_c- - dE_contact = 0 (Conservation of energy)

  // Solve n equations for q_next_next:
  //     D2_Ld(q_c, q_next, (1 - alpha) dt) + D1_Ld(q_next, q_next_next, dt) - (2 - alpha)/2 dt tau_ext = 0
  // Interpretation:
  //     d/dt dL/dq' - dL/dq - tau_ext = 0 (Euler-Lagrange)
}

void Integrate(ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> tau, double dt,
               const std::map<size_t, SpatialForced>& f_external,
               const IntegrationOptions& options) {

  auto& drnea = ab.cache_->drnea_data_;

  const Eigen::VectorXd q = ab.q();
  // TODO: add options for initialization
  Eigen::VectorXd q_next = q + dt * ab.dq() + dt * dt * ForwardDynamics(ab, tau, f_external, options);

  for (size_t i = 0; i < options.max_iterations; i++) {
    const Eigen::VectorXd error = InverseDynamics(ab, q_next, dt, f_external, options) - dt * tau;
    if (error.norm() < options.variational_epsilon) break;

    q_next -= dt * InertiaInverse(ab).solve(error);
  }

  // Update position and velocity
  ab.set_q(q_next);
  ab.set_dq((q_next - drnea.q_prev) / (2. * dt));

  // Transfer computations for curr timestep to prev in cache
  drnea.q_prev = q;
  drnea.is_q_prev_valid = true;
  std::swap(drnea.T_from_next, drnea.T_to_prev);
  std::swap(drnea.p, drnea.p_prev);
  drnea.is_prev_computed = true;
}

}  // namespace discrete
}  // namespace spatial_dyn
