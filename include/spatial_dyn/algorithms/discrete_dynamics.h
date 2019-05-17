/**
 * discrete_dynamics.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: March 5, 2019
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_DISCRETE_DYNAMICS_H_
#define SPATIAL_DYN_ALGORITHMS_DISCRETE_DYNAMICS_H_

#include <map>  // std::map

#include "spatial_dyn/eigen/spatial_math.h"
#include "spatial_dyn/structs/articulated_body.h"
#include "spatial_dyn/structs/options.h"

namespace spatial_dyn {
namespace discrete {

/**
 * Compute the inverse dynamics torques given the desired acceleration `ddq`.
 *
 * Implements the Recursive Newton Euler Algorithm (RNEA) to compute the inverse
 * dynamic torques in O(n) time.
 *
 * @param ab ArticulatedBody.
 * @param ddq Desired joint acceleration.
 * @param f_external Map of (index, force) pairs where the force is the sum of
 *                   all external spatial forces (represented in the world
 *                   frame) applied to the associated rigid body index.
 * @param options InverseDynamicsOptions.
 * @return Inverse dynamics torques.
 * @see Python: spatialdyn.inverse_dynamics()
 */
Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::VectorXd& q_next,
                                const double dt,
                                const std::map<size_t, SpatialForced>& f_external = {},
                                const InverseDynamicsOptions& options = {});

void Integrate(ArticulatedBody& ab, const Eigen::VectorXd& tau, double dt,
               const std::map<size_t, SpatialForced>& f_external = {},
               const IntegrationOptions& options = {});

}  // namespace discrete
}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_ALGORITHMS_DISCRETE_DYNAMICS_H_
