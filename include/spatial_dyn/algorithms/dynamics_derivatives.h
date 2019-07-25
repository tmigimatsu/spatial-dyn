/**
 * dynamics_derivatives.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 10, 2019
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_DYNAMICS_DERIVATIVES_H_
#define SPATIAL_DYN_ALGORITHMS_DYNAMICS_DERIVATIVES_H_

#include <map>  // std::map

#include "spatial_dyn/eigen/spatial_math.h"
#include "spatial_dyn/structs/articulated_body.h"
#include "spatial_dyn/structs/options.h"

namespace spatial_dyn {

/**
 * @defgroup cpp_inverse_dynamics Inverse Dynamics
 * @ingroup cpp_algorithms
 *
 * C++ implementation of spatial_dyn inverse dynamics algorithms.
 *
 * @see Python: \ref py_inverse_dynamics
 * @{
 */

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
Eigen::MatrixXd InverseDynamicsPositionDerivative(const ArticulatedBody& ab,
                                                  Eigen::Ref<const Eigen::VectorXd> ddq,
                                                  const std::map<size_t, SpatialForced>& f_external = {},
                                                  const InverseDynamicsOptions& options = { true, true });

Eigen::MatrixXd InverseDynamicsVelocityDerivative(const ArticulatedBody& ab,
                                                  Eigen::Ref<const Eigen::VectorXd> ddq,
                                                  const std::map<size_t, SpatialForced>& f_external = {},
                                                  const InverseDynamicsOptions& options = { true, true });

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_ALGORITHMS_DYNAMICS_DERIVATIVES_H_
