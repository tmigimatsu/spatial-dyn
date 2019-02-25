/**
 * forward_dynamics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 4, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_FORWARD_DYNAMICS_H_
#define SPATIAL_DYN_ALGORITHMS_FORWARD_DYNAMICS_H_

#include <map>  // std::map

#include "spatial_dyn/eigen/spatial_math.h"
#include "spatial_dyn/structs/articulated_body.h"
#include "spatial_dyn/structs/options.h"

namespace spatial_dyn {

/**
 * @defgroup cpp_forward_dynamics Forward Dynamics
 * @ingroup cpp_algorithms
 *
 * C++ implementation of spatial_dyn forward dynamics algorithms.
 *
 * @see Python: \ref py_forward_dynamics
 * @{
 */

/**
 * Compute the forward dynamics by inverting the inverse dynamics.
 *
 * Uses RNEA to compute the bias torques and inverts the inertia matrix computed
 * via CRBA to output the joint accelerations in O(n^2) time.
 *
 * @param ab ArticulatedBody.
 * @param tau Applied joint torques.
 * @param f_external Map of (index, force) pairs where the force is the sum of
 *                   all external spatial forces (represented in the world
 *                   frame) applied to the associated rigid body index.
 * @param options ForwardDynamicsOptions.
 * @return Joint accelerations.
 * @see Python: spatialdyn.forward_dynamics()
 */
Eigen::VectorXd ForwardDynamics(const ArticulatedBody& ab,
                                Eigen::Ref<const Eigen::VectorXd> tau,
                                const std::map<size_t, SpatialForced>& f_external = {},
                                const ForwardDynamicsOptions& options = {});

/**
 * Compute the forward dynamics with ABA.
 *
 * Uses the Articulated Body Algorithm (ABA) to compute the forward dynamics in 
 * O(n) time.
 *
 * @param ab ArticulatedBody.
 * @param tau Applied joint torques.
 * @param f_external Map of (index, force) pairs where the force is the sum of
 *                   all external spatial forces (represented in the world
 *                   frame) applied to the associated rigid body index.
 * @param options ForwardDynamicsOptions.
 * @return Joint accelerations.
 * @see Python: spatialdyn.forward_dynamics_aba()
 */
Eigen::VectorXd ForwardDynamicsAba(const ArticulatedBody& ab,
                                   Eigen::Ref<const Eigen::VectorXd> tau,
                                   const std::map<size_t, SpatialForced>& f_external = {},
                                   const ForwardDynamicsOptions& options = {});

/**
 * Compute the Cholesky decomposition of the joint space inertia matrix.
 *
 * Uses CRBA to compute the inertia matrix in O(n^2) time, and finds the
 * Cholesky decomposition in O(n^2) time. Results are cached such that
 * subsequent calls with same `q` will return in O(1) time.
 *
 * @param ab ArticulatedBody.
 * @return Reference to the cached Cholesky decomposition of the inertia matrix.
 * @see Python: spatialdyn.inertia_inverse()
 */
const Eigen::LDLT<Eigen::MatrixXd>& InertiaInverse(const ArticulatedBody& ab);

/**
 * Compute the inverse of the joint space inertia matrix with ABA.
 *
 * Uses ABA to compute the inverse inertia matrix in O(n^2) time. Results are
 * cached such that subsequent calls with the same `q` will return in O(1) time.
 *
 * @param ab ArticulatedBody.
 * @return Reference to the cached inverse inertia matrix.
 * @see Python: spatialdyn.inertia_inverse_aba()
 */
const Eigen::MatrixXd& InertiaInverseAba(const ArticulatedBody& ab);

/*
 * @}
 */  // defgroup cpp_forward_dynamics

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_ALGORITHMS_FORWARD_DYNAMICS_H_
