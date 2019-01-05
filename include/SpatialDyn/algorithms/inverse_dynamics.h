/**
 * inverse_dynamics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 23, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_INVERSE_DYNAMICS_H_
#define SPATIAL_DYN_ALGORITHMS_INVERSE_DYNAMICS_H_

#include <map>  // std::map

#include "SpatialDyn/structs/articulated_body.h"
#include "SpatialDyn/structs/options.h"
#include "SpatialDyn/utils/spatial_math.h"

namespace SpatialDyn {

/**
 * @defgroup cpp_inverse_dynamics Inverse Dynamics
 * @ingroup cpp_algorithms
 *
 * C++ implementation of SpatialDyn inverse dynamics algorithms.
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
Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::VectorXd& ddq,
                                const std::map<int, SpatialForced>& f_external = {},
                                const InverseDynamicsOptions& options = {});

/**
 * Compute the centrifugal/Coriolis compensation torques.
 *
 * Implements the Recursive Newton Euler Algorithm (RNEA) to compute the inverse
 * dynamic torques in O(n) time. Results are cached such that subsequent calls
 * with the same `q` and `dq` will return in O(1) time.
 *
 * @param ab ArticulatedBody.
 * @return Reference to the cached centrifugal/Coriolis torques.
 * @see Python: spatialdyn.centrifugal_coriolis()
 */
const Eigen::VectorXd& CentrifugalCoriolis(const ArticulatedBody& ab);

/**
 * Compute the gravity compensation torques.
 *
 * Implements the Recursive Newton Euler Algorithm (RNEA) to compute the inverse
 * dynamic torques in O(n) time. Results are cached such that subsequent calls
 * with the same `q` and `dq` will return in O(1) time.
 *
 * @param ab ArticulatedBody.
 * @return Reference to the cached gravity torques.
 * @see Python: spatialdyn.gravity()
 */
const Eigen::VectorXd& Gravity(const ArticulatedBody& ab);

/**
 * Compute the torques to compensate external forces applied on the rigid bodies.
 *
 * Implements the Recursive Newton Euler Algorithm (RNEA) to compute the inverse
 * dynamic torques in O(n) time. Results are cached such that subsequent calls
 * with the same `q` and `dq` will return in O(1) time.
 *
 * @param ab ArticulatedBody.
 * @param f_external Map of (index, force) pairs where the force is the sum of
 *                   all external spatial forces (represented in the world
 *                   frame) applied to the associated rigid body index.
 * @return External torques.
 * @see Python: spatialdyn.external_torque()
 */
Eigen::VectorXd ExternalTorques(const ArticulatedBody& ab,
                                const std::map<int, SpatialForced>& f_external = {});

/**
 * Compute Coulomb and viscous joint friction torques.
 *
 * Friction torques are computed in O(n) time.
 *
 * @param ab ArticulatedBody.
 * @param tau Applied torques (for stiction).
 * @param compensate Return friction compensation torques as opposed to
 *                   resulting friction torques.
 * @param stiction_epsilon Velocity threshold for stiction activation.
 * @return Friction torques.
 * @see Python: spatialdyn.friction()
 */
Eigen::VectorXd Friction(const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> tau,
                         bool compensate = true, double stiction_epsilon = 0.01);

/**
 * Compute the joint space inertia matrix.
 *
 * Implements the Composite Rigid Body Algorithm (CRBA) to compute the inertia
 * matrix in O(n^2) time. Results are cached such that subsequent calls with the
 * same `q` will return in O(1) time.
 *
 * @param ab ArticulatedBody.
 * @return Inertia matrix.
 * @see Forward dynamics: SpatialDyn::InertiaInverse()
 * @see Python: spatialdyn.inertia()
 */
const Eigen::MatrixXd& Inertia(const ArticulatedBody& ab);

/*
 * @}
 */  // defgroup cpp_inverse_dynamics

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_ALGORITHMS_INVERSE_DYNAMICS_H_
