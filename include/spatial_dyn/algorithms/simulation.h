/**
 * simulation.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 4, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_SIMULATION_H_
#define SPATIAL_DYN_ALGORITHMS_SIMULATION_H_

#include <map>  // std::map

#include "spatial_dyn/eigen/spatial_math.h"
#include "spatial_dyn/structs/articulated_body.h"
#include "spatial_dyn/structs/options.h"

namespace spatial_dyn {

/**
 * @defgroup cpp_simulation Simulation
 * @ingroup cpp
 *
 * C++ implementation of spatial_dyn simulation utilities.
 *
 * @see Python: \ref py_simulation
 * @{
 */

/**
 * Apply the commanded joint torques and integrate by the given timestep to get
 * the robot's next state.
 *
 * Computes the robot's acceleration with forward dynamics and integrates the
 * resulting acceleration along with robot's velocity by one timestep to get the
 * robot's next position and velocity. The forward dynamics algorithm and
 * integration method can be specified in the options.
 *
 * @param ab ArticulatedBody.
 * @param tau Commanded joint torques.
 * @param dt Integration timestep.
 * @param f_external Map of (index, force) pairs where the force is the sum of
 *                   all external spatial forces (represented in the world
 *                   frame) applied to the associated rigid body index.
 * @param options IntegrationOptions.
 * @see Python: spatialdyn.integrate()
 */
void Integrate(ArticulatedBody &ab, const Eigen::VectorXd& tau, double dt,
               const std::map<size_t, SpatialForced>& f_external = {},
               const IntegrationOptions& options = {});

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_ALGORITHMS_SIMULATION_H_
