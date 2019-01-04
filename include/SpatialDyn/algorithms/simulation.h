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

#include "SpatialDyn/structs/articulated_body.h"

#include <map>  // std::map

namespace SpatialDyn {

enum class IntegrationMethod { EULER, HEUNS, RK4 };

void Integrate(ArticulatedBody &ab, const Eigen::VectorXd& tau, double dt,
               const std::map<int, SpatialForced>& f_external = {},
               bool gravity = true, bool centrifugal_coriolis = true, bool friction = false,
               IntegrationMethod method = IntegrationMethod::RK4);

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_ALGORITHMS_SIMULATION_H_
