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

#include <utility>  // std::pair
#include <vector>   // std::vector

#include "SpatialDyn/structs/articulated_body.h"
#include "SpatialDyn/utils/spatial_math.h"

namespace SpatialDyn {

// RNEA
Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::VectorXd& ddq,
                                const std::vector<std::pair<int, SpatialForced>>& f_external = {},
                                bool gravity = true, bool centrifugal_coriolis = false,
                                bool friction = false);
const Eigen::VectorXd& CentrifugalCoriolis(const ArticulatedBody& ab);
const Eigen::VectorXd& Gravity(const ArticulatedBody& ab,
                               const std::vector<std::pair<int, SpatialForced>>& f_external = {});
const Eigen::VectorXd& Friction(const ArticulatedBody& ab);

// CRBA
const Eigen::MatrixXd& Inertia(const ArticulatedBody& ab);
// NOTE: InertiaInverse in forward_dynamics.h

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_ALGORITHMS_INVERSE_DYNAMICS_H_
