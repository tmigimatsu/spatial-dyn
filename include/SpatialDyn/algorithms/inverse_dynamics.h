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

#include "structs/articulated_body.h"
#include "utils/spatial_math.h"

namespace SpatialDyn {

// RNEA
Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::VectorXd& ddq,
                                bool gravity = true, bool centrifugal_coriolis = false,
                                bool cache = false);
const Eigen::VectorXd& CentrifugalCoriolis(const ArticulatedBody& ab);
const Eigen::VectorXd& Gravity(const ArticulatedBody& ab);

// CRBA
const Eigen::MatrixXd& Inertia(const ArticulatedBody& ab);
const Eigen::LDLT<Eigen::MatrixXd>& InertiaInverse(const ArticulatedBody& ab);

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_ALGORITHMS_INVERSE_DYNAMICS_H_