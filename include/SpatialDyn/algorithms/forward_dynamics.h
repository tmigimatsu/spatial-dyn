/**
 * forward_dynamics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_FORWARD_DYNAMICS_H_
#define SPATIAL_DYN_ALGORITHMS_FORWARD_DYNAMICS_H_

#include "SpatialDyn/structs/articulated_body.h"
#include "SpatialDyn/utils/spatial_math.h"

#include <vector>   // std::vector
#include <utility>  // std::pair

namespace SpatialDyn {

Eigen::VectorXd ForwardDynamics(const ArticulatedBody& ab, const Eigen::VectorXd& tau,
                                const std::vector<std::pair<int, SpatialForced>>& f_external = {});
Eigen::VectorXd ForwardDynamicsAba(const ArticulatedBody& ab, const Eigen::VectorXd& tau,
                                   const std::vector<std::pair<int, SpatialForced>>& f_external = {});

const Eigen::LDLT<Eigen::MatrixXd>& InertiaInverse(const ArticulatedBody& ab);
const Eigen::MatrixXd& InertiaInverseAba(const ArticulatedBody& ab);

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_ALGORITHMS_FORWARD_DYNAMICS_H_
