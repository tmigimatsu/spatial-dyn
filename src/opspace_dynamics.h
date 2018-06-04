/**
 * opspace_dynamics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 31, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_OPSPACE_DYNAMICS_H_
#define SPATIAL_DYN_OPSPACE_DYNAMICS_H_

#include "spatial_math.h"

namespace SpatialDyn {

class ArticulatedBody;

namespace Opspace {

Eigen::MatrixXd Inertia(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance = 0);

Eigen::MatrixXd InertiaInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance = 0);

Eigen::MatrixXd JacobianDynamicInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance = 0);

Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, const Eigen::MatrixXd& J, int idx_link = -1, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(), double tolerance = 0);

Eigen::VectorXd Gravity(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double tolerance = 0);

// ABA
const Eigen::Matrix6d& Inertia(const ArticulatedBody& ab, int idx_link = -1, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(), double tolerance = 0);

Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, int idx_link = -1, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(), double tolerance = 0);

Eigen::Vector6d Gravity(ArticulatedBody& ab, int idx_link = -1, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(), double tolerance = 0);

}  // namespace Opspace
}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_OPSPACE_DYNAMICS_H_
