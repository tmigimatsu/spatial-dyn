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

Eigen::Vector3d OrientationError(const Eigen::Quaterniond &quat, const Eigen::Quaterniond &quat_des);

Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                const Eigen::VectorXd& ddx, Eigen::MatrixXd *N = nullptr,
                                double svd_epsilon = 0, bool centrifugal_coriolis = false,
                                bool gravity = false);

const Eigen::MatrixXd& Inertia(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                               double svd_epsilon = 0);

const Eigen::MatrixXd& InertiaInverse(const ArticulatedBody& ab, const Eigen::MatrixXd& J);

const Eigen::MatrixXd& JacobianDynamicInverse(const ArticulatedBody& ab,
                                              const Eigen::MatrixXd& J,
                                              double svd_epsilon = 0);

Eigen::Vector6d CentrifugalCoriolis(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                    int idx_link = -1,
                                    const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
                                    double svd_epsilon = 0);

Eigen::VectorXd Gravity(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                        double svd_epsilon = 0);

// ABA
const Eigen::Matrix6d& InertiaAba(const ArticulatedBody& ab,
                                  int idx_link = -1,
                                  const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
                                  double svd_epsilon = 0);

const Eigen::Matrix6d& InertiaInverseAba(const ArticulatedBody& ab,
                                         int idx_link = -1,
                                         const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

Eigen::Vector6d CentrifugalCoriolisAba(const ArticulatedBody& ab,
                                       int idx_link = -1,
                                       const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
                                       double svd_epsilon = 0);

Eigen::Vector6d GravityAba(const ArticulatedBody& ab,
                           int idx_link = -1,
                           const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
                           double svd_epsilon = 0);

}  // namespace Opspace
}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_OPSPACE_DYNAMICS_H_