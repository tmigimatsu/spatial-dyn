/**
 * opspace_dynamics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 4, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_OPSPACE_DYNAMICS_H_
#define SPATIAL_DYN_ALGORITHMS_OPSPACE_DYNAMICS_H_

#include <map>  // std::map

#include "spatial_dyn/eigen/spatial_math.h"
#include "spatial_dyn/structs/articulated_body.h"
#include "spatial_dyn/structs/options.h"

namespace spatial_dyn {
namespace opspace {

bool IsSingular(const ArticulatedBody& ab, const Eigen::MatrixXd& J, double svd_epsilon = 0);

Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                                const Eigen::VectorXd& ddx, Eigen::MatrixXd *N = nullptr,
                                const std::map<size_t, SpatialForced>& f_external = {},
                                const InverseDynamicsOptions& options = {});

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

Eigen::VectorXd ExternalForces(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                               const std::map<size_t, SpatialForced>& f_external = {},
                               double svd_epsilon = 0);

Eigen::VectorXd Friction(const ArticulatedBody& ab, const Eigen::MatrixXd& J,
                         const Eigen::Ref<const Eigen::VectorXd> tau,
                         double svd_epsilon = 0, double stiction_epsilon = 0.01);

// ABA
const Eigen::Matrix6d& InertiaAba(const ArticulatedBody& ab, int idx_link = -1,
                                  const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
                                  double svd_epsilon = 0);

const Eigen::Matrix6d& InertiaInverseAba(const ArticulatedBody& ab, int idx_link = -1,
                                         const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

Eigen::Vector6d CentrifugalCoriolisAba(const ArticulatedBody& ab, int idx_link = -1,
                                       const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
                                       double svd_epsilon = 0);

Eigen::Vector6d GravityAba(const ArticulatedBody& ab, int idx_link = -1,
                           const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
                           const std::map<size_t, SpatialForced>& f_external = {},
                           double svd_epsilon = 0);

}  // namespace opspace
}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_ALGORITHMS_OPSPACE_DYNAMICS_H_
