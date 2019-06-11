/**
 * inverse_kinematics.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 2, 2019
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_INVERSE_KINEMATICS_H_
#define SPATIAL_DYN_ALGORITHMS_INVERSE_KINEMATICS_H_

#include "spatial_dyn/eigen/spatial_math.h"
#include "spatial_dyn/structs/articulated_body.h"

namespace spatial_dyn {

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
Eigen::VectorXd InverseKinematics(const ArticulatedBody& ab,
                                  Eigen::Ref<const Eigen::Vector3d> x,
                                  const Eigen::Quaterniond& quat, int link = -1,
                                  const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_ALGORITHMS_INVERSE_KINEMATICS_H_
