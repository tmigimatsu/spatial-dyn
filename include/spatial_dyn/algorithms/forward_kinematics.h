/**
 * forward_kinematics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 4, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_FORWARD_KINEMATICS_H_
#define SPATIAL_DYN_ALGORITHMS_FORWARD_KINEMATICS_H_

#include "spatial_dyn/eigen/spatial_math.h"
#include "spatial_dyn/structs/articulated_body.h"

namespace spatial_dyn {

/**
 * @defgroup cpp_forward_kinematics Forward Kinematics
 * @ingroup cpp_algorithms
 *
 * C++ implementation of spatial_dyn forward kinematics algorithms.
 *
 * @see Python: \ref py_forward_kinematics
 * @{
 */

/**
 * Compute the world frame position of an offset vector in link `i`.
 *
 * Uses the articulated body's cached world transforms to run in O(1) time.
 *
 * @param ab Articulated body.
 * @param link Index of the desired link.
 * @param offset Vector offset in link `i`.
 * @return World frame position of the offset in link `i`.
 * @see Python: spatialdyn.position()
 */
Eigen::Vector3d Position(const ArticulatedBody& ab, int link = -1,
                         const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

/**
 * Compute the world frame position of an offset vector in link `i` given
 * articulated body configuration q.
 *
 * Runs in O(n) time.
 *
 * @param ab Articulated body.
 * @param q %Joint configuration.
 * @param link Index of the desired link.
 * @param offset Vector offset in link `i`.
 * @return World frame position of the offset in link `i`.
 * @see Python: spatialdyn.position()
 */
Eigen::Vector3d Position(const ArticulatedBody& ab,
                         Eigen::Ref<const Eigen::VectorXd> q, int link = -1,
                         const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

/**
 * Compute the world frame orientation of link `i`.
 *
 * Uses the articulated body's cached world transforms to run in O(1) time.
 *
 * @param ab Articulated body.
 * @param link Index of the desired link.
 * @return World frame orientation of link `i`.
 * @see Python: spatialdyn.orientation()
 */
Eigen::Quaterniond Orientation(const ArticulatedBody& ab, int link = -1);

/**
 * Compute the world frame orientation of link `i` given articulated body
 * configuration q.
 *
 * Runs in O(n) time.
 *
 * @param ab Articulated body.
 * @param q %Joint configuration.
 * @param link Index of the desired link.
 * @return World frame orientation of link `i`.
 * @see Python: spatialdyn.orientation()
 */
Eigen::Quaterniond Orientation(const ArticulatedBody& ab,
                               Eigen::Ref<const Eigen::VectorXd> q, int link = -1);

/**
 * Compute the world frame pose of an offset vector in link `i`.
 *
 * Uses the articulated body's cached world transforms to run in O(1) time.
 *
 * @param ab Articulated body.
 * @param link Index of the desired link.
 * @param offset Vector offset in link `i`.
 * @return World frame pose of the offset in link `i`.
 * @see Python: spatialdyn.cartesian_pose()
 */
Eigen::Isometry3d CartesianPose(const ArticulatedBody& ab, int link = -1,
                                const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

/**
 * Compute the world frame pose of an offset vector in link `i` given
 * articulated body configuration q.
 *
 * Runs in O(n) time.
 *
 * @param ab Articulated body.
 * @param q %Joint configuration.
 * @param link Index of the desired link.
 * @param offset Vector offset in link `i`.
 * @return World frame pose of the offset in link `i`.
 * @see Python: spatialdyn.cartesian_pose()
 */
Eigen::Isometry3d CartesianPose(const ArticulatedBody& ab,
                                Eigen::Ref<const Eigen::VectorXd> q, int link = -1,
                                const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

/**
 * Compute the world frame basic Jacobian of the given operational point.
 *
 * The computation is cached such that the first call to Jacobian() runs in O(n)
 * time, and subsequent calls with the same joint position, link, and offset run
 * in O(1) time.
 *
 * @param ab Articulated body.
 * @param link Index of the desired link.
 * @param offset Vector offset of the operational point in link `i`.
 * @return Reference to the cached basic Jacobian (linear stacked above angular)
 *         represented in the world frame.
 * @see Python: spatialdyn.jacobian()
 */
const Eigen::Matrix6Xd& Jacobian(const ArticulatedBody& ab, int link = -1,
                                 const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

/**
 * Compute the world frame basic Jacobian of the given operational point.
 *
 * Runs in O(n) time.
 *
 * @param ab Articulated body.
 * @param q %Joint configuration.
 * @param link Index of the desired link.
 * @param offset Vector offset of the operational point in link `i`.
 * @return Basic Jacobian (linear stacked above angular) represented in the
 *         world frame.
 * @see Python: spatialdyn.jacobian()
 */
Eigen::Matrix6Xd Jacobian(const ArticulatedBody& ab,
                          Eigen::Ref<const Eigen::VectorXd> q, int link = -1,
                          const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

/**
 * Compute the world frame linear Jacobian of the given operational point.
 *
 * The computation is cached such that the first call to Jacobian() runs in O(n)
 * time, and subsequent calls with the same joint position, link, and offset run
 * in O(1) time.
 *
 * @param ab Articulated body.
 * @param link Index of the desired link.
 * @param offset Vector offset of the operational point in link `i`.
 * @return Reference to the top 3 rows of the cached basic Jacobian represented
 *         in the world frame.
 * @see Python: spatialdyn.linear_jacobian()
 */
Eigen::Ref<const Eigen::Matrix3Xd> LinearJacobian(const ArticulatedBody& ab, int link = -1,
                                                  const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

/**
 * Compute the world frame linear Jacobian of the given operational point.
 *
 A Runs in O(n) time.
 *
 * @param ab Articulated body.
 * @param q Joint configuration.
 * @param link Index of the desired link.
 * @param offset Vector offset of the operational point in link `i`.
 * @return Linear Jacobian (top 3 rows of the basic Jacobian) represented in the
 *         world frame.
 * @see Python: spatialdyn.linear_jacobian()
 */
Eigen::Matrix3Xd LinearJacobian(const ArticulatedBody& ab,
                                Eigen::Ref<const Eigen::VectorXd> q, int link = -1,
                                const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

/**
 * Compute the world frame angular Jacobian of the given operational point.
 *
 * The computation is cached such that the first call to Jacobian() runs in O(n)
 * time, and subsequent calls with the same joint position, link, and offset run
 * in O(1) time. AngularJacobian() uses the last offset passed to Jacobian() or
 * LinearJacobian() for caching, and therefore should be called after Jacobian()
 * or LinearJacobian() to avoid unnecessary computations with a nonzero offset.
 *
 * @param ab Articulated body.
 * @param link Index of the desired link.
 * @return Reference to the bottom 3 rows of the cached basic Jacobian represented
 *         in the world frame.
 * @see Python: spatialdyn.angular_jacobian()
 */
Eigen::Ref<const Eigen::Matrix3Xd> AngularJacobian(const ArticulatedBody& ab, int link = -1);

/**
 * Compute the world frame angular Jacobian of the given operational point.
 *
 * Runs in O(n) time.
 *
 * @param ab Articulated body.
 * @param q %Joint configuration.
 * @param link Index of the desired link.
 * @return Angular Jacobian (bottom 3 rows of the basic Jacobian) represented in
 *         the world frame.
 * @see Python: spatialdyn.angular_jacobian()
 */
Eigen::Matrix3Xd AngularJacobian(const ArticulatedBody& ab,
                                 Eigen::Ref<const Eigen::VectorXd> q, int link = -1);

/**
 * Compute the world frame Hessian of the given operational point.
 *
 * @param ab Articulated body.
 * @param link Index of the desired link.
 * @param offset Vector offset of the operational point in link `i`.
 * @return [n x n x 6] Hessian represented in the world frame.
 * @see Python: spatialdyn.hessian()
 */
Eigen::Tensor3d Hessian(const ArticulatedBody& ab, int link = -1,
                        const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

/**
 * @}
 */  // defgroup cpp_forward_kinematics

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_ALGORITHMS_FORWARD_KINEMATICS_H_
