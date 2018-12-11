/**
 * articulated_body.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 8, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_STRUCTS_ARTICULATED_BODY_H_
#define SPATIAL_DYN_STRUCTS_ARTICULATED_BODY_H_

#include "SpatialDyn/utils/spatial_math.h"
#include "SpatialDyn/structs/rigid_body.h"

#include <functional>  // std::function
#include <memory>      // std::unique_ptr
#include <ostream>     // std::ostream
#include <string>      // std::string
#include <utility>     // std::pair
#include <vector>      // std::vector

namespace SpatialDyn {

/**
 * Main articulated body struct for SpatialDyn.
 *
 * @see Python: spatialdyn.ArticulatedBody
 */
class ArticulatedBody {

 public:
  /// @cond
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// @endcond

  /**
   * Default constructor.
   */
  ArticulatedBody();

  /**
   * Constructor that sets the name of the articulated body.
   *
   * @param name Name of the articulated body.
   */
  ArticulatedBody(const std::string& name);

  /**
   * Copy constructor.
   */
  ArticulatedBody(const ArticulatedBody& ab);

  /**
   * Destructor.
   */
  virtual ~ArticulatedBody();

  /**
   * Name of the articulated body for debugging purposes.
   *
   * @see Python: spatialdyn.ArticulatedBody.name
   */
  std::string name;

  /**
   * Graphics for the base.
   *
   * @see Python: spatialdyn.ArticulatedBody.graphics
   */
  Graphics graphics;

  /**
   * @return Degrees of freedom of the articulated body.
   * @see Python: spatialdyn.ArticulatedBody.dof
   */
  size_t dof() const { return dof_; };

  /**
   * Add a rigid body to the articulated body and return its assigned ID.
   *
   * IDs will be assigned to rigid bodies starting from `0` and incrementing
   * each time AddRigidBody() is called. These IDs can be used to index into the
   * corresponding rigid_bodies() vector or joint variables such as q().
   *
   * @param rb Rigid body to add.
   * @param id_parent ID of the parent rigid body. If `parent_id` is `-1`, the
   *                  rigid body will be added to the base.
   * @return ID assigned to the new rigid body.
   * @see Python: spatialdyn.ArticulatedBody.add_rigid_body()
   */
  int AddRigidBody(RigidBody&& rb, int id_parent = -1);

  /**
   * @see AddRigidBody()
   */
  int AddRigidBody(const RigidBody& rb, int id_parent = -1);

  /**
   * @return Vector of rigid bodies, indexed by IDs assigned in AddRigidBody().
   * @see Python: spatialdyn.ArticulatedBody.rigid_bodies
   */
  const std::vector<RigidBody>& rigid_bodies() const { return rigid_bodies_; };

  /**
   * @return `i`th rigid body with Pythonic indexing (if `i < 0`, count from the back).
   */
  const RigidBody& rigid_bodies(int i) const;

  /**
   * @return Joint positions. Uninitialized values will be `0` by default.
   * @see Python: spatialdyn.ArticulatedBody.q
   */
  const Eigen::VectorXd& q() const { return q_; };

  /**
   * @return `i`th joint position with Pythonic indexing (if `i < 0`, count from the back).
   */
  double q(int i) const;

  /**
   * Set the joint positions and internally precompute the frames of the articulated body.
   */
  void set_q(Eigen::Ref<const Eigen::VectorXd> q);

  /**
   * @return Joint velocities. Uninitialized values will be `0` by default.
   * @see Python: spatialdyn.ArticulatedBody.dq
   */
  const Eigen::VectorXd& dq() const { return dq_; };

  /**
   * @return `i`th joint velocity with Pythonic indexing (if `i < 0`, count from the back).
   */
  double dq(int i) const;

  /**
   * Set the joint velocities.
   */
  void set_dq(Eigen::Ref<const Eigen::VectorXd> dq);

  /**
   * @return Joint accelerations. Uninitialized values will be `0` by default.
   * @see Python: spatialdyn.ArticulatedBody.ddq
   */
  const Eigen::VectorXd& ddq() const { return ddq_; };

  /**
   * @return `i`th joint acceleration with Pythonic indexing (if `i < 0`, count from the back).
   */
  double ddq(int i) const;

  /**
   * Set the joint accelerations.
   */
  void set_ddq(Eigen::Ref<const Eigen::VectorXd> ddq);

  /**
   * @return 6d spatial gravity vector acting on the articulated body. Defaults
   *         to `-9.81` in the linear `z` direction.
   * @see Python: spatialdyn.ArticulatedBody.g
   */
  const SpatialMotiond& g() const { return g_; };

  /**
   * Set the gravity vector acting on the articulated body.
   */
  void set_g(const Eigen::Vector3d& g);

  /**
   * Get the transform from the articulated body's base frame to the world frame.
   *
   * This transformation matrix will transform a vector represented in the
   * articulated body's base frame to one represented in the world frame. Set to
   * identity by default.
   *
   * Example:
   * ```{.cc}
   * // Arbitrary position vector represented in the articulated body's base frame
   * Eigen::Vector3d pos_in_ab(0.1, 0.1, 0.1);
   *
   * // Same position vector now represented in the world frame
   * Eigen::Vector3d pos_in_world = ab.T_base_to_world() * pos_in_ab;
   * ```
   *
   * @return Transform from the base frame to the world frame.
   * @see Python: spatialdyn.ArticulatedBody.T_base_to_world
   */
  const Eigen::Isometry3d& T_base_to_world() const { return T_base_to_world_; };

  /**
   * Set the transform from the articulated body's base frame to the world frame.
   *
   * @param ori_in_world Orientation of the articulated body in the world frame.
   * @param pos_in_world Position of the articulated body in the world frame.
   */
  void set_T_base_to_world(const Eigen::Quaterniond& ori_in_world,
                           const Eigen::Vector3d& pos_in_world);

  /**
   * Set the transform from the articulated body's base frame to the world frame.
   *
   * @param T_to_world Transform from the base frame to the world frame.
   */
  void set_T_base_to_world(const Eigen::Isometry3d& T_to_world);

  /**
   * Get the transform from rigid body `i`'s frame to its parent's frame.
   *
   * This transformation matrix will transform a vector represented in frame `i`
   * to one represented in frame `i-1` according to the `i`th value of the
   * articulated body's current joint positions q(). Uses Pythonic indexing (if
   * `i < 0`, count from the back).
   *
   * __Example__
   * ```{.cc}
   * // Arbitrary position vector represented in the end-effector's frame
   * Eigen::Vector3d pos_in_ee(0.1, 0.1, 0.1);
   *
   * // Same positiion vector represented in the frame of the end-effector's parent
   * Eigen::Vector3d pos_in_ee_parent = ab.T_to_parent(-1);
   * ```
   *
   * @return Transform from the `i`th rigid body's frame to its parent's frame.
   * @see Python: spatialdyn.ArticulatedBody.T_to_parent()
   */
  const Eigen::Isometry3d& T_to_parent(int i) const;

  /**
   * @return Inverse transform of T_to_parent() (from rigid body `i-1` to `i`).
   * @see Python: spatialdyn.ArticulatedBody.T_from_parent()
   */
  const Eigen::Isometry3d& T_from_parent(int i) const;

  /**
   * Get the transform from rigid body `i`'s frame to the world frame.
   *
   * This transformation matrix will transform a vector represented in frame `i`
   * to one represented in the world frame according to the articulated body's
   * current joint positions q(). Uses Pythonic indexing (if `i < 0`, count from
   * the back).
   *
   * __Example__
   * ```{.cc}
   * // Arbitrary position vector represented in the end-effector's frame
   * Eigen::Vector3d pos_in_ee(0.1, 0.1, 0.1);
   *
   * // Same position vector represented in the world frame
   * Eigen::Vector3d pos_in_ee_parent = ab.T_to_parent(-1);
   * ```
   *
   * @return Transform from the `i`th rigid body's frame to the world. Uses
   *         Pythonic indexing (if `i < 0`, count from the back).
   * @see Python: spatialdyn.ArticulatedBody.T_to_world()
   */
  const Eigen::Isometry3d& T_to_world(int i) const;

  /**
   * @return Vector of ancestor IDs of rigid body `i`, from the base to the
   *         rigid body (inclusive). Uses Pythonic indexing (if `i < 0`, count
   *         from the back).
   * @see Python: spatialdyn.ArticulatedBody.ancestors()
   */
  const std::vector<int>& ancestors(int i) const;

  /**
   * @return Vector of IDs in the subtree of rigid body `i`, going from the
   *         rigid body (inclusive) to the leaves. Uses Pythonic indexing (if `i
   *         < 0`, count from the back).
   * @see Python: spatialdyn.ArticulatedBody.subtree()
   */
  const std::vector<int>& subtree(int i) const;

  /**
   * Map the given function across the articulated body structure.
   *
   * @param rb_function Function to apply to each rigid body.
   * @return Vector of doubles containing the mapped function results.
   * @see Python: spatialdyn.ArticulatedBody.map()
   */
  Eigen::VectorXd Map(const std::function<double(const RigidBody& rb)>& rb_function) const;

  /// @cond
  struct Cache;
  std::unique_ptr<Cache> cache_;
  /// @endcond

 protected:

  /// @cond
  void CalculateTransforms();
  void ExpandDof(int id, int id_parent);

  size_t dof_ = 0;
  std::vector<RigidBody> rigid_bodies_;

  Eigen::VectorXd q_;
  Eigen::VectorXd dq_;
  Eigen::VectorXd ddq_;
  SpatialMotiond g_ = -9.81 * SpatialMotiond::UnitLinZ();

  Eigen::Isometry3d T_base_to_world_ = Eigen::Isometry3d::Identity();
  std::vector<Eigen::Isometry3d> T_to_parent_;
  std::vector<Eigen::Isometry3d> T_from_parent_;
  std::vector<Eigen::Isometry3d> T_to_world_;

  std::vector<std::vector<int>> ancestors_;  // Ancestors from base to link i
  std::vector<std::vector<int>> subtrees_;   // Descendants of link i including link i
  /// @endcond

};

/**
 * @return String representation of the articulated body's state.
 */
std::ostream& operator<<(std::ostream& os, const ArticulatedBody& ab);

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_STRUCTS_ARTICULATED_BODY_H_
