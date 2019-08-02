/**
 * articulated_body.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 8, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_STRUCTS_ARTICULATED_BODY_H_
#define SPATIAL_DYN_STRUCTS_ARTICULATED_BODY_H_

#include <functional>  // std::function
#include <map>         // std::map
#include <memory>      // std::unique_ptr
#include <ostream>     // std::ostream
#include <string>      // std::string
#include <utility>     // std::pair
#include <vector>      // std::vector

#include "spatial_dyn/eigen/spatial_math.h"
#include "spatial_dyn/structs/rigid_body.h"

namespace spatial_dyn {

/**
 * @ingroup cpp_structs
 *
 * Main articulated body struct for spatial_dyn.
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
   * @see Python: spatialdyn.ArticulatedBody.__init__()
   */
  ArticulatedBody(const std::string& name);

  /**
   * Copy constructor.
   */
  ArticulatedBody(const ArticulatedBody& ab);

  /**
   * Move constructor.
   */
  ArticulatedBody(ArticulatedBody&& ab);

  /**
   * Destructor.
   */
  virtual ~ArticulatedBody();

  /**
   * Copy assignment operator.
   */
  ArticulatedBody& operator=(const ArticulatedBody& ab);

  /**
   * Move assignment operator.
   */
  // ArticulatedBody& operator=(ArticulatedBody&& ab) = default;

  /**
   * Name of the articulated body for debugging purposes.
   *
   * @see Python: spatialdyn.ArticulatedBody.name
   */
  mutable std::string name;

  /**
   * %Graphics for the base.
   *
   * @see Python: spatialdyn.ArticulatedBody.graphics
   */
  mutable std::vector<Graphics> graphics;

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
  virtual void set_q(Eigen::Ref<const Eigen::VectorXd> q);

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
  virtual void set_dq(Eigen::Ref<const Eigen::VectorXd> dq);

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
   * Add a spatial inertia to the existing load on the specified link.
   *
   * If a load doesn't exist on the link, a new load will be created.
   *
   * @param inertia Spatial inertia of load to add represented in the link's frame.
   * @param idx_link Index of the desired link.
   * @see Python: spatialdyn.ArticulatedBody.add_load
   */
  virtual void AddLoad(const SpatialInertiad& inertia, int idx_link = -1);

  /**
   * Replace the existing inertial load on the specified link.
   *
   * If a load doesn't exist on the link, a new load will be created.
   *
   * @param inertia Spatial inertia of load to add represented in the link's frame.
   * @param idx_link Index of the desired link.
   * @see Python: spatialdyn.ArticulatedBody.replace_load
   */
  virtual void ReplaceLoad(const SpatialInertiad& inertia, int idx_link = -1);

  /**
   * Clear the existing inertial load on the specified link.
   *
   * @param idx_link Index of the desired link.
   * @see Python: spatialdyn.ArticulatedBody.clear_load
   */
  virtual void ClearLoad(int idx_link = -1);

  /**
   * @return Reference to Map of (index, inertia) pairs where the inertia is the
   *         inertia of a load attached to the associated rigid body index.
   * @see Python: spatialdyn.ArticulatedBody.inertia_load
   */
  const std::map<size_t, SpatialInertiad>& inertia_load() const { return inertia_load_; }

  /**
   * Get the transform from the articulated body's base frame to the world frame.
   *
   * This transformation matrix will transform a vector represented in the
   * articulated body's base frame to one represented in the world frame. Set to
   * identity by default.
   *
   * __Example__
   * ~~~~~~~~~~ {.cc}
   * // Arbitrary position vector represented in the articulated body's base frame
   * Eigen::Vector3d pos_in_ab(0.1, 0.1, 0.1);
   *
   * // Same position vector now represented in the world frame
   * Eigen::Vector3d pos_in_world = ab.T_base_to_world() * pos_in_ab;
   * ~~~~~~~~~~
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
  template<typename Derived>
  void set_T_base_to_world(const Eigen::RotationBase<Derived,3>& ori_in_world,
                           const Eigen::Vector3d& pos_in_world) {
    T_base_to_world_ = Eigen::Translation3d(pos_in_world) * ori_in_world;
  }

  /**
   * Set the transform from the articulated body's base frame to the world frame.
   *
   * @param T_to_world Transform from the base frame to the world frame.
   */
  void set_T_base_to_world(const Eigen::Isometry3d& T_to_world);

  /**
   * @return Spatial inertia of the base link. Defaults to a 1kg point mass at
   *         the origin of the rigid body's frame.
   * @see Python: spatialdyn.ArticulatedBody.inertia_base
   */
  const SpatialInertiad& inertia_base() const { return inertia_base_; }

  /**
   * Set the spatial inertia of the base link.
   *
   * @param mass Mass of the rigid body.
   * @param com Center of mass of the rigid body.
   * @param I_com_flat Inertia of the rigid body as a flat vector `{I_xx, I_xy,
   *        I_xz, I_yy, I_yz, I_zz}``.
   */
  void set_inertia_base(double mass, const Eigen::Vector3d& com, const Eigen::Vector6d& I_com_flat);

  /**
   * Set the spatial inertia of the base link.
   *
   * @param inertia Spatial inertia of the rigid body.
   */
  void set_inertia_base(const SpatialInertiad& inertia);

  /**
   * Get the transform from rigid body `i`'s frame to its parent's frame.
   *
   * The computed transform is cached to avoid recomputation in subsequent calls
   * to T_to_parent() for the same joint configuration.
   *
   * This transformation matrix will transform a vector represented in frame `i`
   * to one represented in frame `i-1` according to the `i`th value of the
   * articulated body's current joint positions q(). Uses Pythonic indexing (if
   * `i < 0`, count from the back).
   *
   * __Example__
   * ~~~~~~~~~~ {.cc}
   * // Arbitrary position vector represented in the end-effector's frame
   * Eigen::Vector3d pos_in_ee(0.1, 0.1, 0.1);
   *
   * // Same positiion vector represented in the frame of the end-effector's parent
   * Eigen::Vector3d pos_in_ee_parent = ab.T_to_parent(-1);
   * ~~~~~~~~~~
   *
   * @param i Index of the desired frame. Uses Pythonic indexing (if `i < 0`,
   *          count from the back).
   * @return Reference to the ached transform from the `i`th rigid body's frame
   *         to its parent's frame.
   * @see Python: spatialdyn.ArticulatedBody.T_to_parent()
   */
  const Eigen::Isometry3d& T_to_parent(int i) const;

  /**
   * Get the transform from rigid body `i`'s frame to its parent's frame given
   * joint position q.
   *
   * @param i Index of the desired frame. Uses Pythonic indexing (if `i < 0`,
   *          count from the back).
   * @param q Position of joint `i`.
   * @return Transform from the `i`th rigid body's frame to its parent's frame.
   * @see Python: spatialdyn.ArticulatedBody.T_to_parent()
   */
  Eigen::Isometry3d T_to_parent(int i, double q) const;

  /**
   * Get the transform from rigid body `i`'s parent frame to its own frame.
   *
   * @param i Index of the desired frame. Uses Pythonic indexing (if `i < 0`,
   *          count from the back).
   * @return Inverse transform of T_to_parent(int).
   * @see Python: spatialdyn.ArticulatedBody.T_from_parent()
   */
  Eigen::Isometry3d T_from_parent(int i) const { return T_to_parent(i).inverse(); };

  /**
   * Get the transform from rigid body `i`'s parent frame to its own frame given
   * joint position q.
   *
   * @param i Index of the desired frame. Uses Pythonic indexing (if `i < 0`,
   *          count from the back).
   * @param q Position of joint `i`.
   * @return Inverse transform of T_to_parent(int, double).
   * @see Python: spatialdyn.ArticulatedBody.T_from_parent()
   */
  Eigen::Isometry3d T_from_parent(int i, double q) const { return T_to_parent(i, q).inverse(); };

  /**
   * Get the transform from rigid body `i`'s frame to the world frame.
   *
   * The computed transform is cached to avoid recomputation in subsequent calls
   * to T_to_world() for the same joint configuration.
   *
   * This transformation matrix will transform a vector represented in frame `i`
   * to one represented in the world frame according to the articulated body's
   * current joint positions q(). Uses Pythonic indexing (if `i < 0`, count from
   * the back).
   *
   * __Example__
   * ~~~~~~~~~~ {.cc}
   * // Arbitrary position vector represented in the end-effector's frame
   * Eigen::Vector3d pos_in_ee(0.1, 0.1, 0.1);
   *
   * // Same position vector represented in the world frame
   * Eigen::Vector3d pos_in_ee_parent = ab.T_to_parent(-1);
   * ~~~~~~~~~~
   *
   * @param i Index of the desired frame. Uses Pythonic indexing (if `i < 0`,
   *          count from the back).
   * @return Reference to the cached transform from the `i`th rigid body's frame to the world.
   * @see Python: spatialdyn.ArticulatedBody.T_to_world()
   */
  const Eigen::Isometry3d& T_to_world(int i) const;

  /**
   * Get the transform from rigid body `i`'s frame to the world frame given
   * joint position q.
   *
   * @param i Index of the desired frame. Uses Pythonic indexing (if `i < 0`,
   *          count from the back).
   * @param q %Joint configuration of articulated body.
   * @return Transform from the `i`th rigid body's frame to the world.
   * @see Python: spatialdyn.ArticulatedBody.T_to_world()
   */
  Eigen::Isometry3d T_to_world(int i, Eigen::Ref<const Eigen::VectorXd> q) const;

  /**
   * Get the transform from the world frame to rigid body `i`'s frame.
   *
   * @param i Index of the desired frame. Uses Pythonic indexing (if `i < 0`,
   *          count from the back).
   * @return Inverse transform of T_to_world(int).
   * @see Python: spatialdyn.ArticulatedBody.T_from_world()
   */
  Eigen::Isometry3d T_from_world(int i) const { return T_to_world(i).inverse(); };

  /**
   * Get the transform from the world frame to rigid body `i`'s frame given
   * joint position q.
   *
   * @param i Index of the desired frame. Uses Pythonic indexing (if `i < 0`,
   *          count from the back).
   * @param q %Joint configuration of articulated body.
   * @return Inverse transform of T_to_world(int, double).
   * @see Python: spatialdyn.ArticulatedBody.T_from_world()
   */
  Eigen::Isometry3d T_from_world(int i, Eigen::Ref<const Eigen::VectorXd> q) const {
    return T_to_world(i, q).inverse();
  };

  /**
   * Get the ancestors of rigid body `i`.
   *
   * @param i Index of the desired frame. Uses Pythonic indexing (if `i < 0`,
   *          count from the back).
   * @return Vector of ancestor IDs of rigid body `i`, from the base to the
   *         rigid body (inclusive).
   * @see Python: spatialdyn.ArticulatedBody.ancestors()
   */
  const std::vector<int>& ancestors(int i) const;

  /**
   * Get the subtree of rigid body `i`.
   *
   * @param i Index of the desired frame. Uses Pythonic indexing (if `i < 0`,
   *          count from the back).
   * @return Vector of IDs in the subtree of rigid body `i`, going from the
   *         rigid body (inclusive) to the leaves.
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
  struct Cache;                   // Defined in articulated_body_cache.h
  std::unique_ptr<Cache> cache_;  // Pointer to cache for internal use
  /// @endcond

 protected:

  /// @cond
  void ExpandDof(int id, int id_parent);

  size_t dof_ = 0;
  std::vector<RigidBody> rigid_bodies_;

  Eigen::VectorXd q_;
  Eigen::VectorXd dq_;
  SpatialMotiond g_ = -9.81 * SpatialMotiond::UnitLinZ();

  std::map<size_t, SpatialInertiad> inertia_load_;

  Eigen::Isometry3d T_base_to_world_ = Eigen::Isometry3d::Identity();
  SpatialInertiad inertia_base_ = SpatialInertiad(1, Eigen::Vector3d::Zero(), Eigen::Vector6d::Zero());

  std::vector<std::vector<int>> ancestors_;  // Ancestors from base to link i
  std::vector<std::vector<int>> subtrees_;   // Descendants of link i including link i
  /// @endcond

};

/**
 * @ingroup cpp_structs
 * @return Stream representation of the articulated body for debugging.
 * @see Python: spatialdyn.ArticulatedBody.__repr__()
 */
std::ostream& operator<<(std::ostream& os, const ArticulatedBody& ab);

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_STRUCTS_ARTICULATED_BODY_H_
