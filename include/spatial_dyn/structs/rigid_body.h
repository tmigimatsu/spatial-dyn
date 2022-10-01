/**
 * rigid_body.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 4, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_STRUCTS_RIGID_BODY_H_
#define SPATIAL_DYN_STRUCTS_RIGID_BODY_H_

#include <ostream>  // std::ostream
#include <string>   // std::string
#include <vector>   // std::vector

#include "spatial_dyn/eigen/spatial_math.h"
#include "spatial_dyn/structs/graphics.h"
#include "spatial_dyn/structs/joint.h"

namespace spatial_dyn {

class ArticulatedBody;

/**
 * @ingroup cpp_structs
 *
 * Rigid body struct for spatial_dyn.
 *
 * Comprised of an articulated joint and the attached rigid body.
 *
 * @see Python: spatialdyn.RigidBody
 */
class RigidBody {

 public:
  /// @cond
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// @endcond

  /**
   * Default constructor.
   */
  RigidBody() = default;
  virtual ~RigidBody() = default;

  /**
   * Constructor that sets the name of the rigid body.
   *
   * @param name Name of the rigid body.
   * @see Python: spatialdyn.RigidBody.__init__()
   */
  RigidBody(const std::string& name) : name(name) {}

  /**
   * Name of the rigid body for debugging purposes.
   *
   * @see Python: spatialdyn.RigidBody.name
   */
  mutable std::string name;

  /**
   * %Graphics for the rigid body.
   *
   * @see Python: spatialdyn.RigidBody.graphics
   */
  mutable std::vector<Graphics> graphics;

  /**
   * @return ID of the rigid body as assigned by ArticulatedBody::AddRigidBody().
   * @see Python: spatialdyn.RigidBody.id
   */
  int id() const { return id_; }
  void set_id(int id) { id_ = id; }

  /**
   * @return ID of the rigid body's parent as assigned by ArticulatedBody::AddRigidBody().
   * @see Python: spatialdyn.RigidBody.id_parent
   */
  int id_parent() const { return id_parent_; }
  void set_id_parent(int id_parent) { id_parent_ = id_parent; }

  /**
   * @return Fixed transform from the rigid body's frame to its parent's frame
   *         when the joint position is 0. Defaults to identity.
   * @see Python: spatialdyn.RigidBody.T_to_parent
   */
  const Eigen::Isometry3d& T_to_parent() const { return T_to_parent_; }

  /**
   * Set the fixed transform from the rigid body's frame to its parent's frame
   * when the joint position is 0.
   *
   * @param ori_in_parent Orientation of the rigid body in its parent's frame.
   * @param pos_in_parent Position of the rigid body in its parent's frame.
   */
  template<typename Derived>
  void set_T_to_parent(const Eigen::RotationBase<Derived,3>& ori_in_parent,
                       const Eigen::Vector3d& pos_in_parent) {
    T_to_parent_ = Eigen::Translation3d(pos_in_parent) * ori_in_parent;
  }

  /**
   * Set the fixed transform from the rigid body's frame to its parent's frame
   * when the joint position is 0.
   *
   * @param T_to_parent Transform from the rigid body frame to its parent's frame.
   */
  void set_T_to_parent(const Eigen::Isometry3d& T_to_parent) { T_to_parent_ = T_to_parent; }

  /**
   * @return Spatial inertia of the rigid body. Defaults to a 1kg point mass at
   *         the origin of the rigid body's frame.
   * @see Python: spatialdyn.RigidBody.inertia
   */
  const SpatialInertiad& inertia() const { return inertia_; }

  /**
   * Set the spatial inertia of the rigid body.
   *
   * @param mass Mass of the rigid body.
   * @param com Center of mass of the rigid body.
   * @param I_com_flat Inertia of the rigid body as a flat vector `{I_xx, I_xy,
   *        I_xz, I_yy, I_yz, I_zz}``.
   */
  void set_inertia(double mass, const Eigen::Vector3d& com, const Eigen::Vector6d& I_com_flat);

  /**
   * Set the spatial inertia of the rigid body.
   *
   * @param inertia Spatial inertia of the rigid body.
   */
  void set_inertia(const SpatialInertiad& inertia);

  /**
   * @return %Joint attached to the rigid body.
   * @see Python: spatialdyn.RigidBody.joint
   */
  const Joint& joint() const { return joint_; }

  /**
   * Set the joint attached to the rigid body.
   *
   * @param joint Joint.
   */
  void set_joint(const Joint& joint) { joint_ = joint; }

 protected:

  /// @cond
  int id_ = -1;
  int id_parent_ = -1;
  Eigen::Isometry3d T_to_parent_ = Eigen::Isometry3d::Identity();
  SpatialInertiad inertia_ = SpatialInertiad(1, Eigen::Vector3d::Zero(), Eigen::Vector6d::Zero());
  Joint joint_;

  friend class ArticulatedBody;
  /// @endcond

};

/**
 * @ingroup cpp_structs
 * @return Stream representation of the rigid body for debugging.
 * @see Python: spatialdyn.RigidBody.__repr__()
 */
std::ostream& operator<<(std::ostream& os, const RigidBody& rb);

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_STRUCTS_RIGID_BODY_H_
