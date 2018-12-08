/**
 * rigid_body.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 8, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_STRUCTS_RIGID_BODY_H_
#define SPATIAL_DYN_STRUCTS_RIGID_BODY_H_

#include "SpatialDyn/structs/graphics.h"
#include "SpatialDyn/structs/joint.h"
#include "SpatialDyn/utils/spatial_math.h"

#include <ostream>  // std::ostream
#include <string>   // std::string

namespace SpatialDyn {

class ArticulatedBody;

class RigidBody {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RigidBody() {}
  RigidBody(const std::string& name);

  std::string name;
  Graphics graphics;

  int id() const;
  int id_parent() const;

  const Eigen::Isometry3d& T_to_parent() const;
  void set_T_to_parent(const Eigen::Quaterniond& ori_in_parent,
                       const Eigen::Vector3d& pos_in_parent);
  void set_T_to_parent(const Eigen::Isometry3d& T_to_parent);

  const SpatialInertiad& inertia() const;
  void set_inertia(double mass, const Eigen::Vector3d& com, const Eigen::Vector6d& I_com_flat);
  void set_inertia(SpatialInertiad&& inertia);
  void set_inertia(const SpatialInertiad& inertia);

  const Joint& joint() const;
  void set_joint(Joint&& joint);
  void set_joint(const Joint& joint);

 protected:

  int id_ = -1;
  int id_parent_ = -1;
  Eigen::Isometry3d T_to_parent_ = Eigen::Isometry3d::Identity();
  SpatialInertiad inertia_ = SpatialInertiad(1, Eigen::Vector3d::Zero(), Eigen::Vector6d::Zero());
  Joint joint_;

  friend class ArticulatedBody;

};

std::ostream& operator<<(std::ostream& os, const RigidBody& rb);

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_STRUCTS_RIGID_BODY_H_
