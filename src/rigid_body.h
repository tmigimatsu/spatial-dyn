/**
 * articulated_body.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 8, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_RIGID_BODY_H_
#define SPATIAL_DYN_RIGID_BODY_H_

#include "joint.h"
#include "spatial_math.h"

#include <string>  // std::string

namespace SpatialDyn {

class ArticulatedBody;

class RigidBody {

 public:

  RigidBody(const std::string& name);

  std::string name;

  int id() const;
  int id_parent() const;

  const Eigen::Affine3d& T_to_parent() const;
  void set_T_to_parent(const Eigen::Quaterniond& ori_in_parent,
                       const Eigen::Vector3d& pos_in_parent);
  void set_T_to_parent(const Eigen::Affine3d& T_to_parent);

  const SpatialInertiad& inertia() const;
  void set_inertia(double mass, const Eigen::Vector3d& com, const Eigen::Vector6d& I_com_flat);

  const Joint& joint() const;
  void set_joint(Joint&& joint);

 protected:

  int id_ = -1;
  int id_parent_ = -1;
  Eigen::Affine3d T_to_parent_ = Eigen::Affine3d::Identity();
  SpatialInertiad inertia_ = SpatialInertiad(1, Eigen::Vector3d::Zero(), Eigen::Vector6d::Zero());
  Joint joint_;

  friend class ArticulatedBody;

};

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_RIGID_BODY_H_
