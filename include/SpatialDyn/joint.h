/**
 * joint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_JOINT_H_
#define SPATIAL_DYN_JOINT_H_

#include "spatial_math.h"

#include <map>  // std::map

namespace SpatialDyn {

enum class JointType { UNDEFINED, RX, RY, RZ, PX, PY, PZ };

class Joint {

 public:

  Joint() {}
  Joint(JointType type);

  JointType type() const;
  void set_type(JointType type);

  // Motion subspace
  const SpatialMotiond& subspace() const;

  Eigen::Isometry3d T_joint(double q) const;

  operator std::string() const;
  static JointType FromString(const std::string& type);

 protected:

  JointType type_ = JointType::UNDEFINED;
  SpatialMotiond subspace_ = SpatialMotiond::Zero();

};

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_JOINT_H_
