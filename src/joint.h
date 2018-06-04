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

  Eigen::Affine3d T_joint(double q) const;

 protected:

  JointType type_ = JointType::UNDEFINED;
  SpatialMotiond subspace_ = SpatialMotiond(0, 0, 0, 0, 0, 0);

};

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_JOINT_H_
