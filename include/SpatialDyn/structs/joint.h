/**
 * joint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_STRUCTS_JOINT_H_
#define SPATIAL_DYN_STRUCTS_JOINT_H_

#include "SpatialDyn/utils/spatial_math.h"

#include <limits>  // std::numeric_limits
#include <string>  // std::string

namespace SpatialDyn {

enum class JointType { UNDEFINED, RX, RY, RZ, PX, PY, PZ };

class Joint {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Joint() {}
  Joint(JointType type);

  JointType type() const;
  void set_type(JointType type);

  // Motion subspace
  const SpatialMotiond& subspace() const;

  // Limits
  double q_min() const;
  void set_q_min(double q_min);

  double q_max() const;
  void set_q_max(double q_max);

  void set_q_limits(double q_min, double q_max);

  double dq_max() const;
  void set_dq_max(double dq_max);

  double fq_max() const;
  void set_fq_max(double fq_max);

  // Friction
  double f_coulomb() const;
  void set_f_coulomb(double f_coulomb);

  double f_stiction() const;
  void set_f_stiction(double f_stiction);

  // Transform
  Eigen::Isometry3d T_joint(double q) const;

  // String
  operator std::string() const;
  static JointType FromString(const std::string& type);

 protected:

  JointType type_ = JointType::UNDEFINED;
  SpatialMotiond subspace_ = SpatialMotiond::Zero();

  double q_min_  = -std::numeric_limits<double>::infinity();
  double q_max_  = std::numeric_limits<double>::infinity();
  double dq_max_ = std::numeric_limits<double>::infinity();
  double fq_max_ = std::numeric_limits<double>::infinity();
  double f_coulomb_  = 0;
  double f_stiction_ = 0;

};

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_STRUCTS_JOINT_H_
