/**
 * joint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "joint.h"

namespace SpatialDyn {

Joint::Joint(JointType type) {
  set_type(type);
}

JointType Joint::type() const {
  return type_;
}
void Joint::set_type(JointType type) {
  type_ = type;
  switch (type_) {
    case JointType::RX:
      subspace_ = SpatialMotiond::UnitAngX();
      break;
    case JointType::RY:
      subspace_ = SpatialMotiond::UnitAngY();
      break;
    case JointType::RZ:
      subspace_ = SpatialMotiond::UnitAngZ();
      break;
    case JointType::PX:
      subspace_ = SpatialMotiond::UnitLinX();
      break;
    case JointType::PY:
      subspace_ = SpatialMotiond::UnitLinY();
      break;
    case JointType::PZ:
      subspace_ = SpatialMotiond::UnitLinZ();
      break;
    default:
      subspace_ = SpatialMotiond::Zero();
  }
}

const SpatialMotiond& Joint::subspace() const {
  return subspace_;
}

Eigen::Isometry3d Joint::T_joint(double q) const {
  switch (type_) {
    case JointType::RX:
      return Eigen::Isometry3d(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitX()));
    case JointType::RY:
      return Eigen::Isometry3d(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitY()));
    case JointType::RZ:
      return Eigen::Isometry3d(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitZ()));
    case JointType::PX:
      return Eigen::Isometry3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitX()));
    case JointType::PY:
      return Eigen::Isometry3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitY()));
    case JointType::PZ:
      return Eigen::Isometry3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitZ()));
    default:
      return Eigen::Isometry3d::Identity();
  }
}

}  // namespace SpatialDyn
