/**
 * joint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "structs/joint.h"

#include <exception>  // std::invalid_argument
#include <map>        // std::map

namespace spatial_dyn {

Joint::Joint(Joint::Type type) {
  set_type(type);
}

Joint::Joint(const std::string& type) {
  set_type(StringToType(type));
}

void Joint::set_type(Joint::Type type) {
  type_ = type;
  switch (type_) {
    case Joint::Type::RX:
      subspace_ = SpatialMotiond::UnitAngX();
      break;
    case Joint::Type::RY:
      subspace_ = SpatialMotiond::UnitAngY();
      break;
    case Joint::Type::RZ:
      subspace_ = SpatialMotiond::UnitAngZ();
      break;
    case Joint::Type::PX:
      subspace_ = SpatialMotiond::UnitLinX();
      break;
    case Joint::Type::PY:
      subspace_ = SpatialMotiond::UnitLinY();
      break;
    case Joint::Type::PZ:
      subspace_ = SpatialMotiond::UnitLinZ();
      break;
    default:
      subspace_ = SpatialMotiond::Zero();
  }
}

bool Joint::is_prismatic() const {
  return type_ == Joint::Type::PX || type_ == Joint::Type::PY || type_ == Joint::Type::PZ;
}
bool Joint::is_revolute() const {
  return type_ == Joint::Type::RX || type_ == Joint::Type::RY || type_ == Joint::Type::RZ;
}

void Joint::set_q_min(double q_min) {
  if (q_min > q_max_) {
    throw std::invalid_argument("Joint::set_q_min(): q_min (" + std::to_string(q_min) +
                                ") cannot be greater than q_max (" + std::to_string(q_max_) + ").");
  }
  q_min_ = q_min;
}

void Joint::set_q_max(double q_max) {
  if (q_max < q_min_) {
    throw std::invalid_argument("Joint::set_q_min(): q_max (" + std::to_string(q_max) +
                                ") cannot be less than q_min (" + std::to_string(q_min_) + ").");
  }
  q_max_ = q_max;
}

void Joint::set_q_limits(double q_min, double q_max) {
  if (q_min > q_max) {
    throw std::invalid_argument("Joint::set_q_limits(): q_min (" + std::to_string(q_min) +
                                ") cannot be greater than q_max (" + std::to_string(q_max) + ").");
  }
  q_min_ = q_min;
  q_max_ = q_max;
}

void Joint::set_dq_max(double dq_max) {
  if (dq_max < 0.) {
    throw std::invalid_argument("Joint::set_dq_max(): dq_max (" + std::to_string(dq_max) +
                                ") cannot be negative.");
  }
  dq_max_ = dq_max;
}

void Joint::set_fq_max(double fq_max) {
  if (fq_max < 0.) {
    throw std::invalid_argument("Joint::set_fq_max(): fq_max (" + std::to_string(fq_max) +
                                ") cannot be negative.");
  }
  fq_max_ = fq_max;
}

void Joint::set_f_coulomb(double f_coulomb) {
  if (f_coulomb < 0.) {
    throw std::invalid_argument("Joint::set_f_coulomb(): f_coulomb (" + std::to_string(f_coulomb) +
                                ") cannot be negative.");
  }
  f_coulomb_ = f_coulomb;
}

void Joint::set_f_viscous(double f_viscous) {
  if (f_viscous < 0.) {
    throw std::invalid_argument("Joint::set_f_viscous(): f_viscous (" + std::to_string(f_viscous) +
                                ") cannot be negative.");
  }
  f_viscous_ = f_viscous;
}

void Joint::set_f_stiction(double f_stiction) {
  if (f_stiction < 0.) {
    throw std::invalid_argument("Joint::set_f_stiction(): f_stiction (" + std::to_string(f_stiction) +
                                ") cannot be negative.");
  }
  f_stiction_ = f_stiction;
}

Eigen::Isometry3d Joint::T_joint(double q) const {
  switch (type_) {
    case Joint::Type::RX:
      return Eigen::Isometry3d(ctrl_utils::Eigen::RotationX(q));
    case Joint::Type::RY:
      return Eigen::Isometry3d(ctrl_utils::Eigen::RotationY(q));
    case Joint::Type::RZ:
      return Eigen::Isometry3d(ctrl_utils::Eigen::RotationZ(q));
    case Joint::Type::PX:
      return Eigen::Isometry3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitX()));
    case Joint::Type::PY:
      return Eigen::Isometry3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitY()));
    case Joint::Type::PZ:
      return Eigen::Isometry3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitZ()));
    default:
      return Eigen::Isometry3d::Identity();
  }
}

static const std::map<std::string, Joint::Type> kStringToJointType = {
  {"RX", Joint::Type::RX}, {"RY", Joint::Type::RY}, {"RZ", Joint::Type::RZ},
  {"PX", Joint::Type::PX}, {"PY", Joint::Type::PY}, {"PZ", Joint::Type::PZ},
  {"UNDEFINED", Joint::Type::UNDEFINED}
};

std::string Joint::TypeToString(const Type& type) {
  switch (type) {
    case Joint::Type::RX:
      return "RX";
    case Joint::Type::RY:
      return "RY";
    case Joint::Type::RZ:
      return "RZ";
    case Joint::Type::PX:
      return "PX";
    case Joint::Type::PY:
      return "PY";
    case Joint::Type::PZ:
      return "PZ";
    default:
      return "UNDEFINED";
  }
}

Joint::Type Joint::StringToType(const std::string& type) {
  try {
    return kStringToJointType.at(type);
  } catch (...) {
    return Joint::Type::UNDEFINED;
  }
}

std::ostream& operator<<(std::ostream& os, const Joint& j) {
  os << "Joint(type=" << std::string(j) << ", q_lim=[" << j.q_min() << " " << j.q_max() 
     << "], dq_max=" << j.dq_max() << ", fq_max=" << j.fq_max() << ", f_coulomb="
     << j.f_coulomb() << ", f_viscous=" << j.f_viscous() << ", f_stiction=" << j.f_stiction() << ")";
  return os;
}

}  // namespace spatial_dyn
