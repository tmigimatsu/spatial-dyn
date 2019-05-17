/**
 * joint.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 3, 2018
 * Authors: Toki Migimatsu
 */

#include "structs/joint.h"

#include <algorithm>  // std::transform
#include <cctype>     // std::tolower
#include <exception>  // std::invalid_argument
#include <map>        // std::map

#include <ctrl_utils/string.h>

namespace spatial_dyn {

Joint::Joint(Joint::Type type) {
  set_type(type);
}

Joint::Joint(const std::string& type) {
  set_type(ctrl_utils::FromString<Joint::Type>(type));
}

void Joint::set_type(Joint::Type type) {
  type_ = type;
  switch (type_) {
    case Joint::Type::kRx:
      subspace_ = SpatialMotiond::UnitAngX();
      break;
    case Joint::Type::kRy:
      subspace_ = SpatialMotiond::UnitAngY();
      break;
    case Joint::Type::kRz:
      subspace_ = SpatialMotiond::UnitAngZ();
      break;
    case Joint::Type::kPx:
      subspace_ = SpatialMotiond::UnitLinX();
      break;
    case Joint::Type::kPy:
      subspace_ = SpatialMotiond::UnitLinY();
      break;
    case Joint::Type::kPz:
      subspace_ = SpatialMotiond::UnitLinZ();
      break;
    default:
      subspace_ = SpatialMotiond::Zero();
  }
}

bool Joint::is_prismatic() const {
  return type_ == Joint::Type::kPx || type_ == Joint::Type::kPy || type_ == Joint::Type::kPz;
}
bool Joint::is_revolute() const {
  return type_ == Joint::Type::kRx || type_ == Joint::Type::kRy || type_ == Joint::Type::kRz;
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
    case Joint::Type::kRx:
      return Eigen::Isometry3d(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitX()));
    case Joint::Type::kRy:
      return Eigen::Isometry3d(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitY()));
    case Joint::Type::kRz:
      return Eigen::Isometry3d(Eigen::AngleAxisd(q, Eigen::Vector3d::UnitZ()));
    case Joint::Type::kPx:
      return Eigen::Isometry3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitX()));
    case Joint::Type::kPy:
      return Eigen::Isometry3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitY()));
    case Joint::Type::kPz:
      return Eigen::Isometry3d(Eigen::Translation3d(q * Eigen::Vector3d::UnitZ()));
    default:
      return Eigen::Isometry3d::Identity();
  }
}

static const std::map<std::string, Joint::Type> kStringToJointType = {
  {"rx", Joint::Type::kRx}, {"ry", Joint::Type::kRy}, {"rz", Joint::Type::kRz},
  {"px", Joint::Type::kPx}, {"py", Joint::Type::kPy}, {"pz", Joint::Type::kPz},
  {"undefined", Joint::Type::kUndefined}
};

static const std::map<Joint::Type, std::string> kJointTypeToString = {
  {Joint::Type::kRx, "rx"}, {Joint::Type::kRy, "ry"}, {Joint::Type::kRz, "rz"},
  {Joint::Type::kPx, "px"}, {Joint::Type::kPy, "py"}, {Joint::Type::kPz, "pz"},
  {Joint::Type::kUndefined, "undefined"}
};

std::ostream& operator<<(std::ostream& os, const Joint::Type& type) {
  os << kJointTypeToString.at(type);
  return os;
}

std::istream& operator>>(std::istream& is, Joint::Type& type) {
  std::string str;
  is >> str;
  std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return std::tolower(c); });
  type = kStringToJointType.at(str);
  return is;
}

std::ostream& operator<<(std::ostream& os, const Joint& j) {
  os << "Joint(type=" << j.type() << ", q_lim=[" << j.q_min() << " " << j.q_max() 
     << "], dq_max=" << j.dq_max() << ", fq_max=" << j.fq_max() << ", f_coulomb="
     << j.f_coulomb() << ", f_viscous=" << j.f_viscous() << ", f_stiction=" << j.f_stiction() << ")";
  return os;
}

}  // namespace spatial_dyn
