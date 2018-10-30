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

namespace SpatialDyn {

Joint::Joint(Joint::Type type) {
  set_type(type);
}

Joint::Type Joint::type() const {
  return type_;
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

const SpatialMotiond& Joint::subspace() const {
  return subspace_;
}

double Joint::q_min() const {
  return q_min_;
}
void Joint::set_q_min(double q_min) {
  if (q_min > q_max_) {
    throw std::invalid_argument("Joint::set_q_min(): q_min (" + std::to_string(q_min) +
                                ") cannot be greater than q_max (" + std::to_string(q_max_) + ").");
  }
  q_min_ = q_min;
}

double Joint::q_max() const {
  return q_max_;
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

double Joint::dq_max() const {
  return dq_max_;
}
void Joint::set_dq_max(double dq_max) {
  if (dq_max < 0.) {
    throw std::invalid_argument("Joint::set_dq_max(): dq_max (" + std::to_string(dq_max) +
                                ") cannot be negative.");
  }
  dq_max_ = dq_max;
}

double Joint::fq_max() const {
  return fq_max_;
}
void Joint::set_fq_max(double fq_max) {
  if (fq_max < 0.) {
    throw std::invalid_argument("Joint::set_fq_max(): fq_max (" + std::to_string(fq_max) +
                                ") cannot be negative.");
  }
  fq_max_ = fq_max;
}

double Joint::f_coulomb() const {
  return f_coulomb_;
}
void Joint::set_f_coulomb(double f_coulomb) {
  if (f_coulomb < 0.) {
    throw std::invalid_argument("Joint::set_f_coulomb(): f_coulomb (" + std::to_string(f_coulomb) +
                                ") cannot be negative.");
  }
  f_coulomb_ = f_coulomb;
}

double Joint::f_viscous() const {
  return f_viscous_;
}
void Joint::set_f_viscous(double f_viscous) {
  if (f_viscous < 0.) {
    throw std::invalid_argument("Joint::set_f_viscous(): f_viscous (" + std::to_string(f_viscous) +
                                ") cannot be negative.");
  }
  f_viscous_ = f_viscous;
}

double Joint::f_stiction() const {
  return f_stiction_;
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
      return Eigen::Isometry3d(Eigen::RotationX(q));
    case Joint::Type::RY:
      return Eigen::Isometry3d(Eigen::RotationY(q));
    case Joint::Type::RZ:
      return Eigen::Isometry3d(Eigen::RotationZ(q));
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

Joint::operator std::string() const {
  switch (type_) {
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

static const std::map<std::string, Joint::Type> kStringToJointType = {
  {"RX", Joint::Type::RX}, {"RY", Joint::Type::RY}, {"RZ", Joint::Type::RZ},
  {"PX", Joint::Type::PX}, {"PY", Joint::Type::PY}, {"PZ", Joint::Type::PZ},
  {"UNDEFINED", Joint::Type::UNDEFINED}
};

Joint::Type Joint::FromString(const std::string& type) {
  try {
    return kStringToJointType.at(type);
  } catch (...) {
    return Joint::Type::UNDEFINED;
  }
}


}  // namespace SpatialDyn
