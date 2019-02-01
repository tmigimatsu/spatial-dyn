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

#include <limits>   // std::numeric_limits
#include <ostream>  // std::ostream
#include <string>   // std::string

#include "spatial_dyn/eigen/spatial_math.h"

namespace spatial_dyn {

/**
 * @ingroup cpp_structs
 *
 * Joint struct for spatial_dyn.
 *
 * @see Python: spatialdyn.Joint
 */
class Joint {

 public:
  /// @cond
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// @endcond

  /**
   * %Joint types.
   */
  enum class Type {
    kUndefined,  ///< Undefined.
    kRx,         ///< Revolute about X.
    kRy,         ///< Revolute about Y.
    kRz,         ///< Revolute about Z.
    kPx,         ///< Prismatic along X.
    kPy,         ///< Prismatic along Y.
    kPz          ///< Prismatic along Z.
  };

  /**
   * Default constructor. Sets joint type to Type::UNDEFINED.
   */
  Joint() {}

  /**
   * Constructor that sets the joint type.
   *
   * @param type %Joint type.
   */
  Joint(Type type);

  /**
   * Constructor that sets the joint type from a string.
   *
   * Valid values: `"undefined"`, `"rx"`, `"ry"`, `"rz"`, `"px"`, `"py"`, `"pz"`.
   *
   * @param type %Joint type string.
   * @see Python: spatialdyn.Joint.__init__()
   */
  Joint(const std::string& type);

  /**
   * @return %Joint type.
   * @see Python: spatialdyn.Joint.type
   */
  Type type() const { return type_; }

  /**
   * Set the joint type.
   *
   * @param type Joint type.
   */
  void set_type(Type type);

  /**
   * @return Boolean indicating whether the joint is prismatic.
   * @see Python: spatialdyn.Joint.is_prismatic
   */
  bool is_prismatic() const;

  /**
   * @return Boolean indicating whether the joint is revolute.
   * @see Python: spatialdyn.Joint.is_revolute
   */
  bool is_revolute() const;

  /**
   * @return Motion subspace spatial vector, determined by the joint type.
   * @see Python: spatialdyn.Joint.subspace
   */
  const SpatialMotiond& subspace() const { return subspace_; }

  /**
   * @return Lower joint limit. Defaults to negative infinity.
   * @see Python: spatialdyn.Joint.q_min
   */
  double q_min() const { return q_min_; }

  /**
   * Set the lower joint limit. Throws an error if `q_min > q_max`.
   *
   * @param q_min Lower joint limit.
   */
  void set_q_min(double q_min);

  /**
   * @return Upper joint limit. Defaults to infinity.
   * @see Python: spatialdyn.Joint.q_max
   */
  double q_max() const { return q_max_; }

  /**
   * Set the upper joint limit. Throws an error if `q_max > q_min`.
   *
   * @param q_max Upper joint limit.
   */
  void set_q_max(double q_max);

  /**
   * Set the lower and upper joint limits. Throws an error if `q_min > q_max`.
   *
   * @param q_min Lower joint limit.
   * @param q_max Upper joint limit.
   */
  void set_q_limits(double q_min, double q_max);

  /**
   * @return Maximum joint velocity. Defaults to infinity.
   * @see Python: spatialdyn.Joint.dq_max
   */
  double dq_max() const { return dq_max_; }

  /**
   * Set the maximum joint velocity. Throws an error if `dq_max < 0`.
   *
   * @param dq_max Maximum joint velocity
   */
  void set_dq_max(double dq_max);

  /**
   * @return Maximum joint torque. Defaults to infinity.
   * @see Python: spatialdyn.Joint.fq_max
   */
  double fq_max() const { return fq_max_; }

  /**
   * Set the maximum joint torque. Throws an error if `fq_max < 0`.
   *
   * @param fq_max Maximum joint torque.
   */
  void set_fq_max(double fq_max);

  /**
   * @return Coulomb friction coefficient used as `f_coulomb * signum(dq)`.
   *         Defaults to `0`.
   * @see Python: spatialdyn.Joint.f_coulomb
   */
  double f_coulomb() const { return f_coulomb_; }

  /**
   * Set the Coulomb friction coefficient. Throws an error if `f_coulomb < 0`.
   *
   * @param f_coulomb Coulomb friction coefficient.
   */
  void set_f_coulomb(double f_coulomb);

  /**
   * @return Viscous friction coefficient used as `f_viscous * dq`. Defaults to `0`.
   * @see Python: spatialdyn.Joint.f_viscous
   */
  double f_viscous() const { return f_viscous_; }

  /**
   * Set the viscous friction coefficient. Throws an error if `f_viscous < 0`.
   *
   * @param f_viscous Viscous friction coefficient.
   */
  void set_f_viscous(double f_viscous);

  /**
   * @return Stiction coefficient. Defaults to `0`.
   * @see Python: spatialdyn.Joint.f_stiction
   */
  double f_stiction() const { return f_stiction_; }

  /**
   * Set the stiction coefficient. Throws an error if `f_stiction < 0`.
   *
   * @param f_stiction Stiction coefficient.
   */
  void set_f_stiction(double f_stiction);

  /**
   * Compute the transform from the joint frame to its parent rigid body's frame.
   *
   * @param q %Joint position.
   * @return Transform given q.
   * @see Python: spatialdyn.Joint.T_joint()
   */
  Eigen::Isometry3d T_joint(double q) const;

 protected:

  /// @cond
  Type type_ = Type::kUndefined;
  SpatialMotiond subspace_ = SpatialMotiond::Zero();

  double q_min_  = -std::numeric_limits<double>::infinity();
  double q_max_  = std::numeric_limits<double>::infinity();
  double dq_max_ = std::numeric_limits<double>::infinity();
  double fq_max_ = std::numeric_limits<double>::infinity();
  double f_coulomb_  = 0.;
  double f_viscous_  = 0.;
  double f_stiction_ = 0.;
  /// @endcond

};

/**
 * @ingroup cpp_structs
 * Output the Joint::Type to an output stream.
 *
 * The string representations of the type are "undefined", "rx", "ry", "rz",
 * "px", "py", and "pz".
 */
std::ostream& operator<<(std::ostream& os, const Joint::Type& type);

/**
 * @ingroup cpp_structs
 * Parse one token from the input stream to a Joint::Type.
 *
 * The string representations of the type are "undefined", "rx", "ry", "rz",
 * "px", "py", and "pz".
 */
std::istream& operator>>(std::istream& is, Joint::Type& type);

/**
 * @ingroup cpp_structs
 * @return Stream representation of the joint for debugging.
 * @see Python: spatialdyn.Joint.__repr__()
 */
std::ostream& operator<<(std::ostream& os, const Joint& j);

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_STRUCTS_JOINT_H_
