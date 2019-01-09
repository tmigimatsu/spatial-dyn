/**
 * options.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: January 05, 2019
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_STRUCTS_OPTIONS_H_
#define SPATIAL_DYN_STRUCTS_OPTIONS_H_

namespace SpatialDyn {

struct InverseDynamicsOptions;
struct ForwardDynamicsOptions;
struct IntegrationOptions;

/**
 * @ingroup cpp_inverse_dynamics
 * Options for SpatialDyn::InverseDynamics().
 */
struct InverseDynamicsOptions {

  InverseDynamicsOptions(bool gravity = true, bool centrifugal_coriolis = false,
                         bool friction = false, double stiction_epsilon = 0.01)
      : gravity(gravity), centrifugal_coriolis(centrifugal_coriolis),
        friction(friction), stiction_epsilon(stiction_epsilon) {}

  InverseDynamicsOptions(const ForwardDynamicsOptions& other);

  InverseDynamicsOptions(const IntegrationOptions& other);

  /**
   * Include gravity torques.
   */
  bool gravity = true;

  /**
   * Include centrifugal/Coriolis torques.
   */
  bool centrifugal_coriolis = false;

  /**
   * Include Coulomb and viscous friction torques at the joints.
   */
  bool friction = false;

  /**
   * Velocity threshold for stiction activation.
   */
  double stiction_epsilon = 0.01;

};

/**
 * @ingroup cpp_forward_dynamics
 * Options for SpatialDyn::ForwardDynamics().
 */
struct ForwardDynamicsOptions /*: InverseDynamicsOptions*/ {

  ForwardDynamicsOptions(bool gravity = true, bool centrifugal_coriolis = true,
                         bool friction = false, double stiction_epsilon = 0.01)
      : gravity(gravity), centrifugal_coriolis(centrifugal_coriolis),
        friction(friction), stiction_epsilon(stiction_epsilon) {}

  ForwardDynamicsOptions(const IntegrationOptions& other);

  /**
   * Include gravity torques.
   */
  bool gravity = true;

  /**
   * Include centrifugal/Coriolis torques.
   */
  bool centrifugal_coriolis = true;

  /**
   * Include Coulomb and viscous friction torques at the joints.
   */
  bool friction = false;

  /**
   * Velocity threshold for stiction activation.
   */
  double stiction_epsilon = 0.01;

};

struct IntegrationOptions /*: ForwardDynamicsOptions*/ {

  /**
   * Integration methods.
   */
  enum class Method {
    EULER,  ///< Euler's method (order 1).
    HEUNS,  ///< Heun's method (order 2).
    RK4     ///< Runge-Kutta 4 (order 4).
  };

  IntegrationOptions(bool gravity = true, bool centrifugal_coriolis = true,
                     bool friction = false, bool joint_limits = false,
                     Method method = Method::RK4, bool aba = false,
                     double stiction_epsilon = 0.01)
      : gravity(gravity), centrifugal_coriolis(centrifugal_coriolis),
        friction(friction), joint_limits(joint_limits), method(method),
        aba(aba), stiction_epsilon(stiction_epsilon) {}

  /**
   * Include gravity torques.
   */
  bool gravity = true;

  /**
   * Include centrifugal/Coriolis torques.
   */
  bool centrifugal_coriolis = true;

  /**
   * Include Coulomb and viscous friction torques at the joints.
   */
  bool friction = false;

  /**
   * Enforce joint limits.
   */
  bool joint_limits = false;

  /**
   * Integration method.
   */
  Method method = Method::RK4;

  /**
   * Use SpatialDyn::ForwardDynamicsAba(). If false, uses SpatialDyn::ForwardDynamics().
   */
  bool aba = false;

  /**
   * Velocity threshold for stiction activation.
   */
  double stiction_epsilon = 0.01;

};

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_STRUCTS_OPTIONS_H_