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

#include <cstddef>  // size_t

namespace spatial_dyn {

struct InverseDynamicsOptions;
struct ForwardDynamicsOptions;
struct IntegrationOptions;

namespace opspace {

struct InverseDynamicsOptions;

}  // namespace opspace

namespace discrete {

struct InverseDynamicsOptions;
struct IntegrationOptions;

}  // namespace discrete

/**
 * @ingroup cpp_inverse_dynamics
 * Options for spatial_dyn::InverseDynamics().
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
 * Options for spatial_dyn::ForwardDynamics().
 */
struct ForwardDynamicsOptions /*: InverseDynamicsOptions*/ {

  ForwardDynamicsOptions(bool gravity = true, bool centrifugal_coriolis = true,
                         bool friction = false, double stiction_epsilon = 0.01)
      : gravity(gravity), centrifugal_coriolis(centrifugal_coriolis),
        friction(friction), stiction_epsilon(stiction_epsilon) {}

  ForwardDynamicsOptions(const IntegrationOptions& other);

  ForwardDynamicsOptions(const discrete::IntegrationOptions& other);

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
    kEuler,  ///< Euler's method (order 1).
    kHeuns,  ///< Heun's method (order 2).
    kRk4     ///< Runge-Kutta 4 (order 4).
  };

  IntegrationOptions(bool gravity = true, bool centrifugal_coriolis = true,
                     bool friction = false, bool joint_limits = false,
                     Method method = Method::kRk4, bool aba = false,
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
  Method method = Method::kRk4;

  /**
   * Use spatial_dyn::ForwardDynamicsAba(). If false, uses spatial_dyn::ForwardDynamics().
   */
  bool aba = false;

  /**
   * Velocity threshold for stiction activation.
   */
  double stiction_epsilon = 0.01;

};

namespace opspace {

struct InverseDynamicsOptions {

  InverseDynamicsOptions(bool gravity = false, bool centrifugal_coriolis = false,
                         bool friction = false, double svd_epsilon = 0.,
                         double stiction_epsilon = 0.01)
      : gravity(gravity), centrifugal_coriolis(centrifugal_coriolis),
        friction(friction), svd_epsilon(svd_epsilon),
        stiction_epsilon(stiction_epsilon) {}

  /**
   * Include gravity torques.
   */
  bool gravity = false;

  /**
   * Include centrifugal/Coriolis torques.
   */
  bool centrifugal_coriolis = false;

  /**
   * Include Coulomb and viscous friction torques at the joints.
   */
  bool friction = false;

  /**
   * Threshold for singular value inversion in the computation of Lambda.
   */
  double svd_epsilon = 0.;

  /**
   * Velocity threshold for stiction activation.
   */
  double stiction_epsilon = 0.01;

  /**
   * Maximum force due to acceleration (F = Lambda * ddx). Forces above the
   * limit will scaled. If f_acc_max is 0, no clipping will occur.
   */
  double f_acc_max = 0.;

};

}  // namespace opspace

namespace discrete {

struct InverseDynamicsOptions {

  InverseDynamicsOptions(bool gravity = true)
      : gravity(gravity) {}

  InverseDynamicsOptions(const discrete::IntegrationOptions& other);

  /**
   * Include gravity torques.
   */
  bool gravity = true;

};

struct IntegrationOptions {

  IntegrationOptions(bool gravity = true)
      : gravity(gravity) {}

  /**
   * Include gravity torques.
   */
  bool gravity = true;

  /**
   * Convergence threshold for finding the roots in the variational integrator.
   */
  double variational_epsilon = 1e-8;

  /**
   * Maximum number of iterations for root finding in the variational integrator.
   */
  size_t max_iterations = 100;

};

}  // namespace discrete

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_STRUCTS_OPTIONS_H_
