/**
 * inverse_dynamics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 23, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_INVERSE_DYNAMICS_H_
#define SPATIAL_DYN_INVERSE_DYNAMICS_H_

#include "articulated_body.h"
#include "spatial_math.h"

#include <vector>  // std::vector

namespace SpatialDyn {

// RNEA
template<typename Derived>
Eigen::VectorXd InverseDynamics(const ArticulatedBody& ab, const Eigen::MatrixBase<Derived>& ddq) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)

  // Forward pass
  std::vector<SpatialMotiond> v(ab.dof());  // Propagate velocities
  std::vector<SpatialMotiond> a(ab.dof());  // Propagate accelerations
  std::vector<SpatialForced> f(ab.dof());   // Propagate forces
  for (int i = 0; i < ab.dof(); i++) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();
    const auto T_from_parent = ab.T_to_parent(i).inverse();
    if (parent < 0) {
      v[i] = ab.dq(i) * s;
      a[i] = T_from_parent * -ab.g() + ddq(i) * s;
    } else {
      v[i] = T_from_parent * v[parent] + ab.dq(i) * s;
      a[i] = T_from_parent * a[parent] + ddq(i) * s + v[i].cross(ab.dq(i) * s);
    }
    const SpatialInertiad& I = ab.rigid_bodies(i).inertia();
    f[i] = I * a[i] + v[i].cross(I * v[i]);
  }

  // Backward pass
  Eigen::VectorXd tau(ab.dof());           // Resulting joint torques
  for (int i = ab.dof() - 1; i >= 0; i--) {
    tau(i) = ab.rigid_bodies(i).joint().subspace().dot(f[i]);

    const int parent = ab.rigid_bodies(i).id_parent();
    if (parent >= 0) {
      f[parent] += ab.T_to_parent(i) * f[i];
    }
  }
  return tau;
}

// CRBA
Eigen::MatrixXd Inertia(const ArticulatedBody& ab) {
  Eigen::MatrixXd A(ab.dof(), ab.dof());

  // Initialize composite rigid body inertia
  std::vector<SpatialInertiad> I_composite;
  I_composite.reserve(ab.dof());
  for (int i = 0; i < ab.dof(); i++) {
    I_composite.push_back(ab.rigid_bodies(i).inertia());
  }

  // Backward pass
  for (int i = ab.dof() - 1; i >= 0; i--) {
    // Add inertia to parent
    const int parent = ab.rigid_bodies(i).id_parent();
    if (parent >= 0) {
      I_composite[parent] += ab.T_to_parent(i) * I_composite[i];
    }

    // Compute kinetic energy
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    SpatialForced f = I_composite[i] * s;  // Force that produces acceleration s
    A(i,i) = f.dot(s);  // Component of f along joint i: A_ii = s_i^T I_c s_i
    int child = i;
    for (int j = parent; j >= 0; j = ab.rigid_bodies(j).id_parent()) {
      f = ab.T_to_parent(child) * f;  // Force in frame j
      A(i,j) = f.dot(ab.rigid_bodies(j).joint().subspace());  // Component of f along joint j
      A(j,i) = A(i,j);  // A_ji = A_ij
      child = j;
    }
  }
  return A;
}


}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_INVERSE_DYNAMICS_H_
