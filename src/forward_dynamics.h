/**
 * forward_dynamics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 28, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_FORWARD_DYNAMICS_H_
#define SPATIAL_DYN_FORWARD_DYNAMICS_H_

#include "articulated_body.h"
#include "spatial_math.h"

#include <vector>  // std::vector

namespace SpatialDyn {

// ABA
template<typename Derived>
Eigen::VectorXd ForwardDynamics(const ArticulatedBody& ab, const Eigen::MatrixBase<Derived>& tau) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)

  // Forward pass
  std::vector<SpatialMotiond> v; v.reserve(ab.dof());  // Propagate velocities
  std::vector<SpatialMotiond> c; c.reserve(ab.dof());
  std::vector<SpatialForced> p; p.reserve(ab.dof());
  std::vector<SpatialInertiaMatrixd> I; I.reserve(ab.dof());
  for (int i = 0; i < ab.dof(); i++) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    const int parent = ab.rigid_bodies(i).id_parent();
    if (parent < 0) {
      v.push_back(ab.dq(i) * s);
    } else {
      const auto T_from_parent = ab.T_to_parent(i).inverse();
      v.push_back(T_from_parent * v[parent] + ab.dq(i) * s);
    }
    I.push_back(ab.rigid_bodies(i).inertia());
    c.push_back(v[i].cross(ab.dq(i) * s));
    p.push_back(v[i].cross(I[i] * v[i]));
  }

  // Backward pass
  std::vector<SpatialForced> h(ab.dof());
  std::vector<double> d(ab.dof());
  std::vector<double> u(ab.dof());
  for (int i = ab.dof() - 1; i >= 0; i--) {
    const SpatialMotiond& s = ab.rigid_bodies(i).joint().subspace();
    h[i] = I[i] * s;
    d[i] = s.dot(h[i]);
    u[i] = tau(i) - s.dot(p[i]);

    const int parent = ab.rigid_bodies(i).id_parent();
    if (parent >= 0) {
      I[i] -= h[i].matrix() / d[i] * h[i].transpose();
      I[parent] += ab.T_to_parent(i) * I[i];
      p[parent] += ab.T_to_parent(i) * (p[i] + I[i] * c[i] + u[i] / d[i] * h[i]);
    }
  }

  std::vector<SpatialMotiond> a; a.reserve(ab.dof());
  Eigen::VectorXd ddq(ab.dof());
  for (int i = 0; i < ab.dof(); i++) {
    const auto T_from_parent = ab.T_to_parent(i).inverse();
    const int parent = ab.rigid_bodies(i).id_parent();
    if (parent < 0) {
      a.push_back(T_from_parent * -ab.g() + c[i]);
    } else {
      a.push_back(T_from_parent * a[parent] + c[i]);
    }
    ddq(i) = (u[i] - h[i].dot(a[i])) / d[i];
    a[i] += ddq(i) * ab.rigid_bodies(i).joint().subspace();
  }
  return ddq;
}

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_FORWARD_DYNAMICS_H_
