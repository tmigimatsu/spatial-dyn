/**
 * control.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 07, 2019
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_ALGORITHMS_CONTROL_H_
#define SPATIAL_DYN_ALGORITHMS_CONTROL_H_

#include "spatial_dyn/eigen/spatial_math.h"
#include "spatial_dyn/algorithms/opspace_kinematics.h"

namespace spatial_dyn {

// General PD control law
template<typename Derived1, typename Derived2, typename Derived3>
inline typename Derived1::PlainObject PdControl(const Eigen::MatrixBase<Derived1>& x,
                                                const Eigen::MatrixBase<Derived2>& x_des,
                                                const Eigen::MatrixBase<Derived3>& dx,
                                                const Eigen::Vector2d& kp_kv,
                                                double dx_max = 0.,
                                                typename Derived1::PlainObject* p_x_err = nullptr) {

  // Output position error
  typename Derived1::PlainObject x_err;
  x_err = x - x_des;
  if (p_x_err != nullptr) {
    *p_x_err = x_err;
  }

  // With velocity clipping
  if (dx_max > 0.) {
    typename Derived1::PlainObject dx_des = -(kp_kv(0) / kp_kv(1)) * x_err;
    if (dx_des.norm() > 0.01) {
      typename Derived1::Scalar v = dx_max / dx_des.norm();
      if (v > 1.) v = 1.;
      return -kp_kv(1) * (dx - v * dx_des);
    }
  }

  // No velocity clipping
  return -kp_kv(0) * x_err - kp_kv(1) * dx;
}

// Special PD control law for orientation
inline Eigen::Vector3d PdControl(const Eigen::Quaterniond& quat,
                                 const Eigen::Quaterniond& quat_des,
                                 Eigen::Ref<const Eigen::Vector3d> w,
                                 Eigen::Ref<const Eigen::Vector2d> kp_kv,
                                 Eigen::Vector3d* p_ori_err = nullptr) {
  Eigen::Vector3d ori_err = spatial_dyn::opspace::OrientationError(quat, quat_des);
  if (p_ori_err != nullptr) {
    *p_ori_err = ori_err;
  }
  return -kp_kv(0) * ori_err - kp_kv(1) * w;
}

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_ALGORITHMS_CONTROL_H_
