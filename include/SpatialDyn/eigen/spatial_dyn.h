/**
 * spatial_dyn.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 16, 2018
 * Authors: Toki Migimatsu
 */

#ifndef EIGEN_SPATIAL_DYN_H_
#define EIGEN_SPATIAL_DYN_H_

namespace Eigen {

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
class SpatialMotion;

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
class SpatialForce;

template<typename Derived>
class SpatialMotionBase;

template<typename Derived>
class SpatialForceBase;

template<typename Scalar>
class SpatialInertia;

template<typename Scalar>
class SpatialInertiaMatrix;

}  // namespace Eigen

#define EIGEN_TRANSFORM_PLUGIN "SpatialDyn/eigen/spatial_transform.h"
#define EIGEN_MATRIX_PLUGIN "SpatialDyn/eigen/matrix_plugin.h"
#define EIGEN_MATRIXBASE_PLUGIN "SpatialDyn/eigen/matrix_base_plugin.h"
#include "spatial_motion.h"
#include "spatial_force.h"
#include "spatial_inertia.h"

#include <algorithm>  // std::max
#include <limits>     // std::numeric_limits

namespace Eigen {

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,Dynamic> Matrix6Xd;

typedef Matrix<float,6,1> Vector6f;
typedef Matrix<float,6,6> Matrix6f;
typedef Matrix<float,6,Dynamic> Matrix6Xf;

template<typename Derived>
inline typename MatrixBase<Derived>::PlainObject
PseudoInverse(const MatrixBase<Derived>& A, double svd_epsilon = 0) {
  unsigned int options = MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic ?
                         Eigen::ComputeThinU | Eigen::ComputeThinV :
                         Eigen::ComputeFullU | Eigen::ComputeFullV;
  Eigen::JacobiSVD<typename MatrixBase<Derived>::PlainObject> svd(A, options);
  const auto& S = svd.singularValues();
  if (svd_epsilon <= 0) {
    svd_epsilon = std::numeric_limits<typename internal::traits<Derived>::Scalar>::epsilon() *
                std::max(A.cols(), A.cols()) * S(0);
  }
  return svd.matrixV() *
         (S.array() > svd_epsilon).select(S.array().inverse(), 0).matrix().asDiagonal() *
         svd.matrixU().adjoint();
}

template<typename Scalar>
Matrix<Scalar,3,3> RotationX(Scalar angle) {
  Matrix<Scalar,3,3> R;
  Scalar ca = std::cos(angle);
  Scalar sa = std::sin(angle);
  R << 1, 0,   0,
       0, ca, -sa,
       0, sa,  ca;
  return R;
}

template<typename Scalar>
Matrix<Scalar,3,3> RotationY(Scalar angle) {
  Matrix<Scalar,3,3> R;
  Scalar ca = std::cos(angle);
  Scalar sa = std::sin(angle);
  R <<  ca, 0, sa,
        0,  1, 0,
       -sa, 0, ca;
  return R;
}

template<typename Scalar>
Matrix<Scalar,3,3> RotationZ(Scalar angle) {
  Matrix<Scalar,3,3> R;
  Scalar ca = std::cos(angle);
  Scalar sa = std::sin(angle);
  R << ca, -sa, 0,
       sa,  ca, 0,
       0,   0,  1;
  return R;
}

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_DYN_H_
