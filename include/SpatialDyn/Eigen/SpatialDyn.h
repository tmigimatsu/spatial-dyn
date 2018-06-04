/**
 * SpatialDyn.h
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

#define EIGEN_TRANSFORM_PLUGIN "Eigen/SpatialTransform.h"
#define EIGEN_MATRIX_PLUGIN "Eigen/MatrixPlugin.h"
#define EIGEN_MATRIXBASE_PLUGIN "Eigen/MatrixBasePlugin.h"
#include "SpatialMotion.h"
#include "SpatialForce.h"
#include "SpatialInertia.h"

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
  Eigen::JacobiSVD<typename MatrixBase<Derived>::PlainObject>
      svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& S = svd.singularValues();
  if (svd_epsilon <= 0) {
    svd_epsilon = std::numeric_limits<typename internal::traits<Derived>::Scalar>::epsilon() *
                std::max(A.cols(), A.cols()) * S(0);
  }
  return svd.matrixV() *
         (S.array() > svd_epsilon).select(S.array().inverse(), 0).matrix().asDiagonal() *
         svd.matrixU().adjoint();
}

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_DYN_H_
