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

#include "SpatialMotion.h"
#include "SpatialForce.h"

namespace Eigen {

template<typename Derived>
Matrix<typename Derived::Scalar,3,3> crossMatrix(const MatrixBase<Derived> &x) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)
  Matrix<typename Derived::Scalar,3,3> result;
  result <<  0,    -x(2),  x(1),
             x(2),  0,    -x(0),
            -x(1),  x(0),  0;
  return result;
}

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,Dynamic> Matrix6Xd;

typedef Matrix<float,6,1> Vector6f;
typedef Matrix<float,6,6> Matrix6f;
typedef Matrix<float,6,Dynamic> Matrix6Xf;

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_DYN_H_
