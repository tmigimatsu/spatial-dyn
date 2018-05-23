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

}  // namespace Eigen

#define EIGEN_TRANSFORM_PLUGIN "Eigen/SpatialTransform.h"
#define EIGEN_MATRIX_PLUGIN "Eigen/MatrixPlugin.h"
#include "SpatialMotion.h"
#include "SpatialForce.h"
#include "SpatialInertia.h"

namespace Eigen {

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,Dynamic> Matrix6Xd;

typedef Matrix<float,6,1> Vector6f;
typedef Matrix<float,6,6> Matrix6f;
typedef Matrix<float,6,Dynamic> Matrix6Xf;

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_DYN_H_
