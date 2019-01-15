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

#define EIGEN_TRANSFORM_PLUGIN "spatial_dyn/eigen/spatial_transform.h"
#define EIGEN_MATRIX_PLUGIN "spatial_dyn/eigen/matrix_plugin.h"
#include <ctrl_utils/eigen.h>
#include "spatial_motion.h"
#include "spatial_force.h"
#include "spatial_inertia.h"

#include <unsupported/Eigen/CXX11/Tensor>

namespace Eigen {

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,Dynamic> Matrix6Xd;

typedef Matrix<float,6,1> Vector6f;
typedef Matrix<float,6,6> Matrix6f;
typedef Matrix<float,6,Dynamic> Matrix6Xf;

typedef Tensor<double,1> Tensor1d;
typedef Tensor<double,2> Tensor2d;
typedef Tensor<double,3> Tensor3d;

typedef Tensor<float,1> Tensor1f;
typedef Tensor<float,2> Tensor2f;
typedef Tensor<float,3> Tensor3f;

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_DYN_H_
