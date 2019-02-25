/**
 * spatial_math.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 16, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_EIGEN_SPATIAL_MATH_H_
#define SPATIAL_DYN_EIGEN_SPATIAL_MATH_H_

#include <ctrl_utils/eigen.h>

#include "spatial_motion.h"
#include "spatial_force.h"
#include "spatial_inertia.h"

namespace spatial_dyn {

typedef SpatialMotion<float,1>  SpatialMotionf;
typedef SpatialMotion<double,1> SpatialMotiond;
typedef SpatialMotion<float,6>  SpatialMotion6f;
typedef SpatialMotion<double,6> SpatialMotion6d;
typedef SpatialMotion<float,Eigen::Dynamic>  SpatialMotionXf;
typedef SpatialMotion<double,Eigen::Dynamic> SpatialMotionXd;

typedef SpatialForce<float,1>  SpatialForcef;
typedef SpatialForce<double,1> SpatialForced;
typedef SpatialForce<float,6>  SpatialForce6f;
typedef SpatialForce<double,6> SpatialForce6d;
typedef SpatialForce<float,Eigen::Dynamic>  SpatialForceXf;
typedef SpatialForce<double,Eigen::Dynamic> SpatialForceXd;

typedef SpatialInertia<float> SpatialInertiaf;
typedef SpatialInertia<double> SpatialInertiad;
typedef SpatialInertiaMatrix<float> SpatialInertiaMatrixf;
typedef SpatialInertiaMatrix<double> SpatialInertiaMatrixd;

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_EIGEN_SPATIAL_MATH_H_
