/**
 * spatial_math.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 6, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_SPATIAL_MATH_H_
#define SPATIAL_DYN_SPATIAL_MATH_H_

#include "Eigen/SpatialDyn.h"

namespace SpatialDyn {

typedef Eigen::SpatialMotion<float,1>  SpatialMotionf;
typedef Eigen::SpatialMotion<double,1> SpatialMotiond;
typedef Eigen::SpatialMotion<float,6>  SpatialMotion6f;
typedef Eigen::SpatialMotion<double,6> SpatialMotion6d;
typedef Eigen::SpatialMotion<float,Eigen::Dynamic>  SpatialMotionXf;
typedef Eigen::SpatialMotion<double,Eigen::Dynamic> SpatialMotionXd;

typedef Eigen::SpatialForce<float,1>  SpatialForcef;
typedef Eigen::SpatialForce<double,1> SpatialForced;
typedef Eigen::SpatialForce<float,6>  SpatialForce6f;
typedef Eigen::SpatialForce<double,6> SpatialForce6d;
typedef Eigen::SpatialForce<float,Eigen::Dynamic>  SpatialForceXf;
typedef Eigen::SpatialForce<double,Eigen::Dynamic> SpatialForceXd;

typedef Eigen::SpatialInertia<float> SpatialInertiaf;
typedef Eigen::SpatialInertia<double> SpatialInertiad;
typedef Eigen::SpatialInertiaMatrix<float> SpatialInertiaMatrixf;
typedef Eigen::SpatialInertiaMatrix<double> SpatialInertiaMatrixd;

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_SPATIAL_MATH_H_
