/**
 * spatial_dyn.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 22, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_SPATIAL_DYN_H_
#define SPATIAL_DYN_SPATIAL_DYN_H_

/**
 * @defgroup cpp C++
 *
 * C++ implementation of spatial_dyn.
 *
 * @see Python: \ref py
 */

#include "spatial_dyn/eigen/spatial_math.h"

/**
 * @defgroup cpp_structs Structs
 * @ingroup cpp
 *
 * C++ implementation of spatial_dyn structs.
 *
 * @see Python: \ref py_structs
 */
#include "spatial_dyn/structs/articulated_body.h"

/**
 * @defgroup cpp_algorithms Algorithms
 * @ingroup cpp
 *
 * C++ implementation of spatial_dyn algorithms.
 *
 * @see Python: \ref py_algorithms
 */
#include "spatial_dyn/algorithms/discrete_dynamics.h"
#include "spatial_dyn/algorithms/forward_dynamics.h"
#include "spatial_dyn/algorithms/forward_kinematics.h"
#include "spatial_dyn/algorithms/inverse_dynamics.h"
#include "spatial_dyn/algorithms/opspace_dynamics.h"
#include "spatial_dyn/algorithms/simulation.h"

// Parsers
#include "spatial_dyn/parsers/json.h"
#include "spatial_dyn/parsers/urdf.h"
#include "spatial_dyn/parsers/yaml.h"

#endif  // SPATIAL_DYN_SPATIAL_DYN_H_
