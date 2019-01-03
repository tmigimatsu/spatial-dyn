/**
 * SpatialDyn.h
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
 * C++ implementation of SpatialDyn.
 *
 * @see Python: \ref py
 */


#include "SpatialDyn/utils/spatial_math.h"

/**
 * @defgroup cpp_structs Structs
 * @ingroup cpp
 *
 * C++ implementation of SpatialDyn structs.
 *
 * @see Python: \ref py_structs
 */
#include "SpatialDyn/structs/articulated_body.h"

/**
 * @defgroup cpp_algorithms Algorithms
 * @ingroup cpp
 *
 * C++ implementation of SpatialDyn algorithms.
 *
 * @see Python: \ref py_algorithms
 */
#include "SpatialDyn/algorithms/forward_dynamics.h"
#include "SpatialDyn/algorithms/forward_kinematics.h"
#include "SpatialDyn/algorithms/inverse_dynamics.h"
#include "SpatialDyn/algorithms/opspace_dynamics.h"
#include "SpatialDyn/algorithms/opspace_kinematics.h"
#include "SpatialDyn/algorithms/simulation.h"

// Parsers
#include "SpatialDyn/parsers/json.h"
#include "SpatialDyn/parsers/urdf.h"

// Utils
#include "SpatialDyn/utils/math.h"
#include "SpatialDyn/utils/redis_client.h"
#include "SpatialDyn/utils/timer.h"

#endif  // SPATIAL_DYN_SPATIAL_DYN_H_
