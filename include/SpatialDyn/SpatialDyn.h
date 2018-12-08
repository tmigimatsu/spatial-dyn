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

// Core
#include "SpatialDyn/utils/spatial_math.h"
#include "SpatialDyn/structs/articulated_body.h"

// Algorithms
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
