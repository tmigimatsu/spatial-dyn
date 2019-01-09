/**
 * options.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: January 05, 2019
 * Authors: Toki Migimatsu
 */

#include "structs/options.h"

namespace SpatialDyn {

InverseDynamicsOptions::InverseDynamicsOptions(const ForwardDynamicsOptions& other)
    : gravity(other.gravity), centrifugal_coriolis(other.centrifugal_coriolis),
      friction(other.friction), stiction_epsilon(other.stiction_epsilon) {}

InverseDynamicsOptions::InverseDynamicsOptions(const IntegrationOptions& other)
    : gravity(other.gravity), centrifugal_coriolis(other.centrifugal_coriolis),
      friction(other.friction), stiction_epsilon(other.stiction_epsilon) {}

ForwardDynamicsOptions::ForwardDynamicsOptions(const IntegrationOptions& other)
    : gravity(other.gravity), centrifugal_coriolis(other.centrifugal_coriolis),
      friction(other.friction), stiction_epsilon(other.stiction_epsilon) {}

}  // namespace SpatialDyn
