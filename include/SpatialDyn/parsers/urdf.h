/**
 * urdf.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 5, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_PARSERS_URDF_H_
#define SPATIAL_DYN_PARSERS_URDF_H_

#include "structs/articulated_body.h"

#include <string>  // std::string

namespace SpatialDyn {
namespace Urdf {

ArticulatedBody ParseModel(const std::string& urdf);

}  // namespace Urdf
}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_PARSERS_URDF_H_
