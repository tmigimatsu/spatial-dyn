/**
 * urdf.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 5, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIALDYN_UTIL_URDF_H_
#define SPATIALDYN_UTIL_URDF_H_

#include "articulated_body.h"

#include <string>  // std::string

namespace SpatialDyn {
namespace Urdf {

ArticulatedBody ParseModel(const std::string& urdf);

}  // namespace Urdf
}  // namespace SpatialDyn

#endif  // SPATIALDYN_UTIL_URDF_H_
