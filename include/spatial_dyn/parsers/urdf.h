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

#include <string>  // std::string

#include "spatial_dyn/structs/articulated_body.h"

namespace spatial_dyn {
namespace urdf {

ArticulatedBody LoadModel(const std::string& urdf, bool expand_paths = false);

}  // namespace urdf
}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_PARSERS_URDF_H_
