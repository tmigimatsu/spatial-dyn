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

/**
 * Load articulated body from a urdf file.
 *
 * @param path_urdf Path of the urdf file.
 * @param path_meshes Path of meshes included in the urdf.
 * @param simplify Merge fixed joints.
 * @return Parsed articulated body.
 * @see Python: spatialdyn.urdf.LoadModel()
 */
ArticulatedBody LoadModel(const std::string& path_urdf,
                          const std::string& path_meshes = "",
                          bool simplify = true);

}  // namespace urdf
}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_PARSERS_URDF_H_
