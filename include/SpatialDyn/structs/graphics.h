/**
 * graphics.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 6, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_STRUCTS_GRAPHICS_H_
#define SPATIAL_DYN_STRUCTS_GRAPHICS_H_

#include <Eigen/Eigen>

#include <string>  // std::string

namespace SpatialDyn {

enum class GeometryType { UNDEFINED, BOX, CYLINDER, SPHERE, MESH };

struct Geometry {
  GeometryType type = GeometryType::UNDEFINED;
  Eigen::Vector3d scale = Eigen::Vector3d::Ones();  // Box size/mesh scale
  double radius = 0                      ;          // Cylinder/sphere radius
  double length = 0;                                // Cylinder length
  std::string mesh;                                 // Mesh filename

  operator std::string() const {
    switch (type) {
      case GeometryType::BOX:      return "BOX";
      case GeometryType::CYLINDER: return "CYLINDER";
      case GeometryType::SPHERE:   return "SPHERE";
      case GeometryType::MESH:     return "MESH";
      default:                     return "UNDEFINED";
    }
  }
};

struct Material {
  std::string name;
  Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
  std::string texture;  // Texture filename
};

struct Graphics {
  std::string name;
  Eigen::Isometry3d T_to_parent = Eigen::Isometry3d::Identity();
  Geometry geometry;
  Material material;
};

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_STRUCTS_RIGID_BODY_H_
