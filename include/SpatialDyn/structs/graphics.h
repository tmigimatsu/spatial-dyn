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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
  static GeometryType FromString(const std::string& type) {
    static const std::map<std::string, GeometryType> kStringToGeometryType = {
      {"BOX", GeometryType::BOX}, {"CYLINDER", GeometryType::CYLINDER},
      {"SPHERE", GeometryType::SPHERE}, {"MESH", GeometryType::MESH},
      {"UNDEFINED", GeometryType::UNDEFINED}
    };
    try {
      return kStringToGeometryType.at(type);
    } catch (...) {
      return GeometryType::UNDEFINED;
    }
  }
};

struct Material {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string name;
  Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
  std::string texture;  // Texture filename
};

struct Graphics {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string name;
  Eigen::Isometry3d T_to_parent = Eigen::Isometry3d::Identity();
  Geometry geometry;
  Material material;
};

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_STRUCTS_RIGID_BODY_H_
