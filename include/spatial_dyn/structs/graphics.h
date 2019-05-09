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

#include <iostream>  // std::istream, std::ostream
#include <string>    // std::string

#include "spatial_dyn/eigen/spatial_math.h"

namespace spatial_dyn {

// TODO: Finish

/**
 * @ingroup cpp_structs
 *
 * Graphics struct for spatial_dyn.
 *
 * @see Python: spatialdyn.Graphics
 */
struct Graphics {
  /// @cond
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// @endcond

  /**
   * %Graphics geometry struct.
   *
   * @see Python: spatialdyn.Graphics.Geometry
   */
  struct Geometry {
    /// @cond
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// @endcond

    /**
     * %Geometry types.
     */
    enum class Type {
      kUndefined,  ///< Undefined.
      kBox,        ///< Box.
      kCapsule,    ///< Capsule.
      kCylinder,   ///< Cylinder.
      kSphere,     ///< Sphere.
      kMesh        ///< Mesh.
    };

    /**
     * Geometry type.
     */
    Type type = Type::kUndefined;

    /**
     * Type::kBox size or Type::kMesh scale.
     */
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();

    /**
     * Type::kCapsule or Type::kCylinder or Type::kSphere radius.
     */
    double radius = 0;

    /**
     * Type::kCapsule or Type::kCylinder length.
     */
    double length = 0;

    /**
     * Type::kMesh filename.
     */
    std::string mesh;
  };

  struct Material {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string name;
    Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
    std::string texture;  // Texture filename
  };

  Graphics(const std::string& name = "") : name(name) {}

  std::string name;
  Eigen::Isometry3d T_to_parent = Eigen::Isometry3d::Identity();
  Geometry geometry;
  Material material;
};

/**
 * Output the Geometry::Type to an output stream.
 *
 * The string representations of the type are "undefined", "box", "cylinder",
 * "sphere", and "mesh".
 */
std::ostream& operator<<(std::ostream& os, const Graphics::Geometry::Type& type);

/**
 * Parse one token from the input stream to a Geometry::Type.
 *
 * The string representations of the type are "undefined", "box", "cylinder",
 * "sphere", and "mesh".
 */
std::istream& operator>>(std::istream& is, Graphics::Geometry::Type& type);

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_STRUCTS_RIGID_BODY_H_
