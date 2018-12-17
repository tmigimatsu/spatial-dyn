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

#include <string>   // std::string

#include "SpatialDyn/utils/spatial_math.h"

namespace SpatialDyn {

/**
 * Graphics struct for SpatialDyn.
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
      UNDEFINED,  ///< Undefined.
      BOX,        ///< Box.
      CYLINDER,   ///< Cylinder.
      SPHERE,     ///< Sphere.
      MESH        ///< Mesh.
    };

    /**
     * Geometry type.
     */
    Type type = Type::UNDEFINED;

    /**
     * Type::BOX size or Type::MESH scale.
     */
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();

    /**
     * Type::CYLINDER or Type::SPHERE radius.
     */
    double radius = 0;

    /**
     * Type::CYLINDER length.
     */
    double length = 0;

    /**
     * Type::MESH filename.
     */
    std::string mesh;

    /**
     * Convert the Geometry::Type to a string.
     *
     * @param type %Geometry type enum.
     * @return %Geometry type string.
     */
    static std::string TypeToString(const Type& type);

    /**
     * Convert the string to a Geometry::Type.
     *
     * @param type %Geometry type string.
     * @return %Geometry type enum.
     */
    static Type StringToType(const std::string& type);

    /**
     * Convert the joint type enum to a string.
     *
     * Example:
     * ```
     * std::string(Graphics::Geometry(Graphics::Geometry::Type::MESH)) -> "MESH"
     * ```
     *
     * @return %Geometry type as a string.
     */
    operator std::string() const { return TypeToString(type); }
  };

  struct Material {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string name;
    Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
    std::string texture;  // Texture filename
  };

  std::string name;
  Eigen::Isometry3d T_to_parent = Eigen::Isometry3d::Identity();
  Geometry geometry;
  Material material;
};

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_STRUCTS_RIGID_BODY_H_
