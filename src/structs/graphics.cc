/**
 * graphics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 6, 2018
 * Authors: Toki Migimatsu
 */

#include "structs/graphics.h"

#include <string>   // std::string
#include <ostream>  // std::ostream

namespace SpatialDyn {

Graphics::Geometry::operator std::string() const {
  switch (type) {
    case Type::BOX:      return "BOX";
    case Type::CYLINDER: return "CYLINDER";
    case Type::SPHERE:   return "SPHERE";
    case Type::MESH:     return "MESH";
    default:             return "UNDEFINED";
  }
}

static const std::map<std::string, Graphics::Geometry::Type> kStringToType = {
  {"BOX",       Graphics::Geometry::Type::BOX},
  {"CYLINDER",  Graphics::Geometry::Type::CYLINDER},
  {"SPHERE",    Graphics::Geometry::Type::SPHERE},
  {"MESH",      Graphics::Geometry::Type::MESH},
  {"UNDEFINED", Graphics::Geometry::Type::UNDEFINED}
};
Graphics::Geometry::Type Graphics::Geometry::FromString(const std::string& type) {
  try {
    return kStringToType.at(type);
  } catch (...) {
    return Type::UNDEFINED;
  }
}

}  // namespace SpatialDyn
