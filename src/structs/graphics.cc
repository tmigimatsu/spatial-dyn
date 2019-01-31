/**
 * graphics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 6, 2018
 * Authors: Toki Migimatsu
 */

#include "structs/graphics.h"

#include <map>  // std::map

namespace spatial_dyn {

static const std::map<std::string, Graphics::Geometry::Type> kStringToType = {
  {"BOX",       Graphics::Geometry::Type::BOX},
  {"CYLINDER",  Graphics::Geometry::Type::CYLINDER},
  {"SPHERE",    Graphics::Geometry::Type::SPHERE},
  {"MESH",      Graphics::Geometry::Type::MESH},
  {"UNDEFINED", Graphics::Geometry::Type::UNDEFINED}
};

static const std::map<Graphics::Geometry::Type, std::string> kTypeToString = {
  {Graphics::Geometry::Type::BOX,       "BOX"},
  {Graphics::Geometry::Type::CYLINDER,  "CYLINDER"},
  {Graphics::Geometry::Type::SPHERE,    "SPHERE"},
  {Graphics::Geometry::Type::MESH,      "MESH"},
  {Graphics::Geometry::Type::UNDEFINED, "UNDEFINED"}
};

// std::string Graphics::Geometry::TypeToString(const Type& type) {
//   switch (type) {
//     case Type::BOX:      return "BOX";
//     case Type::CYLINDER: return "CYLINDER";
//     case Type::SPHERE:   return "SPHERE";
//     case Type::MESH:     return "MESH";
//     default:             return "UNDEFINED";
//   }
// }

// Graphics::Geometry::Type Graphics::Geometry::StringToType(const std::string& type) {
//   try {
//     return kStringToType.at(type);
//   } catch (...) {
//     return Type::UNDEFINED;
//   }
// }

std::ostream& operator<<(std::ostream& os, const Graphics::Geometry::Type& type) {
  os << kTypeToString.at(type);
  return os;
}

std::istream& operator>>(std::istream& is, Graphics::Geometry::Type& type) {
  std::string str;
  is >> str;
  type = kStringToType.at(str);
  return is;
}


}  // namespace spatial_dyn
