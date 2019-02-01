/**
 * graphics.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 6, 2018
 * Authors: Toki Migimatsu
 */

#include "structs/graphics.h"

#include <algorithm>  // std::transform
#include <cctype>     // std::tolower
#include <map>  // std::map

namespace spatial_dyn {

static const std::map<std::string, Graphics::Geometry::Type> kStringToType = {
  {"box",       Graphics::Geometry::Type::kBox},
  {"cylinder",  Graphics::Geometry::Type::kCylinder},
  {"sphere",    Graphics::Geometry::Type::kSphere},
  {"mesh",      Graphics::Geometry::Type::kMesh},
  {"undefined", Graphics::Geometry::Type::kUndefined}
};

static const std::map<Graphics::Geometry::Type, std::string> kTypeToString = {
  {Graphics::Geometry::Type::kBox,       "box"},
  {Graphics::Geometry::Type::kCylinder,  "cylinder"},
  {Graphics::Geometry::Type::kSphere,    "sphere"},
  {Graphics::Geometry::Type::kMesh,      "mesh"},
  {Graphics::Geometry::Type::kUndefined, "undefined"}
};

std::ostream& operator<<(std::ostream& os, const Graphics::Geometry::Type& type) {
  os << kTypeToString.at(type);
  return os;
}

std::istream& operator>>(std::istream& is, Graphics::Geometry::Type& type) {
  std::string str;
  is >> str;
  std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return std::tolower(c); });
  type = kStringToType.at(str);
  return is;
}


}  // namespace spatial_dyn
