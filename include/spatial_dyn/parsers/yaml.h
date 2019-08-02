/**
 * yaml.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 27, 2019
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_PARSERS_YAML_H_
#define SPATIAL_DYN_PARSERS_YAML_H_

#include <ctrl_utils/yaml.h>

#include "spatial_dyn/structs/articulated_body.h"

namespace YAML {

template<>
struct convert<spatial_dyn::ArticulatedBody> {
  static Node encode(const spatial_dyn::ArticulatedBody& ab);
  static bool decode(const Node& node, spatial_dyn::ArticulatedBody& ab);
};

template<>
struct convert<spatial_dyn::RigidBody> {
  static Node encode(const spatial_dyn::RigidBody& rb);
  static bool decode(const Node& node, spatial_dyn::RigidBody& rb);
};

template<>
struct convert<spatial_dyn::Joint> {
  static Node encode(const spatial_dyn::Joint& joint);
  static bool decode(const Node& node, spatial_dyn::Joint& joint);
};

template<>
struct convert<spatial_dyn::Joint::Type> {
  static Node encode(const spatial_dyn::Joint::Type& type);
  static bool decode(const Node& node, spatial_dyn::Joint::Type& type);
};

template<>
struct convert<spatial_dyn::SpatialInertiad> {
  static Node encode(const spatial_dyn::SpatialInertiad& inertia);
  static bool decode(const Node& node, spatial_dyn::SpatialInertiad& inertia);
};

template<>
struct convert<spatial_dyn::Graphics> {
  static Node encode(const spatial_dyn::Graphics& graphics);
  static bool decode(const Node& node, spatial_dyn::Graphics& graphics);
};

template<>
struct convert<spatial_dyn::Graphics::Geometry> {
  static Node encode(const spatial_dyn::Graphics::Geometry& geometry);
  static bool decode(const Node& node, spatial_dyn::Graphics::Geometry& geometry);
};

template<>
struct convert<spatial_dyn::Graphics::Geometry::Type> {
  static Node encode(const spatial_dyn::Graphics::Geometry::Type& type);
  static bool decode(const Node& node, spatial_dyn::Graphics::Geometry::Type& type);
};

template<>
struct convert<spatial_dyn::Graphics::Material> {
  static Node encode(const spatial_dyn::Graphics::Material& material);
  static bool decode(const Node& node, spatial_dyn::Graphics::Material& material);
};

}  // namespace YAML

#endif  // SPATIAL_DYN_PARSERS_YAML_H_
