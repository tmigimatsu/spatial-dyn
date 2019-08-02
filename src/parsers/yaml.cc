/**
 * yaml.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: February 27, 2019
 * Authors: Toki Migimatsu
 */

#include "parsers/yaml.h"

#include <exception>  // std::runtime_error

#include <ctrl_utils/string.h>

namespace YAML {

Node convert<spatial_dyn::ArticulatedBody>::encode(const spatial_dyn::ArticulatedBody& ab) {
  Node node;
  node["name"] = ab.name;
  node["graphics"] = ab.graphics;
  node["T_base_to_world"] = ab.T_base_to_world();
  node["inertia_base"] = ab.inertia_base();
  node["rigid_bodies"] = ab.rigid_bodies();
  return node;
}

bool convert<spatial_dyn::ArticulatedBody>::decode(const Node& node, spatial_dyn::ArticulatedBody& ab) {
  if (node["name"]) {
    ab.name = node["name"].as<std::string>();
  }
  if (node["graphics"]) {
    ab.graphics = node["graphics"].as<std::vector<spatial_dyn::Graphics>>();
  }
  if (node["T_base_to_world"]) {
    ab.set_T_base_to_world(node["T_base_to_world"].as<Eigen::Isometry3d>());
  }
  if (node["inertia_base"]) {
    ab.set_inertia_base(node["inertia_base"].as<spatial_dyn::SpatialInertiad>());
  }
  if (node["rigid_bodies"]) {
    for (const Node& node_rb : node["rigid_bodies"]) {
      spatial_dyn::RigidBody rb = node_rb.as<spatial_dyn::RigidBody>();
      ab.AddRigidBody(rb, rb.id_parent());
    }
  }
  return true;
}

Node convert<spatial_dyn::RigidBody>::encode(const spatial_dyn::RigidBody& rb) {
  Node node;
  node["name"] = rb.name;
  node["graphics"] = rb.graphics;
  node["id"] = rb.id();
  node["id_parent"] = rb.id_parent();
  node["T_to_parent"] = rb.T_to_parent();
  node["inertia"] = rb.inertia();
  node["joint"] = rb.joint();
  return node;
}

bool convert<spatial_dyn::RigidBody>::decode(const Node& node, spatial_dyn::RigidBody& rb) {
  if (node["name"]) {
    rb.name = node["name"].as<std::string>();
  }
  if (node["graphics"]) {
    rb.graphics = node["graphics"].as<std::vector<spatial_dyn::Graphics>>();
  }
  if (node["T_to_parent"]) {
    rb.set_T_to_parent(node["T_to_parent"].as<Eigen::Isometry3d>());
  }
  if (node["inertia"]) {
    rb.set_inertia(node["inertia"].as<spatial_dyn::SpatialInertiad>());
  }
  if (node["joint"]) {
    rb.set_joint(node["joint"].as<spatial_dyn::Joint>());
  }
  return true;
}

Node convert<spatial_dyn::Joint>::encode(const spatial_dyn::Joint& joint) {
  YAML::Node node;
  node["type"] = joint.type();
  node["q_min"] = joint.q_min();
  node["q_max"] = joint.q_max();
  node["dq_max"] = joint.dq_max();
  node["fq_max"] = joint.fq_max();
  node["f_coulomb"] = joint.f_coulomb();
  node["f_viscous"] = joint.f_viscous();
  node["f_stiction"] = joint.f_stiction();
  return node;
}

bool convert<spatial_dyn::Joint>::decode(const Node& node, spatial_dyn::Joint& joint) {
  if (!node["type"]) {
    std::stringstream ss; ss << node;
    throw std::runtime_error("YAML::convert<spatial_dyn::Joint>::decode(): Type field missing.\n" + ss.str());
  }
  joint.set_type(node["type"].as<spatial_dyn::Joint::Type>());

  if (node["q_min"]) {
    joint.set_q_min(node["q_min"].as<double>());
  }
  if (node["q_max"]) {
    joint.set_q_max(node["q_max"].as<double>());
  }
  if (node["dq_max"]) {
    joint.set_dq_max(node["dq_max"].as<double>());
  }
  if (node["fq_max"]) {
    joint.set_fq_max(node["fq_max"].as<double>());
  }
  if (node["f_coulomb"]) {
    joint.set_f_coulomb(node["f_coulomb"].as<double>());
  }
  if (node["f_viscous"]) {
    joint.set_f_viscous(node["f_viscous"].as<double>());
  }
  if (node["f_stiction"]) {
    joint.set_f_stiction(node["f_stiction"].as<double>());
  }
  return true;
}

Node convert<spatial_dyn::Joint::Type>::encode(const spatial_dyn::Joint::Type& type) {
  YAML::Node node;
  node = ctrl_utils::ToString(type);
  return node;
}

bool convert<spatial_dyn::Joint::Type>::decode(const Node& node, spatial_dyn::Joint::Type& type) {
  type = ctrl_utils::FromString<spatial_dyn::Joint::Type>(node.as<std::string>());
  return true;
}

Node convert<spatial_dyn::SpatialInertiad>::encode(const spatial_dyn::SpatialInertiad& inertia) {
  Node node;
  node["mass"] = inertia.mass;
  node["com"] = inertia.com;
  node["I_com_flat"] = inertia.I_com_flat();
  return node;
}

bool convert<spatial_dyn::SpatialInertiad>::decode(const Node& node, spatial_dyn::SpatialInertiad& inertia) {
  if (!node["mass"]) {
    std::stringstream ss; ss << node;
    throw std::runtime_error("YAML::convert<spatial_dyn::SpatialInertiad>::decode(): 'mass' field missing.\n" + ss.str());
  }
  const double mass = node["mass"].as<double>();

  if (!node["com"]) {
    std::stringstream ss; ss << node;
    throw std::runtime_error("YAML::convert<spatial_dyn::SpatialInertiad>::decode(): 'com' field missing.\n" + ss.str());
  }
  const Eigen::Vector3d com = node["com"].as<Eigen::Vector3d>();

  if (!node["I_com_flat"]) {
    std::stringstream ss; ss << node;
    throw std::runtime_error("YAML::convert<spatial_dyn::SpatialInertiad>::decode(): 'I_com_flat' field missing.\n" + ss.str());
  }
  const Eigen::Vector6d I_com_flat = node["I_com_flat"].as<Eigen::Vector6d>();

  inertia = spatial_dyn::SpatialInertiad(mass, com, I_com_flat);
  return true;
}

Node convert<spatial_dyn::Graphics>::encode(const spatial_dyn::Graphics& graphics) {
  Node node;
  node["name"] = graphics.name;
  node["T_to_parent"] = graphics.T_to_parent;
  node["geometry"] = graphics.geometry;
  node["material"] = graphics.material;
  return node;
}

bool convert<spatial_dyn::Graphics>::decode(const Node& node, spatial_dyn::Graphics& graphics) {
  if (node["name"]) {
    graphics.name = node["name"].as<std::string>();
  }
  if (node["T_to_parent"]) {
    graphics.T_to_parent = node["T_to_parent"].as<Eigen::Isometry3d>();
  }
  if (node["geometry"]) {
    graphics.geometry = node["geometry"].as<spatial_dyn::Graphics::Geometry>();
  }
  if (node["material"]) {
    graphics.material = node["material"].as<spatial_dyn::Graphics::Material>();
  }
  return true;
}

Node convert<spatial_dyn::Graphics::Geometry>::encode(const spatial_dyn::Graphics::Geometry& geometry) {
  Node node;
  node["type"] = geometry.type;
  switch (geometry.type) {
    case spatial_dyn::Graphics::Geometry::Type::kBox:
      node["scale"] = geometry.scale;
      break;
    case spatial_dyn::Graphics::Geometry::Type::kCapsule:
    case spatial_dyn::Graphics::Geometry::Type::kCylinder:
      node["radius"] = geometry.radius;
      node["length"] = geometry.length;
      break;
    case spatial_dyn::Graphics::Geometry::Type::kSphere:
      node["radius"] = geometry.radius;
      break;
    case spatial_dyn::Graphics::Geometry::Type::kMesh:
      node["scale"] = geometry.scale;
      node["mesh"] = geometry.mesh;
      break;
    default:
      break;
  }
  return node;
}

bool convert<spatial_dyn::Graphics::Geometry>::decode(const Node& node, spatial_dyn::Graphics::Geometry& geometry) {
  if (!node["type"]) {
    std::stringstream ss; ss << node;
    throw std::runtime_error("YAML::convert<spatial_dyn::Graphics::Geometry>::decode(): 'type' field missing.\n" + ss.str());
  }
  geometry.type = node["type"].as<spatial_dyn::Graphics::Geometry::Type>();
  switch (geometry.type) {
    case spatial_dyn::Graphics::Geometry::Type::kBox:
      if (!node["scale"]) {
        std::stringstream ss; ss << node;
        throw std::runtime_error("YAML::convert<spatial_dyn::Graphics::Geometry>::decode(): 'scale' field missing.\n" + ss.str());
      }
      geometry.scale = node["scale"].as<Eigen::Vector3d>();
      break;
    case spatial_dyn::Graphics::Geometry::Type::kCapsule:
    case spatial_dyn::Graphics::Geometry::Type::kCylinder:
      if (!node["radius"]) {
        std::stringstream ss; ss << node;
        throw std::runtime_error("YAML::convert<spatial_dyn::Graphics::Geometry>::decode(): 'radius' field missing.\n" + ss.str());
      }
      geometry.radius = node["radius"].as<double>();
      if (!node["length"]) {
        std::stringstream ss; ss << node;
        throw std::runtime_error("YAML::convert<spatial_dyn::Graphics::Geometry>::decode(): 'length' field missing.\n" + ss.str());
      }
      geometry.length = node["length"].as<double>();
      break;
    case spatial_dyn::Graphics::Geometry::Type::kSphere:
      if (!node["radius"]) {
        std::stringstream ss; ss << node;
        throw std::runtime_error("YAML::convert<spatial_dyn::Graphics::Geometry>::decode(): 'radius' field missing.\n" + ss.str());
      }
      geometry.radius = node["radius"].as<double>();
      break;
    case spatial_dyn::Graphics::Geometry::Type::kMesh:
      if (node["scale"]) {
        geometry.scale = node["scale"].as<Eigen::Vector3d>();
      }
      if (!node["mesh"]) {
        std::stringstream ss; ss << node;
        throw std::runtime_error("YAML::convert<spatial_dyn::Graphics::Geometry>::decode(): 'mesh' field missing.\n" + ss.str());
      }
      geometry.mesh = node["mesh"].as<std::string>();
      break;
    default:
      break;
  }
  return true;
}

Node convert<spatial_dyn::Graphics::Geometry::Type>::encode(const spatial_dyn::Graphics::Geometry::Type& type) {
  YAML::Node node;
  node = ctrl_utils::ToString(type);
  return node;
}

bool convert<spatial_dyn::Graphics::Geometry::Type>::decode(const Node& node, spatial_dyn::Graphics::Geometry::Type& type) {
  type = ctrl_utils::FromString<spatial_dyn::Graphics::Geometry::Type>(node.as<std::string>());
  return true;
}

Node convert<spatial_dyn::Graphics::Material>::encode(const spatial_dyn::Graphics::Material& material) {
  Node node;
  node["name"] = material.name;
  node["rgba"] = material.rgba;
  node["texture"] = material.texture;
  return node;
}

bool convert<spatial_dyn::Graphics::Material>::decode(const Node& node, spatial_dyn::Graphics::Material& material) {
  if (node["name"]) {
    material.name = node["name"].as<std::string>();
  }
  if (node["rgba"]) {
    material.rgba = node["rgba"].as<Eigen::Vector4d>();
  }
  if (node["texture"]) {
    material.texture = node["texture"].as<std::string>();
  }
  return true;
}

}  // namespace YAML
