/**
 * json.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 6, 2018
 * Authors: Toki Migimatsu
 */

#include "parsers/json.h"

#include <exception>  // std::runtime_error

#include <ctrl_utils/string.h>

namespace spatial_dyn {

void to_json(nlohmann::json& json, const ArticulatedBody& ab) {
  json["name"] = ab.name;
  json["graphics"] = ab.graphics;
  json["T_base_to_world"] = ab.T_base_to_world();
  json["inertia_base"] = ab.inertia_base();
  json["rigid_bodies"] = ab.rigid_bodies();
}

void from_json(const nlohmann::json& json, ArticulatedBody& ab) {
  if (json.count("name")) {
    ab.name = json.at("name").get<std::string>();
  }
  if (json.count("graphics")) {
    ab.graphics = json.at("graphics").get<std::vector<Graphics>>();
  }
  if (json.count("T_base_to_world")) {
    ab.set_T_base_to_world(json.at("T_base_to_world").get<Eigen::Isometry3d>());
  }
  if (json.count("inertia_base")) {
    ab.set_inertia_base(json.at("inertia_base").get<SpatialInertiad>());
  }
  if (json.count("rigid_bodies")) {
    for (const nlohmann::json& json_rb : json.at("rigid_bodies")) {
      RigidBody rb = json_rb.get<RigidBody>();
      ab.AddRigidBody(rb, rb.id_parent());
    }
  }
}

void to_json(nlohmann::json& json, const RigidBody& rb) {
  json["name"] = rb.name;
  json["graphics"] = rb.graphics;
  json["id"] = rb.id();
  json["id_parent"] = rb.id_parent();
  json["T_to_parent"] = rb.T_to_parent();
  json["inertia"] = rb.inertia();
  json["joint"] = rb.joint();
}

void from_json(const nlohmann::json& json, RigidBody& rb) {
  if (json.count("name")) {
    rb.name = json.at("name").get<std::string>();
  }
  if (json.count("graphics")) {
    rb.graphics = json.at("graphics").get<std::vector<Graphics>>();
  }
  if (json.count("id")) {
    rb.set_id(json.at("id").get<int>());
  }
  if (json.count("id_parent")) {
    rb.set_id_parent(json.at("id_parent").get<int>());
  }
  if (json.count("T_to_parent")) {
    rb.set_T_to_parent(json.at("T_to_parent").get<Eigen::Isometry3d>());
  }
  if (json.count("inertia")) {
    rb.set_inertia(json.at("inertia").get<SpatialInertiad>());
  }
  if (json.count("joint")) {
    rb.set_joint(json.at("joint").get<Joint>());
  }
}

void to_json(nlohmann::json& json, const Joint& joint) {
  json["type"] = joint.type();
  json["q_min"] = joint.q_min();
  json["q_max"] = joint.q_max();
  json["dq_max"] = joint.dq_max();
  json["fq_max"] = joint.fq_max();
  json["f_coulomb"] = joint.f_coulomb();
  json["f_viscous"] = joint.f_viscous();
  json["f_stiction"] = joint.f_stiction();
}

void from_json(const nlohmann::json& json, Joint& joint) {
  if (!json.count("type")) {
    throw std::runtime_error("from_json(const nlohmann::json&, spatial_dyn::Joint&): Type field missing.\n" + json.dump());
  }
  joint.set_type(json.at("type").get<spatial_dyn::Joint::Type>());

  if (json.count("q_min")) {
    joint.set_q_min(json.at("q_min").get<double>());
  }
  if (json.count("q_max")) {
    joint.set_q_max(json.at("q_max").get<double>());
  }
  if (json.count("dq_max")) {
    joint.set_dq_max(json.at("dq_max").get<double>());
  }
  if (json.count("fq_max")) {
    joint.set_fq_max(json.at("fq_max").get<double>());
  }
  if (json.count("f_coulomb")) {
    joint.set_f_coulomb(json.at("f_coulomb").get<double>());
  }
  if (json.count("f_viscous")) {
    joint.set_f_viscous(json.at("f_viscous").get<double>());
  }
  if (json.count("f_stiction")) {
    joint.set_f_stiction(json.at("f_stiction").get<double>());
  }
}

void to_json(nlohmann::json& json, const Joint::Type& type) {
  json = ctrl_utils::ToString(type);
}

void from_json(const nlohmann::json& json, Joint::Type& type) {
  type = ctrl_utils::FromString<Joint::Type>(json.get<std::string>());
}

void to_json(nlohmann::json& json, const SpatialInertiad& inertia) {
  json["mass"] = inertia.mass;
  json["com"] = inertia.com;
  json["I_com_flat"] = inertia.I_com_flat();
}

void from_json(const nlohmann::json& json, SpatialInertiad& inertia) {
  if (!json.count("mass")) {
    throw std::runtime_error("from_json(const nlohmann::json&, SpatialInertiad&): 'mass' field missing.\n" + json.dump());
  }
  const double mass = json.at("mass").get<double>();

  if (!json.count("com")) {
    throw std::runtime_error("from_json(const nlohmann::json&, SpatialInertiad&): 'com' field missing.\n" + json.dump());
  }
  const Eigen::Vector3d com = json.at("com").get<Eigen::Vector3d>();

  if (!json.count("I_com_flat")) {
    throw std::runtime_error("from_json(const nlohmann::json&, SpatialInertiad&): 'I_com_flat' field missing.\n" + json.dump());
  }
  const Eigen::Vector6d I_com_flat = json.at("I_com_flat").get<Eigen::Vector6d>();

  inertia = spatial_dyn::SpatialInertiad(mass, com, I_com_flat);
}

void to_json(nlohmann::json& json, const Graphics& graphics) {
  json["name"] = graphics.name;
  json["T_to_parent"] = graphics.T_to_parent;
  json["geometry"] = graphics.geometry;
  json["material"] = graphics.material;
}

void from_json(const nlohmann::json& json, Graphics& graphics) {
  if (json.count("name")) {
    graphics.name = json.at("name").get<std::string>();
  }
  if (json.count("T_to_parent")) {
    graphics.T_to_parent = json.at("T_to_parent").get<Eigen::Isometry3d>();
  }
  if (json.count("geometry")) {
    graphics.geometry = json.at("geometry").get<Graphics::Geometry>();
  }
  if (json.count("material")) {
    graphics.material = json.at("material").get<Graphics::Material>();
  }
}

void to_json(nlohmann::json& json, const Graphics::Geometry& geometry) {
  json["type"] = geometry.type;
  switch (geometry.type) {
    case Graphics::Geometry::Type::kBox:
      json["scale"] = geometry.scale;
      break;
    case Graphics::Geometry::Type::kCapsule:
    case Graphics::Geometry::Type::kCylinder:
      json["radius"] = geometry.radius;
      json["length"] = geometry.length;
      break;
    case Graphics::Geometry::Type::kSphere:
      json["radius"] = geometry.radius;
      break;
    case Graphics::Geometry::Type::kMesh:
      json["mesh"] = geometry.mesh;
      json["scale"] = geometry.scale;
      break;
    default:
      break;
  }
}

void from_json(const nlohmann::json& json, Graphics::Geometry& geometry) {
  if (!json.count("type")) {
    throw std::runtime_error("from_json(const nlohmann::json&, SpatialInertiad&): 'type' field missing.\n" + json.dump());
  }
  geometry.type = json.at("type").get<Graphics::Geometry::Type>();
  switch (geometry.type) {
    case Graphics::Geometry::Type::kBox:
      if (!json.count("scale")) {
        throw std::runtime_error("from_json(const nlohmann::json&, SpatialInertiad&): 'scale' field missing.\n" + json.dump());
      }
      geometry.scale = json.at("scale").get<Eigen::Vector3d>();
      break;
    case Graphics::Geometry::Type::kCapsule:
    case Graphics::Geometry::Type::kCylinder:
      if (!json.count("radius")) {
        throw std::runtime_error("from_json(const nlohmann::json&, SpatialInertiad&): 'radius' field missing.\n" + json.dump());
      }
      geometry.radius = json.at("radius").get<double>();
      if (!json.count("length")) {
        throw std::runtime_error("from_json(const nlohmann::json&, SpatialInertiad&): 'length' field missing.\n" + json.dump());
      }
      geometry.length = json.at("length").get<double>();
      break;
    case Graphics::Geometry::Type::kSphere:
      if (!json.count("radius")) {
        throw std::runtime_error("from_json(const nlohmann::json&, SpatialInertiad&): 'radius' field missing.\n" + json.dump());
      }
      geometry.radius = json.at("radius").get<double>();
      break;
    case Graphics::Geometry::Type::kMesh:
      if (json.count("scale")) {
        geometry.scale = json.at("scale").get<Eigen::Vector3d>();
      }
      if (!json.count("mesh")) {
        throw std::runtime_error("from_json(const nlohmann::json&, SpatialInertiad&): 'mesh' field missing.\n" + json.dump());
      }
      geometry.mesh = json.at("mesh").get<std::string>();
      break;
    default:
      break;
  }
}

void to_json(nlohmann::json& json, const Graphics::Geometry::Type& type) {
  json = ctrl_utils::ToString(type);
}

void from_json(const nlohmann::json& json, Graphics::Geometry::Type& type) {
  type = ctrl_utils::FromString<Graphics::Geometry::Type>(json.get<std::string>());
}

void to_json(nlohmann::json& json, const Graphics::Material& material) {
  json["name"] = material.name;
  json["rgba"] = material.rgba;
  json["texture"] = material.texture;
}

void from_json(const nlohmann::json& json, Graphics::Material& material) {
  if (json.count("name")) {
    material.name = json.at("name").get<std::string>();
  }
  if (json.count("rgba")) {
    material.rgba = json.at("rgba").get<Eigen::Vector4d>();
  }
  if (json.count("texture")) {
    material.texture = json.at("texture").get<std::string>();
  }
}

}  // namespace spatial_dyn
