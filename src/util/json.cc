/**
 * json.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 6, 2018
 * Authors: Toki Migimatsu
 */

#include "util/json.h"

namespace SpatialDyn {
namespace Json {

nlohmann::json Serialize(const ArticulatedBody& ab) {
  nlohmann::json json;
  json["name"] = ab.name;
  json["pos_in_world"] = Serialize(ab.T_base_to_world().translation());
  json["ori_in_world"] = Serialize(Eigen::Quaterniond(ab.T_base_to_world().linear()));
  nlohmann::json json_rigid_bodies;
  for (const RigidBody& rb : ab.rigid_bodies()) {
    json_rigid_bodies.push_back(Serialize(rb));
  }
  json["rigid_bodies"] = std::move(json_rigid_bodies);
  return json;
}

nlohmann::json Serialize(const RigidBody& rb) {
  nlohmann::json json;
  json["name"] = rb.name;
  json["id"] = rb.id();
  json["id_parent"] = rb.id_parent();
  json["pos_in_parent"] = Serialize(rb.T_to_parent().translation());
  json["ori_in_parent"] = Serialize(Eigen::Quaterniond(rb.T_to_parent().linear()));
  json["inertia"] = Serialize(rb.inertia().matrix().flatArray());
  json["joint"] = Serialize(rb.joint());
  if (rb.graphics.geometry.type != GeometryType::UNDEFINED) {
    json["graphics"] = Serialize(rb.graphics);
  }
  return json;
}

nlohmann::json Serialize(const Joint& joint) {
  nlohmann::json json;
  json["type"] = std::string(joint);
  json["q_min"] = joint.q_min();
  json["q_max"] = joint.q_max();
  json["dq_max"] = joint.dq_max();
  json["fq_max"] = joint.fq_max();
  json["f_coulomb"] = joint.f_coulomb();
  json["f_stiction"] = joint.f_stiction();
  return json;
}

nlohmann::json Serialize(const Graphics& graphics) {
  nlohmann::json json;
  json["name"] = graphics.name;
  json["pos_in_parent"] = Serialize(graphics.T_to_parent.translation());
  json["ori_in_parent"] = Serialize(Eigen::Quaterniond(graphics.T_to_parent.linear()));
  json["geometry"] = Serialize(graphics.geometry);
  if (!graphics.material.name.empty()) {
    json["material"] = Serialize(graphics.material);
  }
  return json;
}

nlohmann::json Serialize(const Geometry& geometry) {
  nlohmann::json json;
  json["type"] = std::string(geometry);
  switch (geometry.type) {
    case GeometryType::BOX:
      json["scale"] = Serialize(geometry.scale);
      break;
    case GeometryType::CYLINDER:
      json["radius"] = geometry.radius;
      json["length"] = geometry.length;
      break;
    case GeometryType::SPHERE:
      json["radius"] = geometry.radius;
      break;
    case GeometryType::MESH:
      json["mesh"] = geometry.mesh;
      json["scale"] = Serialize(geometry.scale);
      break;
    default:
      break;
  }
  return json;
}

nlohmann::json Serialize(const Material& material) {
  nlohmann::json json;
  json["name"] = material.name;
  json["rgba"] = Serialize(material.rgba);
  json["texture"] = material.texture;
  return json;
}

/*
template<typename Derived>
Eigen::MatrixBase<Derived>::PlainObject
Deserialize(const nlohmann::json& json) {
  // Vector
  if (matrix.cols() == 1) {
    if (matrix.rows() == 0) {
      // Dynamic vector
      matrix.resize(json.size());
    } else if (json.size() != matrix.rows()) {
      // Static vector
      throw std::runtime_error("nlohmann::adl_serializer<Eigen::EigenBase>::from_json(): json and vector sizes do not match (" + std::to_string(json.size()) + " vs. " + std::to_string(matrix.rows()) + ").");
    }

    for (size_t i = 0; i < matrix.rows(); i++) {
      matrix(i) = json.at(i).get<double>();
    }
    return;
  }

  // Fixed rows
  if (matrix.rows() > 0 && json.size() != matrix.rows()) {
    throw std::runtime_error("nlohmann::adl_serializer<Eigen::EigenBase>::from_json(): json and matrix do not have the same number of rows (" + std::to_string(json.size()) + " vs. " + std::to_string(matrix.rows()) + ").");
  }

  // Fixed columns
  if (matrix.cols() > 0 && json[0].size() != matrix.cols()) {
    throw std::runtime_error("nlohmann::adl_serializer<Eigen::EigenBase>::from_json(): json and matrix do not have the same number of columns (" + std::to_string(json[0].size()) + " vs. " + std::to_string(matrix.cols()) + ").");
  }

  // Dynamic matrix
  if (matrix.rows() == 0 || matrix.cols() == 0) {
    matrix.resize(json.size(), json[0].size());
  }

  // Assign matrix
  for (size_t i = 0; i < matrix.rows(); i++) {
    for (size_t j = 0; j < matrix.cols(); j++) {
      matrix(i,j) = json.at(i).at(j).get<double>();
    }
  }
}
*/

}  // namespace Json
}  // namespace SpatialDyn
