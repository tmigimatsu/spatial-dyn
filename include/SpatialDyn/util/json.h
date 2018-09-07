/**
 * json.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 6, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_JSON_H_
#define SPATIAL_DYN_JSON_H_

#include "articulated_body.h"

#include "nlohmann/json.hpp"

namespace SpatialDyn {
namespace Json {

nlohmann::json Serialize(const ArticulatedBody& ab);
nlohmann::json Serialize(const RigidBody& rb);
nlohmann::json Serialize(const Joint& joint);
nlohmann::json Serialize(const Graphics& graphics);
nlohmann::json Serialize(const Geometry& geometry);
nlohmann::json Serialize(const Material& material);

template<typename Derived>
nlohmann::json Serialize(const Eigen::DenseBase<Derived>& matrix);

template<typename Derived>
nlohmann::json Serialize(const Eigen::QuaternionBase<Derived>& quat);

template<typename Derived>
nlohmann::json Serialize(const Eigen::DenseBase<Derived>& matrix) {
  nlohmann::json json;
  if (matrix.cols() == 1) {
    // Vector
    for (size_t i = 0; i < matrix.rows(); i++) {
      json.push_back(matrix(i));
    }
  } else {
    // Matrix
    for (size_t i = 0; i < matrix.rows(); i++) {
      nlohmann::json json_row;
      for (size_t j = 0; j < matrix.cols(); j++) {
        json_row.push_back(matrix(i,j));
      }
      json.push_back(json_row);
    }
  }
  return json;
}

template<typename Derived>
nlohmann::json Serialize(const Eigen::QuaternionBase<Derived>& quat) {
  nlohmann::json json;
  json["w"] = quat.w();
  json["x"] = quat.x();
  json["y"] = quat.y();
  json["z"] = quat.z();
  return json;
}

}  // namespace Json
}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_JSON_H_
