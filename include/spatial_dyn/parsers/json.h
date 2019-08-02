/**
 * json.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 6, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_PARSERS_JSON_H_
#define SPATIAL_DYN_PARSERS_JSON_H_

#include <ctrl_utils/json.h>

#include "spatial_dyn/structs/articulated_body.h"

namespace spatial_dyn {

void to_json(nlohmann::json& j, const ArticulatedBody& ab);
void from_json(const nlohmann::json& j, ArticulatedBody& ab);

void to_json(nlohmann::json& j, const RigidBody& rb);
void from_json(const nlohmann::json& j, RigidBody& rb);

void to_json(nlohmann::json& j, const Joint& joint);
void from_json(const nlohmann::json& j, Joint& joint);

void to_json(nlohmann::json& j, const Joint::Type& type);
void from_json(const nlohmann::json& j, Joint::Type& type);

void to_json(nlohmann::json& j, const SpatialInertiad& inertia);
void from_json(const nlohmann::json& j, SpatialInertiad& inertia);

void to_json(nlohmann::json& j, const Graphics& graphics);
void from_json(const nlohmann::json& j, Graphics& graphics);

void to_json(nlohmann::json& j, const Graphics::Geometry& geometry);
void from_json(const nlohmann::json& j, Graphics::Geometry& geometry);

void to_json(nlohmann::json& j, const Graphics::Geometry::Type& type);
void from_json(const nlohmann::json& j, Graphics::Geometry::Type& type);

void to_json(nlohmann::json& j, const Graphics::Material& material);
void from_json(const nlohmann::json& j, Graphics::Material& material);

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_PARSERS_JSON_H_
