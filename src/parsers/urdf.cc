/**
 * urdf.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 5, 2018
 * Authors: Toki Migimatsu
 */

#include "parsers/urdf.h"

#include <exception>  // std::runtime_error
#include <limits>     // std::numeric_limits
#include <list>       // std::list
#include <map>        // std::map
#include <set>        // std::set
#include <string>     // std::string
#include <utility>    // std::move, std::pair, std::tie

#include <tinyxml2.h>
#include <ctrl_utils/eigen_string.h>
#include <ctrl_utils/filesystem.h>

namespace spatial_dyn {
namespace urdf {

const tinyxml2::XMLElement* ParseElement(const tinyxml2::XMLDocument& xml_doc, const std::string& element) {
  const tinyxml2::XMLElement* xml_element = xml_doc.FirstChildElement(element.c_str());
  if (xml_element == nullptr) {
    throw std::runtime_error("spatial_dyn::urdf::ParseElement(): <" + element + "> element missing from document.");
  }
  return xml_element;
}
const tinyxml2::XMLElement* ParseElement(const tinyxml2::XMLElement* xml_parent, const std::string& element) {
  const tinyxml2::XMLElement* xml_element = xml_parent->FirstChildElement(element.c_str());
  if (xml_element == nullptr) {
    throw std::runtime_error("spatial_dyn::urdf::ParseElement(): <" + element +
                             "> element missing from <" + xml_parent->Name() + "> element.");
  }
  return xml_element;
}

std::string ParseAttribute(const tinyxml2::XMLElement* xml_element, const std::string& attribute) {
  const char *value = xml_element->Attribute(attribute.c_str());
  if (value == nullptr) {
    throw std::runtime_error("spatial_dyn::urdf::ParseAttribute(): \"link\" attribute missing from <" +
                             std::string(xml_element->Name()) + "> element.");
  }
  return std::string(value);
}

double ParseDoubleAttribute(const tinyxml2::XMLElement* xml_element, const std::string& attribute) {
  double value;
  tinyxml2::XMLError status = xml_element->QueryDoubleAttribute(attribute.c_str(), &value);
  if (status != tinyxml2::XMLError::XML_SUCCESS) {
    throw std::runtime_error("spatial_dyn::urdf::ParseDoubleAttribute(): Invalid \"" + attribute +
                            "\" attribute in <" + std::string(xml_element->Name()) +
                            "> element - " +
                            std::string(tinyxml2::XMLDocument::ErrorIDToName(status)) + ".");
  }
  return value;
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond>
ParseOriginElement(const tinyxml2::XMLElement* xml_element) {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond ori = Eigen::Quaterniond::Identity();

  const tinyxml2::XMLElement* xml_origin = xml_element->FirstChildElement("origin");
  if (xml_origin != nullptr) {
    const char* attr_xyz = xml_origin->Attribute("xyz");
    if (attr_xyz != nullptr) {
      pos = ctrl_utils::DecodeMatlab<Eigen::Vector3d>(attr_xyz);
    }

    const char* attr_rpy = xml_origin->Attribute("rpy");
    if (attr_rpy != nullptr) {
      Eigen::Vector3d rpy = ctrl_utils::DecodeMatlab<Eigen::Vector3d>(std::string(attr_rpy));
      // TODO: Check Euler angles
      ori = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
    }
  }
  return std::make_pair(std::move(pos), std::move(ori));
}

Graphics ParseGraphics(const tinyxml2::XMLElement* xml_visual, const std::string& path_meshes) {
  Graphics graphics;

  // Parse name
  const char* attr_name = xml_visual->Attribute("name");
  if (attr_name != nullptr) {
    graphics.name = attr_name;
  }

  // Parse origin
  Eigen::Vector3d pos;
  Eigen::Quaterniond ori;
  std::tie(pos, ori) = ParseOriginElement(xml_visual);
  graphics.T_to_parent = Eigen::Translation3d(pos) * ori;

  // Parse geometry
  const tinyxml2::XMLElement* xml_geometry = ParseElement(xml_visual, "geometry");
  const tinyxml2::XMLElement* xml_type = xml_geometry->FirstChildElement();
  if (xml_type == nullptr) {
    throw std::runtime_error("spatial_dyn::urdf::ParseGraphics(): <box>|<cylinder>|<sphere>|<mesh> element missing from <geometry> element.");
  }
  Graphics::Geometry& geometry = graphics.geometry;
  std::string str_type = xml_type->Name();
  if (str_type == "box") {
    geometry.type = Graphics::Geometry::Type::kBox;
    geometry.scale = ctrl_utils::DecodeMatlab<Eigen::Vector3d>(ParseAttribute(xml_type, "size"));
  } else if (str_type == "cylinder") {
    geometry.type = Graphics::Geometry::Type::kCylinder;
    geometry.radius = ParseDoubleAttribute(xml_type, "radius");
    geometry.length = ParseDoubleAttribute(xml_type, "length");
  } else if (str_type == "sphere") {
    geometry.type = Graphics::Geometry::Type::kSphere;
    geometry.radius = ParseDoubleAttribute(xml_type, "radius");
  } else if (str_type == "mesh") {
    geometry.type = Graphics::Geometry::Type::kMesh;
    geometry.mesh = (std::filesystem::path(path_meshes) / ParseAttribute(xml_type, "filename")).string();

    const char* attr_scale = xml_type->Attribute("scale");
    if (attr_scale != nullptr) {
      geometry.scale = ctrl_utils::DecodeMatlab<Eigen::Vector3d>(std::string(attr_scale));
    } else {
      geometry.scale.setOnes();
    }
  } else {
    throw std::runtime_error("spatial_dyn::urdf::ParseGraphics(): <box>|<cylinder>|<sphere>|<mesh> element missing from <geometry> element. Found <" + str_type + "> instead.");
  }

  // Parse material
  Graphics::Material& material = graphics.material;
  const tinyxml2::XMLElement* xml_material = xml_geometry->FirstChildElement("material");
  if (xml_material != nullptr) {
    material.name = ParseAttribute(xml_material, "name");

    const tinyxml2::XMLElement* xml_color = xml_material->FirstChildElement("color");
    if (xml_color != nullptr) {
      std::string str_rgba = ParseAttribute(xml_color, "rgba");
      material.rgba = ctrl_utils::DecodeMatlab<Eigen::Vector4d>(str_rgba);
    }

    const tinyxml2::XMLElement* xml_texture = xml_material->FirstChildElement("texture");
    if (xml_texture != nullptr) {
      material.texture = ParseAttribute(xml_texture, "filename");
    }
  }

  return graphics;
}

RigidBody ParseRigidBody(const tinyxml2::XMLElement* xml_link, const std::string& path_meshes) {
  std::string attr_name = ParseAttribute(xml_link, "name");
  RigidBody rb(attr_name);

  // Parse inertial
  const tinyxml2::XMLElement* xml_inertial = xml_link->FirstChildElement("inertial");
  if (xml_inertial != nullptr) {
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> com_R = ParseOriginElement(xml_inertial);
    Eigen::Vector3d com = com_R.first;
    Eigen::Matrix3d R = com_R.second.toRotationMatrix();

    const tinyxml2::XMLElement* xml_mass = ParseElement(xml_inertial, "mass");
    double mass = ParseDoubleAttribute(xml_mass, "value");

    const tinyxml2::XMLElement* xml_inertia = ParseElement(xml_inertial, "inertia");
    Eigen::Vector6d I_com_flat(ParseDoubleAttribute(xml_inertia, "ixx"),
                               ParseDoubleAttribute(xml_inertia, "iyy"),
                               ParseDoubleAttribute(xml_inertia, "izz"),
                               ParseDoubleAttribute(xml_inertia, "ixy"),
                               ParseDoubleAttribute(xml_inertia, "ixz"),
                               ParseDoubleAttribute(xml_inertia, "iyz"));
    Eigen::Matrix3d I_com;
    I_com << I_com_flat(0), I_com_flat(3), I_com_flat(4),
             I_com_flat(3), I_com_flat(1), I_com_flat(5),
             I_com_flat(4), I_com_flat(5), I_com_flat(2);
    I_com = R.transpose() * I_com * R;  // TODO: Check rotation order
    I_com_flat << I_com(0,0), I_com(1,1), I_com(2,2), I_com(0,1), I_com(0,2), I_com(1,2);

    rb.set_inertia(mass, com, I_com_flat);
  }

  // Parse graphics
  const tinyxml2::XMLElement* xml_visual = xml_link->FirstChildElement("visual");
  while (xml_visual != nullptr) {
    rb.graphics.push_back(ParseGraphics(xml_visual, path_meshes));
    xml_visual = xml_visual->NextSiblingElement("visual");
  }

  // Parse collision
  const tinyxml2::XMLElement* xml_collision = xml_link->FirstChildElement("collision");
  if (xml_collision != nullptr) {
    // TODO
  }

  return rb;
}

void AddRigidBody(ArticulatedBody& ab,
                  std::map<std::string, std::pair<RigidBody, std::string>>& rigid_bodies,
                  std::list<std::string>& rb_list,
                  std::map<std::string, int>& rb_ids,
                  const std::string& name_rb) {
  auto it_rb_list = rb_list.begin();
  for ( ; it_rb_list != rb_list.end(); ++it_rb_list) {
    if (*it_rb_list == name_rb) break;
  }

  RigidBody& rb = rigid_bodies.at(name_rb).first;
  const std::string& name_parent = rigid_bodies.at(name_rb).second;

  int id_new;
  if (name_parent.empty()) {
    ab.set_T_base_to_world(rb.T_to_parent());
    ab.set_inertia_base(rb.inertia());
    ab.graphics = rb.graphics;
    id_new = -1;
  } else {
    const int id_parent = rb_ids.at(name_parent);
    id_new = ab.AddRigidBody(std::move(rb), id_parent);
  }

  rb_ids[name_rb] = id_new;
  rb_list.erase(it_rb_list);
}

void AddChildrenDfs(const std::map<std::string, std::pair<RigidBody, std::string>>& rigid_bodies,
                    const std::list<std::string>& unsorted,
                    const std::list<std::string>::const_iterator& it,
                    std::list<std::string>* output) {
  output->push_back(*it);
  for (auto it_child = unsorted.begin(); it_child != unsorted.end(); ++it_child) {
    if (rigid_bodies.at(*it_child).second != *it) continue;
    AddChildrenDfs(rigid_bodies, unsorted, it_child, output);
  }
}

ArticulatedBody LoadModel(const std::string& path_urdf, const std::string& path_meshes,
                          bool simplify) {
  // Open xml document
  tinyxml2::XMLDocument doc;
  doc.LoadFile(path_urdf.c_str());
  if (doc.Error()) {
    throw std::runtime_error("spatial_dyn::urdf::LoadModel(): Unable to parse " + path_urdf +
                             " - " + std::string(doc.ErrorName()));
  }

  // Parse robot
  const tinyxml2::XMLElement* xml_robot = ParseElement(doc, "robot");
  std::string robot_name = ParseAttribute(xml_robot, "name");
  ArticulatedBody ab(robot_name);

  // Parse links
  std::map<std::string, std::pair<RigidBody, std::string>> rigid_bodies;
  std::list<std::string> rb_list;
  const tinyxml2::XMLElement* xml_link = ParseElement(xml_robot, "link");
  while (xml_link != nullptr) {
    RigidBody rb = ParseRigidBody(xml_link, path_meshes);
    if (rigid_bodies.find(rb.name) != rigid_bodies.end()) {
      throw std::runtime_error("spatial_dyn::urdf::LoadModel(): Multiple <link> elements with the name " +
                               rb.name + " cannot exist.");
    }
    std::string rb_name = rb.name;
    rigid_bodies[rb_name] = std::make_pair(std::move(rb), "");
    rb_list.push_back(rb_name);
    xml_link = xml_link->NextSiblingElement("link");
  }

  // Parse joints
  const tinyxml2::XMLElement* xml_joint = ParseElement(xml_robot, "joint");
  while (xml_joint != nullptr) {
    // Parse parent
    const tinyxml2::XMLElement* xml_parent = ParseElement(xml_joint, "parent");
    std::string parent_link = ParseAttribute(xml_parent, "link");
    if (rigid_bodies.find(parent_link) == rigid_bodies.end()) {
      throw std::runtime_error("spatial_dyn::urdf::LoadModel(): Parent link (" + parent_link + ") does not exist.");
    }

    // Parse child
    const tinyxml2::XMLElement* xml_child = xml_joint->FirstChildElement("child");
    std::string child_link = ParseAttribute(xml_child, "link");
    if (rigid_bodies.find(child_link) == rigid_bodies.end()) {
      throw std::runtime_error("spatial_dyn::urdf::LoadModel(): Child link (" + child_link + ") does not exist.");
    }

    // Parse origin
    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
    std::tie(pos, ori) = ParseOriginElement(xml_joint);

    // Parse axis
    int axis_num = 0;  // 0: x, 1: y, 2: z
    const tinyxml2::XMLElement* xml_axis = xml_joint->FirstChildElement("axis");
    if (xml_axis != nullptr) {
      // TODO: Support arbitrary axes
      Eigen::Vector3d axis = ctrl_utils::DecodeMatlab<Eigen::Vector3d>(ParseAttribute(xml_axis, "xyz")).normalized();
      if (axis(1) > 0.8) {
        axis_num = 1;
      } else if (axis(2) > 0.8) {
        axis_num = 2;
      }
    }

    // Parse dynamics
    double f_coulomb = 0., f_stiction = 0., f_viscous = 0.;
    const tinyxml2::XMLElement* xml_dynamics = xml_joint->FirstChildElement("dynamics");
    if (xml_dynamics != nullptr) {
      xml_dynamics->QueryDoubleAttribute("damping", &f_viscous);
      xml_dynamics->QueryDoubleAttribute("friction", &f_stiction);
      xml_dynamics->QueryDoubleAttribute("coulomb", &f_coulomb);
    }

    // Parse limits
    double q_min = -std::numeric_limits<double>::infinity(),
           q_max = std::numeric_limits<double>::infinity(),
           dq_max = std::numeric_limits<double>::infinity(),
           fq_max = std::numeric_limits<double>::infinity();
    const tinyxml2::XMLElement* xml_limit = xml_joint->FirstChildElement("limit");
    if (xml_limit != nullptr) {
      xml_limit->QueryDoubleAttribute("lower", &q_min);
      xml_limit->QueryDoubleAttribute("upper", &q_max);
      xml_limit->QueryDoubleAttribute("velocity", &dq_max);
      xml_limit->QueryDoubleAttribute("effort", &fq_max);
    }

    // Populate rigid body fields
    RigidBody& rb = rigid_bodies.at(child_link).first;
    std::string& name_parent = rigid_bodies.at(child_link).second;

    std::string attr_name = ParseAttribute(xml_joint, "name");  // TODO: Add to joint
    std::string attr_type = ParseAttribute(xml_joint, "type");
    if (attr_type == "revolute" || attr_type == "continuous" || attr_type == "prismatic") {
      Joint::Type type;
      if (attr_type == "prismatic") {
        switch (axis_num) {
          case 0: type = Joint::Type::kPx; break;
          case 1: type = Joint::Type::kPy; break;
          case 2: type = Joint::Type::kPz; break;
          default: type = Joint::Type::kUndefined; break;
        }
      } else {
        switch (axis_num) {
          case 0: type = Joint::Type::kRx; break;
          case 1: type = Joint::Type::kRy; break;
          case 2: type = Joint::Type::kRz; break;
          default: type = Joint::Type::kUndefined; break;
        }
      }

      Joint joint(type);
      if (attr_type != "continuous") {
        joint.set_q_limits(q_min, q_max);
      }
      joint.set_dq_max(dq_max);
      joint.set_fq_max(fq_max);
      joint.set_f_coulomb(f_coulomb);
      joint.set_f_stiction(f_stiction);
      joint.set_f_viscous(f_viscous);

      rb.set_joint(std::move(joint));
      rb.set_T_to_parent(ori, pos);
      name_parent = parent_link;
    } else if (attr_type == "fixed") {
      rb.set_joint(Joint::Type::kUndefined);
      rb.set_T_to_parent(ori, pos);
      name_parent = parent_link;
    } else if (attr_type == "floating") {
      throw std::runtime_error("urdf::LoadModel(): Floating joint not implemented yet.");
      // TODO
    } else if (attr_type == "planar") {
      throw std::runtime_error("urdf::LoadModel(): Planar joint not implemented yet.");
      // TODO
    }

    xml_joint = xml_joint->NextSiblingElement("joint");
  }

  // Sort rigid bodies so parents come first
  std::list<std::string> unsorted_rbs = std::move(rb_list);
  auto it_root = unsorted_rbs.begin();
  for ( ; it_root != unsorted_rbs.end(); ++it_root) {
    const std::string& name_parent = rigid_bodies.at(*it_root).second;
    if (name_parent.empty()) break;
  }
  AddChildrenDfs(rigid_bodies, unsorted_rbs, it_root, &rb_list);

  // Merge fixed joints
  if (simplify) {
    for (auto it = rb_list.begin(); it != rb_list.end(); ) {
      if (it == rb_list.begin()) { ++it; continue; }

      RigidBody& rb = rigid_bodies.at(*it).first;
      if (rb.joint().type() != Joint::Type::kUndefined) { ++it; continue; }

      const std::string& name_parent = rigid_bodies.at(*it).second;
      if (name_parent.empty()) {
        throw std::runtime_error("TODO: Merge with articulated body base.");
      }
      RigidBody& rb_parent = rigid_bodies.at(name_parent).first;

      // Merge graphics
      rb_parent.graphics.reserve(rb_parent.graphics.size() + rb.graphics.size());
      for (Graphics& g : rb.graphics) {
        g.T_to_parent = rb.T_to_parent() * g.T_to_parent;
        rb_parent.graphics.push_back(std::move(g));
      }

      // Merge inertia
      const SpatialInertiad inertia = rb_parent.inertia() + rb.T_to_parent() * rb.inertia();
      rb_parent.set_inertia(inertia);

      // Transfer children
      for (auto it_child = it; it_child != rb_list.end(); ++it_child) {
        if (it_child == it) continue;
        RigidBody& rb_child = rigid_bodies.at(*it_child).first;
        std::string& child_name_parent = rigid_bodies.at(*it_child).second;
        if (child_name_parent != rb.name) continue;

        // Merge transform
        const Eigen::Isometry3d T_to_parent = rb.T_to_parent() * rb_child.T_to_parent();
        rb_child.set_T_to_parent(T_to_parent);

        // Set parent
        child_name_parent = name_parent;
      }

      auto it_old = it;
      ++it;
      rb_list.erase(it_old);
    }
  }

  // Add rigid bodies to articulated body
  std::map<std::string, int> rb_ids;
  for (auto it = rb_list.begin(); it != rb_list.end(); ) {
    const std::string& name_rb = *it;
    AddRigidBody(ab, rigid_bodies, rb_list, rb_ids, name_rb);
    it = rb_list.begin();  // Restart after list element has been removed
  }

  return ab;
}

}  // namespace urdf
}  // namespace spatial_dyn
