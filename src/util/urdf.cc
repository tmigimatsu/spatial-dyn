/**
 * urdf.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 5, 2018
 * Authors: Toki Migimatsu
 */

#include "util/urdf.h"

#include "tinyxml2.h"

#include <exception>  // std::runtime_error
#include <list>       // std::list
#include <map>        // std::map
#include <utility>    // std::pair

namespace SpatialDyn {
namespace Urdf {

const tinyxml2::XMLElement* ParseElement(const tinyxml2::XMLDocument& xml_doc, const std::string& element) {
  const tinyxml2::XMLElement* xml_element = xml_doc.FirstChildElement(element.c_str());
  if (xml_element == nullptr) {
    throw std::runtime_error("ParseElement(): <" + element + "> element missing from document.");
  }
  return xml_element;
}
const tinyxml2::XMLElement* ParseElement(const tinyxml2::XMLElement* xml_parent, const std::string& element) {
  const tinyxml2::XMLElement* xml_element = xml_parent->FirstChildElement(element.c_str());
  if (xml_element == nullptr) {
    throw std::runtime_error("ParseElement(): <" + element +
                             "> element missing from <" + xml_parent->Name() + "> element.");
  }
  return xml_element;
}

std::string ParseAttribute(const tinyxml2::XMLElement* xml_element, const std::string& attribute) {
  const char *value = xml_element->Attribute(attribute.c_str());
  if (value == nullptr) {
    throw std::runtime_error("ParseAttribute(): \"link\" attribute missing from <" +
                             std::string(xml_element->Name()) + "> element.");
  }
  return std::string(value);
}

double ParseDoubleAttribute(const tinyxml2::XMLElement* xml_element, const std::string& attribute) {
  double value;
  tinyxml2::XMLError status = xml_element->QueryDoubleAttribute(attribute.c_str(), &value);
  if (status != tinyxml2::XMLError::XML_SUCCESS) {
    throw std::runtime_error("ParseDoubleAttribute(): Invalid \"" + attribute +
                            "\" attribute in <" + std::string(xml_element->Name()) +
                            "> element - " +
                            std::string(tinyxml2::XMLDocument::ErrorIDToName(status)) + ".");
  }
  return value;
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond>
ParseOrigin(const tinyxml2::XMLElement* xml_origin) {
  Eigen::Vector3d pos;
  Eigen::Quaterniond ori;
  const char* attr_xyz = xml_origin->Attribute("xyz");
  if (attr_xyz != nullptr) {
    pos = Eigen::Vector3d::FromMatlab(attr_xyz);
  } else {
    pos.setZero();
  }

  const char* attr_rpy = xml_origin->Attribute("rpy");
  if (attr_rpy != nullptr) {
    Eigen::Vector3d rpy = Eigen::Vector3d::FromMatlab(std::string(attr_rpy));
    // TODO: Check Euler angles
    ori = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
  } else {
    ori.setIdentity();
  }
  return std::make_pair(std::move(pos), std::move(ori));
}

RigidBody ParseRigidBody(const tinyxml2::XMLElement* xml_link) {
  std::string attr_name = ParseAttribute(xml_link, "name");
  RigidBody rb(attr_name);

  const tinyxml2::XMLElement* xml_inertial = xml_link->FirstChildElement("inertial");
  if (xml_inertial != nullptr) {
    Eigen::Vector3d com;
    Eigen::Matrix3d R;
    const tinyxml2::XMLElement* xml_origin = xml_inertial->FirstChildElement("origin");
    if (xml_origin != nullptr) {
      std::pair<Eigen::Vector3d, Eigen::Quaterniond> com_R = ParseOrigin(xml_origin);
      com = com_R.first;
      R = com_R.second.toRotationMatrix();
    }

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

  const tinyxml2::XMLElement* xml_visual = xml_link->FirstChildElement("visual");
  if (xml_visual != nullptr) {

  }

  const tinyxml2::XMLElement* xml_collision = xml_link->FirstChildElement("collision");
  if (xml_collision != nullptr) {

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

  RigidBody& rb                  = rigid_bodies.at(name_rb).first;
  const std::string& name_parent = rigid_bodies.at(name_rb).second;

  int id_new;
  if (name_parent.empty()) {
    id_new = ab.AddRigidBody(std::move(rb));
  } else {
    if (rb_ids.find(name_parent) == rb_ids.end()) {
      AddRigidBody(ab, rigid_bodies, rb_list, rb_ids, name_parent);
    }
    int id_parent = rb_ids.at(name_parent);
    id_new = ab.AddRigidBody(std::move(rb), id_parent);
  }

  rb_ids[name_rb] = id_new;
  rb_list.erase(it_rb_list);
}

ArticulatedBody ParseModel(const std::string& urdf) {
  // Open xml document
  tinyxml2::XMLDocument doc;
  doc.LoadFile(urdf.c_str());
  if (doc.Error()) {
    throw std::runtime_error("ParseModel(): Unable to parse " + urdf + " - " + std::string(doc.ErrorName()));
  }

  // Parse robot element
  const tinyxml2::XMLElement* xml_robot = ParseElement(doc, "robot");
  std::string robot_name = ParseAttribute(xml_robot, "name");
  ArticulatedBody ab(robot_name);

  // Parse link elements
  std::map<std::string, std::pair<RigidBody, std::string>> rigid_bodies;
  std::list<std::string> rb_list;
  const tinyxml2::XMLElement* xml_link = ParseElement(xml_robot, "link");
  while (xml_link != nullptr) {
    RigidBody rb = ParseRigidBody(xml_link);
    if (rigid_bodies.find(rb.name) != rigid_bodies.end()) {
      throw std::runtime_error("ParseModel(): Multiple <link> elements with the name " +
                               rb.name + " cannot exist.");
    }
    std::string rb_name = rb.name;
    rigid_bodies[rb_name] = std::make_pair(std::move(rb), std::string());
    rb_list.push_back(rb_name);
    xml_link = xml_link->NextSiblingElement("link");
  }


  const tinyxml2::XMLElement* xml_joint = ParseElement(xml_robot, "joint");
  while (xml_joint != nullptr) {
    // Parent
    const tinyxml2::XMLElement* xml_parent = ParseElement(xml_joint, "parent");
    std::string parent_link = ParseAttribute(xml_parent, "link");
    if (rigid_bodies.find(parent_link) == rigid_bodies.end()) {
      throw std::runtime_error("ParseModel(): Parent link (" + parent_link + ") does not exist.");
    }

    // Child
    const tinyxml2::XMLElement* xml_child = xml_joint->FirstChildElement("child");
    std::string child_link = ParseAttribute(xml_child, "link");
    if (rigid_bodies.find(child_link) == rigid_bodies.end()) {
      throw std::runtime_error("ParseModel(): Child link (" + child_link + ") does not exist.");
    }

    // Origin
    const tinyxml2::XMLElement* xml_origin = xml_joint->FirstChildElement("origin");
    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
    if (xml_origin != nullptr) {
      std::tie(pos, ori) = ParseOrigin(xml_origin);
    } else {
      pos.setZero();
      ori.setIdentity();
    }

    // Axis
    int axis_num = 0;  // 0: x, 1: y, 2: z
    const tinyxml2::XMLElement* xml_axis = xml_joint->FirstChildElement("axis");
    if (xml_axis != nullptr) {
      // TODO: Support arbitrary axes
      Eigen::Vector3d axis = Eigen::Vector3d::FromMatlab(ParseAttribute(xml_axis, "xyz")).normalized();
      if (axis(1) > 0.8) {
        axis_num = 1;
      } else if (axis(2) > 0.8) {
        axis_num = 2;
      }
    }

    RigidBody& rb = rigid_bodies.at(child_link).first;
    std::string& name_parent = rigid_bodies.at(child_link).second;

    std::string attr_name = ParseAttribute(xml_joint, "name");  // TODO: Add to joint
    std::string attr_type = ParseAttribute(xml_joint, "type");
    if (attr_type == "revolute" || attr_type == "continuous" || attr_type == "prismatic") {
      JointType type;
      if (attr_type == "prismatic") {
        switch (axis_num) {
          case 0: type = JointType::PX; break;
          case 1: type = JointType::PY; break;
          case 2: type = JointType::PZ; break;
          default: type = JointType::UNDEFINED; break;
        }
      } else {
        switch (axis_num) {
          case 0: type = JointType::RX; break;
          case 1: type = JointType::RY; break;
          case 2: type = JointType::RZ; break;
          default: type = JointType::UNDEFINED; break;
        }
      }
      Joint joint(type);
      rb.set_joint(std::move(joint));
      rb.set_T_to_parent(ori, pos);
      name_parent = parent_link;
    } else if (attr_type == "fixed") {
      // TODO
    } else if (attr_type == "floating") {
      // TODO
    } else if (attr_type == "planar") {
      // TODO
    }

    xml_joint = xml_joint->NextSiblingElement("joint");
  }

  std::map<std::string, int> rb_ids;
  for (auto it = rb_list.begin(); it != rb_list.end(); ) {
    const std::string& name_rb = *it;
    AddRigidBody(ab, rigid_bodies, rb_list, rb_ids, name_rb);
    it = rb_list.begin();  // Restart after list element has been removed
  }

  return ab;
}

}  // namespace Urdf
}  // namespace SpatialDyn
