/**
 * spatial_dyn.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 7, 2018
 * Authors: Toki Migimatsu
 */

#include "SpatialDyn/algorithms/forward_dynamics.h"
#include "SpatialDyn/algorithms/forward_kinematics.h"
#include "SpatialDyn/algorithms/inverse_dynamics.h"
#include "SpatialDyn/algorithms/simulation.h"
#include "SpatialDyn/parsers/urdf.h"
#include "SpatialDyn/parsers/json.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace SpatialDyn {

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(spatialdyn, m) {

  // Articulated body
  py::class_<ArticulatedBody>(m, "ArticulatedBody")
      .def_readwrite("name", &ArticulatedBody::name)
      .def_property_readonly("dof", &ArticulatedBody::dof)
      .def_property("T_base_to_world", &ArticulatedBody::T_base_to_world,
                    (void (ArticulatedBody::*)(const Eigen::Isometry3d&)) &ArticulatedBody::set_T_base_to_world)
      .def("AddRigidBody", (int (ArticulatedBody::*)(const RigidBody&, int)) &ArticulatedBody::AddRigidBody, "rb"_a, "id_parent"_a = -1)
      .def_property_readonly("rigid_bodies", (const std::vector<RigidBody>& (ArticulatedBody::*)(void) const) &ArticulatedBody::rigid_bodies)
      .def("ancestors", &ArticulatedBody::ancestors)
      .def_property("q",
                    (const Eigen::VectorXd& (ArticulatedBody::*)(void) const) &ArticulatedBody::q,
                    (void (ArticulatedBody::*)(const Eigen::VectorXd&)) &ArticulatedBody::set_q)
      .def_property("dq",
                    (const Eigen::VectorXd& (ArticulatedBody::*)(void) const) &ArticulatedBody::dq,
                    (void (ArticulatedBody::*)(const Eigen::VectorXd&)) &ArticulatedBody::set_dq)
      .def_property("ddq",
                    (const Eigen::VectorXd& (ArticulatedBody::*)(void) const) &ArticulatedBody::ddq,
                    (void (ArticulatedBody::*)(const Eigen::VectorXd&)) &ArticulatedBody::set_ddq)
      .def_property("tau",
                    (const Eigen::VectorXd& (ArticulatedBody::*)(void) const) &ArticulatedBody::tau,
                    (void (ArticulatedBody::*)(const Eigen::VectorXd&)) &ArticulatedBody::set_tau)
      .def_property("g",
                    // TODO: Make spatial
                    [](const ArticulatedBody& ab) { return ab.g().linear(); },
                    &ArticulatedBody::set_g)
      .def("T_to_parent", &ArticulatedBody::T_to_parent)
      .def("T_from_parent", &ArticulatedBody::T_from_parent)
      .def("T_to_world", &ArticulatedBody::T_to_world)
      .def("subtree", &ArticulatedBody::subtree)
      .def("__repr__",
           [](const ArticulatedBody& ab) {
             return "<spatialdyn.ArticulatedBody (name=" + ab.name + ")>";
           })
      .def("__str__",
           [](const ArticulatedBody& ab) {
             return SpatialDyn::Json::Serialize(ab).dump();
           });

  // Rigid body
  py::class_<RigidBody>(m, "RigidBody")
      .def_readwrite("name", &RigidBody::name)
      .def_readwrite("graphics", &RigidBody::graphics)
      .def_property_readonly("id", &RigidBody::id)
      .def_property_readonly("id_parent", &RigidBody::id_parent)
      .def_property("T_to_parent", &RigidBody::T_to_parent,
                    (void (RigidBody::*)(const Eigen::Isometry3d&)) &RigidBody::set_T_to_parent)
      .def_property("inertia", &RigidBody::inertia,
                    (void (RigidBody::*)(const SpatialInertiad&)) &RigidBody::set_inertia)
      .def_property("joint", &RigidBody::joint,
                    (void (RigidBody::*)(const Joint&)) &RigidBody::set_joint);

  // Graphics
  py::class_<Graphics>(m, "Graphics")
      .def_readwrite("name", &Graphics::name)
      .def_readwrite("T_to_parent", &Graphics::T_to_parent)
      .def_readwrite("geometry", &Graphics::geometry)
      .def_readwrite("material", &Graphics::material);

  // Geometry
  py::class_<Geometry>(m, "Geometry")
      .def_property("type",
                    [](const Geometry& geometry) {
                      return std::string(geometry);
                    },
                    [](Geometry& geometry, const std::string& type) {
                      geometry.type = Geometry::FromString(type);
                    })
      .def_readwrite("scale", &Geometry::scale)
      .def_readwrite("radius", &Geometry::radius)
      .def_readwrite("length", &Geometry::length)
      .def_readwrite("mesh", &Geometry::mesh);

  // Material
  py::class_<Material>(m, "Material")
      .def_readwrite("name", &Material::name)
      .def_readwrite("rgba", &Material::rgba)
      .def_readwrite("texture", &Material::texture);

  // Joint
  py::class_<Joint>(m, "Joint")
      .def_property("type",
                    [](const Joint& joint) {
                      return std::string(joint);
                    },
                    [](Joint& joint, const std::string& type) {
                      joint.set_type(Joint::FromString(type));
                    })
      .def_property("q_min", &Joint::q_min, &Joint::set_q_min)
      .def_property("q_max", &Joint::q_max, &Joint::set_q_max)
      .def_property("dq_max", &Joint::dq_max, &Joint::set_dq_max)
      .def_property("fq_max", &Joint::fq_max, &Joint::set_fq_max)
      .def_property("f_coulomb", &Joint::f_coulomb, &Joint::set_f_coulomb)
      .def_property("f_stiction", &Joint::f_stiction, &Joint::set_f_stiction)
      .def("T_joint", &Joint::T_joint)
      .def("__repr__",
           [](const Joint& joint) {
             return "<spatialdyn.Joint (type=" + std::string(joint) + ")>";
           });

  // Forward dynamics
  m.def("forward_dynamics", &ForwardDynamics, "ab"_a, "tau"_a,
        "f_external"_a = std::vector<std::pair<int, SpatialForced>>());
  m.def("forward_dynamics_aba", &ForwardDynamicsAba, "ab"_a, "tau"_a,
        "f_external"_a = std::vector<std::pair<int, SpatialForced>>());

  // Forward kinematics
  m.def("position", &Position, "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero());
  m.def("orientation", &Orientation, "ab"_a, "link"_a = -1);
  m.def("jacobian", &Jacobian, "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero());
  m.def("linear_jacobian", &LinearJacobian, "ab"_a, "link"_a = -1,
        "offset"_a = Eigen::Vector3d::Zero());
  m.def("angular_jacobian", &AngularJacobian, "ab"_a, "link"_a = -1);

  // Inverse dynamics
  m.def("inverse_dynamics", &InverseDynamics, "ab"_a, "ddq"_a, "gravity"_a = true,
        "centrifugal_coriolis"_a = false, "f_external"_a = std::vector<std::pair<int, SpatialForced>>());
  m.def("centrifugal_coriolis", &CentrifugalCoriolis, "ab"_a);
  m.def("gravity", &Gravity, "ab"_a, "f_external"_a = std::vector<std::pair<int, SpatialForced>>());
  m.def("inertia", &Inertia, "ab"_a);
  m.def("inertia_inverse", &InertiaInverse, "ab"_a);

  // Simulation
  m.def("integrate", &Integrate, "ab"_a, "tau"_a, "dt"_a);

  // Spatial inertia
  py::class_<SpatialInertiad>(m, "SpatialInertiad")
      .def_readwrite("mass", &SpatialInertiad::mass)
      .def_readwrite("com", &SpatialInertiad::com)
      .def_readwrite("I_com", &SpatialInertiad::I_com)
      .def("__repr__",
           [](const SpatialInertiad& inertia) {
             return "<spatialdyn.SpatialInertiad (mass=" + std::to_string(inertia.mass) +
                    ", com=[" + inertia.com.toMatlab() + "], I_com=[" +
                    inertia.I_com_flat().toMatlab() + "])>";
           });

  // Opspace dynamics
  py::module m_op = m.def_submodule("opspace");
  m_op.def("orientation_error", &Opspace::OrientationError, "quat"_a, "quat_des"_a);
  m_op.def("inverse_dynamics",
           [](const ArticulatedBody& ab, const Eigen::MatrixXd& J,
              const Eigen::VectorXd& ddx, py::EigenDRef<Eigen::MatrixXd> N,
              double svd_epsilon, bool gravity, bool centrifugal_coriolis) {
             Eigen::MatrixXd N_temp = N;
             Eigen::VectorXd tau = Opspace::InverseDynamics(ab, J, ddx, &N_temp, svd_epsilon, gravity, centrifugal_coriolis);
             N = N_temp;
             return tau;
           }, "ab"_a, "J"_a, "ddx"_a, "N"_a, "svd_epsilon"_a = 0,
           "gravity"_a = false, "centrifugal_coriolis"_a = false);
  m_op.def("inertia", &Opspace::Inertia, "ab"_a, "J"_a, "svd_epsilon"_a = 0);
  m_op.def("inertia_inverse", &Opspace::InertiaInverse, "ab"_a, "J"_a);
  m_op.def("jacobian_dynamic_inverse", &Opspace::JacobianDynamicInverse, "ab"_a, "J"_a,
           "svd_epsilon"_a = 0);
  m_op.def("centrifugal_coriolis", &Opspace::CentrifugalCoriolis, "ab"_a, "J"_a,
           "idx_link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(), "svd_epsilon"_a = 0);
  m_op.def("gravity", &Opspace::Gravity, "ab"_a, "J"_a,
           "f_external"_a = std::vector<std::pair<int, SpatialForced>>(), "svd_epsilon"_a = 0);

  m_op.def("inertia_aba", &Opspace::InertiaAba, "ab"_a, "idx_link"_a = -1,
           "offset"_a = Eigen::Vector3d::Zero(), "svd_epsilon"_a = 0);
  m_op.def("inertia_inverse_aba", &Opspace::InertiaInverseAba, "ab"_a, "idx_link"_a = -1,
           "offset"_a = Eigen::Vector3d::Zero());
  m_op.def("centrifugal_coriolis_aba", &Opspace::CentrifugalCoriolisAba, "ab"_a,
           "idx_link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(), "svd_epsilon"_a = 0);
  m_op.def("gravity_aba", &Opspace::GravityAba, "ab"_a, "idx_link"_a = -1,
           "offset"_a = Eigen::Vector3d::Zero(),
           "f_external"_a = std::vector<std::pair<int, SpatialForced>>(), "svd_epsilon"_a = 0);

  // Urdf parser
  py::module m_urdf = m.def_submodule("urdf");
  m_urdf.def("load_model", &Urdf::LoadModel, "urdf"_a);

}

}  // namespace SpatialDyn
