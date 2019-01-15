/**
 * spatial_dyn.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 7, 2018
 * Authors: Toki Migimatsu
 */

#include <exception>  // std::invalid_argument
#include <sstream>    // std::stringstream

#include <pybind11/pybind11.h>
#include "spatial_dyn/utils/spatial_math.h"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <ctrl_utils/eigen_string.h>

#include "spatial_dyn/algorithms/forward_dynamics.h"
#include "spatial_dyn/algorithms/forward_kinematics.h"
#include "spatial_dyn/algorithms/inverse_dynamics.h"
#include "spatial_dyn/algorithms/opspace_dynamics.h"
#include "spatial_dyn/algorithms/simulation.h"
#include "spatial_dyn/parsers/urdf.h"
#include "spatial_dyn/parsers/json.h"

namespace spatial_dyn {

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(spatialdyn, m) {

  // Articulated body
  py::class_<ArticulatedBody>(m, "ArticulatedBody")
      .def(py::init<>())
      .def(py::init<const std::string&>())
      .def(py::init<const ArticulatedBody&>())
      .def_readwrite("name", &ArticulatedBody::name)
      .def_readwrite("graphics", &ArticulatedBody::graphics)
      .def_property_readonly("dof", &ArticulatedBody::dof)
      .def("add_rigid_body", &ArticulatedBody::AddRigidBody, "rb"_a, "id_parent"_a = -1)
      .def_property_readonly("rigid_bodies", (const std::vector<RigidBody>& (ArticulatedBody::*)(void) const) &ArticulatedBody::rigid_bodies)
      .def_property("q",
                    (const Eigen::VectorXd& (ArticulatedBody::*)(void) const) &ArticulatedBody::q,
                    &ArticulatedBody::set_q)
      .def_property("dq",
                    (const Eigen::VectorXd& (ArticulatedBody::*)(void) const) &ArticulatedBody::dq,
                    &ArticulatedBody::set_dq)
      .def_property("g",
                    // TODO: Make spatial
                    [](const ArticulatedBody& ab) { return ab.g().linear(); },
                    &ArticulatedBody::set_g)
      .def("add_load", &ArticulatedBody::AddLoad, "inertia"_a, "idx_link"_a = -1)
      .def("replace_load", &ArticulatedBody::ReplaceLoad, "inertia"_a, "idx_link"_a = -1)
      .def("clear_load", &ArticulatedBody::ClearLoad, "idx_link"_a = -1)
      .def_property_readonly("inertia_load", &ArticulatedBody::inertia_load)
      .def_property("T_base_to_world", &ArticulatedBody::T_base_to_world,
                    (void (ArticulatedBody::*)(const Eigen::Isometry3d&)) &ArticulatedBody::set_T_base_to_world)
      .def("T_to_parent",
           [](const ArticulatedBody& ab, int i, py::object q) -> Eigen::Isometry3d {
             return q.is_none() ? ab.T_to_parent(i) : ab.T_to_parent(i, q.cast<double>());
           }, "i"_a, "q"_a = py::none())
      .def("T_from_parent",
           [](const ArticulatedBody& ab, int i, py::object q) -> Eigen::Isometry3d {
             return q.is_none() ? ab.T_from_parent(i) : ab.T_from_parent(i, q.cast<double>());
           }, "i"_a, "q"_a = py::none())
      .def("T_to_world",
           [](const ArticulatedBody& ab, int i, py::object q) -> Eigen::Isometry3d {
             return q.is_none() ? ab.T_to_world(i)
                                : ab.T_to_world(i, q.cast<Eigen::Ref<const Eigen::VectorXd>>());
           }, "i"_a, "q"_a = py::none())
      .def("T_from_world",
           [](const ArticulatedBody& ab, int i, py::object q) -> Eigen::Isometry3d {
             return q.is_none() ? ab.T_from_world(i)
                                : ab.T_from_world(i, q.cast<Eigen::Ref<const Eigen::VectorXd>>());
           }, "i"_a, "q"_a = py::none())
      .def("ancestors", &ArticulatedBody::ancestors)
      .def("subtree", &ArticulatedBody::subtree)
      .def("map", &ArticulatedBody::Map)
      .def("__str__",
           [](const ArticulatedBody& ab) {
             return spatial_dyn::json::Serialize(ab).dump();
           })
      .def("__repr__",
           [](const ArticulatedBody& ab) {
             std::stringstream ss;
             ss << "spatialdyn." << ab;
             return ss.str();
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
      .def_property("joint", &RigidBody::joint, &RigidBody::set_joint)
      .def("__str__",
           [](const RigidBody& rb) {
             return spatial_dyn::json::Serialize(rb).dump();
           })
      .def("__repr__",
           [](const RigidBody& rb) {
             std::stringstream ss;
             ss << "spatialdyn." << rb;
             return ss.str();
           });

  // Joint
  py::class_<Joint>(m, "Joint")
      .def_property("type",
                    [](const Joint& joint) {
                      return std::string(joint);
                    },
                    [](Joint& joint, const std::string& type) {
                      joint.set_type(Joint::StringToType(type));
                    })
      .def_property_readonly("is_prismatic", &Joint::is_prismatic)
      .def_property_readonly("is_revolute", &Joint::is_revolute)
      // TODO: subspace
      .def_property("q_min", &Joint::q_min, &Joint::set_q_min)
      .def_property("q_max", &Joint::q_max, &Joint::set_q_max)
      .def("set_q_limits", &Joint::set_q_limits)
      .def_property("dq_max", &Joint::dq_max, &Joint::set_dq_max)
      .def_property("fq_max", &Joint::fq_max, &Joint::set_fq_max)
      .def_property("f_coulomb", &Joint::f_coulomb, &Joint::set_f_coulomb)
      .def_property("f_viscous", &Joint::f_viscous, &Joint::set_f_viscous)
      .def_property("f_stiction", &Joint::f_stiction, &Joint::set_f_stiction)
      .def("T_joint", &Joint::T_joint)
      .def("__str__",
           [](const Joint& joint) {
             return spatial_dyn::json::Serialize(joint).dump();
           })
      .def("__repr__",
           [](const Joint& joint) {
             std::stringstream ss;
             ss << "spatialdyn." << joint;
             return ss.str();
           });

  // Graphics
  py::class_<Graphics>(m, "Graphics")
      .def_readwrite("name", &Graphics::name)
      .def_readwrite("T_to_parent", &Graphics::T_to_parent)
      .def_readwrite("geometry", &Graphics::geometry)
      .def_readwrite("material", &Graphics::material);

  // Geometry
  py::class_<Graphics::Geometry>(m, "Geometry")
      .def_property("type",
                    [](const Graphics::Geometry& geometry) {
                      return std::string(geometry);
                    },
                    [](Graphics::Geometry& geometry, const std::string& type) {
                      geometry.type = Graphics::Geometry::StringToType(type);
                    })
      .def_readwrite("scale", &Graphics::Geometry::scale)
      .def_readwrite("radius", &Graphics::Geometry::radius)
      .def_readwrite("length", &Graphics::Geometry::length)
      .def_readwrite("mesh", &Graphics::Geometry::mesh);

  // Material
  py::class_<Graphics::Material>(m, "Material")
      .def_readwrite("name", &Graphics::Material::name)
      .def_readwrite("rgba", &Graphics::Material::rgba)
      .def_readwrite("texture", &Graphics::Material::texture);

  // Forward dynamics
  m.def("forward_dynamics",
        [](const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> tau,
           const std::map<size_t, SpatialForced>& f_external,
           bool gravity, bool centrifugal_coriolis, bool friction, double stiction_epsilon) {
          return ForwardDynamics(ab, tau, f_external,
                                 { gravity, centrifugal_coriolis, friction, stiction_epsilon });
        }, "ab"_a, "tau"_a, "f_external"_a = std::map<size_t, SpatialForced>(),
        "gravity"_a = true, "centrifugal_coriolis"_a = true, "friction"_a = false,
        "stiction_epsilon"_a = 0.01)
   .def("forward_dynamics_aba",
        [](const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> tau,
           const std::map<size_t, SpatialForced>& f_external,
           bool gravity, bool centrifugal_coriolis, bool friction, double stiction_epsilon) {
          return ForwardDynamicsAba(ab, tau, f_external,
                                    { gravity, centrifugal_coriolis, friction, stiction_epsilon });
        }, "ab"_a, "tau"_a, "f_external"_a = std::map<size_t, SpatialForced>(),
        "gravity"_a = true, "centrifugal_coriolis"_a = true, "friction"_a = false,
        "stiction_epsilon"_a = 0.01)
   .def("inertia_inverse", &InertiaInverse, "ab"_a)
   .def("inertia_inverse_aba", &InertiaInverseAba, "ab"_a);

  // Forward kinematics
  m.def("position",
        [](const ArticulatedBody& ab, int link, const Eigen::Vector3d& offset, py::object q) -> Eigen::Vector3d {
          return q.is_none() ? Position(ab, link, offset)
                             : Position(ab, q.cast<Eigen::Ref<const Eigen::VectorXd>>(), link, offset);
        }, "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(), "q"_a = py::none())
   .def("orientation",
        [](const ArticulatedBody& ab, int link, py::object q) -> Eigen::Quaterniond {
          return q.is_none() ? Orientation(ab, link)
                             : Orientation(ab, q.cast<Eigen::Ref<const Eigen::VectorXd>>(), link);
        }, "ab"_a, "link"_a = -1, "q"_a = py::none())
   .def("cartesian_pose",
        [](const ArticulatedBody& ab, int link, const Eigen::Vector3d& offset, py::object q) -> Eigen::Isometry3d {
          return q.is_none() ? CartesianPose(ab, link, offset)
                             : CartesianPose(ab, q.cast<Eigen::Ref<const Eigen::VectorXd>>(), link, offset);
        }, "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(), "q"_a = py::none())
   .def("jacobian",
        [](const ArticulatedBody& ab, int link, const Eigen::Vector3d& offset, py::object q) -> Eigen::Matrix6Xd {
          return q.is_none() ? Jacobian(ab, link, offset)
                             : Jacobian(ab, q.cast<Eigen::Ref<const Eigen::VectorXd>>(), link, offset);
        }, "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(), "q"_a = py::none())
   .def("linear_jacobian",
        [](const ArticulatedBody& ab, int link, const Eigen::Vector3d& offset, py::object q) -> Eigen::Matrix3Xd {
          return q.is_none() ? Eigen::Matrix3Xd(LinearJacobian(ab, link, offset))
                             : LinearJacobian(ab, q.cast<Eigen::Ref<const Eigen::VectorXd>>(), link, offset);
        }, "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(), "q"_a = py::none())
   .def("angular_jacobian",
        [](const ArticulatedBody& ab, int link, const Eigen::Vector3d& offset, py::object q) -> Eigen::Matrix3Xd {
          return q.is_none() ? Eigen::Matrix3Xd(AngularJacobian(ab, link))
                             : AngularJacobian(ab, q.cast<Eigen::Ref<const Eigen::VectorXd>>(), link);
        }, "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(), "q"_a = py::none())
   .def("hessian",
        [](const ArticulatedBody& ab, int link, const Eigen::Vector3d& offset) -> py::array {
          Eigen::Tensor3d H = Hessian(ab, link, offset);
          constexpr ssize_t elem_size = sizeof(double);
          constexpr ssize_t six = 6;
          ssize_t dof = static_cast<ssize_t>(ab.dof());
          return py::array({ dof, dof, six },
                           { elem_size, elem_size * dof, elem_size * dof * dof },
                           H.data(), py::handle());
        }, "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero());

  // Inverse dynamics
  m.def("inverse_dynamics",
        [](const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> ddq,
           const std::map<size_t, SpatialForced>& f_external,
           bool gravity, bool centrifugal_coriolis, bool friction, double stiction_epsilon) {
          return InverseDynamics(ab, ddq, f_external,
                                 { gravity, centrifugal_coriolis, friction, stiction_epsilon });
        }, "ab"_a, "ddq"_a, "f_external"_a = std::map<size_t, SpatialForced>(),
        "gravity"_a = true, "centrifugal_coriolis"_a = false, "friction"_a = false,
        "stiction_epsilon"_a = 0.01)
   .def("centrifugal_coriolis", &CentrifugalCoriolis, "ab"_a)
   .def("gravity", &Gravity, "ab"_a)
   .def("external_torques", &ExternalTorques, "ab"_a, "f_external"_a = std::map<size_t, SpatialForced>())
   .def("friction", &Friction, "ab"_a, "tau"_a, "compensate"_a = true, "stiction_epsilon"_a = 0.01)
   .def("inertia", &Inertia, "ab"_a);

  // Simulation
  m.def("integrate",
        [](ArticulatedBody &ab, const Eigen::VectorXd& tau, double dt,
               const std::map<size_t, SpatialForced>& f_external,
               bool gravity, bool centrifugal_coriolis, bool friction, bool joint_limits,
               const std::string& method, bool aba, double stiction_epsilon) {
          static const std::map<std::string, IntegrationOptions::Method> kStringToMethod = {
            {"EULER", IntegrationOptions::Method::EULER},
            {"HEUNS", IntegrationOptions::Method::HEUNS},
            {"RK4", IntegrationOptions::Method::RK4}
          };
          if (kStringToMethod.find(method) == kStringToMethod.end()) {
            throw std::invalid_argument("spatialdyn.integrate(): Invalid integration method " + method);
          }
          Integrate(ab, tau, dt, f_external,
                    { gravity, centrifugal_coriolis, friction, joint_limits,
                      kStringToMethod.at(method), aba, stiction_epsilon });
        }, "ab"_a, "tau"_a, "dt"_a, "f_external"_a = std::map<size_t, SpatialForced>(),
        "gravity"_a = true, "centrifugal_coriolis"_a = true, "friction"_a = false,
        "joint_limits"_a = false, "method"_a = "RK4", "aba"_a = false,
        "stiction_epsilon"_a = 0.01);

  // Spatial inertia
  py::class_<SpatialInertiad>(m, "SpatialInertiad")
      .def(py::init<>())
      .def(py::init<double, const Eigen::Vector3d&, const Eigen::Vector6d>())
      .def_readwrite("mass", &SpatialInertiad::mass)
      .def_readwrite("com", &SpatialInertiad::com)
      .def_readwrite("I_com", &SpatialInertiad::I_com)
      .def("__repr__",
           [](const SpatialInertiad& inertia) {
             return "<spatialdyn.SpatialInertiad (mass=" + std::to_string(inertia.mass) +
                    ", com=[" + utils::Eigen::EncodeMatlab(inertia.com) + "], I_com=[" +
                    utils::Eigen::EncodeMatlab(inertia.I_com_flat()) + "])>";
           });

  // opspace dynamics
  py::module m_op = m.def_submodule("opspace");
  m_op.def("orientation_error", &opspace::OrientationError, "quat"_a, "quat_des"_a);
  m_op.def("inverse_dynamics",
           [](const ArticulatedBody& ab, const Eigen::MatrixXd& J,
              const Eigen::VectorXd& ddx, py::EigenDRef<Eigen::MatrixXd> N,
              const std::map<size_t, SpatialForced>& f_external,
              bool gravity, bool centrifugal_coriolis, bool friction,
              double svd_epsilon, double stiction_epsilon) {
             Eigen::MatrixXd N_temp = N;
             Eigen::VectorXd tau = opspace::InverseDynamics(ab, J, ddx, &N_temp, f_external, gravity, centrifugal_coriolis, friction, svd_epsilon, stiction_epsilon);
             N = N_temp;
             return tau;
           }, "ab"_a, "J"_a, "ddx"_a, "N"_a,
           "f_external"_a = std::map<size_t, SpatialForced>(),
           "gravity"_a = false, "centrifugal_coriolis"_a = false, "friction"_a = false,
           "svd_epsilon"_a = 0, "stiction_epsilon"_a = 0.01);
  m_op.def("inertia", &opspace::Inertia, "ab"_a, "J"_a, "svd_epsilon"_a = 0);
  m_op.def("inertia_inverse", &opspace::InertiaInverse, "ab"_a, "J"_a);
  m_op.def("jacobian_dynamic_inverse", &opspace::JacobianDynamicInverse, "ab"_a, "J"_a,
           "svd_epsilon"_a = 0);
  m_op.def("centrifugal_coriolis", &opspace::CentrifugalCoriolis, "ab"_a, "J"_a,
           "idx_link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(), "svd_epsilon"_a = 0);
  m_op.def("gravity", &opspace::Gravity, "ab"_a, "J"_a, "svd_epsilon"_a = 0);
  m_op.def("friction", &opspace::Friction, "ab"_a, "J"_a, "tau"_a, "svd_epsilon"_a = 0,
           "stiction_epsilon"_a = 0.01);

  m_op.def("inertia_aba", &opspace::InertiaAba, "ab"_a, "idx_link"_a = -1,
           "offset"_a = Eigen::Vector3d::Zero(), "svd_epsilon"_a = 0);
  m_op.def("inertia_inverse_aba", &opspace::InertiaInverseAba, "ab"_a, "idx_link"_a = -1,
           "offset"_a = Eigen::Vector3d::Zero());
  m_op.def("centrifugal_coriolis_aba", &opspace::CentrifugalCoriolisAba, "ab"_a,
           "idx_link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(), "svd_epsilon"_a = 0);
  m_op.def("gravity_aba", &opspace::GravityAba, "ab"_a, "idx_link"_a = -1,
           "offset"_a = Eigen::Vector3d::Zero(),
           "f_external"_a = std::map<size_t, SpatialForced>(), "svd_epsilon"_a = 0);

  // urdf parser
  py::module m_urdf = m.def_submodule("urdf");
  m_urdf.def("load_model", &urdf::LoadModel, "urdf"_a, "expand_paths"_a = false);

}

}  // namespace spatial_dyn
