/**
 * spatial_dyn.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 7, 2018
 * Authors: Toki Migimatsu
 */

#include <ctrl_utils/control.h>
#include <ctrl_utils/eigen_string.h>
#include <ctrl_utils/string.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>  // std::transform
#include <cctype>     // std::tolower
#include <exception>  // std::invalid_argument
#include <sstream>    // std::stringstream

#include "spatial_dyn/algorithms/forward_dynamics.h"
#include "spatial_dyn/algorithms/forward_kinematics.h"
#include "spatial_dyn/algorithms/inverse_dynamics.h"
#include "spatial_dyn/algorithms/opspace_dynamics.h"
#include "spatial_dyn/algorithms/simulation.h"
#include "spatial_dyn/eigen/spatial_math.h"
#include "spatial_dyn/parsers/json.h"
#include "spatial_dyn/parsers/urdf.h"

namespace {

const uint32_t MASK_X_CONVERGED = 1 << 0;
const uint32_t MASK_DX_CONVERGED = 1 << 1;
const uint32_t MASK_ORI_CONVERGED = 1 << 2;
const uint32_t MASK_W_CONVERGED = 1 << 3;
const uint32_t MASK_SINGULAR_6 = 1 << 4;
const uint32_t MASK_SINGULAR_3 = 1 << 5;

namespace dyn = ::spatial_dyn;

bool IsConverged(double threshold, Eigen::Ref<const Eigen::VectorXd> x_err) {
  if (threshold < 0.) return true;
  return x_err.squaredNorm() < threshold * threshold;
}

std::pair<Eigen::VectorXd, uint32_t> ComputeOpspaceControl(
    spatial_dyn::ArticulatedBody& ab, Eigen::Ref<const Eigen::Vector3d> x_des,
    const Eigen::Quaterniond& quat_des, Eigen::Ref<const Eigen::VectorXd> q_des,
    const Eigen::Matrix<double, 3, 2>& kp_kv_pos,
    Eigen::Ref<const Eigen::Vector2d> kp_kv_ori,
    Eigen::Ref<const Eigen::Vector2d> kp_kv_joint,
    Eigen::Ref<const Eigen::Vector3d> x_task_to_ee,
    const Eigen::Quaterniond* quat_ee_to_task, double ddx_max, double dw_max,
    double x_threshold, double dx_threshold, double ori_threshold,
    double w_threshold, bool gravity_comp, double dt) {
  // Compute Jacobian.
  Eigen::MatrixXd J = dyn::Jacobian(ab, -1, x_task_to_ee);
  Eigen::Ref<Eigen::Matrix<double, 3, -1>> J_v = J.topRows<3>();
  Eigen::Ref<Eigen::Matrix<double, 3, -1>> J_w = J.bottomRows<3>();
  if (quat_ee_to_task) {
    // Rotation Jacobian to task frame.
    const Eigen::Matrix3d R_ee_to_task = quat_ee_to_task->matrix();
    J_v = R_ee_to_task * J_v;
    J_w = R_ee_to_task * J_w;
  }

  // Compute position PD control.
  const Eigen::Vector3d x = dyn::Position(ab, -1, x_task_to_ee);
  const Eigen::Vector3d dx = J_v * ab.dq();
  Eigen::Vector3d x_err;
  const Eigen::Vector3d ddx =
      ctrl_utils::PdControl(x, x_des, dx, kp_kv_pos, ddx_max, &x_err);

  // Get current orientation.
  Eigen::Quaterniond quat = dyn::Orientation(ab);
  if (quat_ee_to_task != nullptr) {
    // Rotate quaternion to task frame.
    quat = *quat_ee_to_task * quat;
  }
  // Choose the quaternion closer to quat_des so the end-effector doesn't rotate
  // the far way around.
  quat = ctrl_utils::NearQuaternion(quat, quat_des);

  // Compute orientation PD control.
  const Eigen::Vector3d w = J_w * ab.dq();
  Eigen::Vector3d ori_err;
  const Eigen::Vector3d dw =
      ctrl_utils::PdControl(quat, quat_des, w, kp_kv_ori, dw_max, &ori_err);

  // Compute opspace torques.
  Eigen::VectorXd tau_cmd;
  Eigen::MatrixXd N = Eigen::MatrixXd::Identity(ab.dof(), ab.dof());
  const bool is_singular_6 = dyn::opspace::IsSingular(ab, J, 0.01);
  bool is_singular_3 = false;
  if (is_singular_6) {
    // Do only position control if the arm is in a 6-dof singularity.
    tau_cmd = dyn::opspace::InverseDynamics(ab, J_v, ddx, &N);
    is_singular_3 = dyn::opspace::IsSingular(ab, J_v, 0.01);
  } else {
    // Otherwise do combined position and orientation control.
    Eigen::Vector6d ddx_dw;
    ddx_dw << ddx, dw;
    tau_cmd = dyn::opspace::InverseDynamics(ab, J, ddx_dw, &N);
  }

  // Add joint task in nullspace.
  Eigen::VectorXd q_err;
  const Eigen::VectorXd ddq =
      ctrl_utils::PdControl(ab.q(), q_des, ab.dq(), kp_kv_joint, 0., &q_err);
  tau_cmd += dyn::opspace::InverseDynamics(
      ab, Eigen::MatrixXd::Identity(ab.dof(), ab.dof()), ddq, &N);

  // Add gravity compensation.
  if (gravity_comp) {
    tau_cmd += dyn::Gravity(ab);
  }

  // Compute convergence.
  uint32_t status = 0;
  if (IsConverged(x_threshold, x_err)) status |= MASK_X_CONVERGED;
  if (IsConverged(dx_threshold, dx)) status |= MASK_DX_CONVERGED;
  if (IsConverged(ori_threshold, ori_err)) status |= MASK_ORI_CONVERGED;
  if (IsConverged(w_threshold, w)) status |= MASK_W_CONVERGED;
  if (is_singular_6) status |= MASK_SINGULAR_6;
  if (is_singular_3) status |= MASK_SINGULAR_3;

  // Apply command torques to update ab.q, ab.dq.
  if (dt > 0.) {
    dyn::Integrate(ab, tau_cmd, dt);
  }

  return {tau_cmd, status};
}

}  // namespace

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
      .def("add_rigid_body", &ArticulatedBody::AddRigidBody, "rb"_a,
           "id_parent"_a = -1)
      .def_property_readonly(
          "rigid_bodies",
          (const std::vector<RigidBody>& (ArticulatedBody::*)(void) const) &
              ArticulatedBody::rigid_bodies)
      .def_property("q",
                    (const Eigen::VectorXd& (ArticulatedBody::*)(void) const) &
                        ArticulatedBody::q,
                    &ArticulatedBody::set_q)
      .def_property("dq",
                    (const Eigen::VectorXd& (ArticulatedBody::*)(void) const) &
                        ArticulatedBody::dq,
                    &ArticulatedBody::set_dq)
      .def_property(
          "g",
          // TODO: Make spatial
          [](const ArticulatedBody& ab) { return ab.g().linear(); },
          &ArticulatedBody::set_g)
      .def("add_load", &ArticulatedBody::AddLoad, "inertia"_a,
           "idx_link"_a = -1)
      .def("replace_load", &ArticulatedBody::ReplaceLoad, "inertia"_a,
           "idx_link"_a = -1)
      .def("clear_load", &ArticulatedBody::ClearLoad, "idx_link"_a = -1)
      .def_property_readonly("inertia_load", &ArticulatedBody::inertia_load)
      .def_property("T_base_to_world", &ArticulatedBody::T_base_to_world,
                    (void(ArticulatedBody::*)(const Eigen::Isometry3d&)) &
                        ArticulatedBody::set_T_base_to_world)
      .def_property("inertia_base", &ArticulatedBody::inertia_base,
                    (void(ArticulatedBody::*)(const SpatialInertiad&)) &
                        ArticulatedBody::set_inertia_base)
      .def(
          "T_to_parent",
          [](const ArticulatedBody& ab, int i,
             py::object q) -> Eigen::Isometry3d {
            return q.is_none() ? ab.T_to_parent(i)
                               : ab.T_to_parent(i, q.cast<double>());
          },
          "i"_a, "q"_a = py::none())
      .def(
          "T_from_parent",
          [](const ArticulatedBody& ab, int i,
             py::object q) -> Eigen::Isometry3d {
            return q.is_none() ? ab.T_from_parent(i)
                               : ab.T_from_parent(i, q.cast<double>());
          },
          "i"_a, "q"_a = py::none())
      .def(
          "T_to_world",
          [](const ArticulatedBody& ab, int i,
             py::object q) -> Eigen::Isometry3d {
            return q.is_none()
                       ? ab.T_to_world(i)
                       : ab.T_to_world(
                             i, q.cast<Eigen::Ref<const Eigen::VectorXd>>());
          },
          "i"_a, "q"_a = py::none())
      .def(
          "T_from_world",
          [](const ArticulatedBody& ab, int i,
             py::object q) -> Eigen::Isometry3d {
            return q.is_none()
                       ? ab.T_from_world(i)
                       : ab.T_from_world(
                             i, q.cast<Eigen::Ref<const Eigen::VectorXd>>());
          },
          "i"_a, "q"_a = py::none())
      .def("ancestors", &ArticulatedBody::ancestors)
      .def("subtree", &ArticulatedBody::subtree)
      .def("map", &ArticulatedBody::Map)
      .def("__str__",
           [](const ArticulatedBody& ab) { return nlohmann::json(ab).dump(); })
      .def("__repr__", [](const ArticulatedBody& ab) {
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
                    (void(RigidBody::*)(const Eigen::Isometry3d&)) &
                        RigidBody::set_T_to_parent)
      .def_property(
          "inertia", &RigidBody::inertia,
          (void(RigidBody::*)(const SpatialInertiad&)) & RigidBody::set_inertia)
      .def_property("joint", &RigidBody::joint, &RigidBody::set_joint)
      .def("__str__",
           [](const RigidBody& rb) { return nlohmann::json(rb).dump(); })
      .def("__repr__", [](const RigidBody& rb) {
        std::stringstream ss;
        ss << "spatialdyn." << rb;
        return ss.str();
      });

  // Joint
  py::class_<Joint>(m, "Joint")
      .def_property(
          "type",
          [](const Joint& joint) { return ctrl_utils::ToString(joint.type()); },
          [](Joint& joint, const std::string& type) {
            joint.set_type(ctrl_utils::FromString<Joint::Type>(type));
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
           [](const Joint& joint) { return nlohmann::json(joint).dump(); })
      .def("__repr__", [](const Joint& joint) {
        std::stringstream ss;
        ss << "spatialdyn." << joint;
        return ss.str();
      });

  // Graphics
  py::class_<Graphics>(m, "Graphics")
      .def(py::init<const std::string&>())
      .def_readwrite("name", &Graphics::name)
      .def_readwrite("T_to_parent", &Graphics::T_to_parent)
      .def_readwrite("geometry", &Graphics::geometry)
      .def_readwrite("material", &Graphics::material)
      .def("__str__", [](const Graphics& graphics) {
        return nlohmann::json(graphics).dump();
      });
  // .def("__repr__",
  //      [](const Graphics& graphics) {
  //        std::stringstream ss;
  //        ss << "spatialdyn." << graphics;
  //        return ss.str();
  //      });

  // Geometry
  py::class_<Graphics::Geometry>(m, "Geometry")
      .def_property(
          "type",
          [](const Graphics::Geometry& geometry) {
            return ctrl_utils::ToString(geometry.type);
          },
          [](Graphics::Geometry& geometry, const std::string& type) {
            ctrl_utils::FromString(type, geometry.type);
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
  m.def(
       "forward_dynamics",
       [](const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> tau,
          const std::map<size_t, SpatialForced>& f_external, bool gravity,
          bool centrifugal_coriolis, bool friction, double stiction_epsilon) {
         return ForwardDynamics(
             ab, tau, f_external,
             {gravity, centrifugal_coriolis, friction, stiction_epsilon});
       },
       "ab"_a, "tau"_a, "f_external"_a = std::map<size_t, SpatialForced>(),
       "gravity"_a = true, "centrifugal_coriolis"_a = true,
       "friction"_a = false, "stiction_epsilon"_a = 0.01)
      .def(
          "forward_dynamics_aba",
          [](const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> tau,
             const std::map<size_t, SpatialForced>& f_external, bool gravity,
             bool centrifugal_coriolis, bool friction,
             double stiction_epsilon) {
            return ForwardDynamicsAba(
                ab, tau, f_external,
                {gravity, centrifugal_coriolis, friction, stiction_epsilon});
          },
          "ab"_a, "tau"_a, "f_external"_a = std::map<size_t, SpatialForced>(),
          "gravity"_a = true, "centrifugal_coriolis"_a = true,
          "friction"_a = false, "stiction_epsilon"_a = 0.01)
      .def("inertia_inverse", &InertiaInverse, "ab"_a)
      .def("inertia_inverse_aba", &InertiaInverseAba, "ab"_a);

  // Forward kinematics
  m.def(
       "position",
       [](const ArticulatedBody& ab, int link, const Eigen::Vector3d& offset,
          py::object q) -> Eigen::Vector3d {
         return q.is_none()
                    ? Position(ab, link, offset)
                    : Position(ab, q.cast<Eigen::Ref<const Eigen::VectorXd>>(),
                               link, offset);
       },
       "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(),
       "q"_a = py::none())
      .def(
          "orientation",
          [](const ArticulatedBody& ab, int link,
             py::object q) -> Eigen::Quaterniond {
            return q.is_none()
                       ? Orientation(ab, link)
                       : Orientation(
                             ab, q.cast<Eigen::Ref<const Eigen::VectorXd>>(),
                             link);
          },
          "ab"_a, "link"_a = -1, "q"_a = py::none())
      .def(
          "cartesian_pose",
          [](const ArticulatedBody& ab, int link, const Eigen::Vector3d& offset,
             py::object q) -> Eigen::Isometry3d {
            return q.is_none()
                       ? CartesianPose(ab, link, offset)
                       : CartesianPose(
                             ab, q.cast<Eigen::Ref<const Eigen::VectorXd>>(),
                             link, offset);
          },
          "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(),
          "q"_a = py::none())
      .def(
          "jacobian",
          [](const ArticulatedBody& ab, int link, const Eigen::Vector3d& offset,
             py::object q) -> Eigen::Matrix6Xd {
            return q.is_none()
                       ? Jacobian(ab, link, offset)
                       : Jacobian(ab,
                                  q.cast<Eigen::Ref<const Eigen::VectorXd>>(),
                                  link, offset);
          },
          "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(),
          "q"_a = py::none())
      .def(
          "linear_jacobian",
          [](const ArticulatedBody& ab, int link, const Eigen::Vector3d& offset,
             py::object q) -> Eigen::Matrix3Xd {
            return q.is_none()
                       ? Eigen::Matrix3Xd(LinearJacobian(ab, link, offset))
                       : LinearJacobian(
                             ab, q.cast<Eigen::Ref<const Eigen::VectorXd>>(),
                             link, offset);
          },
          "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(),
          "q"_a = py::none())
      .def(
          "angular_jacobian",
          [](const ArticulatedBody& ab, int link, const Eigen::Vector3d& offset,
             py::object q) -> Eigen::Matrix3Xd {
            return q.is_none()
                       ? Eigen::Matrix3Xd(AngularJacobian(ab, link))
                       : AngularJacobian(
                             ab, q.cast<Eigen::Ref<const Eigen::VectorXd>>(),
                             link);
          },
          "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(),
          "q"_a = py::none())
      .def(
          "hessian",
          [](const ArticulatedBody& ab, int link,
             const Eigen::Vector3d& offset) -> py::array {
            Eigen::Tensor3d H = Hessian(ab, link, offset);
            constexpr ssize_t elem_size = sizeof(double);
            constexpr ssize_t six = 6;
            ssize_t dof = static_cast<ssize_t>(ab.dof());
            return py::array(
                {dof, dof, six},
                {elem_size, elem_size * dof, elem_size * dof * dof}, H.data(),
                py::handle());
          },
          "ab"_a, "link"_a = -1, "offset"_a = Eigen::Vector3d::Zero());

  // Inverse dynamics
  m.def(
       "inverse_dynamics",
       [](const ArticulatedBody& ab, Eigen::Ref<const Eigen::VectorXd> ddq,
          const std::map<size_t, SpatialForced>& f_external, bool gravity,
          bool centrifugal_coriolis, bool friction, double stiction_epsilon) {
         return InverseDynamics(
             ab, ddq, f_external,
             {gravity, centrifugal_coriolis, friction, stiction_epsilon});
       },
       "ab"_a, "ddq"_a, "f_external"_a = std::map<size_t, SpatialForced>(),
       "gravity"_a = true, "centrifugal_coriolis"_a = false,
       "friction"_a = false, "stiction_epsilon"_a = 0.01)
      .def("centrifugal_coriolis", &CentrifugalCoriolis, "ab"_a)
      .def("gravity", &Gravity, "ab"_a)
      .def("external_torques", &ExternalTorques, "ab"_a,
           "f_external"_a = std::map<size_t, SpatialForced>())
      .def("friction", &Friction, "ab"_a, "tau"_a, "compensate"_a = true,
           "stiction_epsilon"_a = 0.01)
      .def("inertia", &Inertia, "ab"_a)
      .def("composite_inertia", &CompositeInertia, "ab"_a, "link"_a = 0);

  // Simulation
  m.def(
      "integrate",
      [](ArticulatedBody& ab, const Eigen::VectorXd& tau, double dt,
         const std::map<size_t, SpatialForced>& f_external, bool gravity,
         bool centrifugal_coriolis, bool friction, bool joint_limits,
         const std::string& method, bool aba, double stiction_epsilon) {
        static const std::map<std::string, IntegrationOptions::Method>
            kStringToMethod = {{"euler", IntegrationOptions::Method::kEuler},
                               {"heuns", IntegrationOptions::Method::kHeuns},
                               {"rk4", IntegrationOptions::Method::kRk4}};
        std::string str = method;
        std::transform(str.begin(), str.end(), str.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (kStringToMethod.find(str) == kStringToMethod.end()) {
          throw std::invalid_argument(
              "spatialdyn.integrate(): Invalid integration method " + method);
        }
        Integrate(ab, tau, dt, f_external,
                  {gravity, centrifugal_coriolis, friction, joint_limits,
                   kStringToMethod.at(str), aba, stiction_epsilon});
      },
      "ab"_a, "tau"_a, "dt"_a,
      "f_external"_a = std::map<size_t, SpatialForced>(), "gravity"_a = true,
      "centrifugal_coriolis"_a = true, "friction"_a = false,
      "joint_limits"_a = false, "method"_a = "rk4", "aba"_a = false,
      "stiction_epsilon"_a = 0.01);

  // Spatial inertia
  py::class_<SpatialInertiad>(m, "SpatialInertiad")
      .def(py::init<>())
      .def(py::init<double, const Eigen::Vector3d&, const Eigen::Vector6d>())
      .def_readwrite("mass", &SpatialInertiad::mass)
      .def_readwrite("com", &SpatialInertiad::com)
      .def_readwrite("I_com", &SpatialInertiad::I_com)
      .def("__add__", &SpatialInertiad::operator+)
      .def("__iadd__", &SpatialInertiad::operator+=)
      .def("__repr__", [](const SpatialInertiad& inertia) {
        return "<spatialdyn.SpatialInertiad (mass=" +
               std::to_string(inertia.mass) + ", com=[" +
               ctrl_utils::EncodeMatlab(inertia.com) + "], I_com=[" +
               ctrl_utils::EncodeMatlab(inertia.I_com_flat()) + "])>";
      });

  // opspace dynamics
  py::module m_op = m.def_submodule("opspace");
  // m_op.def("orientation_error", &opspace::OrientationError, "quat"_a,
  // "quat_des"_a); m_op.def("near_quaternion",
  //          (Eigen::Quaterniond (*)(const Eigen::Quaterniond&, const
  //          Eigen::Quaterniond&)) &opspace::NearQuaternion, "quat"_a,
  //          "quat_reference"_a);
  m_op.def("is_singular", &opspace::IsSingular, "ab"_a, "J"_a,
           "svd_epsilon"_a = 0.);
  m_op.def(
      "inverse_dynamics",
      [](const ArticulatedBody& ab, const Eigen::MatrixXd& J,
         const Eigen::VectorXd& ddx, py::array_t<double> N,
         const std::map<size_t, SpatialForced>& f_external, bool gravity,
         bool centrifugal_coriolis, bool friction, double svd_epsilon,
         double stiction_epsilon) {
        Eigen::VectorXd tau;
        py::buffer_info info = N.request();
        if (info.ndim == 0) {
          tau =
              opspace::InverseDynamics(ab, J, ddx, nullptr, f_external,
                                       {gravity, centrifugal_coriolis, friction,
                                        svd_epsilon, stiction_epsilon});
          return tau;
        }

        typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> Strides;
        if (info.format != py::format_descriptor<double>::format()) {
          throw std::runtime_error(
              "spatialdyn.opspace.inverse_dynamics(): Expected a double array "
              "for N.");
        }
        if (info.ndim != 2) {
          throw std::runtime_error(
              "spatialdyn.opspace.inverse_dynamics(): Expected a 2D array for "
              "N." +
              std::to_string(info.ndim));
        }
        auto strides = Strides(info.strides[1] / (py::ssize_t)sizeof(double),
                               info.strides[0] / (py::ssize_t)sizeof(double));
        auto N_map = Eigen::Map<Eigen::MatrixXd, 0, Strides>(
            static_cast<double*>(info.ptr), info.shape[0], info.shape[1],
            strides);
        Eigen::MatrixXd N_temp = N_map;
        tau = opspace::InverseDynamics(ab, J, ddx, &N_temp, f_external,
                                       {gravity, centrifugal_coriolis, friction,
                                        svd_epsilon, stiction_epsilon});
        N_map = N_temp;
        return tau;
      },
      "ab"_a, "J"_a, "ddx"_a, "N"_a = py::none(),
      "f_external"_a = std::map<size_t, SpatialForced>(), "gravity"_a = false,
      "centrifugal_coriolis"_a = false, "friction"_a = false,
      "svd_epsilon"_a = 0., "stiction_epsilon"_a = 0.01);
  m_op.def("inertia", &opspace::Inertia, "ab"_a, "J"_a, "svd_epsilon"_a = 0);
  m_op.def("inertia_inverse", &opspace::InertiaInverse, "ab"_a, "J"_a);
  m_op.def("jacobian_dynamic_inverse", &opspace::JacobianDynamicInverse, "ab"_a,
           "J"_a, "svd_epsilon"_a = 0.);
  m_op.def("centrifugal_coriolis", &opspace::CentrifugalCoriolis, "ab"_a, "J"_a,
           "idx_link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(),
           "svd_epsilon"_a = 0.);
  m_op.def("gravity", &opspace::Gravity, "ab"_a, "J"_a, "svd_epsilon"_a = 0);
  m_op.def("external_forces", &opspace::ExternalForces, "ab"_a, "J"_a,
           "f_external"_a = std::map<size_t, SpatialForced>(),
           "svd_epsilon"_a = 0);
  m_op.def("friction", &opspace::Friction, "ab"_a, "J"_a, "tau"_a,
           "svd_epsilon"_a = 0., "stiction_epsilon"_a = 0.01);

  m_op.def("inertia_aba", &opspace::InertiaAba, "ab"_a, "idx_link"_a = -1,
           "offset"_a = Eigen::Vector3d::Zero(), "svd_epsilon"_a = 0.);
  m_op.def("inertia_inverse_aba", &opspace::InertiaInverseAba, "ab"_a,
           "idx_link"_a = -1, "offset"_a = Eigen::Vector3d::Zero());
  m_op.def("centrifugal_coriolis_aba", &opspace::CentrifugalCoriolisAba, "ab"_a,
           "idx_link"_a = -1, "offset"_a = Eigen::Vector3d::Zero(),
           "svd_epsilon"_a = 0.);
  m_op.def("gravity_aba", &opspace::GravityAba, "ab"_a, "idx_link"_a = -1,
           "offset"_a = Eigen::Vector3d::Zero(),
           "f_external"_a = std::map<size_t, SpatialForced>(),
           "svd_epsilon"_a = 0.);

  // Opspace control.
  m.def("compute_opspace_control", &ComputeOpspaceControl);

  // urdf parser
  py::module m_urdf = m.def_submodule("urdf");
  m_urdf.def("load_model", &urdf::LoadModel, "path_urdf"_a,
             "path_meshes"_a = "", "simplify"_a = true);

  // Eigen
  m.def("fdot", [](const Eigen::Isometry3d& T, const SpatialForced& f) {
    return T * f;
  });
  m.def("mdot", [](const Eigen::Isometry3d& T, const SpatialMotiond& m) {
    return T * m;
  });
  m.def("fdot", [](const Eigen::Translation3d& T, const SpatialForced& f) {
    return T * f;
  });
  m.def("mdot", [](const Eigen::Translation3d& T, const SpatialMotiond& m) {
    return T * m;
  });
}

}  // namespace spatial_dyn
