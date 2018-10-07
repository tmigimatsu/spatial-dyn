/**
 * eigen.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 8, 2018
 * Authors: Toki Migimatsu
 */

#include "SpatialDyn/utils/spatial_math.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <string>  // std::to_string

namespace Eigen {

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(eigen, m) {

  // Isometry3d
  py::class_<Isometry3d>(m, "Isometry3d")
      .def(py::init<const Isometry3d&>())
      .def(py::init<const Matrix<double,4,4>&>())
      .def(py::init<const Matrix3d&>())
      .def(py::init<const Quaterniond&>())
      .def("matrix", (Matrix<double,4,4>& (Isometry3d::*)(void)) &Isometry3d::matrix,
           py::return_value_policy::reference_internal)
      .def("linear", (Block<Matrix<double,4,4>,3,3> (Isometry3d::*)(void)) &Isometry3d::linear,
           py::return_value_policy::reference_internal)
      .def("translation", (Block<Matrix<double,4,4>,3,1,true> (Isometry3d::*)(void)) &Isometry3d::translation,
           py::return_value_policy::reference_internal)
      .def("inverse", &Isometry3d::inverse)
      .def("fdot",
           [](const Isometry3d& T, const SpatialDyn::SpatialForced& f) {
             return T * f;
           })
      .def("mdot",
           [](const Isometry3d& T, const SpatialDyn::SpatialMotiond& m) {
             return T * m;
           });

  // Quaterniond
  py::class_<Quaterniond>(m, "Quaterniond")
      .def(py::init<const Quaterniond&>())
      .def(py::init<const double&, const double&, const double&, const double&>())
      .def(py::init<const Matrix3d&>())
      .def_property("w", (const double& (Quaterniond::*)(void) const) &Quaterniond::w,
                    [](Quaterniond& quat, double w) { quat.w() = w; })
      .def_property("x", (const double& (Quaterniond::*)(void) const) &Quaterniond::x,
                    [](Quaterniond& quat, double x) { quat.x() = x; })
      .def_property("y", (const double& (Quaterniond::*)(void) const) &Quaterniond::y,
                    [](Quaterniond& quat, double y) { quat.y() = y; })
      .def_property("z", (const double& (Quaterniond::*)(void) const) &Quaterniond::z,
                    [](Quaterniond& quat, double z) { quat.z() = z; })
      .def_property("coeffs", (const Vector4d& (Quaterniond::*)(void) const) &Quaterniond::coeffs,
                    [](Quaterniond& quat, const Eigen::Vector4d& coeffs) {
                      quat.coeffs() = coeffs;
                    })
      .def("inverse", &Quaterniond::inverse)
      .def("__repr__",
           [](const Quaterniond& quat) {
             return "<eigen.Quaterniond (x=" + std::to_string(quat.x()) +
                    ", y=" + std::to_string(quat.y()) +
                    ", z=" + std::to_string(quat.z()) +
                    ", w=" + std::to_string(quat.w()) + ")>";
           });

  // LDLT<Eigen::MatrixXd>
  py::class_<LDLT<MatrixXd>>(m, "LdltMatrixXd")
    .def("solve",
         [](const LDLT<MatrixXd>& ldlt, const MatrixXd& x) {
           return ldlt.solve(x);
         });

}

}  // namespace Eigen
