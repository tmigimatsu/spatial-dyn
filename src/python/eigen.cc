/**
 * eigen.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: September 8, 2018
 * Authors: Toki Migimatsu
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include "spatial_dyn/eigen/spatial_math.h"

namespace Eigen {

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(spatialeigen, m) {

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
           [](const Isometry3d& T, const spatial_dyn::SpatialForced& f) {
             return T * f;
           })
      .def("mdot",
           [](const Isometry3d& T, const spatial_dyn::SpatialMotiond& m) {
             return T * m;
           });

  // Translation3d
  py::class_<Translation3d>(m, "Translation3d")
      .def(py::init<const Translation3d&>())
      .def(py::init<const Vector3d&>())
      .def("translation", (Vector3d& (Translation3d::*)(void)) &Translation3d::translation,
           py::return_value_policy::reference_internal)
      .def("inverse", &Translation3d::inverse)
      .def("fdot",
           [](const Translation3d& T, const spatial_dyn::SpatialForced& f) {
             return T * f;
           })
      .def("mdot",
           [](const Translation3d& T, const spatial_dyn::SpatialMotiond& m) {
             return T * m;
           });
}

}  // namespace Eigen
