/**
 * jacobian.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 2, 2019
 * Authors: Toki Migimatsu
 */

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "spatial_dyn/algorithms/forward_kinematics.h"

class MexFunction : public matlab::mex::Function {

 public:

  MexFunction() = default;
  virtual ~MexFunction() = default;

  void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) override {
    if (inputs.empty()) return;
    const matlab::data::Array matlab_ab(inputs[0]);
    const matlab::data::TypedArray<uintptr_t>arr_ab = matlab_->getProperty(matlab_ab, u"ptr_ab");
    const uintptr_t ptr_ab = arr_ab[0];
    spatial_dyn::ArticulatedBody* ab = reinterpret_cast<spatial_dyn::ArticulatedBody*>(ptr_ab);
    if (ab == nullptr) return;

    // TODO: Add optional 'link' and 'offset' as named arguments

    // No input q - return current J of ab
    if (inputs.size() == 1) {
      matlab::data::TypedArray<double> arr_J = factory_.createArray<double>({6, ab->dof()});
      Eigen::Map<Eigen::Matrix6Xd> J(&*arr_J.begin(), 6, ab->dof());
      J = spatial_dyn::Jacobian(*ab);
      outputs[0] = std::move(arr_J);
      return;
    }

    // Parse input q
    const matlab::data::TypedArray<double> arr_Q = inputs[1];
    const matlab::data::ArrayDimensions dim_Q = arr_Q.getDimensions();
    if (dim_Q.empty()) return;
    if (dim_Q[0] != ab->dof()) {
      std::cerr << "Input dimensions must match dof." << std::endl;
      return;
    }

    // Parse link/offset
    int link = -1;
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    for (size_t i = 3; i < inputs.size(); i += 2) {
      const matlab::data::CharArray arg = inputs[i-1];
      if (arg.toAscii() == "offset") {
        const matlab::data::TypedArray<double> arr_offset = inputs[i];
        const matlab::data::ArrayDimensions dim_offset = arr_offset.getDimensions();
        if (dim_offset.empty() || dim_offset[0] != 3) {
          std::cerr << "Offset must be 3 x 1." << std::endl;
          return;
        }
        offset << arr_offset[0], arr_offset[1], arr_offset[2];
      } else if (arg.toAscii() == "link") {
        const matlab::data::TypedArray<double> arr_link = inputs[i];
        const matlab::data::ArrayDimensions dim_link = arr_link.getDimensions();
        if (dim_link.empty() || dim_link[0] != 1) {
          std::cerr << "Link must be a scalar" << std::endl;
          return;
        }
        link = arr_link[0];
      }
    }

    // Prepare output x
    size_t n = 1;
    matlab::data::ArrayDimensions dim_output(dim_Q.size() + 1);
    dim_output[0] = 6;
    dim_output[1] = ab->dof();
    for (size_t i = 1; i < dim_Q.size(); i++) {
      dim_output[i+1] = dim_Q[i];
      n *= dim_Q[i];
    }
    matlab::data::TypedArray<double> arr_J = factory_.createArray<double>(dim_output);
    double* data_J = &*arr_J.begin();

    // Compute J
    const Eigen::Map<const Eigen::MatrixXd> Q(&*arr_Q.begin(), ab->dof(), n);
    for (size_t i = 0; i < Q.cols(); i++) {
      double* data_J_i = &data_J[ab->dof() * 6 * i];
      Eigen::Map<Eigen::Matrix6Xd> J(data_J_i, 6, ab->dof());
      J = spatial_dyn::Jacobian(*ab, Q.col(i), link, offset);
    }
    outputs[0] = std::move(arr_J);
  }

 protected:

  // Pointer to MATLAB engine
  std::shared_ptr<matlab::engine::MATLABEngine> matlab_ = getEngine();

  // Factory to create MATLAB data arrays
  matlab::data::ArrayFactory factory_;

};
