/**
 * hessian.cc
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

    // No input q - return current H of ab
    if (inputs.size() == 1) {
      matlab::data::TypedArray<double> arr_H = factory_.createArray<double>({ab->dof(), ab->dof(), 6});
      Eigen::TensorMap<Eigen::Tensor3d> H(&*arr_H.begin(), ab->dof(), ab->dof(), 6);
      H = spatial_dyn::Hessian(*ab);
      outputs[0] = std::move(arr_H);
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

    // Prepare output x
    size_t n = 1;
    matlab::data::ArrayDimensions dim_output(dim_Q.size() + 2);
    dim_output[0] = ab->dof();
    dim_output[1] = ab->dof();
    dim_output[2] = 6;
    for (size_t i = 1; i < dim_Q.size(); i++) {
      dim_output[i+2] = dim_Q[i];
      n *= dim_Q[i];
    }
    matlab::data::TypedArray<double> arr_H = factory_.createArray<double>(dim_output);
    double* data_H = &*arr_H.begin();

    // Compute H
    const Eigen::Map<const Eigen::MatrixXd> Q(&*arr_Q.begin(), ab->dof(), n);
    for (size_t i = 0; i < Q.cols(); i++) {
      double* data_H_i = &data_H[ab->dof() * ab->dof() * 6 * i];
      Eigen::TensorMap<Eigen::Tensor3d> H(data_H_i, ab->dof(), ab->dof(), 6);
      ab->set_q(Q.col(i));
      H = spatial_dyn::Hessian(*ab);
    }
    outputs[0] = std::move(arr_H);
  }

 protected:

  // Pointer to MATLAB engine
  std::shared_ptr<matlab::engine::MATLABEngine> matlab_ = getEngine();

  // Factory to create MATLAB data arrays
  matlab::data::ArrayFactory factory_;

};
