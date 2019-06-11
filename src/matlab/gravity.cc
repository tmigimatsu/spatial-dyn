/**
 * gravity.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: May 28, 2019
 * Authors: Toki Migimatsu
 */

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "spatial_dyn/algorithms/inverse_dynamics.h"

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

    // No input q - return current G of ab
    if (inputs.size() == 1) {
      matlab::data::TypedArray<double> arr_G = factory_.createArray<double>({ab->dof()});
      Eigen::Map<Eigen::VectorXd> G(&*arr_G.begin(), ab->dof());
      G = spatial_dyn::Gravity(*ab);
      outputs[0] = std::move(arr_G);
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

    // Prepare output G
    size_t n = 1;
    matlab::data::ArrayDimensions dim_output(dim_Q.size());
    dim_output[0] = ab->dof();
    for (size_t i = 1; i < dim_Q.size(); i++) {
      dim_output[i] = dim_Q[i];
      n *= dim_Q[i];
    }
    matlab::data::TypedArray<double> arr_G = factory_.createArray<double>(dim_output);
    double* data_G = &*arr_G.begin();

    // Compute A
    const Eigen::Map<const Eigen::MatrixXd> Q(&*arr_Q.begin(), ab->dof(), n);
    for (size_t i = 0; i < Q.cols(); i++) {
      double* data_G_i = &data_G[ab->dof() * i];
      Eigen::Map<Eigen::VectorXd> G(data_G_i, ab->dof());

      ab->set_q(Q.col(i));
      G = spatial_dyn::Gravity(*ab);
    }
    outputs[0] = std::move(arr_G);
  }

 protected:

  // Pointer to MATLAB engine
  std::shared_ptr<matlab::engine::MATLABEngine> matlab_ = getEngine();

  // Factory to create MATLAB data arrays
  matlab::data::ArrayFactory factory_;

};
