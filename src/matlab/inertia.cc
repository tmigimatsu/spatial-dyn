/**
 * inertia.cc
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

    // No input q - return current inertia of ab
    if (inputs.size() == 1) {
      matlab::data::TypedArray<double> arr_A = factory_.createArray<double>({ab->dof(), ab->dof()});
      Eigen::Map<Eigen::MatrixXd> A(&*arr_A.begin(), ab->dof(), ab->dof());
      A = spatial_dyn::Inertia(*ab);
      outputs[0] = std::move(arr_A);
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

    // Prepare output A
    size_t n = 1;
    matlab::data::ArrayDimensions dim_A(dim_Q.size() + 1);
    dim_A[0] = ab->dof();
    dim_A[1] = ab->dof();
    for (size_t i = 1; i < dim_Q.size(); i++) {
      dim_A[i+1] = dim_Q[i];
      n *= dim_Q[i];
    }
    matlab::data::TypedArray<double> arr_A = factory_.createArray<double>(dim_A);
    double* data_A = &*arr_A.begin();

    // Compute A
    const Eigen::Map<const Eigen::MatrixXd> Q(&*arr_Q.begin(), ab->dof(), n);
    for (size_t i = 0; i < Q.cols(); i++) {
      double* data_A_i = &data_A[(ab->dof() * ab->dof()) * i];
      Eigen::Map<Eigen::MatrixXd> A(data_A_i, ab->dof(), ab->dof());

      ab->set_q(Q.col(i));
      A = spatial_dyn::Inertia(*ab);
    }
    outputs[0] = std::move(arr_A);
  }

 protected:

  // Pointer to MATLAB engine
  std::shared_ptr<matlab::engine::MATLABEngine> matlab_ = getEngine();

  // Factory to create MATLAB data arrays
  matlab::data::ArrayFactory factory_;

};
