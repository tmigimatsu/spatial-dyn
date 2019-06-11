/**
 * centrifugal_coriolis.cc
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

    // No input q - return current V of ab
    if (inputs.size() == 1) {
      matlab::data::TypedArray<double> arr_V = factory_.createArray<double>({ab->dof()});
      Eigen::Map<Eigen::VectorXd> V(&*arr_V.begin(), ab->dof());
      V = spatial_dyn::CentrifugalCoriolis(*ab);
      outputs[0] = std::move(arr_V);
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

    // Parse input dq
    const matlab::data::TypedArray<double> arr_dQ = inputs[1];
    const matlab::data::ArrayDimensions dim_dQ = arr_dQ.getDimensions();
    if (dim_dQ.empty()) return;
    if (dim_dQ[0] != ab->dof()) {
      std::cerr << "Input dimensions must match dof." << std::endl;
      return;
    }
    // TODO: Allow vector Q or dQ, but not both
    if (dim_Q.size() != 1 && dim_dQ.size() != 1 && dim_Q != dim_dQ) {
      std::cerr << "Q and dQ must have the same dimensions" << std::endl;
    }

    // Prepare output V
    size_t n = 1;
    matlab::data::ArrayDimensions dim_output(dim_Q.size());
    dim_output[0] = ab->dof();
    for (size_t i = 1; i < dim_Q.size(); i++) {
      dim_output[i] = dim_Q[i];
      n *= dim_Q[i];
    }
    matlab::data::TypedArray<double> arr_V = factory_.createArray<double>(dim_output);
    double* data_V = &*arr_V.begin();

    // Compute V
    const Eigen::Map<const Eigen::MatrixXd> Q(&*arr_Q.begin(), ab->dof(), n);
    const Eigen::Map<const Eigen::MatrixXd> dQ(&*arr_dQ.begin(), ab->dof(), n);
    for (size_t i = 0; i < Q.cols(); i++) {
      double* data_V_i = &data_V[ab->dof() * i];
      Eigen::Map<Eigen::VectorXd> V(data_V_i, ab->dof());

      ab->set_q(Q.col(i));
      ab->set_dq(dQ.col(i));
      V = spatial_dyn::CentrifugalCoriolis(*ab);
    }
    outputs[0] = std::move(arr_V);
  }

 protected:

  // Pointer to MATLAB engine
  std::shared_ptr<matlab::engine::MATLABEngine> matlab_ = getEngine();

  // Factory to create MATLAB data arrays
  matlab::data::ArrayFactory factory_;

};
