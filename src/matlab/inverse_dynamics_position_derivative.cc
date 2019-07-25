/**
 * inverse_dynamics_position_derivative.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 10, 2019
 * Authors: Toki Migimatsu
 */

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "spatial_dyn/algorithms/dynamics_derivatives.h"

class MexFunction : public matlab::mex::Function {

 public:

  MexFunction() = default;
  virtual ~MexFunction() = default;

  void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) override {
    if (inputs.size() < 4) return;
    const matlab::data::Array matlab_ab(inputs[0]);
    const matlab::data::TypedArray<uintptr_t>arr_ab = matlab_->getProperty(matlab_ab, u"ptr_ab");
    const uintptr_t ptr_ab = arr_ab[0];
    spatial_dyn::ArticulatedBody* ab = reinterpret_cast<spatial_dyn::ArticulatedBody*>(ptr_ab);
    if (ab == nullptr) return;

    // Parse input q
    const matlab::data::TypedArray<double> arr_Q = inputs[1];
    const matlab::data::ArrayDimensions dim_Q = arr_Q.getDimensions();
    if (dim_Q.size() != 2) {
      std::cerr << "Q must be a 2d matrix" << std::endl;
      return;
    }
    if (dim_Q[0] != ab->dof()) {
      std::cerr << "Input dimensions must match dof." << std::endl;
      return;
    }

    // Parse input dq
    const matlab::data::TypedArray<double> arr_dQ = inputs[2];
    const matlab::data::ArrayDimensions dim_dQ = arr_dQ.getDimensions();
    if (dim_dQ.size() != 2) {
      std::cerr << "dQ must be a 2d matrix" << std::endl;
      return;
    }
    if (dim_dQ[0] != ab->dof()) {
      std::cerr << "Input dimensions must match dof." << std::endl;
      return;
    }
    if (dim_dQ != dim_Q) {
      std::cerr << "Q and dQ must be the same size." << std::endl;
      return;
    }

    // Parse input ddq
    const matlab::data::TypedArray<double> arr_ddQ = inputs[3];
    const matlab::data::ArrayDimensions dim_ddQ = arr_ddQ.getDimensions();
    if (dim_ddQ.size() != 2) {
      std::cerr << "ddQ must be a 2d matrix" << std::endl;
      return;
    }
    if (dim_ddQ[0] != ab->dof()) {
      std::cerr << "Input dimensions must match dof." << std::endl;
      return;
    }
    if (dim_ddQ != dim_Q) {
      std::cerr << "Q and ddQ must be the same size." << std::endl;
      return;
    }

    // Prepare output Tau
    size_t n = 1;
    matlab::data::ArrayDimensions dim_output(dim_Q.size() + 1);
    dim_output[0] = ab->dof();
    dim_output[1] = ab->dof();
    for (size_t i = 1; i < dim_Q.size(); i++) {
      dim_output[i+1] = dim_Q[i];
      n *= dim_Q[i];
    }
    matlab::data::TypedArray<double> arr_dTau = factory_.createArray<double>(dim_output);
    double* data_dTau = &*arr_dTau.begin();

    // Compute Tau
    const Eigen::Map<const Eigen::MatrixXd> Q(&*arr_Q.begin(), ab->dof(), n);
    const Eigen::Map<const Eigen::MatrixXd> dQ(&*arr_dQ.begin(), ab->dof(), n);
    const Eigen::Map<const Eigen::MatrixXd> ddQ(&*arr_ddQ.begin(), ab->dof(), n);
    const spatial_dyn::InverseDynamicsOptions options(true, true);
    for (size_t i = 0; i < Q.cols(); i++) {
      double* data_dTau_i = &data_dTau[ab->dof() * ab->dof() * i];
      Eigen::Map<Eigen::MatrixXd> dTau(data_dTau_i, ab->dof(), ab->dof());
      ab->set_q(Q.col(i));
      ab->set_dq(dQ.col(i));
      dTau = spatial_dyn::InverseDynamicsPositionDerivative(*ab, ddQ.col(i), {}, options);
    }
    outputs[0] = std::move(arr_dTau);
  }

 protected:

  // Pointer to MATLAB engine
  std::shared_ptr<matlab::engine::MATLABEngine> matlab_ = getEngine();

  // Factory to create MATLAB data arrays
  matlab::data::ArrayFactory factory_;

};
