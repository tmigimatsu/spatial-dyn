/**
 * discrete_inverse_dynamics.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: May 29, 2019
 * Authors: Toki Migimatsu
 */

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "spatial_dyn/algorithms/discrete_dynamics.h"

class MexFunction : public matlab::mex::Function {

 public:

  MexFunction() = default;
  virtual ~MexFunction() = default;

  void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) override {
    if (inputs.size() <= 2) return;
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
    if (dim_Q[1] < 2) {
      std::cerr << "Q must have at least 3 columns." << std::endl;
      return;
    }
    const size_t T = dim_Q[1] - 2;

    // Parse input dt
    const matlab::data::TypedArray<double> arr_dt = inputs[2];
    const double dt = arr_dt[0];

    // Prepare output V
    const matlab::data::ArrayDimensions dim_output({ab->dof(), T});
    matlab::data::TypedArray<double> arr_Tau = factory_.createArray<double>(dim_output);
    double* data_V = &*arr_Tau.begin();

    // Compute V
    const Eigen::Map<const Eigen::MatrixXd> Q(&*arr_Q.begin(), ab->dof(), T + 2);
    Eigen::Map<Eigen::MatrixXd> Tau(&*arr_Tau.begin(), ab->dof(), T);
    for (size_t i = 0; i < T; i++) {
      ab->set_q(Q.col(i + 1));
      ab->set_dq((Q.col(i + 1) - Q.col(i)) / dt);
      Tau.col(i) = spatial_dyn::discrete::InverseDynamics(*ab, Q.col(i + 2), dt);
    }
    outputs[0] = std::move(arr_Tau);
  }

 protected:

  // Pointer to MATLAB engine
  std::shared_ptr<matlab::engine::MATLABEngine> matlab_ = getEngine();

  // Factory to create MATLAB data arrays
  matlab::data::ArrayFactory factory_;

};
