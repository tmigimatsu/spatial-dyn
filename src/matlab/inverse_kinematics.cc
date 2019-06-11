/**
 * inverse_kinematics.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 2, 2019
 * Authors: Toki Migimatsu
 */

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "spatial_dyn/algorithms/inverse_kinematics.h"

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
    if (ab == nullptr) {
      std::cerr << "Null ArticulatedBody pointer." << std::endl;
      return;
    }

    // TODO: Add optional 'link' and 'offset' as named arguments

    // No input x or quat
    if (inputs.size() < 3) {
      std::cerr << "Missing x or quat input." << std::endl;
      return;
    }

    // Parse input x
    const matlab::data::TypedArray<double> arr_x = inputs[1];
    const matlab::data::ArrayDimensions dim_x = arr_x.getDimensions();
    if (dim_x.empty() || dim_x[0] != 3) {
      std::cerr << "Input x must be of size 3 x 1." << std::endl;
      return;
    }
    const Eigen::Map<const Eigen::Vector3d> x(&*arr_x.begin());

    // Parse input quat
    const matlab::data::TypedArray<double> arr_quat = inputs[2];
    const matlab::data::ArrayDimensions dim_quat = arr_quat.getDimensions();
    if (dim_quat.empty() || dim_quat[0] != 4) {
      std::cerr << "Input quat must be of size 4 x 1." << std::endl;
      return;
    }
    const Eigen::Map<const Eigen::Vector4d> quat_coeffs(&*arr_x.begin());
    const Eigen::Quaterniond quat(quat_coeffs(0), quat_coeffs(1), quat_coeffs(2), quat_coeffs(3));

    // Parse guess q
    const matlab::data::TypedArray<double> arr_q_0 = inputs[3];
    const matlab::data::ArrayDimensions dim_q_0 = arr_q_0.getDimensions();
    if (dim_q_0.empty() || dim_q_0[0] != ab->dof()) {
      std::cerr << "Input q must be of size " << ab->dof() << " x 1." << std::endl;
      return;
    }
    const Eigen::Map<const Eigen::VectorXd> q_0(&*arr_q_0.begin(), ab->dof());
    ab->set_q(q_0);

    // Parse link/offset
    int link = -1;
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    for (size_t i = 5; i < inputs.size(); i += 2) {
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

    // Prepare output q
    matlab::data::TypedArray<double> arr_q = factory_.createArray<double>({ab->dof()});
    Eigen::Map<Eigen::VectorXd> q(&*arr_q.begin(), ab->dof());
    q = spatial_dyn::InverseKinematics(*ab, x, quat, link, offset);
    outputs[0] = std::move(arr_q);
  }

 protected:

  // Pointer to MATLAB engine
  std::shared_ptr<matlab::engine::MATLABEngine> matlab_ = getEngine();

  // Factory to create MATLAB data arrays
  matlab::data::ArrayFactory factory_;

};
