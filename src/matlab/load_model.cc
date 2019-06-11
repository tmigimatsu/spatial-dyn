/**
 * load_model.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: May 27, 2019
 * Authors: Toki Migimatsu
 */

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "spatial_dyn/parsers/urdf.h"

class MexFunction : public matlab::mex::Function {

 public:

  MexFunction() = default;
  virtual ~MexFunction() = default;

  void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) override {
    if (inputs.empty()) return;
    matlab::data::CharArray arr_urdf = inputs[0];
    spatial_dyn::ArticulatedBody ab = spatial_dyn::urdf::LoadModel(arr_urdf.toAscii());
    if (outputs.empty()) return;

    spatial_dyn::ArticulatedBody* ptr_ab = new spatial_dyn::ArticulatedBody(std::move(ab));
    matlab::data::TypedArray<uintptr_t> output = factory_.createArray<uintptr_t>({1});
    output[0] = reinterpret_cast<uintptr_t>(ptr_ab);
    outputs[0] = std::move(output);
  }

 protected:

  // Factory to create MATLAB data arrays
  matlab::data::ArrayFactory factory_;

};
