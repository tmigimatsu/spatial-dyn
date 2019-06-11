/**
 * delete_articulated_body.cc
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: May 27, 2019
 * Authors: Toki Migimatsu
 */

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "spatial_dyn/structs/articulated_body.h"

class MexFunction : public matlab::mex::Function {

 public:

  MexFunction() = default;
  virtual ~MexFunction() = default;

  void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) override {
    if (inputs.empty()) return;
    matlab::data::TypedArray<uintptr_t>arr_ab = inputs[0];
    uintptr_t ptr_ab = arr_ab[0];
    spatial_dyn::ArticulatedBody* ab = reinterpret_cast<spatial_dyn::ArticulatedBody*>(ptr_ab);
    if (ab != nullptr) delete ab;
  }

};
