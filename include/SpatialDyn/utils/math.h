/**
 * math.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: October 7, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_UTILS_MATH_H_
#define SPATIAL_DYN_UTILS_MATH_H_

namespace SpatialDyn {

template <typename T>
T Signum(T x) {
  return (T(0) < x) - (x < T(0));
}

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_UTILS_MATH_H_
