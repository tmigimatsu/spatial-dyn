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

template <typename T>
T signum(T x) {
  return (T(0) < x) - (x < T(0));
}

namespace SpatialDyn {

}  // namespace SpatialDyn

#endif  // SPATIAL_DYN_UTILS_MATH_H_
