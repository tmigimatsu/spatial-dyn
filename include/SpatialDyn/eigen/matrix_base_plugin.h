/**
 * matrix_base_plugin.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 23, 2018
 * Authors: Toki Migimatsu
 */

EIGEN_DEVICE_FUNC
EIGEN_STRONG_INLINE Matrix<typename internal::traits<Derived>::Scalar,3,3>
doubleCrossMatrix() const {
  typedef typename internal::traits<Derived>::Scalar Scalar;
  typename internal::nested_eval<Derived,4>::type x(derived());
  Scalar aa = -x.coeff(0) * x.coeff(0);
  Scalar bb = -x.coeff(1) * x.coeff(1);
  Scalar cc = -x.coeff(2) * x.coeff(2);
  Matrix<Scalar,3,3> result;
  result(0,0) = bb + cc;
  result(1,0) = x.coeff(0) * x.coeff(1);
  result(2,0) = x.coeff(0) * x.coeff(2);
  result(0,1) = result(1,0);
  result(1,1) = aa + cc;
  result(2,1) = x.coeff(1) * x.coeff(2);
  result(0,2) = result(2,0);
  result(1,2) = result(2,1);
  result(2,2) = aa + bb;
  return result;
}

EIGEN_DEVICE_FUNC
EIGEN_STRONG_INLINE std::string toMatlab() const {
  typedef typename internal::traits<Derived>::Scalar Scalar;
  std::stringstream ss;
  ss.precision(std::numeric_limits<Scalar>::digits10);
  if (derived().cols() == 1) { // Column vector
    // [[1],[2],[3],[4]] => "1 2 3 4"
    for (int i = 0; i < derived().rows(); ++i) {
      if (i > 0) ss << " ";
      ss << derived().coeff(i);
    }
  } else { // matrix
    // [1,2,3,4]     => "1 2 3 4"
    // [[1,2],[3,4]] => "1 2; 3 4"
    for (int i = 0; i < derived().rows(); ++i) {
      if (i > 0) ss << "; ";
      for (int j = 0; j < derived().cols(); ++j) {
        if (j > 0) ss << " ";
        ss << derived().coeff(i,j);
      }
    }
  }
  return ss.str();
}
