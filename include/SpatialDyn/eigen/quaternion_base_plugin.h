/**
 * quaternion_base_plugin.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 5, 2018
 * Authors: Toki Migimatsu
 */

EIGEN_DEVICE_FUNC
EIGEN_STRONG_INLINE Matrix<Scalar,4,4>
leftProductMatrix() const {
  Matrix<Scalar,4,4> result;

  result(0,0) = w();
  result(1,0) = z();
  result(2,0) = -y();
  result(3,0) = -x();

  result(0,1) = -z();
  result(1,1) = w();
  result(2,1) = x();
  result(3,1) = -y();

  result(0,2) = y();
  result(1,2) = -x();
  result(2,2) = w();
  result(3,2) = -z();

  result(0,3) = x();
  result(1,3) = y();
  result(2,3) = z();
  result(3,3) = w();

  return result;
}

EIGEN_DEVICE_FUNC
EIGEN_STRONG_INLINE Matrix<Scalar,4,4>
rightProductMatrix() const {
  Matrix<Scalar,4,4> result;

  result(0,0) = w();
  result(1,0) = -z();
  result(2,0) = y();
  result(3,0) = -x();

  result(0,1) = z();
  result(1,1) = w();
  result(2,1) = -x();
  result(3,1) = -y();

  result(0,2) = -y();
  result(1,2) = x();
  result(2,2) = w();
  result(3,2) = -z();

  result(0,3) = x();
  result(1,3) = y();
  result(2,3) = z();
  result(3,3) = w();

  return result;
}

