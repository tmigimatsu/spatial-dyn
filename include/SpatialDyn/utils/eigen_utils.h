/**
 * eigen_string.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: July 1, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_UTILS_EIGEN_UTILS_H_
#define SPATIAL_DYN_UTILS_EIGEN_UTILS_H_

#include <algorithm>  // std::max
#include <cmath>      // std::cos, std::sin
#include <limits>     // std::numeric_limits

#include <Eigen/Eigen>

namespace EigenUtils {

template<typename Derived>
inline typename Eigen::MatrixBase<Derived>::PlainObject
PseudoInverse(const Eigen::MatrixBase<Derived>& A, double svd_epsilon = 0) {
  unsigned int options = Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic ?
                         Eigen::ComputeThinU | Eigen::ComputeThinV :
                         Eigen::ComputeFullU | Eigen::ComputeFullV;
  Eigen::JacobiSVD<typename Derived::PlainObject> svd(A, options);
  const auto& S = svd.singularValues();
  if (svd_epsilon <= 0) {
    svd_epsilon = std::numeric_limits<typename Derived::Scalar>::epsilon() *
                std::max(A.cols(), A.cols()) * S(0);
  }
  return svd.matrixV() *
         (S.array() > svd_epsilon).select(S.array().inverse(), 0).matrix().asDiagonal() *
         svd.matrixU().adjoint();
}

template<typename Scalar>
Eigen::Matrix<Scalar,3,3> RotationX(Scalar angle) {
  Eigen::Matrix<Scalar,3,3> R;
  Scalar ca = std::cos(angle);
  Scalar sa = std::sin(angle);
  R << 1, 0,   0,
       0, ca, -sa,
       0, sa,  ca;
  return R;
}

template<typename Scalar>
Eigen::Matrix<Scalar,3,3> RotationY(Scalar angle) {
  Eigen::Matrix<Scalar,3,3> R;
  Scalar ca = std::cos(angle);
  Scalar sa = std::sin(angle);
  R <<  ca, 0, sa,
        0,  1, 0,
       -sa, 0, ca;
  return R;
}

template<typename Scalar>
Eigen::Matrix<Scalar,3,3> RotationZ(Scalar angle) {
  Eigen::Matrix<Scalar,3,3> R;
  Scalar ca = std::cos(angle);
  Scalar sa = std::sin(angle);
  R << ca, -sa, 0,
       sa,  ca, 0,
       0,   0,  1;
  return R;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar,3,3>
CrossMatrix(const Eigen::DenseBase<Derived>& x) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  eigen_assert(x.size() == 3);

  typedef typename Derived::Scalar Scalar;
  typename Eigen::internal::nested_eval<Derived,2>::type z(x.derived());
  Eigen::Matrix<Scalar,3,3> result;

  result(0,0) = 0;
  result(1,0) = -z.coeff(2);
  result(2,0) = z.coeff(1);

  result(0,1) = z.coeff(2);
  result(1,1) = 0;
  result(2,1) = -z.coeff(0);

  result(0,2) = -z.coeff(1);
  result(1,2) = z.coeff(0);
  result(2,2) = 0;

  return result;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar,3,3>
DoubleCrossMatrix(const Eigen::DenseBase<Derived>& x) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  eigen_assert(x.size() == 3);

  typedef typename Derived::Scalar Scalar;
  typename Eigen::internal::nested_eval<Derived,4>::type z(x.derived());
  Eigen::Matrix<Scalar,3,3> result;

  Scalar aa = -z.coeff(0) * z.coeff(0);
  Scalar bb = -z.coeff(1) * z.coeff(1);
  Scalar cc = -z.coeff(2) * z.coeff(2);

  result(0,0) = bb + cc;
  result(1,0) = z.coeff(0) * z.coeff(1);
  result(2,0) = z.coeff(0) * z.coeff(2);

  result(0,1) = result(1,0);
  result(1,1) = aa + cc;
  result(2,1) = z.coeff(1) * z.coeff(2);

  result(0,2) = result(2,0);
  result(1,2) = result(2,1);
  result(2,2) = aa + bb;

  return result;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar,4,4>
LeftProductMatrix(const Eigen::QuaternionBase<Derived>& quat) {
  Eigen::Matrix<typename Derived::Scalar,4,4> result;

  result(0,0) = quat.w();
  result(1,0) = quat.z();
  result(2,0) = -quat.y();
  result(3,0) = -quat.x();

  result(0,1) = -quat.z();
  result(1,1) = quat.w();
  result(2,1) = quat.x();
  result(3,1) = -quat.y();

  result(0,2) = quat.y();
  result(1,2) = -quat.x();
  result(2,2) = quat.w();
  result(3,2) = -quat.z();

  result(0,3) = quat.x();
  result(1,3) = quat.y();
  result(2,3) = quat.z();
  result(3,3) = quat.w();

  return result;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar,4,4>
RightProductMatrix(const Eigen::QuaternionBase<Derived>& quat) {
  Eigen::Matrix<typename Derived::Scalar,4,4> result;

  result(0,0) = quat.w();
  result(1,0) = -quat.z();
  result(2,0) = quat.y();
  result(3,0) = -quat.x();

  result(0,1) = quat.z();
  result(1,1) = quat.w();
  result(2,1) = -quat.x();
  result(3,1) = -quat.y();

  result(0,2) = -quat.y();
  result(1,2) = quat.x();
  result(2,2) = quat.w();
  result(3,2) = -quat.z();

  result(0,3) = quat.x();
  result(1,3) = quat.y();
  result(2,3) = quat.z();
  result(3,3) = quat.w();

  return result;
}

}  // namespace EigenUtils

#endif  // SPATIAL_DYN_UTILS_EIGEN_UTILS_H_
