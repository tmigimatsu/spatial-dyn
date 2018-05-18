/**
 * SpatialInertia.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 17, 2018
 * Authors: Toki Migimatsu
 */

#ifndef EIGEN_SPATIAL_INERTIA_H_
#define EIGEN_SPATIAL_INERTIA_H_

#include "SpatialMotionBase.h"
#include "SpatialForceBase.h"

namespace Eigen {

template<typename Derived>
Matrix<typename Derived::Scalar,3,3> crossMatrix(const MatrixBase<Derived>& x) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)
  Matrix<typename Derived::Scalar,3,3> result;
  result <<  0,    -x(2),  x(1),
             x(2),  0,    -x(0),
            -x(1),  x(0),  0;
  return result;
}

template<typename Scalar>
class SpatialInertia : public Matrix<Scalar,6,6> {

 public:
  SpatialInertia() : Matrix<Scalar,6,6>() {};

  template<typename ComDerived, typename IComDerived>
  SpatialInertia(Scalar mass,
                 const Eigen::MatrixBase<ComDerived>& com,
                 const Eigen::MatrixBase<IComDerived>& I_com_flat);

};

template<typename Scalar, typename Derived>
SpatialForce<typename ScalarBinaryOpTraits<Scalar, typename internal::traits<Derived>::Scalar>::ReturnType,
             internal::traits<Derived>::ColsAtCompileTime>
operator*(const SpatialInertia<Scalar>& I, const SpatialMotionBase<Derived>& m) {
  return I * m.matrix();
}


template<typename Scalar>
template<typename ComDerived, typename IComDerived>
SpatialInertia<Scalar>::SpatialInertia(Scalar m,
    const Eigen::MatrixBase<ComDerived>& c,
    const Eigen::MatrixBase<IComDerived>& I) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(ComDerived, 3)
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(IComDerived, 6)

  const double mcx = m * c(0);
  const double mcy = m * c(1);
  const double mcz = m * c(2);
  const double Imcxx = I(0) + m * (c(1) * c(1) + c(2) * c(2));
  const double Imcyy = I(1) + m * (c(0) * c(0) + c(2) * c(2));
  const double Imczz = I(2) + m * (c(0) * c(0) + c(1) * c(1));
  const double Imcxy = I(3) - m * c(0) * c(1);
  const double Imcxz = I(4) - m * c(0) * c(2);
  const double Imcyz = I(5) - m * c(1) * c(2);
  *this <<  m,    0,    0,    0,      mcz,   -mcy,
            0,    m,    0,   -mcz,    0,      mcx,
            0,    0,    m,    mcy,   -mcx,    0,
            0,   -mcz,  mcy,  Imcxx,  Imcxy,  Imcxz,
            mcz,  0,   -mcx,  Imcxy,  Imcyy,  Imcyz,
           -mcy,  mcx,  0,    Imcxz,  Imcyz,  Imczz;
}

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_INERTIA_H_
