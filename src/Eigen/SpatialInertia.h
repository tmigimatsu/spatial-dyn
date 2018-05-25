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

#include <utility>  // std::move

namespace Eigen {

template<typename _Scalar>
class SpatialInertia {

 public:
  SpatialInertia() {}

  SpatialInertia(const SpatialInertia<_Scalar>& other);
  // SpatialInertia(SpatialInertia<_Scalar>&& other);

  template<typename ComDerived, typename IComDerived>
  SpatialInertia(_Scalar mass,
                 const MatrixBase<ComDerived>& com,
                 const MatrixBase<IComDerived>& I_com_flat);

  Matrix<_Scalar,6,6> matrix() const;

  template<typename OtherDerived> struct motion_product_return_type {
    typedef typename ScalarBinaryOpTraits<_Scalar, typename internal::traits<OtherDerived>::Scalar>::ReturnType Scalar;
    typedef SpatialForce<Scalar, internal::traits<OtherDerived>::ColsAtCompileTime> type;
  };

  template<typename OtherDerived>
  typename motion_product_return_type<OtherDerived>::type
  operator*(const SpatialMotionBase<OtherDerived>& other) const;

  SpatialInertia<_Scalar>& operator+=(const SpatialInertia<_Scalar>& other) {
    Eigen::Vector3d com_new = (mass * com + other.mass * other.com) / (mass + other.mass);
    I_com += other.I_com -
             mass * (com_new - com).doubleCrossMatrix() -
             other.mass * (com_new - other.com).doubleCrossMatrix();
    mass += other.mass;
    com = std::move(com_new);
    return *this;
  }

  double mass;
  Matrix<_Scalar,3,1> com;
  Matrix<_Scalar,3,3> I_com;

};

template<typename _Scalar>
Matrix<_Scalar,6,6> SpatialInertia<_Scalar>::matrix() const {
  const double mcx = mass * com(0);
  const double mcy = mass * com(1);
  const double mcz = mass * com(2);
  const double Imcxx = I_com(0,0) + mass * (com(1) * com(1) + com(2) * com(2));
  const double Imcyy = I_com(1,1) + mass * (com(0) * com(0) + com(2) * com(2));
  const double Imczz = I_com(2,2) + mass * (com(0) * com(0) + com(1) * com(1));
  const double Imcxy = I_com(0,1) - mass * com(0) * com(1);
  const double Imcxz = I_com(0,2) - mass * com(0) * com(2);
  const double Imcyz = I_com(1,2) - mass * com(1) * com(2);
  Matrix<_Scalar,6,6> result;
  result <<  mass, 0,    0,    0,      mcz,   -mcy,
             0,    mass, 0,   -mcz,    0,      mcx,
             0,    0,    mass, mcy,   -mcx,    0,
             0,   -mcz,  mcy,  Imcxx,  Imcxy,  Imcxz,
             mcz,  0,   -mcx,  Imcxy,  Imcyy,  Imcyz,
            -mcy,  mcx,  0,    Imcxz,  Imcyz,  Imczz;
  return result;
}

template<typename _Scalar>
SpatialInertia<_Scalar>::SpatialInertia(const SpatialInertia<_Scalar>& other)
    : mass(other.mass) {
  // TODO: Why can't this be initialized?
  com = other.com;
  I_com = other.I_com;
}

// template<typename _Scalar>
// SpatialInertia<_Scalar>::SpatialInertia(SpatialInertia<_Scalar>&& other)
//     : mass(other.mass), com(std::move(other.com)), I_com(std::move(other.I_com)) {}

template<typename _Scalar>
template<typename ComDerived, typename IComDerived>
SpatialInertia<_Scalar>::SpatialInertia(_Scalar m,
    const MatrixBase<ComDerived>& c,
    const MatrixBase<IComDerived>& I) : mass(m), com(c) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(ComDerived, 3)
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(IComDerived, 6)

  I_com << I(0), I(3), I(4),
           I(3), I(1), I(5),
           I(4), I(5), I(2);
}

template<typename _Scalar>
template<typename OtherDerived>
typename SpatialInertia<_Scalar>::template motion_product_return_type<OtherDerived>::type
SpatialInertia<_Scalar>::operator*(const SpatialMotionBase<OtherDerived>& other) const {
  typename motion_product_return_type<OtherDerived>::type result;
  Matrix<_Scalar,3,1> mc = mass * com;
  result.template topRows<3>() = mass * other.matrix().template topRows<3>() +
                                 other.matrix().template bottomRows<3>().cross(mc);
  result.template bottomRows<3>() = I_com * other.matrix().template bottomRows<3>() -
                                    other.matrix().template topRows<3>().cross(mc) -
                                    other.matrix().template bottomRows<3>().cross(com).cross(mc);
  return result;
}

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_INERTIA_H_
