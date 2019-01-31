/**
 * spatial_inertia.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 17, 2018
 * Authors: Toki Migimatsu
 */

#ifndef EIGEN_SPATIAL_INERTIA_H_
#define EIGEN_SPATIAL_INERTIA_H_

#include "spatial_motion_base.h"
#include "spatial_force_base.h"

#include <utility>  // std::move

namespace Eigen {

template<typename _Scalar>
class SpatialInertia {

 public:
  SpatialInertia() {}

  SpatialInertia(const SpatialInertia<_Scalar>& other);
  SpatialInertia(SpatialInertia<_Scalar>&& other);

  SpatialInertia<_Scalar>& operator=(const SpatialInertia<_Scalar>& other) {
    mass = other.mass;
    com = other.com;
    I_com = other.I_com;
    return *this;
  }
  SpatialInertia<_Scalar>& operator=(SpatialInertia<_Scalar>&& other) {
    mass = other.mass;
    com = std::move(other.com);
    I_com = std::move(other.I_com);
    return *this;
  }

  template<typename ComDerived, typename IComDerived>
  SpatialInertia(_Scalar mass,
                 const MatrixBase<ComDerived>& com,
                 const MatrixBase<IComDerived>& I_com_flat);

  SpatialInertiaMatrix<_Scalar> matrix() const;

  template<typename OtherDerived> struct motion_product_return_type {
    typedef typename ScalarBinaryOpTraits<_Scalar, typename internal::traits<OtherDerived>::Scalar>::ReturnType Scalar;
    typedef SpatialForce<Scalar, internal::traits<OtherDerived>::ColsAtCompileTime> type;
  };

  template<typename OtherDerived>
  typename motion_product_return_type<OtherDerived>::type
  operator*(const SpatialMotionBase<OtherDerived>& other) const;

  // TODO: inline
  SpatialInertia<_Scalar>& operator+=(const SpatialInertia<_Scalar>& other) {
    Eigen::Vector3d com_new = (mass * com + other.mass * other.com) / (mass + other.mass);
    I_com += other.I_com -
             mass * ctrl_utils::Eigen::DoubleCrossMatrix(com_new - com) -
             other.mass * ctrl_utils::Eigen::DoubleCrossMatrix(com_new - other.com);
    mass += other.mass;
    com = std::move(com_new);
    return *this;
  }

  SpatialInertia<_Scalar> operator+(const SpatialInertia<_Scalar>& other) const {
    SpatialInertia<_Scalar> I_total;
    I_total.com = (mass * com + other.mass * other.com) / (mass + other.mass);
    I_total.I_com = I_com + other.I_com -
                    mass * ctrl_utils::Eigen::DoubleCrossMatrix(I_total.com - com) -
                    other.mass * ctrl_utils::Eigen::DoubleCrossMatrix(I_total.com - other.com);
    I_total.mass = mass + other.mass;
    return I_total;
  }

  bool operator==(const SpatialInertia<_Scalar>& other) const {
    return mass == other.mass && com == other.com && I_com == other.I_com;
  }

  bool operator!=(const SpatialInertia<_Scalar>& other) const {
    return !(*this == other);
  }

  Eigen::Matrix<_Scalar,6,1> I_com_flat() const {
    Eigen::Matrix<_Scalar,6,1> result;
    result << I_com(0,0), I_com(1,1), I_com(2,2), I_com(0,1), I_com(0,2), I_com(1,2);
    return result;
  }

  double mass = 0;
  Matrix<_Scalar,3,1> com = Matrix<_Scalar,3,1>::Zero();
  Matrix<_Scalar,3,3> I_com = Matrix<_Scalar,3,3>::Zero();

};

template<typename _Scalar>
class SpatialInertiaMatrix : public Matrix<_Scalar,6,6> {

 public:
  SpatialInertiaMatrix() {
    this->setZero();
  }

  SpatialInertiaMatrix(const SpatialInertia<_Scalar>& other);

  template<typename OtherDerived>
  SpatialInertiaMatrix(const MatrixBase<OtherDerived>& other) : Matrix<_Scalar,6,6>(other) {};

  template<typename OtherDerived>
  SpatialInertiaMatrix(MatrixBase<OtherDerived>&& other) : Matrix<_Scalar,6,6>(std::move(other)) {};

  template<typename OtherDerived> struct motion_product_return_type {
    typedef typename ScalarBinaryOpTraits<_Scalar, typename internal::traits<OtherDerived>::Scalar>::ReturnType Scalar;
    typedef SpatialForce<Scalar, internal::traits<OtherDerived>::ColsAtCompileTime> type;
  };

  template<typename OtherDerived>
  typename motion_product_return_type<OtherDerived>::type
  operator*(const SpatialMotionBase<OtherDerived>& other) const {
    return Matrix<_Scalar,6,6>::operator*(other.matrix());
  };

  // TODO: inline
  SpatialInertiaMatrix<_Scalar>& operator+=(const SpatialInertia<_Scalar>& other) {
    *this += other.matrix();
    return *this;
  }
  template<typename OtherDerived>
  SpatialInertiaMatrix<_Scalar>& operator+=(const MatrixBase<OtherDerived>& other) {
    Matrix<_Scalar,6,6>::operator+=(other);
    return *this;
  }

};

// Transform operations
template<typename TransformScalar, int Dim, typename Scalar>
inline SpatialInertia<Scalar>
operator*(const Translation<TransformScalar, Dim>& T, const SpatialInertia<Scalar>& I) {
  EIGEN_STATIC_ASSERT(Dim==3,YOU_CAN_ONLY_APPLY_3D_TRANSFORMS_TO_SPATIAL_INERTIAS);

  SpatialInertia<Scalar> result;
  result.mass = I.mass;
  result.com = T.translation() + I.com;
  result.I_com = I.I_com;
  return result;
}

template<typename TransformScalar, int Dim, int Mode, int Options, typename Scalar>
inline SpatialInertia<Scalar>
operator*(const Transform<TransformScalar, Dim, Mode, Options>& T, const SpatialInertia<Scalar>& I) {
  EIGEN_STATIC_ASSERT(Dim==3,YOU_CAN_ONLY_APPLY_3D_TRANSFORMS_TO_SPATIAL_INERTIAS);
  EIGEN_STATIC_ASSERT(Mode==int(Isometry),YOU_CAN_ONLY_APPLY_ISOMETRY_TRANSFORMS_TO_SPATIAL_INERTIAS);

  SpatialInertia<Scalar> result;
  result.mass = I.mass;
  result.com = T * I.com;
  result.I_com = T.linear() * I.I_com * T.linear().transpose();
  return result;
}

template<typename TransformScalar, int Dim, typename Scalar>
inline SpatialInertiaMatrix<Scalar>
operator*(const Translation<TransformScalar, Dim>& T, const SpatialInertiaMatrix<Scalar>& I) {
  EIGEN_STATIC_ASSERT(Dim==3,YOU_CAN_ONLY_APPLY_3D_TRANSFORMS_TO_SPATIAL_INERTIAS);

  SpatialInertiaMatrix<Scalar> result = I;
  result.template rightCols<3>() -= result.template leftCols<3>().rowwise().cross(T.translation());
  result.template bottomRows<3>() -= result.template topRows<3>().colwise().cross(T.translation());
  return result;
}

template<typename TransformScalar, int Dim, int Mode, int Options, typename Scalar>
inline SpatialInertiaMatrix<Scalar>
operator*(const Transform<TransformScalar, Dim, Mode, Options>& T, const SpatialInertiaMatrix<Scalar>& I) {
  EIGEN_STATIC_ASSERT(Dim==3,YOU_CAN_ONLY_APPLY_3D_TRANSFORMS_TO_SPATIAL_INERTIAS);
  EIGEN_STATIC_ASSERT(Mode==int(Isometry),YOU_CAN_ONLY_APPLY_ISOMETRY_TRANSFORMS_TO_SPATIAL_INERTIAS);

  SpatialInertiaMatrix<Scalar> result = I;
  result.template leftCols<3>() = result.template leftCols<3>() * T.linear().transpose();
  result.template rightCols<3>() = result.template rightCols<3>() * T.linear().transpose() -
      result.template leftCols<3>().rowwise().cross(T.translation());
  result.template topRows<3>() = T.linear() * result.template topRows<3>();
  result.template bottomRows<3>() = T.linear() * result.template bottomRows<3>() -
      result.template topRows<3>().colwise().cross(T.translation());
  return result;
}

template<typename _Scalar>
SpatialInertiaMatrix<_Scalar> SpatialInertia<_Scalar>::matrix() const {
  const double mcx = mass * com(0);
  const double mcy = mass * com(1);
  const double mcz = mass * com(2);
  const double Imcxx = I_com(0,0) + mass * (com(1) * com(1) + com(2) * com(2));
  const double Imcyy = I_com(1,1) + mass * (com(0) * com(0) + com(2) * com(2));
  const double Imczz = I_com(2,2) + mass * (com(0) * com(0) + com(1) * com(1));
  const double Imcxy = I_com(0,1) - mass * com(0) * com(1);
  const double Imcxz = I_com(0,2) - mass * com(0) * com(2);
  const double Imcyz = I_com(1,2) - mass * com(1) * com(2);
  SpatialInertiaMatrix<_Scalar> result;
  result <<  mass, 0,    0,    0,      mcz,   -mcy,
             0,    mass, 0,   -mcz,    0,      mcx,
             0,    0,    mass, mcy,   -mcx,    0,
             0,   -mcz,  mcy,  Imcxx,  Imcxy,  Imcxz,
             mcz,  0,   -mcx,  Imcxy,  Imcyy,  Imcyz,
            -mcy,  mcx,  0,    Imcxz,  Imcyz,  Imczz;
  return result;
}

template<typename _Scalar>
SpatialInertiaMatrix<_Scalar>::SpatialInertiaMatrix(const SpatialInertia<_Scalar>& other) {
  const double& mass               = other.mass;
  const Matrix<_Scalar,3,1>& com   = other.com;
  const Matrix<_Scalar,3,3>& I_com = other.I_com;
  const double mcx = mass * com(0);
  const double mcy = mass * com(1);
  const double mcz = mass * com(2);
  const double Imcxx = I_com(0,0) + mass * (com(1) * com(1) + com(2) * com(2));
  const double Imcyy = I_com(1,1) + mass * (com(0) * com(0) + com(2) * com(2));
  const double Imczz = I_com(2,2) + mass * (com(0) * com(0) + com(1) * com(1));
  const double Imcxy = I_com(0,1) - mass * com(0) * com(1);
  const double Imcxz = I_com(0,2) - mass * com(0) * com(2);
  const double Imcyz = I_com(1,2) - mass * com(1) * com(2);
  *this <<  mass, 0,    0,    0,      mcz,   -mcy,
            0,    mass, 0,   -mcz,    0,      mcx,
            0,    0,    mass, mcy,   -mcx,    0,
            0,   -mcz,  mcy,  Imcxx,  Imcxy,  Imcxz,
            mcz,  0,   -mcx,  Imcxy,  Imcyy,  Imcyz,
           -mcy,  mcx,  0,    Imcxz,  Imcyz,  Imczz;
}

template<typename _Scalar>
SpatialInertia<_Scalar>::SpatialInertia(const SpatialInertia<_Scalar>& other)
    : mass(other.mass), com(other.com), I_com(other.I_com) {}

template<typename _Scalar>
SpatialInertia<_Scalar>::SpatialInertia(SpatialInertia<_Scalar>&& other)
    : mass(other.mass), com(std::move(other.com)), I_com(std::move(other.I_com)) {}

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
