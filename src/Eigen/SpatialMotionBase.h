/**
 * SpatialMotionBase.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 11, 2018
 * Authors: Toki Migimatsu
 */

#ifndef EIGEN_SPATIAL_MOTION_BASE_H_
#define EIGEN_SPATIAL_MOTION_BASE_H_

#include "SpatialDeclarations.h"
#include "SpatialForceBase.h"

namespace Eigen {

template<typename Derived>
class SpatialMotionBase : public DenseBase<Derived> {

 public:
  typedef SpatialMotionBase StorageBaseType;
  typedef typename internal::traits<Derived>::StorageKind StorageKind;
  typedef typename internal::traits<Derived>::StorageIndex StorageIndex;
  typedef typename internal::traits<Derived>::Scalar Scalar;
  typedef typename internal::packet_traits<Scalar>::type PacketScalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;

  typedef DenseBase<Derived> Base;
  using Base::RowsAtCompileTime;
  using Base::ColsAtCompileTime;
  using Base::SizeAtCompileTime;
  using Base::MaxRowsAtCompileTime;
  using Base::MaxColsAtCompileTime;
  using Base::MaxSizeAtCompileTime;
  using Base::IsVectorAtCompileTime;
  using Base::Flags;

  using Base::derived;
  using Base::const_cast_derived;
  using Base::rows;
  using Base::cols;
  using Base::size;
  using Base::coeff;
  using Base::coeffRef;
  using Base::lazyAssign;
  using Base::eval;
  using Base::operator+=;
  using Base::operator-=;
  using Base::operator*=;
  using Base::operator/=;

  typedef typename Base::CoeffReturnType CoeffReturnType;
  typedef typename Base::ConstTransposeReturnType ConstTransposeReturnType;
  typedef typename Base::RowXpr RowXpr;
  typedef typename Base::ColXpr ColXpr;

  typedef SpatialMotion<Scalar,6> SquareMatrixType;

  typedef SpatialMotion<Scalar, ColsAtCompileTime, AutoAlign | (Flags & RowMajorBit ? RowMajor : ColMajor),
                        MaxColsAtCompileTime> PlainObject;

  typedef CwiseNullaryOp<internal::scalar_constant_op<Scalar>,PlainObject> ConstantReturnType;

  typedef CwiseNullaryOp<internal::scalar_identity_op<Scalar>,PlainObject> IdentityReturnType;

  typedef Block<const CwiseNullaryOp<internal::scalar_identity_op<Scalar>, SquareMatrixType>,
                6, internal::traits<Derived>::ColsAtCompileTime> BasisReturnType;

  template<typename OtherDerived> struct cross_product_return_type {
    typedef typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,typename internal::traits<OtherDerived>::Scalar>::ReturnType Scalar;
    typedef SpatialMotion<Scalar,SpatialMotionBase::ColsAtCompileTime> type;
  };

#define EIGEN_CURRENT_STORAGE_BASE_CLASS Eigen::SpatialMotionBase
#define EIGEN_DOC_UNARY_ADDONS(X,Y)
#include "Eigen/src/plugins/CommonCwiseUnaryOps.h"
#include "Eigen/src/plugins/CommonCwiseBinaryOps.h"
#include "Eigen/src/plugins/MatrixCwiseUnaryOps.h"
#include "Eigen/src/plugins/MatrixCwiseBinaryOps.h"
#undef EIGEN_CURRENT_STORAGE_BASE_CLASS
#undef EIGEN_DOC_UNARY_ADDONS

  Derived& operator=(const SpatialMotionBase& other);

  template <typename OtherDerived>
  Derived& operator=(const DenseBase<OtherDerived>& other);

  template <typename OtherDerived>
  Derived& operator=(const EigenBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator=(const ReturnByValue<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator+=(const SpatialMotionBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator+=(const SpatialForceBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator-=(const SpatialMotionBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator-=(const SpatialForceBase<OtherDerived>& other);

  template<typename OtherDerived>
  const Product<Derived,OtherDerived> operator*(const SpatialMotionBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  Derived& operator*=(const SpatialMotionBase<OtherDerived>& other);

  template<typename CustomNullaryOp>
  static const CwiseNullaryOp<CustomNullaryOp, PlainObject>
  NullaryExpr(Index rows, Index cols, const CustomNullaryOp& func);

  template<typename CustomNullaryOp>
  static const CwiseNullaryOp<CustomNullaryOp, PlainObject>
  NullaryExpr(Index cols, const CustomNullaryOp& func);

  template<typename CustomNullaryOp>
  static const CwiseNullaryOp<CustomNullaryOp, PlainObject>
  NullaryExpr(const CustomNullaryOp& func);

  static const ConstantReturnType Constant(Index cols, const Scalar& value);
  static const ConstantReturnType Constant(const Scalar& value);
  static const ConstantReturnType Zero(Index cols);
  static const ConstantReturnType Zero();
  static const ConstantReturnType Ones(Index cols);
  static const ConstantReturnType Ones();

  static const IdentityReturnType Identity();
  static const IdentityReturnType Identity(Index cols);
  static const BasisReturnType Unit(Index i);
  static const BasisReturnType UnitLinX();
  static const BasisReturnType UnitLinY();
  static const BasisReturnType UnitLinZ();
  static const BasisReturnType UnitAngX();
  static const BasisReturnType UnitAngY();
  static const BasisReturnType UnitAngZ();

  Derived& setIdentity();
  Derived& setIdentity(Index cols);
  Derived& setUnit(Index i);

  template<typename OtherDerived>
  typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,typename internal::traits<OtherDerived>::Scalar>::ReturnType
  dot(const SpatialForceBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,typename internal::traits<OtherDerived>::Scalar>::ReturnType
  dot(const SpatialMotionBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  typename cross_product_return_type<OtherDerived>::type
  cross(const SpatialMotionBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  inline typename SpatialForceBase<Derived>::template cross_product_return_type<OtherDerived>::type
  cross(const SpatialForceBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  bool operator==(const SpatialMotionBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  bool operator!=(const SpatialMotionBase<OtherDerived>& other) const;

  MatrixWrapper<Derived> matrix();
  const MatrixWrapper<const Derived> matrix() const;

  ArrayWrapper<Derived> array();
  const ArrayWrapper<const Derived> array() const;

  const Matrix<Scalar, SpatialMotionBase::ColsAtCompileTime, 6> transpose() const;

 protected:
  SpatialMotionBase() : Base() {}

 private:
  // TODO: Implement?
  template<typename OtherDerived>
  explicit SpatialMotionBase(const SpatialMotionBase<OtherDerived>&);

};

// from Assign.h
template<typename Derived>
inline Derived& SpatialMotionBase<Derived>::operator=(const SpatialMotionBase& other) {
  internal::call_assignment(derived(), other.derived());
  return derived();
}

template<typename Derived>
template <typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator=(const DenseBase<OtherDerived>& other) {
  internal::call_assignment(derived(), other.derived());
  return derived();
}

template<typename Derived>
template <typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator=(const EigenBase<OtherDerived>& other) {
  internal::call_assignment(derived(), other.derived());
  return derived();
}

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator=(const ReturnByValue<OtherDerived>& other) {
  other.derived().evalTo(derived());
  return derived();
}

// from CwiseBinaryOp.h
template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator+=(const SpatialMotionBase<OtherDerived>& other) {
  call_assignment(derived(), other.derived(), internal::add_assign_op<Scalar,typename OtherDerived::Scalar>());
  return derived();
}

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator+=(const SpatialForceBase<OtherDerived>& other) {
  EIGEN_STATIC_ASSERT(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar))==-1,YOU_CANNOT_ADD_SPATIAL_MOTION_AND_SPATIAL_FORCE_VECTORS);
  return *this;
};

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator-=(const SpatialMotionBase<OtherDerived>& other) {
  call_assignment(derived(), other.derived(), internal::sub_assign_op<Scalar,typename OtherDerived::Scalar>());
  return derived();
}

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator-=(const SpatialForceBase<OtherDerived>& other) {
  EIGEN_STATIC_ASSERT(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar))==-1,YOU_CANNOT_ADD_SPATIAL_MOTION_AND_SPATIAL_FORCE_VECTORS);
  return *this;
}

template<typename Derived>
template<typename OtherDerived>
inline const Product<Derived,OtherDerived>
SpatialMotionBase<Derived>::operator*(const SpatialMotionBase<OtherDerived>& other) const {
  EIGEN_STATIC_ASSERT(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar))==-1,YOU_CANNOT_MULTIPLY_TWO_SPATIAL_MOTION_VECTORS);
};

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator*=(const SpatialMotionBase<OtherDerived>& other) {
  EIGEN_STATIC_ASSERT(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar))==-1,YOU_CANNOT_MULTIPLY_TWO_SPATIAL_MOTION_VECTORS);
};

// from CwiseNullaryOp.h
template<typename Derived>
template<typename CustomNullaryOp>
inline const CwiseNullaryOp<CustomNullaryOp, typename SpatialMotionBase<Derived>::PlainObject>
SpatialMotionBase<Derived>::NullaryExpr(Index rows, Index cols, const CustomNullaryOp& func) {
  return CwiseNullaryOp<CustomNullaryOp, PlainObject>(6, cols, func);
}

template<typename Derived>
template<typename CustomNullaryOp>
inline const CwiseNullaryOp<CustomNullaryOp, typename SpatialMotionBase<Derived>::PlainObject>
SpatialMotionBase<Derived>::NullaryExpr(Index cols, const CustomNullaryOp& func) {
  return CwiseNullaryOp<CustomNullaryOp, PlainObject>(6, cols, func);
}

template<typename Derived>
template<typename CustomNullaryOp>
inline const CwiseNullaryOp<CustomNullaryOp, typename SpatialMotionBase<Derived>::PlainObject>
SpatialMotionBase<Derived>::NullaryExpr(const CustomNullaryOp& func) {
  return CwiseNullaryOp<CustomNullaryOp, PlainObject>(6, ColsAtCompileTime, func);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::ConstantReturnType
SpatialMotionBase<Derived>::Constant(Index cols, const Scalar& value) {
  return NullaryExpr(cols, internal::scalar_constant_op<Scalar>(value));
};

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::ConstantReturnType
SpatialMotionBase<Derived>::Constant(const Scalar& value) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived)
  return NullaryExpr(ColsAtCompileTime, internal::scalar_constant_op<Scalar>(value));
};

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::ConstantReturnType
SpatialMotionBase<Derived>::Zero(Index cols) {
  return Constant(cols, Scalar(0));
};

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::ConstantReturnType
SpatialMotionBase<Derived>::Zero() {
  return Constant(Scalar(0));
};

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::ConstantReturnType
SpatialMotionBase<Derived>::Ones(Index cols) {
  return Constant(cols, Scalar(1));
};

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::ConstantReturnType
SpatialMotionBase<Derived>::Ones() {
  return Constant(Scalar(1));
};

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::IdentityReturnType
SpatialMotionBase<Derived>::Identity() {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived)
  return SpatialMotionBase<Derived>::NullaryExpr(ColsAtCompileTime, internal::scalar_identity_op<Scalar>());
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::IdentityReturnType
SpatialMotionBase<Derived>::Identity(Index cols) {
  return SpatialMotionBase<Derived>::NullaryExpr(cols, internal::scalar_identity_op<Scalar>());
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType SpatialMotionBase<Derived>::Unit(Index i) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
  return BasisReturnType(SquareMatrixType::Identity(), i);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType SpatialMotionBase<Derived>::UnitLinX() {
  return Derived::Unit(0);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType SpatialMotionBase<Derived>::UnitLinY() {
  return Derived::Unit(1);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType SpatialMotionBase<Derived>::UnitLinZ() {
  return Derived::Unit(2);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType SpatialMotionBase<Derived>::UnitAngX() {
  return Derived::Unit(3);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType SpatialMotionBase<Derived>::UnitAngY() {
  return Derived::Unit(4);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType SpatialMotionBase<Derived>::UnitAngZ() {
  return Derived::Unit(5);
}

template<typename Derived>
inline Derived& SpatialMotionBase<Derived>::setIdentity() {
  return internal::setIdentity_impl<Derived>::run(derived());
}

template<typename Derived>
inline Derived& SpatialMotionBase<Derived>::setIdentity(Index cols) {
  derived().resize(NoChange, cols);
  return setIdentity();
}

template<typename Derived>
inline Derived& SpatialMotionBase<Derived>::setUnit(Index i) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  eigen_assert(i < size());
  derived().setZero();
  derived().coeffRef(i) = Scalar(1);
  return derived();
}

// from Dot.h
template<typename Derived>
template<typename OtherDerived>
inline typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,
                                     typename internal::traits<OtherDerived>::Scalar>::ReturnType
SpatialMotionBase<Derived>::dot(const SpatialMotionBase<OtherDerived>& other) const {
  EIGEN_STATIC_ASSERT(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar))==-1,YOU_CANNOT_DOT_TWO_SPATIAL_MOTION_VECTORS);
  return 0;
}

template<typename Derived>
template<typename OtherDerived>
inline typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,
                                     typename internal::traits<OtherDerived>::Scalar>::ReturnType
SpatialMotionBase<Derived>::dot(const SpatialForceBase<OtherDerived>& other) const {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(OtherDerived)
  typedef internal::scalar_conj_product_op<typename internal::traits<Derived>::Scalar,
                                           typename internal::traits<OtherDerived>::Scalar> conj_prod;
  return CwiseBinaryOp<conj_prod, const Derived, const OtherDerived>(
      derived(), other.derived(), conj_prod()).sum();
}

// from Geometry/OrthoMethods.h
template<typename Derived>
template<typename OtherDerived>
inline typename SpatialMotionBase<Derived>::template cross_product_return_type<OtherDerived>::type
SpatialMotionBase<Derived>::cross(const SpatialMotionBase<OtherDerived>& other) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,6)
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(OtherDerived,6)
  typename internal::nested_eval<Derived,4>::type lhs(derived());
  typename internal::nested_eval<OtherDerived,4>::type rhs(other.derived());
  return typename cross_product_return_type<OtherDerived>::type(
    lhs.coeff(1) * rhs.coeff(5) - lhs.coeff(2) * rhs.coeff(4) + lhs.coeff(4) * rhs.coeff(2) - lhs.coeff(5) * rhs.coeff(1),
    lhs.coeff(2) * rhs.coeff(3) - lhs.coeff(0) * rhs.coeff(5) + lhs.coeff(5) * rhs.coeff(0) - lhs.coeff(3) * rhs.coeff(2),
    lhs.coeff(0) * rhs.coeff(4) - lhs.coeff(1) * rhs.coeff(3) + lhs.coeff(3) * rhs.coeff(1) - lhs.coeff(4) * rhs.coeff(0),
    lhs.coeff(4) * rhs.coeff(5) - lhs.coeff(5) * rhs.coeff(4),
    lhs.coeff(5) * rhs.coeff(3) - lhs.coeff(3) * rhs.coeff(5),
    lhs.coeff(3) * rhs.coeff(4) - lhs.coeff(4) * rhs.coeff(3)
  );
}

template<typename Derived>
template<typename OtherDerived>
inline typename SpatialForceBase<Derived>::template cross_product_return_type<OtherDerived>::type
SpatialMotionBase<Derived>::cross(const SpatialForceBase<OtherDerived>& other) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,6)
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(OtherDerived,6)
  typename internal::nested_eval<Derived,4>::type lhs(derived());
  typename internal::nested_eval<OtherDerived,4>::type rhs(other.derived());
  return typename cross_product_return_type<OtherDerived>::type(
    lhs.coeff(4) * rhs.coeff(2) - lhs.coeff(5) * rhs.coeff(1),
    lhs.coeff(5) * rhs.coeff(0) - lhs.coeff(3) * rhs.coeff(2),
    lhs.coeff(3) * rhs.coeff(1) - lhs.coeff(4) * rhs.coeff(0),
    lhs.coeff(1) * rhs.coeff(2) - lhs.coeff(2) * rhs.coeff(1) + lhs.coeff(4) * rhs.coeff(5) - lhs.coeff(5) * rhs.coeff(4),
    lhs.coeff(2) * rhs.coeff(0) - lhs.coeff(0) * rhs.coeff(2) + lhs.coeff(5) * rhs.coeff(3) - lhs.coeff(3) * rhs.coeff(5),
    lhs.coeff(0) * rhs.coeff(1) - lhs.coeff(1) * rhs.coeff(0) + lhs.coeff(3) * rhs.coeff(4) - lhs.coeff(4) * rhs.coeff(3)
  );
}

template<typename Derived>
template<typename OtherDerived>
inline bool SpatialMotionBase<Derived>::operator==(const SpatialMotionBase<OtherDerived>& other) const {
  return cwiseEqual(other).all();
}

template<typename Derived>
template<typename OtherDerived>
inline bool SpatialMotionBase<Derived>::operator!=(const SpatialMotionBase<OtherDerived>& other) const {
  return cwiseNotEqual(other).any();
}

template<typename Derived>
inline MatrixWrapper<Derived> SpatialMotionBase<Derived>::matrix() {
  return MatrixWrapper<Derived>(derived());
}

template<typename Derived>
inline const MatrixWrapper<const Derived> SpatialMotionBase<Derived>::matrix() const {
  return MatrixWrapper<const Derived>(derived());
}

template<typename Derived>
inline ArrayWrapper<Derived> SpatialMotionBase<Derived>::array() {
  return ArrayWrapper<Derived>(derived());
}

template<typename Derived>
inline const ArrayWrapper<const Derived> SpatialMotionBase<Derived>::array() const {
  return ArrayWrapper<const Derived>(derived());
}

template<typename Derived>
inline const Matrix<typename SpatialMotionBase<Derived>::Scalar,
                    SpatialMotionBase<Derived>::ColsAtCompileTime, 6>
SpatialMotionBase<Derived>::transpose() const {
  return MatrixWrapper<const Derived>(derived()).transpose();
}

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_MOTION_BASE_H_
