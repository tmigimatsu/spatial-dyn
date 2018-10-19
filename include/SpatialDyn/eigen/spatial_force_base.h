/**
 * spatial_force_base.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 11, 2018
 * Authors: Toki Migimatsu
 */

#ifndef EIGEN_SPATIAL_FORCE_BASE_H_
#define EIGEN_SPATIAL_FORCE_BASE_H_

#include "spatial_declarations.h"

namespace Eigen {

template<typename Derived>
class SpatialForceBase : public DenseBase<Derived> {

 public:
  typedef SpatialForceBase StorageBaseType;
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

  typedef SpatialForce<Scalar,6> SquareMatrixType;

  typedef SpatialForce<Scalar, ColsAtCompileTime, AutoAlign | (Flags & RowMajorBit ? RowMajor : ColMajor),
                        MaxColsAtCompileTime> PlainObject;

  typedef CwiseNullaryOp<internal::scalar_constant_op<Scalar>,PlainObject> ConstantReturnType;

  typedef CwiseNullaryOp<internal::scalar_identity_op<Scalar>,PlainObject> IdentityReturnType;

  typedef Block<const CwiseNullaryOp<internal::scalar_identity_op<Scalar>, SquareMatrixType>,
                6, internal::traits<Derived>::ColsAtCompileTime> BasisReturnType;

  typedef Block<MatrixWrapper<Derived>, 3, ColsAtCompileTime> CartesianBlockType;
  typedef const Block<const MatrixWrapper<const Derived>, 3, ColsAtCompileTime> ConstCartesianBlockType;

  template<typename OtherDerived> struct cross_product_return_type {
    typedef typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,typename internal::traits<OtherDerived>::Scalar>::ReturnType Scalar;
    typedef SpatialForce<Scalar,SpatialForceBase::ColsAtCompileTime> type;
  };

#define EIGEN_CURRENT_STORAGE_BASE_CLASS Eigen::SpatialForceBase
#define EIGEN_DOC_UNARY_ADDONS(X,Y)
#include "Eigen/src/plugins/CommonCwiseUnaryOps.h"
#include "Eigen/src/plugins/CommonCwiseBinaryOps.h"
#include "Eigen/src/plugins/MatrixCwiseUnaryOps.h"
#include "Eigen/src/plugins/MatrixCwiseBinaryOps.h"
#undef EIGEN_CURRENT_STORAGE_BASE_CLASS
#undef EIGEN_DOC_UNARY_ADDONS

  Derived& operator=(const SpatialForceBase& other);

  template <typename OtherDerived>
  Derived& operator=(const DenseBase<OtherDerived>& other);

  template <typename OtherDerived>
  Derived& operator=(const EigenBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator=(const ReturnByValue<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator+=(const SpatialForceBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator+=(const SpatialMotionBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator-=(const SpatialForceBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator-=(const SpatialMotionBase<OtherDerived>& other);

  template<typename OtherDerived>
  const Product<Derived,OtherDerived> operator*(const SpatialForceBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  SpatialForce<typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,
                                             typename internal::traits<OtherDerived>::Scalar>::ReturnType,
               OtherDerived::ColsAtCompileTime>
  operator*(const MatrixBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  Derived& operator*=(const SpatialForceBase<OtherDerived>& other);

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
  dot(const SpatialMotionBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,typename internal::traits<OtherDerived>::Scalar>::ReturnType
  dot(const SpatialForceBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  Matrix<typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,typename internal::traits<OtherDerived>::Scalar>::ReturnType,
         internal::traits<Derived>::ColsAtCompileTime,
         internal::traits<OtherDerived>::ColsAtCompileTime>
  transposeProduct(const SpatialMotionBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  bool operator==(const SpatialForceBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  bool operator!=(const SpatialForceBase<OtherDerived>& other) const;

  MatrixWrapper<Derived> matrix();
  const MatrixWrapper<const Derived> matrix() const;

  ArrayWrapper<Derived> array();
  const ArrayWrapper<const Derived> array() const;

  const Matrix<typename internal::traits<Derived>::Scalar,
               internal::traits<Derived>::ColsAtCompileTime, 6> transpose() const;

  CartesianBlockType linear();
  ConstCartesianBlockType linear() const;

  CartesianBlockType angular();
  ConstCartesianBlockType angular() const;

 protected:
  SpatialForceBase() : Base() {}

 private:
  // TODO: Implement?
  template<typename OtherDerived>
  explicit SpatialForceBase(const SpatialForceBase<OtherDerived>&);

};

// from Assign.h
template<typename Derived>
inline Derived& SpatialForceBase<Derived>::operator=(const SpatialForceBase& other) {
  internal::call_assignment(derived(), other.derived());
  return derived();
}

template<typename Derived>
template <typename OtherDerived>
inline Derived& SpatialForceBase<Derived>::operator=(const DenseBase<OtherDerived>& other) {
  internal::call_assignment(derived(), other.derived());
  return derived();
}

template<typename Derived>
template <typename OtherDerived>
inline Derived& SpatialForceBase<Derived>::operator=(const EigenBase<OtherDerived>& other) {
  internal::call_assignment(derived(), other.derived());
  return derived();
}

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialForceBase<Derived>::operator=(const ReturnByValue<OtherDerived>& other) {
  other.derived().evalTo(derived());
  return derived();
}

// from CwiseBinaryOp.h
template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialForceBase<Derived>::operator+=(const SpatialForceBase<OtherDerived>& other) {
  call_assignment(derived(), other.derived(), internal::add_assign_op<Scalar,typename OtherDerived::Scalar>());
  return derived();
}

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialForceBase<Derived>::operator+=(const SpatialMotionBase<OtherDerived>&) {
  EIGEN_STATIC_ASSERT(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar))==-1,YOU_CANNOT_ADD_SPATIAL_MOTION_AND_SPATIAL_FORCE_VECTORS);
  return *this;
};

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialForceBase<Derived>::operator-=(const SpatialForceBase<OtherDerived>& other) {
  call_assignment(derived(), other.derived(), internal::sub_assign_op<Scalar,typename OtherDerived::Scalar>());
  return derived();
}

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialForceBase<Derived>::operator-=(const SpatialMotionBase<OtherDerived>&) {
  EIGEN_STATIC_ASSERT(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar))==-1,YOU_CANNOT_ADD_SPATIAL_MOTION_AND_SPATIAL_FORCE_VECTORS);
  return *this;
}

template<typename Derived>
template<typename OtherDerived>
inline const Product<Derived,OtherDerived>
SpatialForceBase<Derived>::operator*(const SpatialForceBase<OtherDerived>&) const {
  EIGEN_STATIC_ASSERT(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar))==-1,YOU_CANNOT_MULTIPLY_TWO_SPATIAL_FORCE_VECTORS);
};

template<typename Derived>
template<typename OtherDerived>
inline SpatialForce<typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,
                                                  typename internal::traits<OtherDerived>::Scalar>::ReturnType,
                    OtherDerived::ColsAtCompileTime>
SpatialForceBase<Derived>::operator*(const MatrixBase<OtherDerived>& other) const {
  EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == Dynamic ||
                      OtherDerived::RowsAtCompileTime == Dynamic ||
                      int(Derived::ColsAtCompileTime) == int(OtherDerived::RowsAtCompileTime),
                      INVALID_MATRIX_PRODUCT);
  // TODO: Make not lazy
  return Product<Derived, OtherDerived, LazyProduct>(derived(), other.derived());
};

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialForceBase<Derived>::operator*=(const SpatialForceBase<OtherDerived>&) {
  EIGEN_STATIC_ASSERT(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar))==-1,YOU_CANNOT_MULTIPLY_TWO_SPATIAL_FORCE_VECTORS);
};

// from CwiseNullaryOp.h
template<typename Derived>
template<typename CustomNullaryOp>
inline const CwiseNullaryOp<CustomNullaryOp, typename SpatialForceBase<Derived>::PlainObject>
SpatialForceBase<Derived>::NullaryExpr(Index, Index cols, const CustomNullaryOp& func) {
  return CwiseNullaryOp<CustomNullaryOp, PlainObject>(6, cols, func);
}

template<typename Derived>
template<typename CustomNullaryOp>
inline const CwiseNullaryOp<CustomNullaryOp, typename SpatialForceBase<Derived>::PlainObject>
SpatialForceBase<Derived>::NullaryExpr(Index cols, const CustomNullaryOp& func) {
  return CwiseNullaryOp<CustomNullaryOp, PlainObject>(6, cols, func);
}

template<typename Derived>
template<typename CustomNullaryOp>
inline const CwiseNullaryOp<CustomNullaryOp, typename SpatialForceBase<Derived>::PlainObject>
SpatialForceBase<Derived>::NullaryExpr(const CustomNullaryOp& func) {
  return CwiseNullaryOp<CustomNullaryOp, PlainObject>(6, ColsAtCompileTime, func);
}

template<typename Derived>
inline const typename SpatialForceBase<Derived>::ConstantReturnType
SpatialForceBase<Derived>::Constant(Index cols, const Scalar& value) {
  return NullaryExpr(cols, internal::scalar_constant_op<Scalar>(value));
};

template<typename Derived>
inline const typename SpatialForceBase<Derived>::ConstantReturnType
SpatialForceBase<Derived>::Constant(const Scalar& value) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived)
  return NullaryExpr(ColsAtCompileTime, internal::scalar_constant_op<Scalar>(value));
};

template<typename Derived>
inline const typename SpatialForceBase<Derived>::ConstantReturnType
SpatialForceBase<Derived>::Zero(Index cols) {
  return Constant(cols, Scalar(0));
};

template<typename Derived>
inline const typename SpatialForceBase<Derived>::ConstantReturnType
SpatialForceBase<Derived>::Zero() {
  return Constant(Scalar(0));
};

template<typename Derived>
inline const typename SpatialForceBase<Derived>::ConstantReturnType
SpatialForceBase<Derived>::Ones(Index cols) {
  return Constant(cols, Scalar(1));
};

template<typename Derived>
inline const typename SpatialForceBase<Derived>::ConstantReturnType
SpatialForceBase<Derived>::Ones() {
  return Constant(Scalar(1));
};

template<typename Derived>
inline const typename SpatialForceBase<Derived>::IdentityReturnType
SpatialForceBase<Derived>::Identity() {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived)
  return SpatialForceBase<Derived>::NullaryExpr(ColsAtCompileTime, internal::scalar_identity_op<Scalar>());
}

template<typename Derived>
inline const typename SpatialForceBase<Derived>::IdentityReturnType
SpatialForceBase<Derived>::Identity(Index cols) {
  return SpatialForceBase<Derived>::NullaryExpr(cols, internal::scalar_identity_op<Scalar>());
}

template<typename Derived>
inline const typename SpatialForceBase<Derived>::BasisReturnType SpatialForceBase<Derived>::Unit(Index i) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
  return BasisReturnType(SquareMatrixType::Identity(), i);
}

template<typename Derived>
inline const typename SpatialForceBase<Derived>::BasisReturnType SpatialForceBase<Derived>::UnitLinX() {
  return Derived::Unit(0);
}

template<typename Derived>
inline const typename SpatialForceBase<Derived>::BasisReturnType SpatialForceBase<Derived>::UnitLinY() {
  return Derived::Unit(1);
}

template<typename Derived>
inline const typename SpatialForceBase<Derived>::BasisReturnType SpatialForceBase<Derived>::UnitLinZ() {
  return Derived::Unit(2);
}

template<typename Derived>
inline const typename SpatialForceBase<Derived>::BasisReturnType SpatialForceBase<Derived>::UnitAngX() {
  return Derived::Unit(3);
}

template<typename Derived>
inline const typename SpatialForceBase<Derived>::BasisReturnType SpatialForceBase<Derived>::UnitAngY() {
  return Derived::Unit(4);
}

template<typename Derived>
inline const typename SpatialForceBase<Derived>::BasisReturnType SpatialForceBase<Derived>::UnitAngZ() {
  return Derived::Unit(5);
}

template<typename Derived>
inline Derived& SpatialForceBase<Derived>::setIdentity() {
  return internal::setIdentity_impl<Derived>::run(derived());
}

template<typename Derived>
inline Derived& SpatialForceBase<Derived>::setIdentity(Index cols) {
  derived().resize(NoChange, cols);
  return setIdentity();
}

template<typename Derived>
inline Derived& SpatialForceBase<Derived>::setUnit(Index i) {
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
SpatialForceBase<Derived>::dot(const SpatialForceBase<OtherDerived>&) const {
  EIGEN_STATIC_ASSERT(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar))==-1,YOU_CANNOT_DOT_TWO_SPATIAL_FORCE_VECTORS);
  return 0;
}

template<typename Derived>
template<typename OtherDerived>
inline typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,
                                     typename internal::traits<OtherDerived>::Scalar>::ReturnType
SpatialForceBase<Derived>::dot(const SpatialMotionBase<OtherDerived>& other) const {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(OtherDerived)
  typedef internal::scalar_conj_product_op<typename internal::traits<Derived>::Scalar,
                                           typename internal::traits<OtherDerived>::Scalar> conj_prod;
  return CwiseBinaryOp<conj_prod, const Derived, const OtherDerived>(
      derived(), other.derived(), conj_prod()).sum();
}

template<typename Derived>
template<typename OtherDerived>
inline Matrix<typename ScalarBinaryOpTraits<typename internal::traits<Derived>::Scalar,
                                            typename internal::traits<OtherDerived>::Scalar>::ReturnType,
              internal::traits<Derived>::ColsAtCompileTime,
              internal::traits<OtherDerived>::ColsAtCompileTime>
SpatialForceBase<Derived>::transposeProduct(const SpatialMotionBase<OtherDerived>& other) const {
  return derived().transpose() * other.derived().matrix();
}

template<typename Derived>
template<typename OtherDerived>
inline bool SpatialForceBase<Derived>::operator==(const SpatialForceBase<OtherDerived>& other) const {
  return cwiseEqual(other).all();
}

template<typename Derived>
template<typename OtherDerived>
inline bool SpatialForceBase<Derived>::operator!=(const SpatialForceBase<OtherDerived>& other) const {
  return cwiseNotEqual(other).any();
}

template<typename Derived>
inline MatrixWrapper<Derived> SpatialForceBase<Derived>::matrix() {
  return MatrixWrapper<Derived>(derived());
}

template<typename Derived>
inline const MatrixWrapper<const Derived> SpatialForceBase<Derived>::matrix() const {
  return MatrixWrapper<const Derived>(derived());
}

template<typename Derived>
inline ArrayWrapper<Derived> SpatialForceBase<Derived>::array() {
  return ArrayWrapper<Derived>(derived());
}

template<typename Derived>
inline const ArrayWrapper<const Derived> SpatialForceBase<Derived>::array() const {
  return ArrayWrapper<const Derived>(derived());
}

template<typename Derived>
inline const Matrix<typename internal::traits<Derived>::Scalar,
                    internal::traits<Derived>::ColsAtCompileTime, 6>
SpatialForceBase<Derived>::transpose() const {
  return MatrixWrapper<const Derived>(derived()).transpose();
}

template<typename Derived>
inline typename SpatialForceBase<Derived>::CartesianBlockType
SpatialForceBase<Derived>::linear() {
  return this->matrix().template topRows<3>();
}

template<typename Derived>
inline typename SpatialForceBase<Derived>::ConstCartesianBlockType
SpatialForceBase<Derived>::linear() const {
  return this->matrix().template topRows<3>();
}

template<typename Derived>
inline typename SpatialForceBase<Derived>::CartesianBlockType
SpatialForceBase<Derived>::angular() {
  return this->matrix().template bottomRows<3>();
}

template<typename Derived>
inline typename SpatialForceBase<Derived>::ConstCartesianBlockType
SpatialForceBase<Derived>::angular() const {
  return this->matrix().template bottomRows<3>();
}

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_FORCE_BASE_H_
