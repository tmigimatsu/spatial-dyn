/**
 * spatial_motion_base.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 11, 2018
 * Authors: Toki Migimatsu
 */

#ifndef EIGEN_SPATIAL_MOTION_BASE_H_
#define EIGEN_SPATIAL_MOTION_BASE_H_

#include "spatial_declarations.h"

namespace spatial_dyn {

template<typename Derived>
class SpatialMotionBase : public Eigen::DenseBase<Derived> {

 public:
  typedef SpatialMotionBase StorageBaseType;
  typedef typename Eigen::internal::traits<Derived>::StorageKind StorageKind;
  typedef typename Eigen::internal::traits<Derived>::StorageIndex StorageIndex;
  typedef typename Eigen::internal::traits<Derived>::Scalar Scalar;
  typedef typename Eigen::internal::packet_traits<Scalar>::type PacketScalar;
  typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;

  typedef Eigen::DenseBase<Derived> Base;
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

  typedef SpatialMotion<Scalar, ColsAtCompileTime,
                        Eigen::AutoAlign |
                        (Flags & Eigen::RowMajorBit ? Eigen::RowMajor : Eigen::ColMajor),
                        MaxColsAtCompileTime> PlainObject;

  typedef Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<Scalar>,
                                PlainObject> ConstantReturnType;

  typedef Eigen::CwiseNullaryOp<Eigen::internal::scalar_identity_op<Scalar>,
                                PlainObject> IdentityReturnType;

  typedef Eigen::Block<const Eigen::CwiseNullaryOp<Eigen::internal::scalar_identity_op<Scalar>,
                                                   SquareMatrixType>,
                       6, ColsAtCompileTime> BasisReturnType;

  typedef Eigen::Block<Eigen::MatrixWrapper<Derived>, 3, ColsAtCompileTime> CartesianBlockType;

  typedef const Eigen::Block<const Eigen::MatrixWrapper<const Derived>, 3,
                             ColsAtCompileTime> ConstCartesianBlockType;

  template<typename OtherDerived>
  struct SpatialOpTraits {
    typedef typename Eigen::ScalarBinaryOpTraits<Scalar,
                                                   typename OtherDerived::Scalar>::ReturnType ScalarT;
    typedef SpatialMotion<ScalarT, ColsAtCompileTime> MotionReturnType;
    typedef SpatialForce<ScalarT, ColsAtCompileTime> ForceReturnType;
  };

  // #include "Eigen/src/plugins/CommonCwiseUnaryOps.h"
  typedef Eigen::CwiseUnaryOp<Eigen::internal::scalar_opposite_op<Scalar>,
                              const Derived> NegativeReturnType;

  const NegativeReturnType operator-() const;

  template<class NewType>
  struct CastXpr {
    typedef typename Eigen::internal::cast_return_type<Derived,
        const Eigen::CwiseUnaryOp<Eigen::internal::scalar_cast_op<Scalar, NewType>,
                                  const Derived>>::type Type;
  };

  template<typename NewType>
  typename CastXpr<NewType>::Type cast() const;

  template<typename CustomUnaryOp>
  inline const Eigen::CwiseUnaryOp<CustomUnaryOp, const Derived>
  unaryExpr(const CustomUnaryOp& func = CustomUnaryOp()) const;

  template<typename CustomViewOp>
  inline const Eigen::CwiseUnaryView<CustomViewOp, const Derived>
  unaryViewExpr(const CustomViewOp& func = CustomViewOp()) const;

  // #include "Eigen/src/plugins/CommonCwiseBinaryOps.h"
  template<typename OtherDerived>
  struct BinaryOpTraits {
    typedef Eigen::CwiseBinaryOp<Eigen::internal::scalar_difference_op<Scalar,
                                                                       typename OtherDerived::Scalar>,
                                 const Derived, const OtherDerived> DifferenceType;
    typedef Eigen::CwiseBinaryOp<Eigen::internal::scalar_sum_op<Scalar,
                                                                typename OtherDerived::Scalar>,
                                 const Derived, const OtherDerived> SumType;
  };

  // EIGEN_MAKE_CWISE_BINARY_OP(operator-,difference)
  template<typename OtherDerived>
  const typename BinaryOpTraits<OtherDerived>::DifferenceType
  operator-(const SpatialMotionBase<OtherDerived> &other) const;

  // EIGEN_MAKE_CWISE_BINARY_OP(operator+,sum)
  template<typename OtherDerived>
  inline const typename BinaryOpTraits<OtherDerived>::SumType
  operator+(const SpatialMotionBase<OtherDerived> &other) const;

  template<typename T>
  struct ScalarOpTraits {
    typedef typename Eigen::ScalarBinaryOpTraits<Scalar, T,
        Eigen::internal::scalar_product_op<Scalar, T>> RightProduct;
    typedef typename Eigen::ScalarBinaryOpTraits<T, Scalar,
        Eigen::internal::scalar_product_op<T, Scalar>> LeftProduct;
    typedef typename Eigen::ScalarBinaryOpTraits<Scalar, T,
        Eigen::internal::scalar_product_op<Scalar, T>> RightQuotient;
  };

  // EIGEN_MAKE_SCALAR_BINARY_OP_ONTHERIGHT(operator*,product)
  template<typename T>
  Eigen::CwiseBinaryOp<
      Eigen::internal::scalar_product_op<
          Scalar,
          typename Eigen::internal::promote_scalar_arg<Scalar, T,
              Eigen::internal::has_ReturnType<typename ScalarOpTraits<T>::RightProduct>::value>::type>,
      const Derived,
      const typename Eigen::internal::plain_constant_type<
          Derived,
          typename Eigen::internal::promote_scalar_arg<Scalar, T,
              Eigen::internal::has_ReturnType<typename ScalarOpTraits<T>::RightProduct>::value>::type>::type>
  operator*(const T& scalar) const;

  // EIGEN_MAKE_SCALAR_BINARY_OP_ONTHELEFT(operator*,product)
  // See defined function below

  // EIGEN_MAKE_SCALAR_BINARY_OP_ONTHERIGHT(operator/,quotient)
  template<typename T>
  Eigen::CwiseBinaryOp<
      Eigen::internal::scalar_quotient_op<
          Scalar,
          typename Eigen::internal::promote_scalar_arg<Scalar, T,
              Eigen::internal::has_ReturnType<typename ScalarOpTraits<T>::RightQuotient>::value>::type>,
      const Derived,
      const typename Eigen::internal::plain_constant_type<
          Derived,
          typename Eigen::internal::promote_scalar_arg<Scalar, T,
              Eigen::internal::has_ReturnType<typename ScalarOpTraits<T>::RightQuotient>::value>::type>::type>
  operator/(const T& scalar) const;

  template<typename CustomBinaryOp, typename OtherDerived>
  inline const Eigen::CwiseBinaryOp<CustomBinaryOp, const Derived, const OtherDerived>
  binaryExpr(const SpatialMotionBase<OtherDerived> &other,
             const CustomBinaryOp& func = CustomBinaryOp()) const;

  Derived& operator=(const SpatialMotionBase& other);

  template <typename OtherDerived>
  Derived& operator=(const Eigen::DenseBase<OtherDerived>& other);

  template <typename OtherDerived>
  Derived& operator=(const Eigen::EigenBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator=(const Eigen::ReturnByValue<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator+=(const SpatialMotionBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator+=(const SpatialForceBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator-=(const SpatialMotionBase<OtherDerived>& other);

  template<typename OtherDerived>
  Derived& operator-=(const SpatialForceBase<OtherDerived>& other);

  template<typename OtherDerived>
  const Eigen::Product<Derived, OtherDerived>
  operator*(const SpatialMotionBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  SpatialMotion<typename SpatialOpTraits<OtherDerived>::ScalarT, OtherDerived::ColsAtCompileTime>
  operator*(const Eigen::MatrixBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  Derived& operator*=(const SpatialMotionBase<OtherDerived>& other);

  template<typename CustomNullaryOp>
  static const Eigen::CwiseNullaryOp<CustomNullaryOp, PlainObject>
  NullaryExpr(Eigen::Index rows, Eigen::Index cols, const CustomNullaryOp& func);

  template<typename CustomNullaryOp>
  static const Eigen::CwiseNullaryOp<CustomNullaryOp, PlainObject>
  NullaryExpr(Eigen::Index cols, const CustomNullaryOp& func);

  template<typename CustomNullaryOp>
  static const Eigen::CwiseNullaryOp<CustomNullaryOp, PlainObject>
  NullaryExpr(const CustomNullaryOp& func);

  static const ConstantReturnType Constant(Eigen::Index cols, const Scalar& value);
  static const ConstantReturnType Constant(const Scalar& value);
  static const ConstantReturnType Zero(Eigen::Index cols);
  static const ConstantReturnType Zero();
  static const ConstantReturnType Ones(Eigen::Index cols);
  static const ConstantReturnType Ones();

  static const IdentityReturnType Identity();
  static const IdentityReturnType Identity(Eigen::Index cols);
  static const BasisReturnType Unit(Eigen::Index i);
  static const BasisReturnType UnitLinX();
  static const BasisReturnType UnitLinY();
  static const BasisReturnType UnitLinZ();
  static const BasisReturnType UnitAngX();
  static const BasisReturnType UnitAngY();
  static const BasisReturnType UnitAngZ();

  Derived& setIdentity();
  Derived& setIdentity(Eigen::Index cols);
  Derived& setUnit(Eigen::Index i);

  template<typename OtherDerived>
  typename SpatialOpTraits<OtherDerived>::ScalarT
  dot(const SpatialForceBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  typename SpatialOpTraits<OtherDerived>::ScalarT
  dot(const SpatialMotionBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  typename SpatialOpTraits<OtherDerived>::MotionReturnType
  cross(const SpatialMotionBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  typename SpatialOpTraits<OtherDerived>::ForceReturnType
  cross(const SpatialForceBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  bool operator==(const SpatialMotionBase<OtherDerived>& other) const;

  template<typename OtherDerived>
  bool operator!=(const SpatialMotionBase<OtherDerived>& other) const;

  Eigen::MatrixWrapper<Derived> matrix();
  const Eigen::MatrixWrapper<const Derived> matrix() const;

  Eigen::ArrayWrapper<Derived> array();
  const Eigen::ArrayWrapper<const Derived> array() const;

  Eigen::Transpose<Eigen::MatrixWrapper<Derived>> transpose();
  const Eigen::Transpose<Eigen::MatrixWrapper<const Derived>> transpose() const;

  CartesianBlockType linear();
  ConstCartesianBlockType linear() const;

  CartesianBlockType angular();
  ConstCartesianBlockType angular() const;

 protected:
  SpatialMotionBase() : Base() {}

 private:
  // TODO: Implement?
  template<typename OtherDerived>
  explicit SpatialMotionBase(const SpatialMotionBase<OtherDerived>&);

};

// Transform operations
template<typename Scalar, int Dim, typename Derived>
inline typename Derived::PlainObject
operator*(const Eigen::Translation<Scalar, Dim>& T, const SpatialMotionBase<Derived>& m) {
  static_assert(Dim == 3, "YOU_CAN_ONLY_APPLY_3D_TRANSFORMS_TO_SPATIAL_MOTIONS");
  typename Derived::PlainObject result = m;  // Evaluate expr
  result.template topRows<3>() -= m.matrix().template bottomRows<3>().colwise().cross(T.translation());
  return result;
}

template<typename Scalar, int Dim, int Mode, int Options, typename Derived>
inline typename Derived::PlainObject
operator*(const Eigen::Transform<Scalar, Dim, Mode, Options>& T, const SpatialMotionBase<Derived>& m) {
  static_assert(Dim == 3, "YOU_CAN_ONLY_APPLY_3D_TRANSFORMS_TO_SPATIAL_MOTIONS");
  static_assert(Mode == int(Eigen::Isometry), "YOU_CAN_ONLY_APPLY_ISOMETRY_TRANSFORMS_TO_SPATIAL_MOTIONS");

  typename Derived::PlainObject result = m;  // Evaluate expr
  result.template bottomRows<3>() = T.linear() * result.matrix().template bottomRows<3>();
  result.template topRows<3>() = T.linear() * result.matrix().template topRows<3>() -
      result.matrix().template bottomRows<3>().colwise().cross(T.translation());
  return result;
}

// from src/plugins/CommonCwiseUnaryOps.h
template<typename Derived>
inline const typename SpatialMotionBase<Derived>::NegativeReturnType
SpatialMotionBase<Derived>::operator-() const {
  return NegativeReturnType(derived());
}

template<typename Derived>
template<typename NewType>
inline typename SpatialMotionBase<Derived>::template CastXpr<NewType>::Type
SpatialMotionBase<Derived>::cast() const {
  return typename CastXpr<NewType>::Type(derived());
}

template<typename Derived>
template<typename CustomUnaryOp>
inline const Eigen::CwiseUnaryOp<CustomUnaryOp, const Derived>
SpatialMotionBase<Derived>::unaryExpr(const CustomUnaryOp& func) const {
  return Eigen::CwiseUnaryOp<CustomUnaryOp, const Derived>(derived(), func);
}

template<typename Derived>
template<typename CustomViewOp>
inline const Eigen::CwiseUnaryView<CustomViewOp, const Derived>
SpatialMotionBase<Derived>::unaryViewExpr(const CustomViewOp& func) const {
  return Eigen::CwiseUnaryView<CustomViewOp, const Derived>(derived(), func);
}

// from src/plugins/CommonCwiseBinaryOps.h
template<typename Derived>
template<typename OtherDerived>
inline const typename SpatialMotionBase<Derived>::template BinaryOpTraits<OtherDerived>::DifferenceType
SpatialMotionBase<Derived>::operator-(const SpatialMotionBase<OtherDerived> &other) const {
  return typename BinaryOpTraits<OtherDerived>::DifferenceType(derived(), other.derived());
}

template<typename Derived>
template<typename OtherDerived>
inline const typename SpatialMotionBase<Derived>::template BinaryOpTraits<OtherDerived>::SumType
SpatialMotionBase<Derived>::operator+(const SpatialMotionBase<OtherDerived> &other) const {
  return typename BinaryOpTraits<OtherDerived>::SumType(derived(), other.derived());
}

template<typename Derived>
template<typename T>
inline Eigen::CwiseBinaryOp<
    Eigen::internal::scalar_product_op<
        typename SpatialMotionBase<Derived>::Scalar,
        typename Eigen::internal::promote_scalar_arg<typename SpatialMotionBase<Derived>::Scalar, T,
            Eigen::internal::has_ReturnType<
                typename SpatialMotionBase<Derived>::template ScalarOpTraits<T>::RightProduct>::value>::type>,
    const Derived,
    const typename Eigen::internal::plain_constant_type<
        Derived,
        typename Eigen::internal::promote_scalar_arg<typename SpatialMotionBase<Derived>::Scalar, T,
            Eigen::internal::has_ReturnType<
                typename SpatialMotionBase<Derived>::template ScalarOpTraits<T>::RightProduct>::value>::type>::type>
SpatialMotionBase<Derived>::operator*(const T& scalar) const {
  typedef typename Eigen::internal::promote_scalar_arg<Scalar, T,
      Eigen::internal::has_ReturnType<typename ScalarOpTraits<T>::RightProduct>::value>::type PromotedT;
  typedef typename Eigen::internal::plain_constant_type<Derived, PromotedT>::type PlainType;
  typedef typename Eigen::CwiseBinaryOp<Eigen::internal::scalar_product_op<Scalar, PromotedT>,
                                        const Derived, const PlainType> ReturnType;
  typedef typename Eigen::internal::scalar_constant_op<PromotedT> ScalarOp;
  return ReturnType(derived(), PlainType(derived().rows(), derived().cols(), ScalarOp(scalar)));
}

template<typename T, typename Derived>
inline Eigen::CwiseBinaryOp<
    Eigen::internal::scalar_product_op<
        typename Eigen::internal::promote_scalar_arg<typename SpatialMotionBase<Derived>::Scalar, T,
            Eigen::internal::has_ReturnType<
                typename SpatialMotionBase<Derived>::template ScalarOpTraits<T>::LeftProduct>::value>::type,
        typename SpatialMotionBase<Derived>::Scalar>,
    const typename Eigen::internal::plain_constant_type<
        Derived,
        typename Eigen::internal::promote_scalar_arg<typename SpatialMotionBase<Derived>::Scalar, T,
            Eigen::internal::has_ReturnType<
              typename SpatialMotionBase<Derived>::template ScalarOpTraits<T>::LeftProduct>::value>::type>::type,
    const Derived>
operator*(const T& scalar, const SpatialMotionBase<Derived>& m) {
  typedef typename SpatialMotionBase<Derived>::template ScalarOpTraits<T> ScalarOpTraits;
  typedef typename SpatialMotionBase<Derived>::Scalar Scalar;

  typedef typename Eigen::internal::promote_scalar_arg<Scalar, T,
      Eigen::internal::has_ReturnType<typename ScalarOpTraits::LeftProduct>::value>::type PromotedT;
  typedef typename Eigen::internal::plain_constant_type<Derived, PromotedT>::type PlainType;
  typedef typename Eigen::CwiseBinaryOp<Eigen::internal::scalar_product_op<PromotedT, Scalar>,
                                        const PlainType, const Derived> ReturnType;
  typedef typename Eigen::internal::scalar_constant_op<PromotedT> ScalarOp;
  return ReturnType(PlainType(m.derived().rows(), m.derived().cols(), ScalarOp(scalar)), m.derived());
}

template<typename Derived>
template<typename T>
inline Eigen::CwiseBinaryOp<
    Eigen::internal::scalar_quotient_op<
        typename SpatialMotionBase<Derived>::Scalar,
        typename Eigen::internal::promote_scalar_arg<typename SpatialMotionBase<Derived>::Scalar, T,
            Eigen::internal::has_ReturnType<
                typename SpatialMotionBase<Derived>::template ScalarOpTraits<T>::RightQuotient>::value>::type>,
    const Derived,
    const typename Eigen::internal::plain_constant_type<
        Derived,
        typename Eigen::internal::promote_scalar_arg<typename SpatialMotionBase<Derived>::Scalar, T,
            Eigen::internal::has_ReturnType<
                typename SpatialMotionBase<Derived>::template ScalarOpTraits<T>::RightQuotient>::value>::type>::type>
SpatialMotionBase<Derived>::operator/(const T& scalar) const {
  typedef typename Eigen::internal::promote_scalar_arg<Scalar, T,
      Eigen::internal::has_ReturnType<typename ScalarOpTraits<T>::RightQuotient>::value>::type PromotedT;
  typedef typename Eigen::internal::plain_constant_type<Derived, PromotedT>::type PlainType;
  typedef typename Eigen::CwiseBinaryOp<Eigen::internal::scalar_quotient_op<Scalar, PromotedT>,
                                        const Derived, const PlainType> ReturnType;
  typedef typename Eigen::internal::scalar_constant_op<PromotedT> ScalarOp;
  return ReturnType(derived(), PlainType(derived().rows(), derived().cols(), ScalarOp(scalar)));
}

template<typename Derived>
template<typename CustomBinaryOp, typename OtherDerived>
inline const Eigen::CwiseBinaryOp<CustomBinaryOp, const Derived, const OtherDerived>
SpatialMotionBase<Derived>::binaryExpr(const SpatialMotionBase<OtherDerived> &other,
                                       const CustomBinaryOp& func) const {
  return Eigen::CwiseBinaryOp<CustomBinaryOp, const Derived,
                                const OtherDerived>(derived(), other.derived(), func);
}

// from Assign.h
template<typename Derived>
inline Derived& SpatialMotionBase<Derived>::operator=(const SpatialMotionBase& other) {
  Eigen::internal::call_assignment(derived(), other.derived());
  return derived();
}

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator=(const Eigen::DenseBase<OtherDerived>& other) {
  Eigen::internal::call_assignment(derived(), other.derived());
  return derived();
}

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator=(const Eigen::EigenBase<OtherDerived>& other) {
  Eigen::internal::call_assignment(derived(), other.derived());
  return derived();
}

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator=(const Eigen::ReturnByValue<OtherDerived>& other) {
  other.derived().evalTo(derived());
  return derived();
}

// from CwiseBinaryOp.h
template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator+=(const SpatialMotionBase<OtherDerived>& other) {
  Eigen::internal::call_assignment(derived(), other.derived(),
                                     Eigen::internal::add_assign_op<Scalar, typename OtherDerived::Scalar>());
  return derived();
}

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator+=(const SpatialForceBase<OtherDerived>&) {
  static_assert(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar)) == -1,
                "YOU_CANNOT_ADD_SPATIAL_MOTION_AND_SPATIAL_FORCE_VECTORS");
  return *this;
};

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator-=(const SpatialMotionBase<OtherDerived>& other) {
  Eigen::internal::call_assignment(derived(), other.derived(),
                                     Eigen::internal::sub_assign_op<Scalar,typename OtherDerived::Scalar>());
  return derived();
}

template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator-=(const SpatialForceBase<OtherDerived>&) {
  static_assert(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar)) == -1,
                "YOU_CANNOT_ADD_SPATIAL_MOTION_AND_SPATIAL_FORCE_VECTORS");
  return *this;
}

template<typename Derived>
template<typename OtherDerived>
inline const Eigen::Product<Derived, OtherDerived>
SpatialMotionBase<Derived>::operator*(const SpatialMotionBase<OtherDerived>&) const {
  static_assert(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar)) == -1,
                "YOU_CANNOT_MULTIPLY_TWO_SPATIAL_MOTION_VECTORS");
}

template<typename Derived>
template<typename OtherDerived>
inline SpatialMotion<typename SpatialMotionBase<Derived>::template SpatialOpTraits<OtherDerived>::ScalarT,
                     OtherDerived::ColsAtCompileTime>
SpatialMotionBase<Derived>::operator*(const Eigen::MatrixBase<OtherDerived>& other) const {
  static_assert(ColsAtCompileTime == Eigen::Dynamic ||
                OtherDerived::RowsAtCompileTime == Eigen::Dynamic ||
                int(ColsAtCompileTime) == int(OtherDerived::RowsAtCompileTime),
                "INVALID_MATRIX_PRODUCT");
  // TODO: Make not lazy
  return Eigen::Product<Derived, OtherDerived, Eigen::LazyProduct>(derived(), other.derived());
}


template<typename Derived>
template<typename OtherDerived>
inline Derived& SpatialMotionBase<Derived>::operator*=(const SpatialMotionBase<OtherDerived>&) {
  static_assert(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar)) == -1,
                "YOU_CANNOT_MULTIPLY_TWO_SPATIAL_MOTION_VECTORS");
}

// from CwiseNullaryOp.h
template<typename Derived>
template<typename CustomNullaryOp>
inline const Eigen::CwiseNullaryOp<CustomNullaryOp,
                                   typename SpatialMotionBase<Derived>::PlainObject>
SpatialMotionBase<Derived>::NullaryExpr(Eigen::Index rows, Eigen::Index cols,
                                        const CustomNullaryOp& func) {
  return Eigen::CwiseNullaryOp<CustomNullaryOp, PlainObject>(6, cols, func);
}

template<typename Derived>
template<typename CustomNullaryOp>
inline const Eigen::CwiseNullaryOp<CustomNullaryOp,
                                   typename SpatialMotionBase<Derived>::PlainObject>
SpatialMotionBase<Derived>::NullaryExpr(Eigen::Index cols, const CustomNullaryOp& func) {
  return Eigen::CwiseNullaryOp<CustomNullaryOp, PlainObject>(6, cols, func);
}

template<typename Derived>
template<typename CustomNullaryOp>
inline const Eigen::CwiseNullaryOp<CustomNullaryOp,
                                   typename SpatialMotionBase<Derived>::PlainObject>
SpatialMotionBase<Derived>::NullaryExpr(const CustomNullaryOp& func) {
  return Eigen::CwiseNullaryOp<CustomNullaryOp, PlainObject>(6, ColsAtCompileTime, func);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::ConstantReturnType
SpatialMotionBase<Derived>::Constant(Eigen::Index cols, const Scalar& value) {
  return NullaryExpr(cols, Eigen::internal::scalar_constant_op<Scalar>(value));
};

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::ConstantReturnType
SpatialMotionBase<Derived>::Constant(const Scalar& value) {
  static_assert(ColsAtCompileTime != Eigen::Dynamic, "MATRIX_MUST_BE_OF_FIXED_SIZE");
  return NullaryExpr(ColsAtCompileTime, Eigen::internal::scalar_constant_op<Scalar>(value));
};

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::ConstantReturnType
SpatialMotionBase<Derived>::Zero(Eigen::Index cols) {
  return Constant(cols, Scalar(0));
};

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::ConstantReturnType
SpatialMotionBase<Derived>::Zero() {
  return Constant(Scalar(0));
};

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::ConstantReturnType
SpatialMotionBase<Derived>::Ones(Eigen::Index cols) {
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
  static_assert(ColsAtCompileTime != Eigen::Dynamic, "MATRIX_MUST_BE_OF_FIXED_SIZE");
  return NullaryExpr(ColsAtCompileTime, Eigen::internal::scalar_identity_op<Scalar>());
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::IdentityReturnType
SpatialMotionBase<Derived>::Identity(Eigen::Index cols) {
  return NullaryExpr(cols, Eigen::internal::scalar_identity_op<Scalar>());
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType
SpatialMotionBase<Derived>::Unit(Eigen::Index i) {
  static_assert(ColsAtCompileTime == 1, "MATRIX_MUST_BE_A_VECTOR");
  return BasisReturnType(SquareMatrixType::Identity(), i);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType
SpatialMotionBase<Derived>::UnitLinX() {
  return Derived::Unit(0);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType
SpatialMotionBase<Derived>::UnitLinY() {
  return Derived::Unit(1);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType
SpatialMotionBase<Derived>::UnitLinZ() {
  return Derived::Unit(2);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType
SpatialMotionBase<Derived>::UnitAngX() {
  return Derived::Unit(3);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType
SpatialMotionBase<Derived>::UnitAngY() {
  return Derived::Unit(4);
}

template<typename Derived>
inline const typename SpatialMotionBase<Derived>::BasisReturnType
SpatialMotionBase<Derived>::UnitAngZ() {
  return Derived::Unit(5);
}

template<typename Derived>
inline Derived& SpatialMotionBase<Derived>::setIdentity() {
  return Eigen::internal::setIdentity_impl<Derived>::run(derived());
}

template<typename Derived>
inline Derived& SpatialMotionBase<Derived>::setIdentity(Eigen::Index cols) {
  derived().resize(Eigen::NoChange, cols);
  return setIdentity();
}

template<typename Derived>
inline Derived& SpatialMotionBase<Derived>::setUnit(Eigen::Index i) {
  static_assert(ColsAtCompileTime == 1, "MATRIX_MUST_BE_A_VECTOR");
  eigen_assert(i < size());
  derived().setZero();
  derived().coeffRef(i) = Scalar(1);
  return derived();
}

// from Dot.h
template<typename Derived>
template<typename OtherDerived>
inline typename SpatialMotionBase<Derived>::template SpatialOpTraits<OtherDerived>::ScalarT
SpatialMotionBase<Derived>::dot(const SpatialMotionBase<OtherDerived>&) const {
  static_assert(std::ptrdiff_t(sizeof(typename OtherDerived::Scalar)) == -1,
                "YOU_CANNOT_DOT_TWO_SPATIAL_MOTION_VECTORS");
  return 0;
}

template<typename Derived>
template<typename OtherDerived>
inline typename SpatialMotionBase<Derived>::template SpatialOpTraits<OtherDerived>::ScalarT
SpatialMotionBase<Derived>::dot(const SpatialForceBase<OtherDerived>& other) const {
  static_assert(ColsAtCompileTime == 1, "MATRIX_MUST_BE_A_VECTOR");
  static_assert(OtherDerived::ColsAtCompileTime == 1, "MATRIX_MUST_BE_A_VECTOR");
  typedef Eigen::internal::scalar_conj_product_op<Scalar, typename OtherDerived::Scalar> conj_prod;
  return Eigen::CwiseBinaryOp<conj_prod, const Derived, const OtherDerived>(
      derived(), other.derived(), conj_prod()).sum();
}

// from Geometry/OrthoMethods.h
template<typename Derived>
template<typename OtherDerived>
inline typename SpatialMotionBase<Derived>::template SpatialOpTraits<OtherDerived>::MotionReturnType
SpatialMotionBase<Derived>::cross(const SpatialMotionBase<OtherDerived>& other) const {
  static_assert(ColsAtCompileTime == 1, "MATRIX_MUST_BE_A_VECTOR");
  static_assert(OtherDerived::ColsAtCompileTime == 1, "MATRIX_MUST_BE_A_VECTOR");
  typename Eigen::internal::nested_eval<Derived,4>::type lhs(derived());
  typename Eigen::internal::nested_eval<OtherDerived,4>::type rhs(other.derived());
  return typename SpatialOpTraits<OtherDerived>::MotionReturnType(
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
inline typename SpatialMotionBase<Derived>::template SpatialOpTraits<OtherDerived>::ForceReturnType
SpatialMotionBase<Derived>::cross(const SpatialForceBase<OtherDerived>& other) const {
  static_assert(ColsAtCompileTime == 1, "MATRIX_MUST_BE_A_VECTOR");
  static_assert(OtherDerived::ColsAtCompileTime == 1, "MATRIX_MUST_BE_A_VECTOR");
  typename Eigen::internal::nested_eval<Derived,4>::type lhs(derived());
  typename Eigen::internal::nested_eval<OtherDerived,4>::type rhs(other.derived());
  return typename SpatialOpTraits<OtherDerived>::ForceReturnType(
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
inline Eigen::MatrixWrapper<Derived> SpatialMotionBase<Derived>::matrix() {
  return Eigen::MatrixWrapper<Derived>(derived());
}

template<typename Derived>
inline const Eigen::MatrixWrapper<const Derived> SpatialMotionBase<Derived>::matrix() const {
  return Eigen::MatrixWrapper<const Derived>(derived());
}

template<typename Derived>
inline Eigen::ArrayWrapper<Derived> SpatialMotionBase<Derived>::array() {
  return Eigen::ArrayWrapper<Derived>(derived());
}

template<typename Derived>
inline const Eigen::ArrayWrapper<const Derived> SpatialMotionBase<Derived>::array() const {
  return Eigen::ArrayWrapper<const Derived>(derived());
}

template<typename Derived>
inline Eigen::Transpose<Eigen::MatrixWrapper<Derived>>
SpatialMotionBase<Derived>::transpose() {
  return Eigen::MatrixWrapper<Derived>(derived()).transpose();
}

template<typename Derived>
inline const Eigen::Transpose<Eigen::MatrixWrapper<const Derived>>
SpatialMotionBase<Derived>::transpose() const {
  return Eigen::MatrixWrapper<const Derived>(derived()).transpose();
}

template<typename Derived>
inline typename SpatialMotionBase<Derived>::CartesianBlockType
SpatialMotionBase<Derived>::linear() {
  return this->matrix().template topRows<3>();
}

template<typename Derived>
inline typename SpatialMotionBase<Derived>::ConstCartesianBlockType
SpatialMotionBase<Derived>::linear() const {
  return this->matrix().template topRows<3>();
}

template<typename Derived>
inline typename SpatialMotionBase<Derived>::CartesianBlockType
SpatialMotionBase<Derived>::angular() {
  return this->matrix().template bottomRows<3>();
}

template<typename Derived>
inline typename SpatialMotionBase<Derived>::ConstCartesianBlockType
SpatialMotionBase<Derived>::angular() const {
  return this->matrix().template bottomRows<3>();
}

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_MOTION_BASE_H_
