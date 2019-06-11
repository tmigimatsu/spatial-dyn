/**
 * spatial_force.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 16, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_EIGEN_SPATIAL_FORCE_H_
#define SPATIAL_DYN_EIGEN_SPATIAL_FORCE_H_

#include "spatial_force_base.h"

namespace spatial_dyn {

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
class SpatialForce : public Eigen::PlainObjectBase<SpatialForce<_Scalar, _Cols, _Options, _MaxCols>> {

 public:
  typedef Eigen::PlainObjectBase<SpatialForce> Base;
  enum { Options = _Options };

  // EIGEN_GENERIC_PUBLIC_INTERFACE(SpatialForce)
  typedef typename Eigen::internal::traits<SpatialForce>::Scalar Scalar;
  typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;
  typedef typename Base::CoeffReturnType CoeffReturnType;
  typedef typename Eigen::internal::ref_selector<SpatialForce>::type Nested;
  typedef typename Eigen::internal::traits<SpatialForce>::StorageKind StorageKind;
  typedef typename Eigen::internal::traits<SpatialForce>::StorageIndex StorageIndex;
  enum CompileTimeTraits {
    RowsAtCompileTime = Eigen::internal::traits<SpatialForce>::RowsAtCompileTime,
    ColsAtCompileTime = Eigen::internal::traits<SpatialForce>::ColsAtCompileTime,
    Flags = Eigen::internal::traits<SpatialForce>::Flags,
    SizeAtCompileTime = Base::SizeAtCompileTime,
    MaxSizeAtCompileTime = Base::MaxSizeAtCompileTime,
    IsVectorAtCompileTime = Base::IsVectorAtCompileTime
  };
  using Base::derived;
  using Base::const_cast_derived;

  // EIGEN_DENSE_PUBLIC_INTERFACE(SpatialForce)
  typedef typename Base::PacketScalar PacketScalar;

  typedef typename Base::PlainObject PlainObject;

  using Base::base;
  using Base::coeffRef;

  SpatialForce();

  SpatialForce(double cols);

  SpatialForce(double rows, double cols);

  SpatialForce(const SpatialForce& other);

  SpatialForce(SpatialForce&& other)
  EIGEN_NOEXCEPT_IF(std::is_nothrow_move_constructible<_Scalar>::value);

  template<typename OtherDerived>
  SpatialForce(const Eigen::EigenBase<OtherDerived>& other);

  SpatialForce(const _Scalar& lin_x, const _Scalar& lin_y, const _Scalar& lin_z,
                const _Scalar& ang_x, const _Scalar& ang_y, const _Scalar& ang_z);

  template<typename DerivedLin, typename DerivedAng>
  SpatialForce(const Eigen::EigenBase<DerivedLin>& lin,
               const Eigen::EigenBase<DerivedAng>& ang);

  SpatialForce& operator=(const SpatialForce& other);

  SpatialForce& operator=(SpatialForce&& other)
  EIGEN_NOEXCEPT_IF(std::is_nothrow_move_assignable<_Scalar>::value);

  template<typename OtherDerived>
  SpatialForce& operator=(const Eigen::DenseBase<OtherDerived>& other);

  template<typename OtherDerived>
  SpatialForce& operator=(const Eigen::EigenBase<OtherDerived>& other);

  template<typename OtherDerived>
  SpatialForce& operator=(const Eigen::ReturnByValue<OtherDerived>& func);

  SpatialForce& operator=(const _Scalar& value);

  SpatialForce& operator*=(const Eigen::Isometry3d& T);

  Eigen::Index innerStride() const;
  Eigen::Index outerStride() const;

 protected:
  using Base::m_storage;

};

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce()
    : SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::Base() {
  Base::_check_template_params();
  EIGEN_INITIALIZE_COEFFS_IF_THAT_OPTION_IS_ENABLED
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce(double cols)
    : SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::Base() {
  Base::resize(6, cols);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce(double rows, double cols)
    : SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::Base() {
  assert(rows == 6);
  Base::resize(rows, cols);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce(const SpatialForce& other)
    : SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::Base(other) {}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce(SpatialForce&& other)
EIGEN_NOEXCEPT_IF(std::is_nothrow_move_constructible<_Scalar>::value)
    : SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::Base(std::move(other)) {
  Base::_check_template_params();
  if (ColsAtCompileTime != Eigen::Dynamic) Base::_set_noalias(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce(
    const Eigen::EigenBase<OtherDerived>& other)
    : Base(other.derived()) {}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce(
    const _Scalar& lin_x, const _Scalar& lin_y, const _Scalar& lin_z,
    const _Scalar& ang_x, const _Scalar& ang_y, const _Scalar& ang_z) {
  m_storage.data()[0] = lin_x;
  m_storage.data()[1] = lin_y;
  m_storage.data()[2] = lin_z;
  m_storage.data()[3] = ang_x;
  m_storage.data()[4] = ang_y;
  m_storage.data()[5] = ang_z;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename DerivedLin, typename DerivedAng>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce(
    const Eigen::EigenBase<DerivedLin>& lin, const Eigen::EigenBase<DerivedAng>& ang) {
  this->linear() = lin;
  this->angular() = ang;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(const SpatialForce& other) {
  return Base::_set(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(SpatialForce&& other)
EIGEN_NOEXCEPT_IF(std::is_nothrow_move_assignable<_Scalar>::value) {
  other.swap(*this);
  return *this;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(
    const Eigen::DenseBase<OtherDerived>& other) {
  return Base::_set(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(
    const Eigen::EigenBase<OtherDerived>& other) {
  return Base::operator=(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(
    const Eigen::ReturnByValue<OtherDerived>& func) {
  return Base::operator=(func);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(const _Scalar& value) {
  Base::setConstant(value);
  return *this;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator*=(const Eigen::Isometry3d& T) {
  *this = T * (*this);
  return *this;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline Eigen::Index SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::innerStride() const {
  return 1;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline Eigen::Index SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::outerStride() const {
  return 6;
}

}  // spatial_dyn

#endif  // SPATIAL_DYN_EIGEN_SPATIAL_FORCE_H_
