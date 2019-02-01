/**
 * spatial_motion.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 10, 2018
 * Authors: Toki Migimatsu
 */

#ifndef EIGEN_SPATIAL_MOTION_H_
#define EIGEN_SPATIAL_MOTION_H_

#include "spatial_motion_base.h"

namespace Eigen {

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
class SpatialMotion : public PlainObjectBase<SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>> {

 public:
  typedef PlainObjectBase<SpatialMotion> Base;
  enum { Options = _Options };
  EIGEN_DENSE_PUBLIC_INTERFACE(SpatialMotion)
  typedef typename Base::PlainObject PlainObject;

  using Base::base;
  using Base::coeffRef;

  SpatialMotion();

  SpatialMotion(double rows, double cols);

  SpatialMotion(const SpatialMotion& other);

  SpatialMotion(SpatialMotion&& other)
  EIGEN_NOEXCEPT_IF(std::is_nothrow_move_constructible<Scalar>::value);

  template<typename OtherDerived>
  SpatialMotion(const EigenBase<OtherDerived>& other);

  SpatialMotion(const Scalar& lin_x, const Scalar& lin_y, const Scalar& lin_z,
                const Scalar& ang_x, const Scalar& ang_y, const Scalar& ang_z);

  template<typename DerivedLin, typename DerivedAng>
  SpatialMotion(const EigenBase<DerivedLin>& lin, const EigenBase<DerivedAng>& ang);

  SpatialMotion& operator=(const SpatialMotion& other);

  SpatialMotion& operator=(SpatialMotion&& other)
  EIGEN_NOEXCEPT_IF(std::is_nothrow_move_assignable<Scalar>::value);

  template<typename OtherDerived>
  SpatialMotion& operator=(const DenseBase<OtherDerived>& other);

  template<typename OtherDerived>
  SpatialMotion& operator=(const EigenBase<OtherDerived>& other);

  template<typename OtherDerived>
  SpatialMotion& operator=(const ReturnByValue<OtherDerived>& func);

  SpatialMotion& operator=(const Scalar &value);

  SpatialMotion& operator*=(const Isometry3d& T);

  Index innerStride() const;
  Index outerStride() const;

 protected:
  using Base::m_storage;

};

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::SpatialMotion()
    : SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::Base() {
  Base::_check_template_params();
  EIGEN_INITIALIZE_COEFFS_IF_THAT_OPTION_IS_ENABLED
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::SpatialMotion(double rows, double cols)
    : SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::Base() {
  Base::resize(rows, cols);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::SpatialMotion(const SpatialMotion& other)
    : SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::Base(other) {}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::SpatialMotion(SpatialMotion&& other)
EIGEN_NOEXCEPT_IF(std::is_nothrow_move_constructible<Scalar>::value)
    : SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::Base(std::move(other)) {
  Base::_check_template_params();
  if (ColsAtCompileTime != Dynamic) Base::_set_noalias(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::SpatialMotion(const EigenBase<OtherDerived>& other)
    : Base(other.derived()) {}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::SpatialMotion(
    const Scalar& lin_x, const Scalar& lin_y, const Scalar& lin_z,
    const Scalar& ang_x, const Scalar& ang_y, const Scalar& ang_z) {
  m_storage.data()[0] = lin_x;
  m_storage.data()[1] = lin_y;
  m_storage.data()[2] = lin_z;
  m_storage.data()[3] = ang_x;
  m_storage.data()[4] = ang_y;
  m_storage.data()[5] = ang_z;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename DerivedLin, typename DerivedAng>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::SpatialMotion(
    const EigenBase<DerivedLin>& lin, const EigenBase<DerivedAng>& ang) {
  this->linear() = lin;
  this->angular() = ang;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>&
SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::operator=(const SpatialMotion& other) {
  return Base::_set(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>&
SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::operator=(SpatialMotion&& other)
EIGEN_NOEXCEPT_IF(std::is_nothrow_move_assignable<Scalar>::value) {
  other.swap(*this);
  return *this;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>&
SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::operator=(const DenseBase<OtherDerived>& other) {
  return Base::_set(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>&
SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::operator=(const EigenBase<OtherDerived>& other) {
  return Base::operator=(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>&
SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::operator=(const ReturnByValue<OtherDerived>& func) {
  return Base::operator=(func);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>&
SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::operator=(const Scalar& value) {
  Base::setConstant(value);
  return *this;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>&
SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::operator*=(const Isometry3d& T) {
  *this = T * (*this);
  return *this;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline Index SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::innerStride() const {
  return 1;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline Index SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>::outerStride() const {
  return 6;
}

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_MOTION_H_
