/**
 * spatial_force.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 16, 2018
 * Authors: Toki Migimatsu
 */

#ifndef EIGEN_SPATIAL_FORCE_H_
#define EIGEN_SPATIAL_FORCE_H_

#include "spatial_force_base.h"

namespace Eigen {

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
class SpatialForce : public PlainObjectBase<SpatialForce<_Scalar, _Cols, _Options, _MaxCols>> {

 public:
  typedef PlainObjectBase<SpatialForce> Base;
  enum { Options = _Options };
  EIGEN_DENSE_PUBLIC_INTERFACE(SpatialForce)
  typedef typename Base::PlainObject PlainObject;

  using Base::base;
  using Base::coeffRef;

  SpatialForce();

  SpatialForce(const SpatialForce& other);

  SpatialForce(SpatialForce&& other)
  EIGEN_NOEXCEPT_IF(std::is_nothrow_move_constructible<Scalar>::value);

  template<typename OtherDerived>
  SpatialForce(const EigenBase<OtherDerived>& other);

  SpatialForce(const Scalar& lin_x, const Scalar& lin_y, const Scalar& lin_z,
                const Scalar& ang_x, const Scalar& ang_y, const Scalar& ang_z);

  SpatialForce& operator=(const SpatialForce& other);

  SpatialForce& operator=(SpatialForce&& other)
  EIGEN_NOEXCEPT_IF(std::is_nothrow_move_assignable<Scalar>::value);

  template<typename OtherDerived>
  SpatialForce& operator=(const DenseBase<OtherDerived>& other);

  template<typename OtherDerived>
  SpatialForce& operator=(const EigenBase<OtherDerived>& other);

  template<typename OtherDerived>
  SpatialForce& operator=(const ReturnByValue<OtherDerived>& func);

  SpatialForce& operator=(const Scalar& value);

  Index innerStride() const;
  Index outerStride() const;

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
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce(const SpatialForce& other)
    : SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::Base(other) {}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce(SpatialForce&& other)
EIGEN_NOEXCEPT_IF(std::is_nothrow_move_constructible<Scalar>::value)
    : SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::Base(std::move(other)) {
  Base::_check_template_params();
  if (ColsAtCompileTime != Dynamic) Base::_set_noalias(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce(const EigenBase<OtherDerived>& other)
    : Base(other.derived()) {}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::SpatialForce(
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
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(const SpatialForce& other) {
  return Base::_set(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(SpatialForce&& other)
EIGEN_NOEXCEPT_IF(std::is_nothrow_move_assignable<Scalar>::value) {
  other.swap(*this);
  return *this;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(const DenseBase<OtherDerived>& other) {
  return Base::_set(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(const EigenBase<OtherDerived>& other) {
  return Base::operator=(other);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
template<typename OtherDerived>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(const ReturnByValue<OtherDerived>& func) {
  return Base::operator=(func);
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline SpatialForce<_Scalar, _Cols, _Options, _MaxCols>&
SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::operator=(const Scalar &value) {
  Base::setConstant(value);
  return *this;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline Index SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::innerStride() const {
  return 1;
}

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
inline Index SpatialForce<_Scalar, _Cols, _Options, _MaxCols>::outerStride() const {
  return 6;
}

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_FORCE_H_
