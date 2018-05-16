/**
 * SpatialMotion.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 10, 2018
 * Authors: Toki Migimatsu
 */

#ifndef EIGEN_SPATIAL_MOTION_H_
#define EIGEN_SPATIAL_MOTION_H_

#include "SpatialMotionBase.h"

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

  inline SpatialMotion() : Base() {
    Base::_check_template_params();
    EIGEN_INITIALIZE_COEFFS_IF_THAT_OPTION_IS_ENABLED
  }

  inline SpatialMotion(const SpatialMotion& other) : Base(other) {}

  SpatialMotion(SpatialMotion&& other)
      EIGEN_NOEXCEPT_IF(std::is_nothrow_move_constructible<Scalar>::value)
      : Base(std::move(other)) {
    Base::_check_template_params();
    if (ColsAtCompileTime != Dynamic) Base::_set_noalias(other);
  }

  template<typename OtherDerived>
  inline SpatialMotion(const EigenBase<OtherDerived> &other) : Base(other.derived()) {}

  inline SpatialMotion(const Scalar& lin_x, const Scalar& lin_y, const Scalar& lin_z,
                       const Scalar& ang_x, const Scalar& ang_y, const Scalar& ang_z) {
    m_storage.data()[0] = lin_x;
    m_storage.data()[1] = lin_y;
    m_storage.data()[2] = lin_z;
    m_storage.data()[3] = ang_x;
    m_storage.data()[4] = ang_y;
    m_storage.data()[5] = ang_z;
  }

  inline SpatialMotion& operator=(const SpatialMotion& other) {
    return Base::_set(other);
  }

  SpatialMotion& operator=(SpatialMotion&& other)
      EIGEN_NOEXCEPT_IF(std::is_nothrow_move_assignable<Scalar>::value) {
    other.swap(*this);
    return *this;
  }

  template<typename OtherDerived>
  inline SpatialMotion& operator=(const DenseBase<OtherDerived>& other) {
    return Base::_set(other);
  }

  template<typename OtherDerived>
  inline SpatialMotion& operator=(const EigenBase<OtherDerived> &other) {
    return Base::operator=(other);
  }

  template<typename OtherDerived>
  inline SpatialMotion& operator=(const ReturnByValue<OtherDerived>& func) {
    return Base::operator=(func);
  }

  inline SpatialMotion& operator=(const Scalar &value) {
    Base::setConstant(value);
    return *this;
  }

  inline Index innerStride() const { return 1; }
  inline Index outerStride() const { return 6; }

 protected:

  using Base::m_storage;

};

typedef SpatialMotion<float,1>  SpatialMotionf;
typedef SpatialMotion<double,1> SpatialMotiond;
typedef SpatialMotion<float,6>  SpatialMotion6f;
typedef SpatialMotion<double,6> SpatialMotion6d;
typedef SpatialMotion<float,Dynamic>  SpatialMotionXf;
typedef SpatialMotion<double,Dynamic> SpatialMotionXd;

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_MOTION_H_
