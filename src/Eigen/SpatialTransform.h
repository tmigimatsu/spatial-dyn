/**
 * SpatialTransform.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 17, 2018
 * Authors: Toki Migimatsu
 */

template<typename OtherDerived>
EIGEN_DEVICE_FUNC inline typename SpatialMotionBase<OtherDerived>::PlainObject
operator*(const SpatialMotionBase<OtherDerived>& other) const {
  typename SpatialMotionBase<OtherDerived>::PlainObject result = other;  // Evaluate expr
  result.template topRows<3>() = this->linear() * (result.matrix().template topRows<3>() +
      result.matrix().template bottomRows<3>().colwise().cross(this->translation()));
  result.template bottomRows<3>() = this->linear() * result.matrix().template bottomRows<3>();
  return result;
}

template<typename OtherDerived>
EIGEN_DEVICE_FUNC inline typename SpatialForceBase<OtherDerived>::PlainObject
operator*(const SpatialForceBase<OtherDerived>& other) const {
  typename SpatialForceBase<OtherDerived>::PlainObject result = other;  // Evaluate expr
  result.template bottomRows<3>() = this->linear() * (result.matrix().template bottomRows<3>() +
      result.matrix().template topRows<3>().colwise().cross(this->translation()));
  result.template topRows<3>() = this->linear() * result.matrix().template topRows<3>();
  return result;
}

template<typename OtherScalar>
EIGEN_DEVICE_FUNC inline SpatialInertia<OtherScalar>
operator*(const SpatialInertia<OtherScalar>& other) const {
  SpatialInertia<OtherScalar> result;
  result.mass = other.mass;
  result.com = this->linear() * (other.com - this->translation());
  result.I_com = this->linear() * other.I_com * this->linear().transpose();
  return result;
}
