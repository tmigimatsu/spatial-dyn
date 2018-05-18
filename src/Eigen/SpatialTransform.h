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
  typename SpatialMotionBase<OtherDerived>::PlainObject result = other;
  Eigen::Matrix3d T_x;
  T_x <<  0,                      -this->translation()(2),  this->translation()(1),
          this->translation()(2),  0,                      -this->translation()(0),
         -this->translation()(1),  this->translation()(0),  0;
  result.template topRows<3>() = this->linear() *
      (result.matrix().template topRows<3>() - T_x * result.matrix().template bottomRows<3>());
  result.template bottomRows<3>() = this->linear() * result.matrix().template bottomRows<3>();
  return result;
}

template<typename OtherDerived>
EIGEN_DEVICE_FUNC inline typename SpatialForceBase<OtherDerived>::PlainObject
operator*(const SpatialForceBase<OtherDerived>& other) const {
  typename SpatialForceBase<OtherDerived>::PlainObject result = other;
  Eigen::Matrix3d T_x;
  T_x <<  0,                      -this->translation()(2),  this->translation()(1),
          this->translation()(2),  0,                      -this->translation()(0),
         -this->translation()(1),  this->translation()(0),  0;
  result.template bottomRows<3>() = this->linear() *
      (result.matrix().template bottomRows<3>() - T_x * result.matrix().template topRows<3>());
  result.template topRows<3>() = this->linear() * result.matrix().template topRows<3>();
  return result;
}

template<typename OtherScalar>
EIGEN_DEVICE_FUNC inline SpatialInertia<OtherScalar>
operator*(const SpatialInertia<OtherScalar>& other) const {
  SpatialInertia<OtherScalar> result;
  Eigen::Matrix3d T_x;
  T_x <<  0,                      -this->translation()(2),  this->translation()(1),
          this->translation()(2),  0,                      -this->translation()(0),
         -this->translation()(1),  this->translation()(0),  0;
  result.template leftCols<3>() = other.template leftCols<3>() * this->linear();
  result.template rightCols<3>() = other.template rightCols<3>() * this->linear() -
                                   result.template leftCols<3>() * T_x;
  T_x = this->linear() * T_x;
  result.template bottomLeftCorner<3,3>() = this->linear() * result.template bottomLeftCorner<3,3>() -
                                    T_x * result.template topLeftCorner<3,3>();
  result.template bottomRightCorner<3,3>() = this->linear() * result.template bottomRightCorner<3,3>() -
                                     T_x * result.template topRightCorner<3,3>();
  result.template topLeftCorner<3,3>() = this->linear() * result.template topLeftCorner<3,3>();
  result.template topRightCorner<3,3>() = this->linear() * result.template topRightCorner<3,3>();
  return result;
}
