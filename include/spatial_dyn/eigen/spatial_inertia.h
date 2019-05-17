/**
 * spatial_inertia.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 17, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_EIGEN_SPATIAL_INERTIA_H_
#define SPATIAL_DYN_EIGEN_SPATIAL_INERTIA_H_

#include "spatial_motion_base.h"
#include "spatial_force_base.h"

#include <utility>  // std::move

namespace spatial_dyn {

template<typename Scalar>
class SpatialInertia {

 public:

  SpatialInertia() {}

  SpatialInertia(const SpatialInertia<Scalar>& other)
      : mass(other.mass), com(other.com), I_com(other.I_com) {}

  SpatialInertia(SpatialInertia<Scalar>&& other)
      : mass(other.mass), com(std::move(other.com)), I_com(std::move(other.I_com)) {}

  template<typename ComDerived, typename IComDerived>
  SpatialInertia(Scalar mass,
                 const Eigen::MatrixBase<ComDerived>& com,
                 const Eigen::MatrixBase<IComDerived>& I_com_flat);

  SpatialInertia<Scalar>& operator=(const SpatialInertia<Scalar>& other);

  SpatialInertia<Scalar>& operator=(SpatialInertia<Scalar>&& other);

  template<typename OtherDerived>
  struct MotionProductTraits {
    typedef typename Eigen::ScalarBinaryOpTraits<Scalar,
                                                   typename OtherDerived::Scalar>::ReturnType ScalarT;
    typedef SpatialForce<ScalarT, OtherDerived::ColsAtCompileTime> ReturnType;
  };

  template<typename OtherDerived>
  typename MotionProductTraits<OtherDerived>::ReturnType
  operator*(const SpatialMotionBase<OtherDerived>& other) const;

  SpatialInertia<Scalar>& operator+=(const SpatialInertia<Scalar>& other);

  SpatialInertia<Scalar> operator+(const SpatialInertia<Scalar>& other) const;

  bool operator==(const SpatialInertia<Scalar>& other) const;

  bool operator!=(const SpatialInertia<Scalar>& other) const;

  SpatialInertiaMatrix<Scalar> matrix() const;

  Eigen::Matrix<Scalar,6,1> I_com_flat() const;

  double mass = 0;
  Eigen::Matrix<Scalar,3,1> com = Eigen::Matrix<Scalar,3,1>::Zero();
  Eigen::Matrix<Scalar,3,3> I_com = Eigen::Matrix<Scalar,3,3>::Zero();

};

template<typename Scalar>
class SpatialInertiaMatrix : public Eigen::Matrix<Scalar,6,6> {

 public:
  SpatialInertiaMatrix() { this->setZero(); }

  SpatialInertiaMatrix(const SpatialInertia<Scalar>& other);

  template<typename OtherDerived>
  SpatialInertiaMatrix(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::Matrix<Scalar,6,6>(other) {};

  template<typename OtherDerived>
  SpatialInertiaMatrix(Eigen::MatrixBase<OtherDerived>&& other)
      : Eigen::Matrix<Scalar,6,6>(std::move(other)) {};

  template<typename OtherDerived>
  struct MotionProductTraits {
    typedef typename Eigen::ScalarBinaryOpTraits<Scalar,
                                                   typename OtherDerived::Scalar>::ReturnType _Scalar;
    typedef SpatialForce<_Scalar, OtherDerived::ColsAtCompileTime> ReturnType;
  };

  template<typename OtherDerived>
  typename MotionProductTraits<OtherDerived>::ReturnType
  operator*(const SpatialMotionBase<OtherDerived>& other) const;

  SpatialInertiaMatrix<Scalar>& operator+=(const SpatialInertia<Scalar>& other);

  template<typename OtherDerived>
  SpatialInertiaMatrix<Scalar>& operator+=(const Eigen::MatrixBase<OtherDerived>& other);

};

// Transform operations
template<typename TransformScalar, int Dim, typename Scalar>
inline SpatialInertia<Scalar>
operator*(const Eigen::Translation<TransformScalar, Dim>& T, const SpatialInertia<Scalar>& I) {
  static_assert(Dim == 3, "YOU_CAN_ONLY_APPLY_3D_TRANSFORMS_TO_SPATIAL_INERTIAS");

  SpatialInertia<Scalar> result;
  result.mass = I.mass;
  result.com = T.translation() + I.com;
  result.I_com = I.I_com;
  return result;
}

template<typename TransformScalar, int Dim, int Mode, int Options, typename Scalar>
inline SpatialInertia<Scalar>
operator*(const Eigen::Transform<TransformScalar, Dim, Mode, Options>& T, const SpatialInertia<Scalar>& I) {
  static_assert(Dim == 3, "YOU_CAN_ONLY_APPLY_3D_TRANSFORMS_TO_SPATIAL_INERTIAS");
  static_assert(Mode == int(Eigen::Isometry),
                "YOU_CAN_ONLY_APPLY_ISOMETRY_TRANSFORMS_TO_SPATIAL_INERTIAS");

  SpatialInertia<Scalar> result;
  result.mass = I.mass;
  result.com = T * I.com;
  result.I_com = T.linear() * I.I_com * T.linear().transpose();
  return result;
}

template<typename TransformScalar, int Dim, typename Scalar>
SpatialInertiaMatrix<Scalar>
operator*(const Eigen::Translation<TransformScalar, Dim>& T, const SpatialInertiaMatrix<Scalar>& I) {
  static_assert(Dim == 3, "YOU_CAN_ONLY_APPLY_3D_TRANSFORMS_TO_SPATIAL_INERTIAS");

  SpatialInertiaMatrix<Scalar> result = I;
  result.template rightCols<3>() -= result.template leftCols<3>().rowwise().cross(T.translation());
  result.template bottomRows<3>() -= result.template topRows<3>().colwise().cross(T.translation());
  return result;
}

template<typename TransformScalar, int Dim, int Mode, int Options, typename Scalar>
SpatialInertiaMatrix<Scalar>
operator*(const Eigen::Transform<TransformScalar, Dim, Mode, Options>& T,
          const SpatialInertiaMatrix<Scalar>& I) {
  static_assert(Dim == 3, "YOU_CAN_ONLY_APPLY_3D_TRANSFORMS_TO_SPATIAL_INERTIAS");
  static_assert(Mode == int(Eigen::Isometry),
                "YOU_CAN_ONLY_APPLY_ISOMETRY_TRANSFORMS_TO_SPATIAL_INERTIAS");

  SpatialInertiaMatrix<Scalar> result = I;
  result.template leftCols<3>() = result.template leftCols<3>() * T.linear().transpose();
  result.template rightCols<3>() = result.template rightCols<3>() * T.linear().transpose() -
      result.template leftCols<3>().rowwise().cross(T.translation());
  result.template topRows<3>() = T.linear() * result.template topRows<3>();
  result.template bottomRows<3>() = T.linear() * result.template bottomRows<3>() -
      result.template topRows<3>().colwise().cross(T.translation());
  return result;
}

template<typename Scalar>
template<typename ComDerived, typename IComDerived>
inline SpatialInertia<Scalar>::SpatialInertia(Scalar m,
    const Eigen::MatrixBase<ComDerived>& c,
    const Eigen::MatrixBase<IComDerived>& I) : mass(m), com(c) {
  static_assert(ComDerived::RowsAtCompileTime == 3 && ComDerived::ColsAtCompileTime == 1,
                "COM_MUST_BE_A_VECTOR_OF_SIZE_3");
  static_assert(IComDerived::RowsAtCompileTime == 6 && ComDerived::ColsAtCompileTime == 1,
                "I_COM_MUST_BE_A_VECTOR_OF_SIZE_6");

  I_com << I(0), I(3), I(4),
           I(3), I(1), I(5),
           I(4), I(5), I(2);
}

template<typename Scalar>
inline SpatialInertia<Scalar>&
SpatialInertia<Scalar>::operator=(const SpatialInertia<Scalar>& other) {
  mass = other.mass;
  com = other.com;
  I_com = other.I_com;
  return *this;
}

template<typename Scalar>
inline SpatialInertia<Scalar>&
SpatialInertia<Scalar>::operator=(SpatialInertia<Scalar>&& other) {
  mass = other.mass;
  com = std::move(other.com);
  I_com = std::move(other.I_com);
  return *this;
}

template<typename Scalar>
template<typename OtherDerived>
typename SpatialInertia<Scalar>::template MotionProductTraits<OtherDerived>::ReturnType
SpatialInertia<Scalar>::operator*(const SpatialMotionBase<OtherDerived>& other) const {
  typename MotionProductTraits<OtherDerived>::ReturnType result;
  Eigen::Matrix<Scalar,3,1> mc = mass * com;
  result.template topRows<3>() = mass * other.matrix().template topRows<3>() +
                                 other.matrix().template bottomRows<3>().cross(mc);
  result.template bottomRows<3>() = I_com * other.matrix().template bottomRows<3>() -
                                    other.matrix().template topRows<3>().cross(mc) -
                                    other.matrix().template bottomRows<3>().cross(com).cross(mc);
  return result;
}

template<typename Scalar>
SpatialInertia<Scalar>&
SpatialInertia<Scalar>::operator+=(const SpatialInertia<Scalar>& other) {
  Eigen::Vector3d com_new = (mass * com + other.mass * other.com) / (mass + other.mass);
  I_com += other.I_com -
           mass * ctrl_utils::DoubleCrossMatrix(com_new - com) -
           other.mass * ctrl_utils::DoubleCrossMatrix(com_new - other.com);
  mass += other.mass;
  com = std::move(com_new);
  return *this;
}

template<typename Scalar>
SpatialInertia<Scalar>
SpatialInertia<Scalar>::operator+(const SpatialInertia<Scalar>& other) const {
  SpatialInertia<Scalar> I_total;
  I_total.com = (mass * com + other.mass * other.com) / (mass + other.mass);
  I_total.I_com = I_com + other.I_com -
                  mass * ctrl_utils::DoubleCrossMatrix(I_total.com - com) -
                  other.mass * ctrl_utils::DoubleCrossMatrix(I_total.com - other.com);
  I_total.mass = mass + other.mass;
  return I_total;
}

template<typename Scalar>
inline bool SpatialInertia<Scalar>::operator==(const SpatialInertia<Scalar>& other) const {
  return mass == other.mass && com == other.com && I_com == other.I_com;
}

template<typename Scalar>
inline bool SpatialInertia<Scalar>::operator!=(const SpatialInertia<Scalar>& other) const {
  return !(*this == other);
}

template<typename Scalar>
SpatialInertiaMatrix<Scalar> SpatialInertia<Scalar>::matrix() const {
  const double mcx = mass * com(0);
  const double mcy = mass * com(1);
  const double mcz = mass * com(2);
  const double Imcxx = I_com(0,0) + mass * (com(1) * com(1) + com(2) * com(2));
  const double Imcyy = I_com(1,1) + mass * (com(0) * com(0) + com(2) * com(2));
  const double Imczz = I_com(2,2) + mass * (com(0) * com(0) + com(1) * com(1));
  const double Imcxy = I_com(0,1) - mass * com(0) * com(1);
  const double Imcxz = I_com(0,2) - mass * com(0) * com(2);
  const double Imcyz = I_com(1,2) - mass * com(1) * com(2);
  SpatialInertiaMatrix<Scalar> result;
  result <<  mass, 0,    0,    0,      mcz,   -mcy,
             0,    mass, 0,   -mcz,    0,      mcx,
             0,    0,    mass, mcy,   -mcx,    0,
             0,   -mcz,  mcy,  Imcxx,  Imcxy,  Imcxz,
             mcz,  0,   -mcx,  Imcxy,  Imcyy,  Imcyz,
            -mcy,  mcx,  0,    Imcxz,  Imcyz,  Imczz;
  return result;
}

template<typename Scalar>
inline Eigen::Matrix<Scalar,6,1> SpatialInertia<Scalar>::I_com_flat() const {
  Eigen::Matrix<Scalar,6,1> result;
  result << I_com(0,0), I_com(1,1), I_com(2,2), I_com(0,1), I_com(0,2), I_com(1,2);
  return result;
}

template<typename Scalar>
inline SpatialInertiaMatrix<Scalar>::SpatialInertiaMatrix(const SpatialInertia<Scalar>& other) {
  const double& mass               = other.mass;
  const Eigen::Matrix<Scalar,3,1>& com   = other.com;
  const Eigen::Matrix<Scalar,3,3>& I_com = other.I_com;
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

template<typename Scalar>
template<typename OtherDerived>
inline typename SpatialInertiaMatrix<Scalar>::template MotionProductTraits<OtherDerived>::ReturnType
SpatialInertiaMatrix<Scalar>::operator*(const SpatialMotionBase<OtherDerived>& other) const {
  return Eigen::Matrix<Scalar,6,6>::operator*(other.matrix());
};

template<typename Scalar>
inline SpatialInertiaMatrix<Scalar>&
SpatialInertiaMatrix<Scalar>::operator+=(const SpatialInertia<Scalar>& other) {
  *this += other.matrix();
  return *this;
}

template<typename Scalar>
template<typename OtherDerived>
inline SpatialInertiaMatrix<Scalar>&
SpatialInertiaMatrix<Scalar>::operator+=(const Eigen::MatrixBase<OtherDerived>& other) {
  Eigen::Matrix<Scalar,6,6>::operator+=(other);
  return *this;
}

}  // namespace spatial_dyn

#endif  // SPATIAL_DYN_EIGEN_SPATIAL_INERTIA_H_
