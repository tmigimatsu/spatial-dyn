/**
 * SpatialDeclarations.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 15, 2018
 * Authors: Toki Migimatsu
 */

#ifndef EIGEN_SPATIAL_DECLARATIONS_H_
#define EIGEN_SPATIAL_DECLARATIONS_H_

#include <Eigen/Eigen>

namespace Eigen {

template<typename _Scalar, int _Cols, int _Options = AutoAlign | Eigen::ColMajor, int _MaxCols = _Cols>
class SpatialMotion;

template<typename _Scalar, int _Cols, int _Options = AutoAlign | Eigen::ColMajor, int _MaxCols = _Cols>
class SpatialForce;

namespace internal {

// From util/XprHelper.h
struct SpatialMotionXpr {};
struct SpatialForceXpr {};

template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
struct traits<SpatialMotion<_Scalar, _Cols, _Options, _MaxCols>>
    : traits<Matrix<_Scalar, 6, _Cols, _Options, 6, _MaxCols>> {
  typedef SpatialMotionXpr XprKind;
};
template<typename _Scalar, int _Cols, int _Options, int _MaxCols>
struct traits<SpatialForce<_Scalar, _Cols, _Options, _MaxCols>>
    : traits<Matrix<_Scalar, 6, _Cols, _Options, 6, _MaxCols>> {
  typedef SpatialForceXpr XprKind;
};

// From util/XprHelper.h
template<typename Derived>
struct dense_xpr_base<Derived, SpatialMotionXpr> {
  typedef SpatialMotionBase<Derived> type;
};
template<typename Derived>
struct dense_xpr_base<Derived, SpatialForceXpr> {
  typedef SpatialForceBase<Derived> type;
};

// From util/XprHelper.h
template<typename T, int Flags>
struct plain_matrix_type_dense<T, SpatialMotionXpr, Flags> {
  typedef SpatialMotion<typename traits<T>::Scalar, traits<T>::ColsAtCompileTime,
                        AutoAlign | (Flags&RowMajorBit ? RowMajor : ColMajor),
                        traits<T>::MaxColsAtCompileTime> type;
};
template<typename T, int Flags>
struct plain_matrix_type_dense<T, SpatialForceXpr, Flags> {
  typedef SpatialForce<typename traits<T>::Scalar, traits<T>::ColsAtCompileTime,
                        AutoAlign | (Flags&RowMajorBit ? RowMajor : ColMajor),
                        traits<T>::MaxColsAtCompileTime> type;
};

// From CoreEvaluators.h
template<typename Scalar, int Cols, int Options, int MaxCols>
struct evaluator<SpatialMotion<Scalar, Cols, Options, MaxCols>>
    : evaluator<PlainObjectBase<SpatialMotion<Scalar, Cols, Options, MaxCols>>> {
  typedef SpatialMotion<Scalar, Cols, Options, MaxCols> XprType;
  evaluator() {}
  explicit evaluator(const XprType& m) : evaluator<PlainObjectBase<XprType> >(m) {}
};
template<typename Scalar, int Cols, int Options, int MaxCols>
struct evaluator<SpatialForce<Scalar, Cols, Options, MaxCols>>
    : evaluator<PlainObjectBase<SpatialForce<Scalar, Cols, Options, MaxCols>>> {
  typedef SpatialForce<Scalar, Cols, Options, MaxCols> XprType;
  evaluator() {}
  explicit evaluator(const XprType& m) : evaluator<PlainObjectBase<XprType> >(m) {}
};

}  // namespace internal

}  // namespace Eigen

#endif  // EIGEN_SPATIAL_DECLARATIONS_H_
