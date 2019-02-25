/**
 * spatial_declarations.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: June 15, 2018
 * Authors: Toki Migimatsu
 */

#ifndef SPATIAL_DYN_EIGEN_SPATIAL_DECLARATIONS_H_
#define SPATIAL_DYN_EIGEN_SPATIAL_DECLARATIONS_H_

#include <Eigen/Eigen>

namespace spatial_dyn {

template<typename _Scalar, int _Cols,
         int _Options = ::Eigen::AutoAlign | ::Eigen::ColMajor, int _MaxCols = _Cols>
class SpatialMotion;

template<typename _Scalar, int _Cols,
         int _Options = ::Eigen::AutoAlign | ::Eigen::ColMajor, int _MaxCols = _Cols>
class SpatialForce;

template<typename Derived>
class SpatialMotionBase;

template<typename Derived>
class SpatialForceBase;

template<typename Scalar>
class SpatialInertia;

template<typename Scalar>
class SpatialInertiaMatrix;

// From util/XprHelper.h
struct SpatialMotionXpr {};
struct SpatialForceXpr {};

}  // namespace spatial_dyn

namespace Eigen {
namespace internal {

template<typename Scalar, int Cols, int Options, int MaxCols>
struct traits<spatial_dyn::SpatialMotion<Scalar, Cols, Options, MaxCols>>
    : traits<Matrix<Scalar, 6, Cols, Options, 6, MaxCols>> {
  typedef spatial_dyn::SpatialMotionXpr XprKind;
};
template<typename Scalar, int Cols, int Options, int MaxCols>
struct traits<spatial_dyn::SpatialForce<Scalar, Cols, Options, MaxCols>>
    : traits<Matrix<Scalar, 6, Cols, Options, 6, MaxCols>> {
  typedef spatial_dyn::SpatialForceXpr XprKind;
};

// From util/XprHelper.h
template<typename Derived>
struct dense_xpr_base<Derived, spatial_dyn::SpatialMotionXpr> {
  typedef spatial_dyn::SpatialMotionBase<Derived> type;
};
template<typename Derived>
struct dense_xpr_base<Derived, spatial_dyn::SpatialForceXpr> {
  typedef spatial_dyn::SpatialForceBase<Derived> type;
};

// From util/XprHelper.h
template<typename T, int Flags>
struct plain_matrix_type_dense<T, spatial_dyn::SpatialMotionXpr, Flags> {
  typedef spatial_dyn::SpatialMotion<typename traits<T>::Scalar,
                                     traits<T>::ColsAtCompileTime,
                                     AutoAlign | (Flags & RowMajorBit ? RowMajor : ColMajor),
                                     traits<T>::MaxColsAtCompileTime> type;
};
template<typename T, int Flags>
struct plain_matrix_type_dense<T, spatial_dyn::SpatialForceXpr, Flags> {
  typedef spatial_dyn::SpatialForce<typename traits<T>::Scalar,
                                    traits<T>::ColsAtCompileTime,
                                    AutoAlign | (Flags & RowMajorBit ? RowMajor : ColMajor),
                                    traits<T>::MaxColsAtCompileTime> type;
};

// From util/XprHelper.h
// TODO: Fix
template<>
struct plain_constant_type<spatial_dyn::SpatialMotion<double,1,0,1>, double> {
  typedef CwiseNullaryOp<scalar_constant_op<double>, const spatial_dyn::SpatialMotion<double,1,0,1>> type;
};
template<>
struct plain_constant_type<spatial_dyn::SpatialForce<double,1,0,1>, double> {
  typedef CwiseNullaryOp<scalar_constant_op<double>, const spatial_dyn::SpatialForce<double,1,0,1>> type;
};

// From CoreEvaluators.h
template<typename Scalar, int Cols, int Options, int MaxCols>
struct evaluator<spatial_dyn::SpatialMotion<Scalar, Cols, Options, MaxCols>>
    : evaluator<PlainObjectBase<spatial_dyn::SpatialMotion<Scalar, Cols, Options, MaxCols>>> {
  typedef spatial_dyn::SpatialMotion<Scalar, Cols, Options, MaxCols> XprType;
  evaluator() {}
  explicit evaluator(const XprType& m) : evaluator<PlainObjectBase<XprType> >(m) {}
};
template<typename Scalar, int Cols, int Options, int MaxCols>
struct evaluator<spatial_dyn::SpatialForce<Scalar, Cols, Options, MaxCols>>
    : evaluator<PlainObjectBase<spatial_dyn::SpatialForce<Scalar, Cols, Options, MaxCols>>> {
  typedef spatial_dyn::SpatialForce<Scalar, Cols, Options, MaxCols> XprType;
  evaluator() {}
  explicit evaluator(const XprType& m) : evaluator<PlainObjectBase<XprType> >(m) {}
};

}  // namespace internal
}  // namespace Eigen

#endif  // SPATIAL_DYN_EIGEN_SPATIAL_DECLARATIONS_H_
