/**
 * matrix_plugin.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: May 22, 2018
 * Authors: Toki Migimatsu
 */

EIGEN_DEVICE_FUNC
EIGEN_STRONG_INLINE Matrix(const Scalar& a, const Scalar& b, const Scalar& c,
                           const Scalar& d, const Scalar& e, const Scalar& f) {
  Base::_check_template_params();
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Matrix, 6)
  m_storage.data()[0] = a;
  m_storage.data()[1] = b;
  m_storage.data()[2] = c;
  m_storage.data()[3] = d;
  m_storage.data()[4] = e;
  m_storage.data()[5] = f;
}
