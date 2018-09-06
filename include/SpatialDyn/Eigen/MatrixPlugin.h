/**
 * MatrixPlugin.h
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

/**
 * Decode an Eigen matrix from Matlab format:
 *
 * Usage:
 *   Eigen::Vector3d x = Eigen::DecodeMatlab<Eigen::Vector3d>("1 2 3");
 *   Eigen::MatrixXd A = Eigen::DecodeMatlab<Eigen::MatrixXd>("1 2 3; 4 5 6");
 */
EIGEN_DEVICE_FUNC
EIGEN_STRONG_INLINE
static Matrix<Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> FromMatlab(const std::string& str) {
  std::string str_local = str;

  // Create matrix to return
  Matrix<Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> matrix;
  size_t num_cols = matrix.cols();
  size_t num_rows = matrix.rows();
  Scalar eof;  // Used to check for eof and print Matrix scalar type

  // Count number of columns
  size_t idx_col_end = 0;
  if (num_cols == 0 || (num_cols == 1 && num_rows == 0)) {
    num_cols = 0;
    size_t idx = 0;
    idx_col_end = str.find_first_of(';');
    while (idx < idx_col_end) {
      // Skip over extra whitespace
      idx = str.find_first_not_of(' ', idx);
      if (idx >= idx_col_end) break;

      // Find next delimiter
      if (str[idx] == ';') break;
      idx = str.find_first_of(' ', idx + 1);
      ++num_cols;
    }
  }

  // Count number of rows
  if (num_rows == 0) {
    size_t idx = idx_col_end;
    if (idx_col_end != 0) {  // First row already traversed
      idx = idx_col_end;
      num_rows = 1;
    }
    while (idx != std::string::npos) {
      // Clear semicolons as we go
      if (idx != 0) str_local[idx] = ' ';

      // Find next delimiter
      idx = str.find_first_of(';', idx + 1);
      ++num_rows;
    }
    // Ignore trailing semicolon
    idx = str.find_last_not_of(' ');
    if (str[idx] == ';') --num_rows;
  } else {
    // Clear remaining semicolons
    for (size_t idx = idx_col_end; idx < str.size(); idx++) {
      if (str[idx] == ';') str_local[idx] = ' ';
    }
  }

  // Check number of rows and columns
  if (num_cols == 0)
    throw std::invalid_argument(
        "Eigen::decode(): Failed to decode Eigen::MatrixX" +
        std::string(typeid(eof).name()) + "(" + std::to_string(num_rows) +
        ", " + std::to_string(num_cols) + ") from: (" + str + ").");
  if (num_rows == 1) {
    // Convert to vector
    num_rows = num_cols;
    num_cols = 1;
  }
  if (matrix.rows() == 0 || matrix.cols() == 0) {
    matrix.resize(num_rows, num_cols);
  }

  // Parse matrix
  std::stringstream ss(str_local);
  for (size_t i = 0; i < num_rows; ++i) {
    for (size_t j = 0; j < num_cols; ++j) {
      ss >> matrix(i,j);
      if (ss.fail()) {
        throw std::invalid_argument(
            "Eigen::decode(): Failed to decode Eigen::MatrixX" +
            std::string(typeid(eof).name()) + "(" + std::to_string(num_rows) +
            ", " + std::to_string(num_cols) + ") from: (" + str + ").");
      }
    }
  }

  // Make sure there are no numbers left
  ss >> eof;
  if (!ss.fail()) {
    throw std::invalid_argument(
        "Eigen::decode(): Failed to decode Eigen::MatrixX" +
        std::string(typeid(eof).name()) + "(" + std::to_string(num_rows) +
        ", " + std::to_string(num_cols) + ") from: (" + str + ").");
  }

  return matrix;
}
