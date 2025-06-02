#ifndef MATRIX_ALGEBRA_H
#define MATRIX_ALGEBRA_H

#include <eigen3/Eigen/Eigen>

namespace math {
/**
 * @brief Compute the Kronecker product. A composite array made of blocks of the
 second array scaled by the first

 * @param[in] m1 first matrix
 * @param[in] m2 second matrix
 *
 * @return A result of the Kronecker product
 */
Eigen::MatrixXd kron(const Eigen::MatrixXd &m1, const Eigen::MatrixXd &m2) {
  uint32_t m1r = m1.rows();
  uint32_t m1c = m1.cols();
  uint32_t m2r = m2.rows();
  uint32_t m2c = m2.cols();

  Eigen::MatrixXd m3(m1r * m2r, m1c * m2c);

  for (int i = 0; i < m1r; i++) {
    for (int j = 0; j < m1c; j++) {
      m3.block(i * m2r, j * m2c, m2r, m2c) = m1(i, j) * m2;
    }
  }

  return m3;
}

/**
 * @brief Create a block diagonal matrix from provided matrices 3 input version
 *
 * @param m1 first matrix
 * @param m2 second matrix
 *
 * @return Created a block diagonal matrix
 */
Eigen::MatrixXd block_diag(const Eigen::MatrixXd &m1,
                           const Eigen::MatrixXd &m2) {
  uint32_t m1r = m1.rows();
  uint32_t m1c = m1.cols();
  uint32_t m2r = m2.rows();
  uint32_t m2c = m2.cols();

  Eigen::MatrixXd mf = Eigen::MatrixXd::Zero(m1r + m2r, m1c + m2c);
  mf.block(0, 0, m1r, m1c) = m1;
  mf.block(m1r, m1c, m2r, m2c) = m2;

  return mf;
}

/**
 * @brief Create a block diagonal matrix from provided matrices 3 input version
 *
 * @param[in] m1 first matrix
 * @param[in] m2 second matrix
 * @param[in] m3 third matrix
 *
 * @return Created a block diagonal matrix
 */
Eigen::MatrixXd block_diag(const Eigen::MatrixXd &m1, const Eigen::MatrixXd &m2,
                           const Eigen::MatrixXd &m3) {
  uint32_t m1r = m1.rows();
  uint32_t m1c = m1.cols();
  uint32_t m2r = m2.rows();
  uint32_t m2c = m2.cols();
  uint32_t m3r = m3.rows();
  uint32_t m3c = m3.cols();

  Eigen::MatrixXd bdm = Eigen::MatrixXd::Zero(m1r + m2r + m3r, m1c + m2c + m3c);
  bdm.block(0, 0, m1r, m1c) = m1;
  bdm.block(m1r, m1c, m2r, m2c) = m2;
  bdm.block(m1r + m2r, m1c + m2c, m3r, m3c) = m3;

  return bdm;
}

/**
 * @brief  Gives a new shape to an array without changing its data.
 *
 * @param[in] x input matrix
 * @param[in] r the number of row elements
 * @param[in] c the number of collum elements
 *
 * @return The new shape matrix
 */
Eigen::MatrixXd reshape(Eigen::MatrixXd x, uint32_t r, uint32_t c) {
  Eigen::Map<Eigen::MatrixXd> rx(x.data(), r, c);

  return rx;
}
}  // namespace math
#endif
