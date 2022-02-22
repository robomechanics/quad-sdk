#include <gtest/gtest.h>
#include <nmpc_controller/quad_nlp.h>
#include <ros/ros.h>

#include <chrono>

TEST(NMPCTest, testUtils) {
  int type = NONE;
  int N = 24;
  int n = 12;
  int n_null = 24;
  int n_complex = n + n_null;
  int m = 12;
  double dt = 0.03;
  double mu = 1;
  double panic_weights = 1;
  Eigen::MatrixXd Q(n, 1);
  Eigen::MatrixXd R(m, 1);
  Eigen::MatrixXd Q_factor(n, 1);
  Eigen::MatrixXd R_factor(m, 1);
  Eigen::MatrixXd x_min(n, 1);
  Eigen::MatrixXd x_max(n, 1);
  Eigen::MatrixXd x_min_complex(n_complex, 1);
  Eigen::MatrixXd x_max_complex(n_complex, 1);
  Eigen::MatrixXd u_min(m, 1);
  Eigen::MatrixXd u_max(m, 1);

  double state_weight = 1;
  double input_weight = 0;

  Q.fill(state_weight);
  R.fill(input_weight);
  Q_factor.fill(1);
  R_factor.fill(1);
  x_min.fill(-1);
  x_max.fill(1);
  x_min_complex.fill(-1);
  x_max_complex.fill(1);
  u_min.fill(-10);
  u_max.fill(10);

  quadNLP nlp(type, N, n, n_null, m, dt, mu, panic_weights, Q, R, Q_factor,
              R_factor, x_min, x_max, x_min_complex, x_max_complex, u_min,
              u_max);

  Eigen::VectorXi complexity_schedule(N + 1);

  // Check num vars when no complex elements
  complexity_schedule.setZero();
  EXPECT_FALSE(nlp.apply_slack_to_complex_);
  nlp.update_complexity_schedule(complexity_schedule);
  EXPECT_EQ(nlp.n_vars_, (n * 3 + m) * N + n);
  int g = n + 16;
  EXPECT_EQ(nlp.n_constraints_, (g + 2 * n) * N);

  std::cout << "nlp.n_vars_ = " << nlp.n_vars_ << std::endl;
  std::cout << "nlp.n_vars_ target = " << (n * 3 + m) * N + n << std::endl;
  std::cout << "nlp.n_constraints_ = " << nlp.n_constraints_ << std::endl;
  std::cout << "nlp.n_constraints_ target = " << (g + 2 * n) * N << std::endl;
  std::cout << "nlp.complexity_schedule_ = "
            << nlp.complexity_schedule_.transpose() << std::endl;
  std::cout << "nlp.sys_id_schedule_ = " << nlp.sys_id_schedule_.transpose()
            << std::endl;
  std::cout << "nlp.nnz_jac_g_ = " << nlp.nnz_jac_g_ << std::endl;
  std::cout << "nlp.nnz_step_jac_g_[0] = " << nlp.nnz_step_jac_g_[0]
            << std::endl;
  std::cout << "nlp.iRow_jac_g_.sum() = " << nlp.iRow_jac_g_.sum() << std::endl;
  std::cout << "nlp.jCol_jac_g_.sum() = " << nlp.jCol_jac_g_.sum() << std::endl;
  std::cout << "nlp.nnz_h_ = " << nlp.nnz_h_ << std::endl;
  std::cout << "nlp.nnz_compact_h_ = " << nlp.nnz_compact_h_ << std::endl;
  std::cout << "nlp.nnz_step_hess_[0] = " << nlp.nnz_step_hess_[0] << std::endl;
  std::cout << "nlp.iRow_h_.sum() = " << nlp.iRow_h_.sum() << std::endl;
  std::cout << "nlp.jCol_h_.sum() = " << nlp.jCol_h_.sum() << std::endl;
  std::cout << "nlp.iRow_compact_h_.sum() = " << nlp.iRow_compact_h_.sum()
            << std::endl;
  std::cout << "nlp.jCol_compact_h_.sum() = " << nlp.jCol_compact_h_.sum()
            << std::endl;

  // // Check against prior implementation
  EXPECT_TRUE(nlp.nnz_jac_g_ == 4248);
  EXPECT_TRUE(nlp.nnz_step_jac_g_[0] == 129);
  EXPECT_TRUE(nlp.iRow_jac_g_.sum() == 2134776);
  EXPECT_TRUE(nlp.jCol_jac_g_.sum() == 1583880);
  EXPECT_TRUE(nlp.nnz_h_ == 1608);
  EXPECT_TRUE(nlp.nnz_compact_h_ == 1539);
  EXPECT_TRUE(nlp.nnz_step_hess_[0] == 43);
  EXPECT_TRUE(nlp.iRow_h_.sum() == 474240);
  EXPECT_TRUE(nlp.jCol_h_.sum() == 462000);
  EXPECT_TRUE(nlp.iRow_compact_h_.sum() == 454092);
  EXPECT_TRUE(nlp.jCol_compact_h_.sum() == 441852);

  // Check num vars when all complex elements
  complexity_schedule.fill(1);
  nlp.update_complexity_schedule(complexity_schedule);
  EXPECT_EQ(nlp.n_vars_, (n_complex + n * 2 + m) * N + nlp.n_vec_[N]);

  // Check finite element indices when no complex elements
  complexity_schedule.fill(0);
  nlp.update_complexity_schedule(complexity_schedule);
  for (int i = 0; i < N; i++) {
    EXPECT_EQ(nlp.getPrimalFEIndex(i), i * (n + m));
  }

  // Check finite element indices when all complex elements
  complexity_schedule.fill(1);
  nlp.update_complexity_schedule(complexity_schedule);
  for (int i = 0; i < N; i++) {
    EXPECT_EQ(nlp.getPrimalFEIndex(i), i * (n_complex + m));
  }
}
