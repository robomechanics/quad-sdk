#include <gtest/gtest.h>
#include <ros/ros.h>

#include <chrono>
#include <nmpc_controller/quad_nlp.h>

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

  Eigen::VectorXi complexity_schedule(N);

  // Check num vars when no complex elements
  complexity_schedule.setZero();
  nlp.update_complexity_schedule(complexity_schedule);
  int n_vars = nlp.getNumVariables();
  int n_constraints = nlp.getNumConstraints();
  EXPECT_EQ(n_vars, (n * 3 + m) * N);

  std::cout << "nlp.complexity_schedule_ = " << nlp.complexity_schedule_.transpose() << std::endl;
  std::cout << "nlp.sys_id_schedule_ = " << nlp.sys_id_schedule_.transpose() << std::endl;
  std::cout << "nlp.nnz_jac_g_ = " << nlp.nnz_jac_g_ << std::endl;
  std::cout << "nlp.nnz_step_jac_g_[0] = " << nlp.nnz_step_jac_g_[0] << std::endl;
  std::cout << "nlp.first_step_idx_mat_(LEG,JAC) = " << nlp.first_step_idx_mat_(LEG,JAC) << std::endl;
  std::cout << "nlp.iRow_jac_g_.sum() = " << nlp.iRow_jac_g_.sum() << std::endl;
  std::cout << "nlp.jCol_jac_g_.sum() = " << nlp.jCol_jac_g_.sum() << std::endl;
  std::cout << "nlp.nnz_h_ = " << nlp.nnz_h_ << std::endl;
  std::cout << "nlp.nnz_compact_h_ = " << nlp.nnz_compact_h_ << std::endl;
  std::cout << "nlp.nnz_step_hess_[0] = " << nlp.nnz_step_hess_[0] << std::endl;
  std::cout << "nlp.first_step_idx_mat_(LEG,HESS) = " << nlp.first_step_idx_mat_(LEG,HESS) << std::endl;
  std::cout << "nlp.iRow_h_.sum() = " << nlp.iRow_h_.sum() << std::endl;
  std::cout << "nlp.jCol_h_.sum() = " << nlp.jCol_h_.sum() << std::endl;
  std::cout << "nlp.iRow_compact_h_.sum() = " << nlp.iRow_compact_h_.sum() << std::endl;
  std::cout << "nlp.jCol_compact_h_.sum() = " << nlp.jCol_compact_h_.sum() << std::endl;

  // Check against prior implementation
  EXPECT_TRUE(nlp.nnz_jac_g_ == 4978);
  EXPECT_TRUE(nlp.nnz_step_jac_g_[0] == 161);
  EXPECT_TRUE(nlp.first_step_idx_mat_(LEG,JAC) == 38);
  EXPECT_TRUE(nlp.iRow_jac_g_.sum() == 2385295);
  EXPECT_TRUE(nlp.jCol_jac_g_.sum() == 1755775);
  EXPECT_TRUE(nlp.nnz_h_ == 2937);
  EXPECT_TRUE(nlp.nnz_compact_h_ == 2543);
  EXPECT_TRUE(nlp.nnz_step_hess_[0] == 101);
  EXPECT_TRUE(nlp.first_step_idx_mat_(LEG,HESS) == 63);
  EXPECT_TRUE(nlp.iRow_h_.sum() == 863430);
  EXPECT_TRUE(nlp.jCol_h_.sum() == 834003);
  EXPECT_TRUE(nlp.iRow_compact_h_.sum() == 750935);
  EXPECT_TRUE(nlp.jCol_compact_h_.sum() == 722543);

  // Check num vars when all complex elements
  complexity_schedule.fill(1);
  nlp.update_complexity_schedule(complexity_schedule);
  n_vars = nlp.getNumVariables();
  n_constraints = nlp.getNumConstraints();
  EXPECT_EQ(n_vars, (n_complex + n * 2 + m) * N);

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