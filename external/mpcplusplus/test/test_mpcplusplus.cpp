
#include "mpcplusplus/mpcplusplus.h"

#include <Eigen/Dense>
#include <iostream>
#include <math.h>

#include <gtest/gtest.h>

const double INF = 1000000;

TEST(TestUseCase, drone) {
  // Linear Drone example+
  // Configurable parameters
  const int Nu = 5; // Appended gravity term
  const int Nx = 6; // Number of states
  const int N = 10;   // Time horizons to consider
  const double dt = 0.1;             // Time horizon
  const int m = 1;                   // Mass of drone

  // Weights on state deviation and control input
  Eigen::MatrixXd Qx(Nx, Nx);
  Qx.setZero();
  Qx.diagonal() << 100, 100, 1000, 1, 1, 1;
  Eigen::MatrixXd Qn = 5 * Qx;
  Eigen::MatrixXd Ru(Nu, Nu);
  Ru.setZero();
  Ru.diagonal() << 1, 1, 1, 1, 1;

  // Bounds on states and controls
  Eigen::MatrixXd xbounds(Nx, 2);
  xbounds.col(0) << -INF, -INF, -INF, -INF, -INF, -INF;
  xbounds.col(1) << INF, INF, INF, INF, INF, INF;

  Eigen::MatrixXd ubounds(Nu, 2);
  ubounds.col(0) << -M_PI / 2, -M_PI / 2, -M_PI / 2, -20, 1;
  ubounds.col(1) << M_PI / 2, M_PI / 2, M_PI / 2, 20, 1;

  // Linearized dynamics
  Eigen::MatrixXd Ad(Nx, Nx);
  Ad.row(0) << 1, 0, 0, dt, 0, 0;
  Ad.row(1) << 0, 1, 0, 0, dt, 0;
  Ad.row(2) << 0, 0, 1, 0, 0, dt;
  Ad.row(3) << 0, 0, 0, 1, 0, 0;
  Ad.row(4) << 0, 0, 0, 0, 1, 0;
  Ad.row(5) << 0, 0, 0, 0, 0, 1;

  Eigen::MatrixXd Bd(Nx, Nu);
  Bd.row(0) << 0, 0, 0, 0, 0;
  Bd.row(1) << 0, 0, 0, 0, 0;
  Bd.row(2) << 0, 0, 0, 0, 0;
  Bd.row(3) << -9.8 * dt, 0, 0, 0, 0;
  Bd.row(4) << 0, 9.8 * dt, 0, 0, 0;
  Bd.row(5) << 0, 0, 0, dt / m, -9.8 * dt;

  // Reference trajectory
  Eigen::MatrixXd ref_traj(Nx, N + 1);
  ref_traj.setZero();
  for (int i = 0; i < N + 1; i++) {
    ref_traj(0, i) = cos(0.5 * i); // xref
    ref_traj(1, i) = sin(0.5 * i); // yref
    ref_traj(2, i) = 0.1 * i;      // zref
  }

  // Initial state
  Eigen::VectorXd x0 = ref_traj.col(0);

  LinearMPC mpc(Ad, Bd, Qx, Qn, Ru, xbounds, ubounds,N);

  // Test Cost Function
  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  mpc.get_cost_function(ref_traj, H, f);
  // TODO eval
  // std::cout << f << std::endl;

  // Test Dynamic Constraint function
  Eigen::MatrixXd Aeq;
  Eigen::MatrixXd beq;
  mpc.get_dynamics_constraint(Aeq, beq);

  // Test state and control bounds
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  mpc.get_state_control_bounds(x0, lb, ub);

  // Test solving
  Eigen::MatrixXd x_out;
  double f_val;
  mpc.solve(x0, ref_traj, x_out, f_val);

  // Test resolving
  mpc.solve(x0, ref_traj, x_out, f_val);

  // Test extracting solution
  Eigen::MatrixXd first_control;
  Eigen::MatrixXd opt_traj;
  mpc.get_output(x_out,first_control,opt_traj);
}

TEST(TestUseCase, quadruped) {
  // Linear Drone example+
  // Configurable parameters
  const int Nu = 5; // Appended gravity term
  const int Nx = 6; // Number of states
  const int N = 10;   // Time horizons to consider
  const double dt = 0.1;             // Time horizon
  const int m = 1;                   // Mass of drone

  // Weights on state deviation and control input
  Eigen::MatrixXd Qx(Nx, Nx);
  Qx.setZero();
  Qx.diagonal() << 100, 100, 1000, 1, 1, 1;
  Eigen::MatrixXd Qn = 5 * Qx;
  Eigen::MatrixXd Ru(Nu, Nu);
  Ru.setZero();
  Ru.diagonal() << 1, 1, 1, 1, 1;

  // Bounds on states and controls
  Eigen::MatrixXd xbounds(Nx, 2);
  xbounds.col(0) << -INF, -INF, -INF, -INF, -INF, -INF;
  xbounds.col(1) << INF, INF, INF, INF, INF, INF;

  Eigen::MatrixXd ubounds(Nu, 2);
  ubounds.col(0) << -M_PI / 2, -M_PI / 2, -M_PI / 2, -20, 1;
  ubounds.col(1) << M_PI / 2, M_PI / 2, M_PI / 2, 20, 1;

  // Linearized dynamics
  Eigen::MatrixXd Ad(Nx, Nx);
  Ad.row(0) << 1, 0, 0, dt, 0, 0;
  Ad.row(1) << 0, 1, 0, 0, dt, 0;
  Ad.row(2) << 0, 0, 1, 0, 0, dt;
  Ad.row(3) << 0, 0, 0, 1, 0, 0;
  Ad.row(4) << 0, 0, 0, 0, 1, 0;
  Ad.row(5) << 0, 0, 0, 0, 0, 1;

  Eigen::MatrixXd Bd(Nx, Nu);
  Bd.row(0) << 0, 0, 0, 0, 0;
  Bd.row(1) << 0, 0, 0, 0, 0;
  Bd.row(2) << 0, 0, 0, 0, 0;
  Bd.row(3) << -9.8 * dt, 0, 0, 0, 0;
  Bd.row(4) << 0, 9.8 * dt, 0, 0, 0;
  Bd.row(5) << 0, 0, 0, dt / m, -9.8 * dt;

  // Reference trajectory
  Eigen::MatrixXd ref_traj(Nx, N + 1);
  ref_traj.setZero();
  for (int i = 0; i < N + 1; i++) {
    ref_traj(0, i) = cos(0.5 * i); // xref
    ref_traj(1, i) = sin(0.5 * i); // yref
    ref_traj(2, i) = 0.1 * i;      // zref
  }

  // Initial state
  Eigen::VectorXd x0 = ref_traj.col(0);

  LinearMPC mpc(Ad, Bd, Qx, Qn, Ru, xbounds, ubounds,N);

  // Test Cost Function
  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  mpc.get_cost_function(ref_traj, H, f);
  // TODO eval
  // std::cout << f << std::endl;

  // Test Dynamic Constraint function
  Eigen::MatrixXd Aeq;
  Eigen::MatrixXd beq;
  mpc.get_dynamics_constraint(Aeq, beq);

  // Test state and control bounds
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  mpc.get_state_control_bounds(x0, lb, ub);

  // Test solving
  Eigen::MatrixXd x_out;
  double f_val;
  mpc.solve(x0, ref_traj, x_out, f_val);

  // Test resolving
  mpc.solve(x0, ref_traj, x_out, f_val);

  // Test extracting solution
  Eigen::MatrixXd first_control;
  Eigen::MatrixXd opt_traj;
  mpc.get_output(x_out,first_control,opt_traj);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}