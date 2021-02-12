
#include "mpcplusplus/mpcplusplus.h"
#include "mpcplusplus/matplotlibcpp.h"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <limits>

#include <gtest/gtest.h>

const double INF = std::numeric_limits<double>::max();
const double NINF = std::numeric_limits<double>::min();
namespace plt = matplotlibcpp;

TEST(TestUseCase, quadVariable) {

  // Configurable parameters
  const int Nu = 13; // Appended gravity term
  const int Nx = 12; // Number of states
  const int N = 10;   // Time horizons to consider
  const double dt = 0.1;             // Time horizon
  const int m = 10;                   // Mass of quad

  // Weights on state deviation and control input
  Eigen::MatrixXd Qx(Nx, Nx);
  Qx.setZero();
  Qx.diagonal() << 1,1,1,1,1,1,1,1,1,1,1,1;
  Eigen::MatrixXd Qn = 5 * Qx;
  Eigen::MatrixXd Ru(Nu, Nu);
  Ru.setZero();
  Ru.diagonal() << 1,1,1,1,1,1,1,1,1,1,1,1,0;

  // State and control bounds (fixed for a given solve)
  Eigen::VectorXd state_lo(Nx);
  state_lo << NINF,NINF,0,NINF,NINF,NINF,NINF,NINF,NINF,NINF,NINF,NINF;
  Eigen::VectorXd state_hi(Nx);
  state_hi << INF,INF,0.4,INF,INF,INF,INF,INF,INF,INF,INF,INF;

  Eigen::VectorXd control_lo(Nu);
  control_lo << NINF,NINF,NINF,NINF,NINF,NINF,NINF,NINF,NINF,NINF,NINF,NINF,0.9999;
  Eigen::VectorXd control_hi(Nu);
  control_hi << INF,INF,INF,INF,INF,INF,INF,INF,INF,INF,INF,INF,1.0001;

  // Create vectors of dynamics matrices at each step,
  // weights at each step and contact sequences at each step
  std::vector<Eigen::MatrixXd> Ad_vec(N);
  std::vector<Eigen::MatrixXd> Bd_vec(N);
  std::vector<Eigen::MatrixXd> Q_vec(N+1);
  std::vector<Eigen::MatrixXd> U_vec(N);
  std::vector<std::vector<bool>> contact_sequences(N);
  Eigen::MatrixXd ref_traj(Nx,N+1);
  Eigen::VectorXd initial_state(Nx);

  initial_state << 0,0,0.4,0,0,0,0,0,0,0,0,0;
  for (int i = 0; i < N; ++i) {
    Ad_vec.at(i) = Eigen::MatrixXd::Ones(Nx,Nx);
    Bd_vec.at(i) = Eigen::MatrixXd::Ones(Nx,Nu);
    Q_vec.at(i) = Qx;
    U_vec.at(i) = Ru;
    ref_traj.col(i) = initial_state;
    contact_sequences.at(i).resize(4);
    for (int j = 0; j < 4; ++j) {
      contact_sequences.at(i).at(j) = true;
    }
  }
  Q_vec.back() = Qx;
  ref_traj.col(N) = initial_state;

  double fmin = 5;
  double fmax = 50;
  double mu = 0.6;

  // Setup MPC class, call necessary functions
  mpcplusplus::LinearMPC mpc(N,Nx,Nu);

  mpc.update_weights(Q_vec,U_vec);
  mpc.update_dynamics(Ad_vec,Bd_vec);
  mpc.update_contact(contact_sequences, fmin, fmax, mu);
  mpc.update_state_bounds(state_lo, state_hi);
  mpc.update_control_bounds(control_lo, control_hi);

  // Solve, collect output and cost val
  Eigen::MatrixXd x_out;
  double f_val;

  mpc.solve(initial_state, ref_traj, x_out, f_val);
  mpc.solve(initial_state, ref_traj, x_out, f_val);
  
  Eigen::MatrixXd opt_traj;
  Eigen::VectorXd first_control;
  mpc.get_output(x_out, first_control, opt_traj);

  // Plot output
  std::vector<double> z(N+1);
  for (int i = 0; i < N+1; ++i) {
    z.at(i) = opt_traj(2,i);
  }

  std::vector<double> first_control_stl(Nu);
  for (int i = 0; i < Nu; ++i) {
    first_control_stl.at(i) = first_control(i);
  }

  plt::clf();
  plt::ion();
  //plt::named_plot("Z", z);
  plt::named_plot("Forces", first_control_stl);
  plt::xlabel("horizon index");
  plt::ylabel("forces");
  plt::legend();
  plt::show();
  plt::pause(3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}