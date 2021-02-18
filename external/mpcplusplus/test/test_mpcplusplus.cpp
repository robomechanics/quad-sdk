
#include "mpcplusplus/mpcplusplus.h"
#include "mpcplusplus/matplotlibcpp.h"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <limits>

#include <gtest/gtest.h>

const double INF = OsqpEigen::INFTY;
const double NINF = -INF;

namespace plt = matplotlibcpp;


TEST(TestUseCase, quadVariable) {

  // Configurable parameters
  const int Nu = 13; // Appended gravity term
  const int Nx = 12; // Number of states
  const int N = 30;   // Time horizons to consider
  const double dt = 0.05;             // Time horizon
  const double m = 10;                   // Mass of quad
  const double g = 9.81;

  // Weights on state deviation and control input
  Eigen::MatrixXd Qx(Nx, Nx);
  Qx.setZero();
  Qx.diagonal() << 0,0,100, //x
                   0,0,0, //theta
                   0,0,0, //dx
                   0,0,0; //dtheta*/

  Eigen::MatrixXd Ru(Nu, Nu);
  Ru.setZero();
  double Rf = 0.00000000001;
  Ru.diagonal() << Rf,Rf,Rf,Rf,Rf,Rf,Rf,Rf,Rf,Rf,Rf,Rf,0;

  // State and control bounds (fixed for a given solve) 
  Eigen::VectorXd state_lo(Nx);
  state_lo << NINF,NINF,0.1,NINF,NINF,NINF,NINF,NINF,-1,NINF,NINF,NINF;
  Eigen::VectorXd state_hi(Nx);
  state_hi << INF,INF,0.4,INF,INF,INF,INF,INF,1,INF,INF,INF;

  Eigen::VectorXd control_lo(Nu);

  double fzmin = 0;
  control_lo << NINF,NINF,fzmin,NINF,NINF,fzmin,NINF,NINF,fzmin,NINF,NINF,fzmin,1;
  
  double fzmax = 100;
  Eigen::VectorXd control_hi(Nu);
  control_hi << INF,INF,fzmax,INF,INF,fzmax,INF,INF,fzmax,INF,INF,fzmax,1;

  // Create vectors of dynamics matrices at each step,
  // weights at each step and contact sequences at each step
  Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(Nx,Nx);
  Ad.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);
  Ad.block(6,6,6,6) = Eigen::MatrixXd::Identity(6,6);
  Ad.block(0,6,3,3) = Eigen::MatrixXd::Identity(3,3)*dt;
  Ad.block(3,9,3,3) = Eigen::MatrixXd::Identity(3,3)*dt; // rotation matrix (fixed to identity for now)
  std::cout << "Ad: " << std::endl << Ad << std::endl;

  Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(Nx,Nu);

  for (int i = 0; i < 4; ++i) {
    Bd.block(6,3*i,3,3) = Eigen::MatrixXd::Identity(3,3)/m*dt;
  }
  Bd(8,12) = -g*dt; // gravity acts downwards here
  std::cout << "Bd: " << std::endl << Bd << std::endl;

  std::vector<Eigen::MatrixXd> Ad_vec(N);
  std::vector<Eigen::MatrixXd> Bd_vec(N);
  std::vector<Eigen::MatrixXd> Q_vec(N+1);
  std::vector<Eigen::MatrixXd> U_vec(N);
  std::vector<std::vector<bool>> contact_sequences(N);
  Eigen::MatrixXd ref_traj(Nx,N+1);
  Eigen::VectorXd initial_state(Nx);

  initial_state << 0,0,0.31,0,0,0,0,0,0,0,0,0;
  for (int i = 0; i < N; ++i) {
    Ad_vec.at(i) = Ad;
    Bd_vec.at(i) = Bd;
    Q_vec.at(i) = Qx;
    U_vec.at(i) = Ru;
    ref_traj.col(i) = initial_state;
    ref_traj(2,i) = 0.25 + 0.1*sin(i/3.0);
    contact_sequences.at(i).resize(4);
    for (int j = 0; j < 4; ++j) {
      contact_sequences.at(i).at(j) = true;
    }
  }

  Q_vec.at(N) = 1*Qx;
  ref_traj.col(N) = initial_state;
  ref_traj(2,N) = 0.25 + 0.1*sin(N/3.0);

  double mu = 0.6;
  double fmin = -INF;
  double fmax = INF;

  // Setup MPC class, call necessary functions
  mpcplusplus::LinearMPC mpc(N,Nx,Nu);

  mpc.update_weights(Q_vec,U_vec);
  mpc.update_dynamics(Ad_vec,Bd_vec);

  mpc.update_contact(contact_sequences, mu, fmin, fmax);
  mpc.update_state_bounds(state_lo, state_hi);
  mpc.update_control_bounds(control_lo, control_hi);

  // Solve, collect output and cost val
  Eigen::MatrixXd x_out;
  double f_val;

  mpc.solve(initial_state, ref_traj, x_out, f_val);
  //mpc.solve(initial_state, ref_traj, x_out, f_val);
  
  //std::cout << "xout: " << std::endl << x_out << std::endl;

  Eigen::MatrixXd opt_traj,control_traj;
  Eigen::VectorXd first_control;
  mpc.get_output(x_out, first_control, opt_traj, control_traj);

  //std::cout << std::endl << "First control: " << first_control << std::endl;

  //std::cout << std::endl << "z ref traj: " << ref_traj.row(2) << std::endl;
  //std::cout << std::endl << "z traj: " << opt_traj.row(2) << std::endl;
  //std::cout << std::endl << "dz traj: " << opt_traj.row(8) << std::endl;
  
  std::vector<double> zref(N+1);
  std::vector<double> z(N+1);

  std::vector<double> dz(N+1);
  for (int i = 0; i < N+1; ++i) {
    zref.at(i) = ref_traj(2,i);
    z.at(i) = opt_traj(2,i);
    dz.at(i) = opt_traj(8,i);
  }

  std::cout << control_traj << std::endl;

  plt::clf();
  plt::ion();
  plt::named_plot("Z", z);
  plt::named_plot("Zref", zref);
  //plt::named_plot("dz", dz);
  plt::xlabel("horizon index");
  plt::ylabel("Z position");
  plt::legend();
  plt::show();
  plt::pause(1000);

  
  
}
/*
TEST(TestUseCase, monoVariable) {

  // Configurable parameters
  const int Nu = 2; // Appended gravity term
  const int Nx = 2; // Number of states
  const int N = 10;   // Time horizons to consider
  const double dt = 0.1;             // Time horizon
  const double g = 9.81;
  const double m = 10;

  // Weights on state deviation and control input
  Eigen::MatrixXd Qx(Nx, Nx);
  Qx.setZero();
  Qx.diagonal() << 1000000,0.000001; //dtheta

  Eigen::MatrixXd Ru(Nu, Nu);
  Ru.setZero();
  double Rf = 0.00001;
  Ru.diagonal() << Rf,0;

  // State and control bounds (fixed for a given solve) 
  Eigen::VectorXd state_lo(Nx);
  state_lo << NINF,NINF;
  Eigen::VectorXd state_hi(Nx);
  state_hi << INF,INF;

  Eigen::VectorXd control_lo(Nu);
  control_lo << 0,1;
  
  Eigen::VectorXd control_hi(Nu);
  control_hi << 20,1;

  // Create vectors of dynamics matrices at each step,
  // weights at each step and contact sequences at each step
  Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(Nx,Nx);
  Ad.block(0,0,2,2) = Eigen::MatrixXd::Identity(2,2);
  Ad(0,1) = dt;
  std::cout << "Ad: " << std::endl << Ad << std::endl;

  Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(Nx,Nu);
  Bd(1,0) = 1/m;
  Bd(1,1) = -g*dt; // gravity acts downwards here
  std::cout << "Bd: " << std::endl << Bd << std::endl;

  std::vector<Eigen::MatrixXd> Ad_vec(N);
  std::vector<Eigen::MatrixXd> Bd_vec(N);
  std::vector<Eigen::MatrixXd> Q_vec(N+1);
  std::vector<Eigen::MatrixXd> U_vec(N);
  std::vector<std::vector<bool>> contact_sequences(N);
  Eigen::MatrixXd ref_traj(Nx,N+1);
  Eigen::VectorXd initial_state(Nx);

  initial_state << 0.3,0;
  for (int i = 0; i < N; ++i) {
    Ad_vec.at(i) = Ad;
    Bd_vec.at(i) = Bd;
    Q_vec.at(i) = Qx;
    U_vec.at(i) = Ru;
    ref_traj.col(i) = initial_state;
    ref_traj(0,i) = 0.25 + 0.1*sin(i/3.0);
    contact_sequences.at(i).resize(4);
    for (int j = 0; j < 4; ++j) {
      contact_sequences.at(i).at(j) = true;
    }
  }

  Q_vec.at(N) = 1*Qx;
  ref_traj.col(N) = initial_state;
  ref_traj(0,N) = 0.25 + 0.1*sin(N/3.0);

  double mu = 0.6;
  double fmin = -INF;
  double fmax = INF;

  // Setup MPC class, call necessary functions
  mpcplusplus::LinearMPC mpc(N,Nx,Nu);
  
  mpc.update_weights(Q_vec,U_vec);
  mpc.update_dynamics(Ad_vec,Bd_vec);

  //mpc.update_contact(contact_sequences, mu, fmin, fmax);
  mpc.update_state_bounds(state_lo, state_hi);
  mpc.update_control_bounds(control_lo, control_hi);

  // Solve, collect output and cost val
  Eigen::MatrixXd x_out;
  double f_val;

  mpc.solve(initial_state, ref_traj, x_out, f_val);
  //mpc.solve(initial_state, ref_traj, x_out, f_val);
  
  //std::cout << "xout: " << std::endl << x_out << std::endl;

  Eigen::MatrixXd opt_traj,control_traj;
  Eigen::VectorXd first_control;
  mpc.get_output(x_out, first_control, opt_traj, control_traj);

  //std::cout << std::endl << "First control: " << first_control << std::endl;

  //std::cout << std::endl << "z ref traj: " << ref_traj.row(2) << std::endl;
  //std::cout << std::endl << "z traj: " << opt_traj.row(2) << std::endl;
  //std::cout << std::endl << "dz traj: " << opt_traj.row(8) << std::endl;
  
  std::vector<double> zref(N+1);
  std::vector<double> z(N+1);
  for (int i = 0; i < N+1; ++i) {
    zref.at(i) = ref_traj(0,i);
    z.at(i) = opt_traj(0,i);
  }

  std::cout << control_traj << std::endl;

  plt::clf();
  plt::ion();
  plt::named_plot("Z", z);
  plt::named_plot("Zref", zref);
  plt::xlabel("horizon index");
  plt::ylabel("Z position");
  plt::legend();
  plt::show();
  plt::pause(1000);
  
}*/

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}