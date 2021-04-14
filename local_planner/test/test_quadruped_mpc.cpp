
#include "local_planner/quadruped_mpc.h"
#include "spirit_utils/matplotlibcpp.h"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <limits>

#include <gtest/gtest.h>

const double INF = OsqpEigen::INFTY;

namespace plt = matplotlibcpp;

TEST(TestUseCase, quadVariable) {

  // Configurable (system) parameters
  const int Nu = 13;      // Appended gravity term
  const int Nx = 12;      // Number of states
  const int N = 20;       // Time horizons to consider
  const double dt = 0.1;  // Time horizon
  const double m = 12;    // Mass of quad
  const double g = 9.81;  // gravitational constant
  const double Ixx = 0.1; // approximately SDF values, but will need refining
  const double Iyy = 0.2; // approximately SDF values, but will need refining
  const double Izz = 0.2; // approximately SDF values, but will need refining

  // Weights on state deviation and control input
  Eigen::MatrixXd Qx(Nx, Nx);
  Qx.setZero();
  double e = 1e-1;
  Qx.diagonal() << 1e6,1e6,5e7,1e5,1e5,1e4,e,e,e,e,e,1e2;

  Eigen::MatrixXd Ru(Nu, Nu);
  Ru.setZero();

  double Rfx = 1;
  double Rfy = 1;
  double Rfz = 1;
  Ru.diagonal() << Rfx,Rfy,Rfz,Rfx,Rfy,Rfz,Rfx,Rfy,Rfz,Rfx,Rfy,Rfz,0;

  // State bounds (fixed for a given solve) 
  Eigen::VectorXd state_lo(Nx);
  state_lo << -100,-100,0.15,-M_PI/4,-M_PI/4,-4*M_PI,-5,-5,-2,-M_PI,-M_PI,-M_PI;
  Eigen::VectorXd state_hi(Nx);
  state_hi << 100,100,0.4,M_PI/4,M_PI/4,4*M_PI,5,5,2,M_PI,M_PI,M_PI;

  // Robot body inertia matrix
  Eigen::Matrix3d Ib = Eigen::Matrix3d::Zero();
  Ib.diagonal() << Ixx,Iyy,Izz;

  // Foot positions in body frame
  const double body_l = 0.6;
  const double body_w = 0.3;

  // x1 y1 z1, x2 y2, z2 ... 
  Eigen::VectorXd foot_position(12);
  foot_position << -body_w/2,body_l/2,-0.25,
                   -body_w/2,-body_l/2,-0.25,
                    body_w/2,body_l/2,-0.25,
                    body_w/2,-body_l/2,-0.25;

  std::vector<Eigen::MatrixXd> Q_vec(N+1);
  std::vector<Eigen::MatrixXd> U_vec(N);
  std::vector<std::vector<bool>> contact_sequences(N);
  Eigen::MatrixXd ref_traj(Nx,N+1);
  Eigen::VectorXd initial_state(Nx);

  initial_state << 0,0,0.3,0,0,0,0,0,0,0,0,0;

  for (int i = 0; i < N+1; ++i) {
    Q_vec.at(i) = Qx;
    ref_traj.col(i) = initial_state;
    ref_traj(0,i) = 0.03*i; // x ramp
    ref_traj(1,i) = i > N/2 ? 0.2 : 0; // y step
    ref_traj(2,i) = 0.25 + 0.1*sin(i/2.0); // z sine
    ref_traj(4,i) = 0.3*cos(i/3.0);
    ref_traj(5,i) = 0.2*sin(i/3.0);
  }

  Eigen::MatrixXd foot_positions(N,12);
  for (int i = 0; i < N; ++i) {
    U_vec.at(i) = Ru;
    contact_sequences.at(i) = {true,true,true,true};
    foot_positions.row(i) = foot_position;
  }

  double mu = 0.6;
  double fmin = 5;
  double fmax = 50;

  // Setup MPC class, call necessary functions
  QuadrupedMPC mpc(N,Nx,Nu);
  mpc.setTimestep(dt);
  mpc.setMassProperties(m,Ib);
  mpc.update_weights(Q_vec,U_vec);
  mpc.update_dynamics(ref_traj,foot_positions);
  mpc.update_friction(mu);
  mpc.update_contact(contact_sequences, fmin, fmax);
  mpc.update_state_bounds(state_lo, state_hi);

  // Solve, collect output and cost val
  Eigen::MatrixXd x_out;
  mpc.solve(initial_state, ref_traj, x_out);
  mpc.solve(initial_state, ref_traj, x_out);

  double f_val;
  Eigen::MatrixXd opt_traj,control_traj;
  mpc.get_output(x_out, opt_traj, control_traj, f_val);
  std::cout << "Final cost: " << f_val << std::endl;

  std::cout << control_traj << std::endl;

  // Accumulate states in stl form for plotting
  std::vector<std::vector<double>> state_ref(Nx);
  std::vector<std::vector<double>> state_opt(Nx);
  for (int i = 0; i < Nx; ++i) {
    state_ref.at(i).resize(N+1);
    state_opt.at(i).resize(N+1);
    for (int j = 0; j < N+1; ++j) {
      state_ref.at(i).at(j) = ref_traj(i,j);
      state_opt.at(i).at(j) = opt_traj(i,j);
    }
  } 

  // Accumulate controls in stl form for plotting
  std::vector<std::vector<double>> control_opt(Nu);
  for (int i = 0; i < Nu; ++i) {
    control_opt.at(i).resize(N-1);
    for (int j = 0; j < N-1; ++j) {
      control_opt.at(i).at(j) = control_traj(i,j);
    }
  } 

  // // Plot everything
  
  // plt::figure();
  // plt::suptitle("Position Tracking");
  // const char* pos_names[6] = {"x","y","z","roll","pitch","yaw"};
  // for (int i = 0; i < 6;++i) {
  //   plt::subplot(2,3,i+1);
  //   plt::plot(state_opt.at(i));
  //   plt::plot(state_ref.at(i));
  //   plt::title(pos_names[i]);
  // }
  // plt::save("/home/nflowers/Desktop/position_mpc.png");

  // plt::figure();
  // plt::suptitle("Velocity Tracking");
  // const char* vel_names[6] = {"vx","vy","vz","wx","wy","wz"};
  // for (int i = 0; i < 6;++i) {
  //   plt::subplot(2,3,i+1);
  //   plt::plot(state_opt.at(i+6));
  //   plt::plot(state_ref.at(i+6));
  //   plt::title(vel_names[i]);
  // }
  // plt::save("/home/nflowers/Desktop/velocity_mpc.png");

  // plt::figure();
  // plt::suptitle("Control Efforts");
  // const char* control_names[3] = {"fx","fy","fz"};
  // for (int i = 0; i < 3;++i) {
  //   plt::subplot(1,3,i+1);
  //   plt::named_plot("FL",control_opt.at(i+0));
  //   plt::named_plot("BL",control_opt.at(i+3));
  //   plt::named_plot("FR",control_opt.at(i+6));
  //   plt::named_plot("BR",control_opt.at(i+9));
  //   plt::legend();
  //   plt::title(control_names[i]);
  // }
  // plt::save("/home/nflowers/Desktop/control_mpc.png");


  // plt::show();
  // plt::pause(1000);
  

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}