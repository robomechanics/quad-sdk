#include "mpcplusplus/mpcplusplus.h"
#include "mpcplusplus/matrix_algebra.h"

#include <iostream>
#include <chrono>

// Comment to remove OSQP printing
//#define PRINT_DEBUG 

// Comment to remove timing prints
#define PRINT_TIMING

using namespace std::chrono;
using namespace mpcplusplus;

LinearMPC::LinearMPC(const Eigen::MatrixXd &Ad,
                     const Eigen::MatrixXd &Bd, const Eigen::MatrixXd &Q,
                     const Eigen::MatrixXd &Qn, const Eigen::MatrixXd &R,
                     const Eigen::MatrixXd &state_bounds,
                     const Eigen::MatrixXd &control_bounds,const int N)
  : m_state_bounds(state_bounds), m_control_bounds(control_bounds), m_N(N) {
  m_Nx = Ad.rows();
  m_Nu = Bd.cols();

  m_Nq = (m_N + 1) * m_Nx + m_N * m_Nu;
  m_Nx_vars = m_N * m_Nx;
  m_Nx_decision = (m_N + 1) * m_Nx;
  m_Nconst = m_Nq + m_N * m_Nx;
  m_num_control_vars = m_N * m_Nu;
  m_num_state_vars = (m_N + 1) * m_Nx;
  m_num_decision_vars = (m_N + 1) * m_Nx + m_N * m_Nu;
  m_num_constraints = m_num_decision_vars + m_N * m_Nx;

  this->update_weights(Q,Qn,R);
  this->update_statespace(Ad,Bd);
}

LinearMPC::LinearMPC(const int N, const int Nx, const int Nu)
  : m_N(N), m_Nx(Nx), m_Nu(Nu) {
  m_Nq = (m_N + 1) * m_Nx + m_N * m_Nu;
  m_Nx_vars = m_N * m_Nx;
  m_Nx_decision = (m_N + 1) * m_Nx;
  m_Nconst = m_Nq + m_N * m_Nx;
  m_num_control_vars = m_N * m_Nu;
  m_num_state_vars = (m_N + 1) * m_Nx;
  m_num_decision_vars = (m_N + 1) * m_Nx + m_N * m_Nu;
  m_num_constraints = m_num_decision_vars + m_N * m_Nx;
}

void LinearMPC::update_weights(const Eigen::MatrixXd &Q, 
                               const Eigen::MatrixXd &Qn,
                               const Eigen::MatrixXd &R) {

  Eigen::MatrixXd Hq = math::kron(Eigen::MatrixXd::Identity(m_N, m_N), Q);
  Eigen::MatrixXd Hu = math::kron(Eigen::MatrixXd::Identity(m_N, m_N), R);
  H_ = math::block_diag(Hq, Qn, Hu);
  H_f_ = math::block_diag(math::kron(Eigen::MatrixXd::Identity(m_N, m_N), Q), Qn);
  H_f_ = H_f_.block(0, 0, m_num_state_vars, m_num_state_vars);
}

void LinearMPC::update_weights_vector(const std::vector<Eigen::MatrixXd> &Q, 
                               const std::vector<Eigen::MatrixXd> &R) {

  Eigen::MatrixXd Hq(m_Nx_decision, m_Nx_decision);
  for (int i = 0; i < m_N+1; ++i) {
    Hq.block(i * m_Nx, i * m_Nx, m_Nx, m_Nx) = Q.at(i);
  }

  Eigen::MatrixXd Hu(m_num_control_vars, m_num_control_vars);
  for (int i = 0; i < m_N; ++i) {
    Hu.block(i*m_Nu,i*m_Nu,m_Nu,m_Nu) = R.at(i);
  }

  H_ = math::block_diag(Hq, Hu);
  H_f_ = Hq;
  updated_weights_ = true;
}

void LinearMPC::update_statespace(const Eigen::MatrixXd &Ad,
                                  const Eigen::MatrixXd &Bd) {

  Eigen::MatrixXd A_padded_eye (m_Nx_vars,m_Nx_decision);
  A_padded_eye.setZero();

  A_padded_eye.block(0, m_Nx,m_Nx_vars, m_Nx_vars) =
      -Eigen::MatrixXd::Identity(m_Nx_vars, m_Nx_vars);

  Eigen::MatrixXd A_padded_ad(m_Nx_vars,m_Nx_decision);
  A_padded_ad.setZero();
  for (int i = 0; i < m_N; i++) {
    A_padded_ad.block(i * m_Nx, i * m_Nx, m_Nx, m_Nx) = Ad;
  }

  A_dyn_dense_.resize(m_Nx_vars,m_Nq);
  A_dyn_dense_.setZero();
  A_dyn_dense_.block(0, 0, m_Nx_vars, m_Nx_decision) = A_padded_eye + A_padded_ad;
  for (int i = 0; i < m_N; i++) {
    A_dyn_dense_.block(i * m_Nx, m_Nx_decision + i * m_Nu, m_Nx, m_Nu) = Bd;
  }

  b_dyn_ = Eigen::MatrixXd::Zero(m_Nx_vars,1);
}

void LinearMPC::update_statespace_vector(const std::vector<Eigen::MatrixXd> &Ad,
                                  const std::vector<Eigen::MatrixXd> &Bd) {
  assert(Ad.size() == m_N);
  assert(Bd.size() == m_N);

  Eigen::MatrixXd A_padded_eye (m_Nx_vars,m_Nx_decision);
  A_padded_eye.setZero();

  A_padded_eye.block(0, m_Nx,m_Nx_vars, m_Nx_vars) =
      -Eigen::MatrixXd::Identity(m_Nx_vars, m_Nx_vars);

  Eigen::MatrixXd A_padded_ad(m_Nx_vars,m_Nx_decision);
  A_padded_ad.setZero();

  for (int i = 0; i < m_N; i++) {
    A_padded_ad.block(i * m_Nx, i * m_Nx, m_Nx, m_Nx) = Ad.at(i);
  }

  A_dyn_dense_.resize(m_Nx_vars,m_Nq);
  A_dyn_dense_.setZero();
  A_dyn_dense_.block(0, 0, m_Nx_vars, m_Nx_decision) = A_padded_eye + A_padded_ad;
  for (int i = 0; i < m_N; i++) {
    A_dyn_dense_.block(i * m_Nx, m_Nx_decision + i * m_Nu, m_Nx, m_Nu) = Bd.at(i);
  }

  b_dyn_ = Eigen::MatrixXd::Zero(m_Nx_vars,1);
  updated_statespace_ = true;
}

void LinearMPC::get_cost_function(const Eigen::MatrixXd &ref_traj,
                                  Eigen::VectorXd &f) {

  // Construct fx vector
  Eigen::MatrixXd y(1,m_Nx_decision);
  y.block(0, 0, 1, m_Nx_decision) =
      math::reshape(ref_traj, 1, ref_traj.cols() * ref_traj.rows());
  
  Eigen::MatrixXd fx = y * H_f_;

  // Construct fu vector
  Eigen::MatrixXd fu = Eigen::MatrixXd::Zero(m_N * m_Nu, 1);

  // Construct f vector
  f.resize(m_Nq);
  f.setZero();
  f << -fx.transpose(), -fu;
}

void LinearMPC::get_state_control_bounds(const Eigen::VectorXd &initial_state,
                                         Eigen::VectorXd &lb,
                                         Eigen::VectorXd &ub) {

  Eigen::VectorXd x_min = m_state_bounds.col(0);
  Eigen::VectorXd x_max = m_state_bounds.col(1);

  Eigen::VectorXd u_min = m_control_bounds.col(0);
  Eigen::VectorXd u_max = m_control_bounds.col(1);

  lb.resize(m_num_decision_vars);
  lb.setZero();
  ub.resize(m_num_decision_vars);
  ub.setZero();

  lb << initial_state, x_min.replicate(m_N, 1), u_min.replicate(m_N, 1);
  ub << initial_state, x_max.replicate(m_N, 1), u_max.replicate(m_N, 1);
}

//========================================================================================
void LinearMPC::get_output(const Eigen::MatrixXd &x_out,
                      Eigen::MatrixXd &first_control,
                      Eigen::MatrixXd &opt_traj) {

  // Resize and wipe output containers
  first_control.resize(m_Nu,1);
  first_control.setZero();
  opt_traj.resize(m_Nx,m_N+1);
  opt_traj.setZero();

  // Collect optimized control
  first_control = x_out.block((m_N+1)*m_Nx,0,m_Nu,1);

  // Collect optimized trajectory
  for (size_t i = 0; i < m_N + 1; ++i)
  {
    for (size_t j = 0; j < m_Nx; ++j)
    {
      opt_traj(j,i) = x_out(i*m_Nx+j,0);
    }
  }
}

void LinearMPC::solve(const Eigen::VectorXd &initial_state,
                      const Eigen::MatrixXd &ref_traj, Eigen::MatrixXd &x_out,
                      double &f_val) {
  #ifdef PRINT_TIMING
    steady_clock::time_point t1 = steady_clock::now();
  #endif

  // Collect MPC Matrices
  Eigen::VectorXd f;
  this->get_cost_function(ref_traj, f);

  Eigen::VectorXd lb_simple, ub_simple; // Should be row vectors
  this->get_state_control_bounds(initial_state, lb_simple, ub_simple);

  // Cast to OSQP style QP
  Eigen::MatrixXd A_dense(A_dyn_dense_.rows() + m_Nq, A_dyn_dense_.cols());
  A_dense.block(0, 0, A_dyn_dense_.rows(), A_dyn_dense_.cols()) = A_dyn_dense_;
  A_dense.block(A_dyn_dense_.rows(), 0, m_Nq, m_Nq) =
      Eigen::MatrixXd::Identity(m_Nq, m_Nq);

  Eigen::SparseMatrix<double> H = H_.sparseView();
  Eigen::SparseMatrix<double> A = A_dense.sparseView();

  Eigen::VectorXd l(m_Nx_vars + m_Nq);
  l.setZero();
  l << b_dyn_, lb_simple;

  Eigen::VectorXd u(m_Nx_vars + m_Nq);
  u.setZero();
  u << b_dyn_, ub_simple;

  // Init solver if not already initialized
  if (!solver_.isInitialized()) {
    solver_.data()->setNumberOfVariables(m_Nq);
    solver_.data()->setNumberOfConstraints(m_Nconst);
    solver_.data()->setHessianMatrix(H);
    solver_.data()->setGradient(f);
    solver_.data()->setLinearConstraintsMatrix(A);
    solver_.data()->setLowerBound(l);
    solver_.data()->setUpperBound(u);
    #ifdef PRINT_DEBUG
      solver_.settings()->setVerbosity(true);
    #else
      solver_.settings()->setVerbosity(false);
    #endif
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setCheckTermination(10);
    solver_.initSolver();
  }
  else { // Update components of QP that change from iter to iter

    if (updated_statespace_) {
      solver_.updateLinearConstraintsMatrix(A);
      updated_statespace_ = false;
    }
    if (updated_weights_) {
      solver_.updateHessianMatrix(H);
      updated_weights_ = false;
    }

    solver_.updateBounds(l,u);
    solver_.updateGradient(f);
  }

  // Call solver
  solver_.solve();
  x_out = solver_.getSolution();

  #ifdef PRINT_TIMING
    steady_clock::time_point t2 = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "LinearMPC::solve completed in "
              << time_span.count()*1000.0 << " milliseconds"
              << std::endl;
  #endif
}
