#include "mpcplusplus/mpcplusplus.h"
#include "mpcplusplus/matrix_algebra.h"

#include <iostream>
#include <chrono>

// Comment to remove OSQP printing
#define PRINT_DEBUG 

// Comment to remove timing prints
#define PRINT_TIMING

using namespace std::chrono;
using namespace mpcplusplus;

// Nc refers only to additional constraints BEYOND dynamics and simple state bounds
LinearMPC::LinearMPC(const int N, const int Nx, const int Nu)
  : N_(N), nx_(Nx), nu_(Nu) {
  
  num_dyn_constraints_ = N * Nx;
  num_contact_constraints_ = Nu;
  
  num_state_vars_= (N_ + 1) * Nx;
  num_control_vars_= N * Nu;
  num_constraints_ = num_dyn_constraints_ + num_contact_constraints_;;
  num_decision_vars_ = num_state_vars_ + num_control_vars_;
}

void LinearMPC::update_weights(const std::vector<Eigen::MatrixXd> &Q, 
                               const std::vector<Eigen::MatrixXd> &R) {

  Eigen::MatrixXd Hq(num_state_vars_, num_state_vars_);
  for (int i = 0; i < N_+1; ++i) {
    Hq.block(i * nx_, i * nx_, nx_, nx_) = Q.at(i);
  }

  Eigen::MatrixXd Hu(num_control_vars_, num_control_vars_);
  for (int i = 0; i < N_; ++i) {
    Hu.block(i*nu_,i*nu_,nu_,nu_) = R.at(i);
  }

  H_ = math::block_diag(Hq, Hu);
  H_f_ = Hq;
  updated_weights_ = true;
}

void LinearMPC::update_dynamics(const std::vector<Eigen::MatrixXd> &Ad,
                                  const std::vector<Eigen::MatrixXd> &Bd) {
  assert(Ad.size() == N_);
  assert(Bd.size() == N_);

  Eigen::MatrixXd A_padded_eye (num_dyn_constraints_,num_state_vars_);
  A_padded_eye.setZero();

  A_padded_eye.block(0, nx_,num_dyn_constraints_, num_dyn_constraints_) =
      -Eigen::MatrixXd::Identity(num_dyn_constraints_, num_dyn_constraints_);

  Eigen::MatrixXd A_padded_ad(num_dyn_constraints_,num_state_vars_);
  A_padded_ad.setZero();

  for (int i = 0; i < N_; i++) {
    A_padded_ad.block(i * nx_, i * nx_, nx_, nx_) = Ad.at(i);
  }

  A_dyn_dense_.resize(num_dyn_constraints_, num_decision_vars_);
  A_dyn_dense_.setZero();
  A_dyn_dense_.block(0, 0, num_dyn_constraints_, num_state_vars_) = A_padded_eye + A_padded_ad;
  for (int i = 0; i < N_; i++) {
    A_dyn_dense_.block(i * nx_, num_state_vars_ + i * nu_, nx_, nu_) = Bd.at(i);
  }

  b_dyn_ = Eigen::MatrixXd::Zero(num_dyn_constraints_,1);
}

void LinearMPC::update_contact_constraint(const Eigen::MatrixXd &constraint,
                             const Eigen::VectorXd &lb,
                             const Eigen::VectorXd &ub) {
  b_contact_lo_ = lb;
  b_contact_hi_ = ub;
}

void LinearMPC::get_cost_function(const Eigen::MatrixXd &ref_traj,
                                  Eigen::VectorXd &f) {

  // Construct fx vector
  Eigen::MatrixXd y(1,num_state_vars_);
  y.block(0, 0, 1, num_state_vars_) =
      math::reshape(ref_traj, 1, ref_traj.cols() * ref_traj.rows());
  
  Eigen::MatrixXd fx = y * H_f_;

  // Construct fu vector
  Eigen::MatrixXd fu = Eigen::MatrixXd::Zero(N_ * nu_, 1);

  // Construct f vector
  f.resize(num_decision_vars_);
  f.setZero();
  f << -fx.transpose(), -fu;
}

//========================================================================================
void LinearMPC::get_output(const Eigen::MatrixXd &x_out,
                      Eigen::MatrixXd &first_control,
                      Eigen::MatrixXd &opt_traj) {

  // Resize and wipe output containers
  first_control.resize(nu_,1);
  first_control.setZero();
  opt_traj.resize(nx_,N_+1);
  opt_traj.setZero();

  // Collect optimized control
  first_control = x_out.block((N_+1)*nx_,0,nu_,1);

  // Collect optimized trajectory
  for (size_t i = 0; i < N_ + 1; ++i)
  {
    for (size_t j = 0; j < nx_; ++j)
    {
      opt_traj(j,i) = x_out(i*nx_+j,0);
    }
  }
}

void LinearMPC::solve(const Eigen::VectorXd &initial_state,
                      const Eigen::MatrixXd &ref_traj, Eigen::MatrixXd &x_out,
                      double &f_val) {
  #ifdef PRINT_TIMING
    steady_clock::time_point t1 = steady_clock::now();
  #endif

  // Update linear component of cost function to reflect new reference traj
  Eigen::VectorXd f;
  this->get_cost_function(ref_traj, f);

  // Create correctly sized linear constraint matrix to be populated with dynamics, contact info
  Eigen::MatrixXd A_dense(num_constraints_, num_decision_vars_);

  #ifdef PRINT_DEBUG 
  std::cout << "A_dyn_dense size: " << A_dyn_dense_.rows() << " x " << A_dyn_dense_.cols() << std::endl;
  std::cout << "Nq: " << num_decision_vars_ << std::endl;
  std::cout << "Nx_vars: " << num_dyn_constraints_ << std::endl;
  std::cout << "Number of constraints: " << num_constraints_ << std::endl;
  std::cout << "Nu: " << nu_ << std::endl;
  std::cout << "Nx: " << nx_ << std::endl;
  #endif

  // Add dynamics matrix to linear constraint matrix
  //A_dense.block(0, 0, num_dyn_constraints_, num_decision_vars_) = A_dyn_dense_;

  A_dense.topRows(num_dyn_constraints_) = A_dyn_dense_;

  // Add contact matrix to linear constraint matrix
  //A_dense.bottomRows(m_)

  // Convert hessian and linear constraint matrix 
  Eigen::SparseMatrix<double> H = H_.sparseView();
  Eigen::SparseMatrix<double> A = A_dense.sparseView();

  // Setup lower and  upper bounds for linear constraints
  Eigen::VectorXd l(num_constraints_);
  l.setZero();
  //l << b_dyn_, b_contact_lo_;

  Eigen::VectorXd u(num_constraints_);
  u.setZero();
  //u << b_dyn_, b_contact_hi_;

   std::cout << "Here" << std::endl;

  // Init solver if not already initialized
  if (!solver_.isInitialized()) {
    solver_.data()->setNumberOfVariables(num_decision_vars_);
    solver_.data()->setNumberOfConstraints(num_constraints_);
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

    solver_.updateLinearConstraintsMatrix(A);
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
