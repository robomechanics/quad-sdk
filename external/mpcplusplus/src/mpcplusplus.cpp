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
  num_contact_constraints_ = 21*N; // num legs * num constraints per leg * num tsteps
  num_state_vars_= (N + 1) * Nx;
  num_control_vars_= N * Nu;
  num_decision_vars_ = num_state_vars_ + num_control_vars_;
  num_constraints_ = num_decision_vars_ + num_dyn_constraints_;// + 5*N; // num_contact_constraints_

  // Setup contact matrices and size vectors
  b_contact_lo_.resize(num_contact_constraints_);
  b_contact_lo_.setZero();

  b_contact_hi_.resize(num_contact_constraints_);
  b_contact_hi_.setZero();
}

void LinearMPC::update_weights(const std::vector<Eigen::MatrixXd> &Q, 
                               const std::vector<Eigen::MatrixXd> &R) {

  Eigen::MatrixXd Hq(num_state_vars_, num_state_vars_);
  Hq.setZero();
  for (int i = 0; i < N_+1; ++i) {
    Hq.block(i * nx_, i * nx_, nx_, nx_) = Q.at(i);
  }

  Eigen::MatrixXd Hu(num_control_vars_, num_control_vars_);
  Hu.setZero();
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

  A_dyn_dense_.resize(num_dyn_constraints_, num_decision_vars_);
  A_dyn_dense_.setZero();

  Eigen::MatrixXd A_padded_eye (num_dyn_constraints_,num_state_vars_);
  A_padded_eye.setZero();

  A_padded_eye.block(0, nx_ ,num_dyn_constraints_, num_dyn_constraints_) =
      -Eigen::MatrixXd::Identity(num_dyn_constraints_, num_dyn_constraints_);

  Eigen::MatrixXd A_padded_ad(num_dyn_constraints_,num_state_vars_);
  A_padded_ad.setZero();

  for (int i = 0; i < N_; i++) {
    A_padded_ad.block(i * nx_, i * nx_, nx_, nx_) = Ad.at(i);
  }

  A_dyn_dense_.block(0, 0, num_dyn_constraints_, num_state_vars_) = A_padded_eye + A_padded_ad;
  for (int i = 0; i < N_; i++) {
    A_dyn_dense_.block(i * nx_, num_state_vars_ + i * nu_, nx_, nu_) = Bd.at(i);
  }
  
  //std::cout << "Dynamics constraint: " << std::endl << A_dyn_dense_ << std::endl;

  b_dyn_ = Eigen::MatrixXd::Zero(num_dyn_constraints_,1);
}

void LinearMPC::update_contact(const std::vector<std::vector<bool>> contact_sequence,
                               const double mu,
                               const double fmin,
                               const double fmax) {
  assert(contact_sequence.size() == N_);
  assert(contact_sequence.front().size() == 4);
  assert(0 <= mu && mu <= 1);

  // Convert contact sequence to constraint matrix
  A_con_dense_ = Eigen::MatrixXd::Zero(num_contact_constraints_, num_decision_vars_);

  Eigen::MatrixXd C_toe(5,3);
  Eigen::VectorXd lo_contact(5);
  Eigen::VectorXd hi_contact(5);

  C_toe << 1, 0, -mu,
       1, 0, mu,
       0, 1, -mu,
       0, 1, mu,
       0, 0, 1;

  Eigen::MatrixXd C_body = Eigen::MatrixXd::Zero(21,13);
  for (int i = 0; i < 4; ++i) {
    C_body.block(5*i,3*i,5,3) = C_toe;
  }
  C_body(20,12) = 1;

  //lo_contact << -INF_,0,-INF_,0,fmin;
  //hi_contact << 0,INF_,0,INF_,fmax; //
  lo_contact << -INF_,-INF_,-INF_,-INF_,-INF_;//INF_;
  hi_contact << INF_,INF_,INF_,INF_,INF_;//INF_;

  b_contact_lo_.resize(num_contact_constraints_);
  b_contact_lo_.setZero();

  b_contact_hi_.resize(num_contact_constraints_);
  b_contact_hi_.setZero();

  for (int i = 0; i < N_; ++i) { // iterate over horizon
    A_con_dense_.block(21*i,num_state_vars_+13*i,21,13) = C_body;
    for (int j = 0; j < 4; ++j) { // iterate over legs
      int row_start = 21*i + 5*j;
      if (contact_sequence.at(i).at(j)) { // ground contact
         b_contact_lo_.segment(row_start,5) = lo_contact;
         b_contact_hi_.segment(row_start,5) = hi_contact;
      }
      else { 
        // do nothing, bounds have been zeroed out earlier in this function
      }
    }
    b_contact_lo_(21*i+20) = 0.999;
    b_contact_hi_(21*i+20) = 1.001; 
  }
  
  //std::cout << "Friction vector lo: " << std::endl << b_contact_lo_ << std::endl;
  //std::cout << "Friction vector hi: " << std::endl << b_contact_hi_ << std::endl;
  
  //std::cout << num_control_vars_ << std::endl;
  //std::cout << "Friction cone matrix: " << std::endl << A_con_dense_.rightCols(num_control_vars_) << std::endl;
  
}

void LinearMPC::update_state_bounds(const Eigen::VectorXd state_lo,
                                    const Eigen::VectorXd state_hi) {
  b_state_lo_ = state_lo.replicate(N_,1);
  b_state_hi_ = state_hi.replicate(N_,1);
}

void LinearMPC::update_control_bounds(const Eigen::VectorXd control_lo,
                                    const Eigen::VectorXd control_hi) {
  b_control_lo_ = control_lo.replicate(N_,1);
  b_control_hi_ = control_hi.replicate(N_,1);
}

void LinearMPC::get_cost_function(const Eigen::MatrixXd &ref_traj,
                                  Eigen::VectorXd &f) {

  // Construct fx vector
  Eigen::MatrixXd y(1,num_state_vars_);
  y.block(0, 0, 1, num_state_vars_) =
      math::reshape(ref_traj, 1, ref_traj.cols() * ref_traj.rows());


  //std::cout << "Y size: " << y.rows() << ", " << y.cols() << std::endl;
  Eigen::MatrixXd fx = -y * H_f_;

  // Construct fu vector
  Eigen::MatrixXd fu = Eigen::MatrixXd::Zero(N_ * nu_, 1);

  // Construct f vector
  f.resize(num_decision_vars_);
  f.setZero();
  f << fx.transpose(), fu;
}

//========================================================================================
void LinearMPC::get_output(const Eigen::MatrixXd &x_out,
                      Eigen::VectorXd &first_control,
                      Eigen::MatrixXd &opt_traj,
                      Eigen::MatrixXd &control_traj) {


  //std::cout << "Friction constraint vals: " << std::endl << A_con_dense_*x_out << std::endl;

  // Resize and wipe output containers
  first_control.resize(nu_);
  first_control.setZero();
  opt_traj.resize(nx_,N_+1);
  opt_traj.setZero();
  control_traj.resize(nu_, N_);
  control_traj.setZero();

  // Collect optimized control
  first_control = x_out.block((N_+1)*nx_,0,nu_,1);

  // Collect control trajectory
  for (size_t i = 0; i < N_; ++i) {
    for (size_t j = 0; j < nu_; ++j) {
      control_traj(j,i) = x_out((N_+1)*nx_ + i*nu_ + j,0);
    }
  }

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
  
  this->get_cost_function(ref_traj, f_);

  // Create correctly sized linear constraint matrix to be populated with dynamics, contact info
  Eigen::MatrixXd A_dense = Eigen::MatrixXd::Zero(num_constraints_, num_decision_vars_);

  // Add identity matrix to linear constraint matrix for state bounds
  A_dense.block(0,0,num_decision_vars_,num_decision_vars_) = Eigen::MatrixXd::Identity(num_decision_vars_,num_decision_vars_);
    
  // Add dynamics matrix to linear constraint matrix
  A_dense.block(num_decision_vars_,0,num_dyn_constraints_,num_decision_vars_) = A_dyn_dense_;

  // Gravity constraint

  /*
  Eigen::MatrixXd gravity_constraint = Eigen::MatrixXd::Zero(N_,num_control_vars_);
  Eigen::VectorXd gravity_lo(N_);
  Eigen::VectorXd gravity_hi(N_);

 for (int i = 0; i < N_; ++i){
    gravity_constraint(i,13*i+12) = 1;
    gravity_lo(i) = 1;
    gravity_hi(i) = 1;
  }

  std::cout << "Gravity constraint: " << std::endl << gravity_constraint << std::endl;

  A_dense.block(num_state_vars_+num_dyn_constraints_,num_state_vars_,N_,num_control_vars_) = gravity_constraint;

  // Normal force constraint
  Eigen::MatrixXd normal_constraint = Eigen::MatrixXd::Zero(4*N_,num_control_vars_);
  Eigen::VectorXd normal_lo(4*N_);
  Eigen::VectorXd normal_hi(4*N_);

  for (int i = 0; i < N_; ++i) {
    for (int j = 0; j < 4; ++j) {
      normal_lo(4*i+j) = -INF_;
      normal_hi(4*i+j) = INF_;
      normal_constraint(4*i+j,13*i + 3*j+2) = 1;
    }
  }

  normal_hi(11) = 50;
  normal_lo(10) = -50;

  std::cout << "Normal constraint: " << std::endl << normal_constraint << std::endl;
  
  A_dense.block(num_state_vars_+num_dyn_constraints_+N_,num_state_vars_,4*N_,num_control_vars_) = normal_constraint;
  */

  // Add contact matrix to linear constraint matrix
  //A_dense.block(num_state_vars_+num_dyn_constraints_,0,num_contact_constraints_,num_decision_vars_) = A_con_dense_;

  // Convert hessian and linear constraint matrix to sparse (wanted by OSQP solver)
  Eigen::SparseMatrix<double> H = H_.sparseView();
  Eigen::SparseMatrix<double> A = A_dense.sparseView();

  std::cout << A << std::endl;

  #ifdef PRINT_DEBUG 
    std::cout << "Number of decision variables: " << num_decision_vars_ << std::endl;
    std::cout << "Number of total constraints: " << num_constraints_ << std::endl;
    std::cout << "Number of dynamics constraints: " << num_dyn_constraints_ << std::endl;
    std::cout << "Number of contact constraints: " << num_contact_constraints_ << std::endl;
    std::cout << "Nu: " << nu_ << std::endl;
    std::cout << "Nx: " << nx_ << std::endl;
  #endif
  
  // Setup lower and  upper bounds for linear constraints

  std::cout << initial_state.size() << ", " << b_state_lo_.size() << ", " << b_control_hi_.size() << " = " << num_decision_vars_ << std::endl;

  Eigen::VectorXd l(num_constraints_);
  l << initial_state,b_state_lo_,b_control_lo_,b_dyn_;//gravity_lo,normal_lo;//,b_contact_lo_;//

  Eigen::VectorXd u(num_constraints_);
  u << initial_state,b_state_hi_,b_control_hi_,b_dyn_;//gravity_hi,normal_hi;//,b_contact_hi_; //

  // Init solver if not already initialized
  if (!solver_.isInitialized()) {
    solver_.data()->setNumberOfVariables(num_decision_vars_);
    solver_.data()->setNumberOfConstraints(num_constraints_);
    solver_.data()->setHessianMatrix(H);
    solver_.data()->setGradient(f_);
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
    solver_.updateGradient(f_);
  }

  // Call solver
  solver_.solve();
  x_out = solver_.getSolution();

  Eigen::MatrixXd p(num_constraints_,3);
  p.col(0) = l;
  p.col(1) = A_dense*x_out;
  p.col(2) = u;

  std::cout << "Constraint vals: " << std::endl << p << std::endl;

  #ifdef PRINT_TIMING
    steady_clock::time_point t2 = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "LinearMPC::solve completed in "
              << time_span.count()*1000.0 << " milliseconds"
              << std::endl;
  #endif
}
