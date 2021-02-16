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
  num_contact_constraints_ = 17*N; // num legs * num constraints per leg * num tsteps
  num_state_vars_= (N_ + 1) * Nx;
  num_control_vars_= N * Nu;
  num_decision_vars_ = num_state_vars_ + num_control_vars_;
  num_constraints_ = num_dyn_constraints_ + num_contact_constraints_ + num_state_vars_;

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

  Eigen::MatrixXd C_toe(4,3);
  Eigen::VectorXd lo_contact(4);
  Eigen::VectorXd hi_contact(4);

  C_toe << 1, 0, -mu,
       1, 0, mu,
       0, 1, -mu,
       0, 1, mu;

  Eigen::MatrixXd C_body = Eigen::MatrixXd::Zero(17,13);
  C_body.block(0,0,4,3) = C_toe;
  C_body.block(4,3,4,3) = C_toe;
  C_body.block(8,6,4,3) = C_toe;
  C_body.block(12,9,4,3) = C_toe;
  C_body(16,12) = 1;

  //std::cout << "C_body: " << std::endl << C_body << std::endl;

  lo_contact << -INF_,0,-INF_,0;
  hi_contact << 0,INF_,0,INF_;


  b_contact_lo_.resize(num_contact_constraints_);
  b_contact_lo_.setZero();

  b_contact_hi_.resize(num_contact_constraints_);
  b_contact_hi_.setZero();

  for (int i = 0; i < N_; ++i) { // iterate over horizon
    A_con_dense_.block(17*i,num_state_vars_+13*i,17,13) = C_body;
    for (int j = 0; j < 4; ++j) { // iterate over legs
      int row_start = 17*i + 4*j;
      if (contact_sequence.at(i).at(j)) { // ground contact
         b_contact_lo_.segment(row_start,4) = lo_contact;
         b_contact_hi_.segment(row_start,4) = hi_contact;
      }
      else { 
        // do nothing, bounds have been zeroed out earlier in this function
      }
    }
    b_contact_lo_(17*i+16) = 1;
    b_contact_hi_(17*i+16) = 1; 
  }
  //std::cout << "Friction vector lo: " << std::endl << b_contact_lo_ << std::endl;
  //std::cout << "Friction vector hi: " << std::endl << b_contact_hi_ << std::endl;
  //std::cout << "Friction cone matrix: " << std::endl << A_con_dense_.rightCols(num_control_vars_) << std::endl;

}

void LinearMPC::update_state_bounds(const Eigen::VectorXd state_lo,
                                    const Eigen::VectorXd state_hi) {
  b_state_lo_ = state_lo.replicate(N_,1);
  b_state_hi_ = state_hi.replicate(N_,1);
}

void LinearMPC::get_cost_function(const Eigen::MatrixXd &ref_traj,
                                  Eigen::VectorXd &f) {

  // Construct fx vector
  Eigen::MatrixXd y(1,num_state_vars_);
  y.block(0, 0, 1, num_state_vars_) =
      math::reshape(ref_traj, 1, ref_traj.cols() * ref_traj.rows());


  std::cout << "Y size: " << y.rows() << ", " << y.cols() << std::endl;
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
                      Eigen::MatrixXd &opt_traj) {


  //std::cout << "Friction constraint vals: " << std::endl << A_con_dense_*x_out << std::endl;

  // Resize and wipe output containers
  first_control.resize(nu_);
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

  // Add contact matrix to linear constraint matrix
  A_dense.topRows(num_contact_constraints_) = A_con_dense_;

  // Add identity matrix to linear constraint matrix for state and control bounds
  A_dense.middleRows(num_contact_constraints_,num_state_vars_) = Eigen::MatrixXd::Identity(num_state_vars_,num_decision_vars_);
  
  // Add dynamics matrix to linear constraint matrix
  A_dense.bottomRows(num_dyn_constraints_) = A_dyn_dense_;

  // Convert hessian and linear constraint matrix to sparse (wanted by OSQP solver)
  Eigen::SparseMatrix<double> H = H_.sparseView();
  Eigen::SparseMatrix<double> A = A_dense.sparseView();

  #ifdef PRINT_DEBUG 
    std::cout << "Number of decision variables: " << num_decision_vars_ << std::endl;
    std::cout << "Number of total constraints: " << num_constraints_ << std::endl;
    std::cout << "Number of dynamics constraints: " << num_dyn_constraints_ << std::endl;
    std::cout << "Number of contact constraints: " << num_contact_constraints_ << std::endl;
    std::cout << "Nu: " << nu_ << std::endl;
    std::cout << "Nx: " << nx_ << std::endl;
  #endif
  
  // Setup lower and  upper bounds for linear constraints
  Eigen::VectorXd l(num_constraints_);
  l.setZero();
  l << b_contact_lo_,initial_state,b_state_lo_,b_dyn_;

  Eigen::VectorXd u(num_constraints_);
  u.setZero();
  u << b_contact_hi_,initial_state,b_state_hi_,b_dyn_; 

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
