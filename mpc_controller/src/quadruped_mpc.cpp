#include "mpc_controller/quadruped_mpc.h"
#include "spirit_utils/matrix_utils.h"

#include <iostream>
#include <chrono>

// Comment to remove OSQP printing
//#define PRINT_DEBUG 

// Comment to remove timing prints
//#define PRINT_TIMING

using namespace std::chrono;

QuadrupedMPC::QuadrupedMPC(const int N, const int Nx, const int Nu)
  : N_(N), nx_(Nx), nu_(Nu) {
  
  num_dyn_constraints_ = N * Nx;
  num_constraints_per_leg_ = 5;
  num_contact_constraints_per_step_ = (num_constraints_per_leg_*4 + 1);
  num_contact_constraints_ = num_contact_constraints_per_step_*N;
  num_state_vars_= (N + 1) * Nx;
  num_control_vars_= N * Nu;
  num_decision_vars_ = num_state_vars_ + num_control_vars_;
  num_constraints_ = num_state_vars_ + num_dyn_constraints_ + num_contact_constraints_;// + num_control_vars_;

  // Initialize vectors
  b_contact_lo_ = Eigen::VectorXd::Zero(num_contact_constraints_);
  b_contact_hi_ = Eigen::VectorXd::Zero(num_contact_constraints_);
  b_dyn_ = Eigen::VectorXd::Zero(num_dyn_constraints_); 
}

void QuadrupedMPC::setMassProperties(const double m, const Eigen::Matrix3d Ib) {
  m_ = m;
  Ib_ = Ib;
  mass_properties_set_ = true;
}

void QuadrupedMPC::setTimestep(const double dt) {
  dt_ = dt;
  dt_set_ = true;
}

void QuadrupedMPC::update_friction(const double mu) {
  assert(0 <= mu && mu <= 1);
  mu_ = mu;
  
  A_con_dense_ = Eigen::MatrixXd::Zero(num_contact_constraints_, num_control_vars_);

  // Friction cone for one leg
  Eigen::MatrixXd C_leg(num_constraints_per_leg_,3);
  C_leg << 1, 0, -mu,
       1, 0, mu,
       0, 1, -mu,
       0, 1, mu,
       0, 0, 1;

  // Full friciton cone for one tsteo
  Eigen::MatrixXd C_step = Eigen::MatrixXd::Zero(num_contact_constraints_per_step_,nu_);
  for (int i = 0; i < 4; ++i) {
    C_step.block(num_constraints_per_leg_*i,3*i,num_constraints_per_leg_,3) = C_leg;
  }
  C_step(num_contact_constraints_per_step_-1,nu_-1) = 1;

  for (int i = 0; i < N_; ++i) { // iterate over horizon
    A_con_dense_.block(num_contact_constraints_per_step_*i,nu_*i,num_contact_constraints_per_step_,nu_) = C_step;
  }
}

void QuadrupedMPC::update_weights(const std::vector<Eigen::MatrixXd> &Q, 
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

void QuadrupedMPC::update_dynamics(const Eigen::MatrixXd &ref_traj,
                                const Eigen::MatrixXd &foot_positions) { // N x 12
  assert(dt_set_);
  assert(mass_properties_set_);
  assert(foot_positions.rows() == N_);
  assert(foot_positions.cols() == 12);

  // Create fixed components of dynamics matrices
  Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(nx_,nx_);
  Ad.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);
  Ad.block(6,6,6,6) = Eigen::MatrixXd::Identity(6,6);
  Ad.block(0,6,3,3) = Eigen::MatrixXd::Identity(3,3)*dt_;

  Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(nx_,nu_);
  for (int i = 0; i < 4; ++i) {
    Bd.block(6,3*i,3,3) = Eigen::MatrixXd::Identity(3,3)/m_*dt_;
  }
  Bd(8,12) = -g_*dt_; // gravity acts downwards here

  //std::cout << "Ad: \n" << Ad << std::endl;
  //std::cout << "Bd: \n" << Bd << std::endl;

  // Reset dynamics matrix (not strictly necessary but good practice)
  A_dyn_dense_ = Eigen::MatrixXd::Zero(num_dyn_constraints_,num_decision_vars_);

  // Placeholder matrices for dynamics
  Eigen::MatrixXd Adi(nx_,nx_);
  Eigen::MatrixXd Bdi(nx_,nu_);
  Eigen::Matrix3d foot_pos_hat;

  for (int i = 0; i < N_; ++i) {
    double yaw_ref = (ref_traj(5,i) + ref_traj(5,i+1))/2.0;
    Eigen::AngleAxisd yaw_aa(yaw_ref,Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d Rz = yaw_aa.toRotationMatrix();
    Eigen::Matrix3d Rzt = Rz.transpose();
    Eigen::Matrix3d Iw = Rz*Ib_*Rzt;
    Eigen::Matrix3d Iw_inv = Iw.inverse();

    // Reset our temporary dynamics matrices
    Adi = Ad;
    Bdi = Bd;

    // Update our linearized angular velocity integration
    Adi.block(3,9,3,3) = Rzt*dt_;

    // Update our foot positions
    for (int j = 0; j < 4; ++j) {
      foot_pos_hat << 0, -foot_positions(i,3*j+2),foot_positions(i,3*j+1),
                      foot_positions(i,3*j+2), 0, -foot_positions(i,3*j+0),
                      -foot_positions(i,3*j+1), foot_positions(i,3*j+0), 0;
      Bdi.block(9,3*j,3,3) = Iw_inv * foot_pos_hat * dt_;//Eigen::Matrix3d::Zero();
    }

    A_dyn_dense_.block(nx_*i,nx_*i,nx_,nx_) = Adi;
    A_dyn_dense_.block(nx_*i,nx_*(i+1),nx_,nx_) = -Eigen::MatrixXd::Identity(nx_,nx_);
    A_dyn_dense_.block(nx_*i,num_state_vars_+nu_*i,nx_,nu_) = Bdi;
  }
}

void QuadrupedMPC::update_contact(const std::vector<std::vector<bool>> contact_sequence,
                               const double fmin,
                               const double fmax) {
  assert(contact_sequence.size() == N_);
  assert(contact_sequence.front().size() == 4);

  Eigen::VectorXd lo_contact(num_constraints_per_leg_);
  lo_contact << -2*mu_*fmax,0,-2*mu_*fmax,0,fmin;
  Eigen::VectorXd hi_contact(num_constraints_per_leg_);
  hi_contact << 0,2*mu_*fmax,0,2*mu_*fmax,fmax;

  b_contact_lo_.setZero();
  b_contact_hi_.setZero();

  for (int i = 0; i < N_; ++i) { // iterate over horizon
    for (int j = 0; j < 4; ++j) { // iterate over legs
      int row_start = num_contact_constraints_per_step_*i + num_constraints_per_leg_*j;
      if (contact_sequence.at(i).at(j)) { // ground contact
         b_contact_lo_.segment(row_start,num_constraints_per_leg_) = lo_contact;
         b_contact_hi_.segment(row_start,num_constraints_per_leg_) = hi_contact;
      }
      else { 
        // do nothing, bounds have been zeroed out earlier in this function
      }
    }
    b_contact_lo_(num_contact_constraints_per_step_*i+num_contact_constraints_per_step_-1) = 1.0;
    b_contact_hi_(num_contact_constraints_per_step_*i+num_contact_constraints_per_step_-1) = 1.0; 
  }
}

void QuadrupedMPC::update_state_bounds(const Eigen::VectorXd state_lo,
                                    const Eigen::VectorXd state_hi) {
  b_state_lo_ = state_lo.replicate(N_,1);
  b_state_hi_ = state_hi.replicate(N_,1);
}

void QuadrupedMPC::get_cost_function(const Eigen::MatrixXd &ref_traj,
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
void QuadrupedMPC::get_output(const Eigen::MatrixXd &x_out,
                      Eigen::MatrixXd &opt_traj,
                      Eigen::MatrixXd &control_traj,
                      double &f_val) {

  // Resize and wipe output containers
  opt_traj.resize(nx_,N_+1);
  opt_traj.setZero();
  control_traj.resize(nu_, N_);
  control_traj.setZero();

  // Collect optimized control trajectory
  for (size_t i = 0; i < N_; ++i) {
    for (size_t j = 0; j < nu_; ++j) {
      control_traj(j,i) = x_out(num_state_vars_ + i*nu_ + j,0);
    }
  }

  // Collect optimized state trajectory
  for (size_t i = 0; i < N_ + 1; ++i)
  {
    for (size_t j = 0; j < nx_; ++j)
    {
      opt_traj(j,i) = x_out(i*nx_+j,0);
    }
  }

  // Get final cost
  f_val = (0.5*x_out.transpose()*H_*x_out)(0,0) + (f_.transpose() * x_out)(0,0);
}

void QuadrupedMPC::solve(const Eigen::VectorXd &initial_state,
                         const Eigen::MatrixXd &ref_traj,
                         Eigen::MatrixXd &x_out) {
  #ifdef PRINT_TIMING
    steady_clock::time_point t1 = steady_clock::now();
  #endif

  // Update linear component of cost function to reflect new reference traj
  this->get_cost_function(ref_traj, f_);

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints_,num_decision_vars_);

  A.block(0,0,num_dyn_constraints_,num_decision_vars_) = A_dyn_dense_;
  A.block(num_dyn_constraints_,0,num_state_vars_,num_state_vars_) = Eigen::MatrixXd::Identity(num_state_vars_,num_state_vars_);
  A.block(num_dyn_constraints_+num_state_vars_,num_state_vars_,num_contact_constraints_,num_control_vars_) = A_con_dense_;

  Eigen::VectorXd lower_bound(num_constraints_);
  lower_bound << b_dyn_, initial_state, b_state_lo_, b_contact_lo_;

  Eigen::VectorXd upper_bound(num_constraints_);
  upper_bound << b_dyn_, initial_state, b_state_hi_, b_contact_hi_;

  // Setup OsqpEigen solver
  Eigen::SparseMatrix<double> A_sparse = A.sparseView();

  // Init solver if not already initialized
  if (!solver_.isInitialized()) {
    Eigen::SparseMatrix<double> hessian_sparse = H_.sparseView();
    solver_.data()->setNumberOfVariables(num_decision_vars_);
    solver_.data()->setNumberOfConstraints(num_constraints_);
    solver_.data()->setHessianMatrix(hessian_sparse);
    solver_.data()->setGradient(f_);
    solver_.data()->setLinearConstraintsMatrix(A_sparse);
    solver_.data()->setLowerBound(lower_bound);
    solver_.data()->setUpperBound(upper_bound);

    #ifdef PRINT_DEBUG
      solver_.settings()->setVerbosity(true);
    #else
      solver_.settings()->setVerbosity(false);
    #endif
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setCheckTermination(10);
    solver_.settings()->setScaling(false);
    solver_.initSolver();

  }
  else { // Update components of QP that change from iter to iter

    solver_.updateLinearConstraintsMatrix(A_sparse);
    if (updated_weights_) {
      Eigen::SparseMatrix<double> hessian_sparse = H_.sparseView();
      solver_.updateHessianMatrix(hessian_sparse);
      updated_weights_ = false;
    }

    solver_.updateBounds(lower_bound,upper_bound);
    solver_.updateGradient(f_);
  }

  // Call solver
  solver_.solve();
  x_out = solver_.getSolution();

  #ifdef PRINT_TIMING
    steady_clock::time_point t2 = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "QuadrupedMPC::solve completed in "
              << time_span.count()*1000.0 << " milliseconds"
              << std::endl;
  #endif
}
