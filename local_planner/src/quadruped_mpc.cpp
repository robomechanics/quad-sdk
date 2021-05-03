#include "local_planner/quadruped_mpc.h"
#include "spirit_utils/matrix_utils.h"

#include <iostream>
#include <chrono>

// Comment to remove OSQP printing
//#define PRINT_DEBUG 

// Comment to remove timing prints
//#define PRINT_TIMING

using namespace std::chrono;

QuadrupedMPC::QuadrupedMPC() {

  int N = N_;
  int Nx = nx_;
  int Nu = nu_;
  
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

  kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();
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

  // Full friciton cone for one tstep
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

void QuadrupedMPC::update_dynamics_hip_projected_feet(const Eigen::MatrixXd &ref_traj) {
  Eigen::MatrixXd foot_positions = Eigen::MatrixXd::Zero(N_,12);

  Eigen::Vector3d nominal_joint_state;
  nominal_joint_state << 0, 0.78, 1.57; // Default stand angles

  for (int i = 0; i < N_; ++i) {
    for (int j = 0; j < 4; ++j) {
    Eigen::Vector3d toe_body_pos;
    this->kinematics_->bodyToFootFK(j, nominal_joint_state, toe_body_pos);
    foot_positions.block(i,j*3,1,3) = toe_body_pos;
    }
  }

  this->update_dynamics(ref_traj, foot_positions);
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

  // Reset dynamics matrix (not strictly necessary but good practice)
  A_dyn_dense_ = Eigen::MatrixXd::Zero(num_dyn_constraints_,num_decision_vars_);

  // Placeholder matrices for dynamics
  Eigen::MatrixXd Adi(nx_,nx_);
  Eigen::MatrixXd Bdi(nx_,nu_);
  
  
  Eigen::VectorXd yaws = ref_traj.row(5);
  for (int i = 0; i < N_; ++i) {
    Eigen::AngleAxisd yaw_aa(yaws(i),Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d Rz = yaw_aa.toRotationMatrix();
    Eigen::Matrix3d Rzt = Rz.transpose();
    Eigen::Matrix3d Iw = Rz*Ib_*Rzt;
    Eigen::Matrix3d Iw_inv = Iw.inverse();

    // Reset our temporary dynamics matrices
    Adi = Ad;
    Bdi = Bd;

    // Update our linearized angular velocity integration
    Adi.block(3,9,3,3) = Rzt*dt_;
    //std::cout << "Rzt: " << std::endl << Rzt << std::endl;

    // Update our foot positions
    for (int j = 0; j < 4; ++j) {
      Eigen::Matrix3d foot_pos_hat;
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

void QuadrupedMPC::update_control_bounds(const double f_min, const double f_max) {
  f_min_ = f_min;
  f_max_ = f_max;
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

bool QuadrupedMPC::solve(const Eigen::VectorXd &initial_state,
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

    solver_.data()->clearLinearConstraintsMatrix();
    solver_.data()->setLinearConstraintsMatrix(A_sparse);
    if (updated_weights_) {
      Eigen::SparseMatrix<double> hessian_sparse = H_.sparseView();
      solver_.updateHessianMatrix(hessian_sparse);
      updated_weights_ = false;
    }

    solver_.updateBounds(lower_bound,upper_bound);
    solver_.updateGradient(f_);
  }

  // Call solver
  bool good_solve = solver_.solve();
  x_out = solver_.getSolution();

  return good_solve;
  #ifdef PRINT_TIMING
    steady_clock::time_point t2 = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "QuadrupedMPC::solve completed in "
              << time_span.count()*1000.0 << " milliseconds"
              << std::endl;
  #endif
}


bool QuadrupedMPC::computePlan(const Eigen::VectorXd &initial_state, 
  const Eigen::MatrixXd &ref_traj, const Eigen::MatrixXd &foot_positions,
  const std::vector<std::vector<bool>> &contact_schedule,
  Eigen::MatrixXd &state_traj, Eigen::MatrixXd &control_traj){

  // Pass inputs into solver and solve
  update_contact(contact_schedule, f_min_, f_max_);
  update_dynamics(ref_traj,foot_positions);

  // Perform the solve
  Eigen::MatrixXd x_out;
  if (!solve(initial_state, ref_traj, x_out)) {
    // std::cout << "Failed solve: " << std::endl;
    // std::cout << "Current state: " << std::endl << initial_state.format(CleanFmt) << std::endl;
    // std::cout << "Reference trajectory: " << std::endl << ref_traj.format(CleanFmt) << std::endl;
    // std::cout << "Foot Placements in body frame: " << std::endl << foot_positions.format(CleanFmt) << std::endl;
    ROS_WARN_THROTTLE(0.1, "Failed OSQP solve!");
    // throw std::runtime_error("Failed OSQP solve!");
  }

  // Get output, remove gravity control
  double f_val;
  get_output(x_out, state_traj, control_traj, f_val);
  control_traj = control_traj.block(0,0,control_traj.rows(), control_traj.cols()-1);
}


// #include "local_planner/quadruped_mpc.h"
// #include "spirit_utils/matrix_utils.h"

// #include <iostream>
// #include <chrono>

// Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

// // Comment to remove OSQP printing
// //#define PRINT_DEBUG 

// // Comment to remove timing prints
// //#define PRINT_TIMING

// using namespace std::chrono;

// QuadrupedMPC::QuadrupedMPC() {
  
//   int N = N_;
//   int Nx = nx_;
//   int Nu = nu_;

//   num_dyn_constraints_ = N * Nx;
//   num_constraints_per_leg_ = 5;
//   num_contact_constraints_per_step_ = (num_constraints_per_leg_*4 + 1);
//   num_contact_constraints_ = num_contact_constraints_per_step_*N;
//   num_state_vars_= (N + 1) * Nx;
//   num_control_vars_= N * Nu;
//   num_decision_vars_ = num_state_vars_ + num_control_vars_;
//   num_constraints_ = num_state_vars_ + num_dyn_constraints_ + num_contact_constraints_;// + num_control_vars_;

//   // Initialize vectors
//   b_contact_lo_ = Eigen::VectorXd::Zero(num_contact_constraints_);
//   b_contact_hi_ = Eigen::VectorXd::Zero(num_contact_constraints_);
//   b_dyn_ = Eigen::VectorXd::Zero(num_dyn_constraints_);

//   kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();
// }

// void QuadrupedMPC::setMassProperties(const double m, const Eigen::Matrix3d Ib) {
//   m_ = m;
//   Ib_ = Ib;
//   mass_properties_set_ = true;
// }

// void QuadrupedMPC::setTimestep(const double dt) {
//   dt_ = dt;
//   dt_set_ = true;
// }

// void QuadrupedMPC::update_friction(const double mu) {
//   assert(0 <= mu && mu <= 1);
//   mu_ = mu;
  
//   A_con_dense_ = Eigen::MatrixXd::Zero(num_contact_constraints_, num_control_vars_);

//   // Friction cone for one leg
//   Eigen::MatrixXd C_leg(num_constraints_per_leg_,3);
//   C_leg << 1, 0, -mu,
//        1, 0, mu,
//        0, 1, -mu,
//        0, 1, mu,
//        0, 0, 1;

//   // Full friciton cone for one tstep
//   Eigen::MatrixXd C_step = Eigen::MatrixXd::Zero(num_contact_constraints_per_step_,nu_);
//   for (int i = 0; i < 4; ++i) {
//     C_step.block(num_constraints_per_leg_*i,3*i,num_constraints_per_leg_,3) = C_leg;
//   }
//   C_step(num_contact_constraints_per_step_-1,nu_-1) = 1;

//   for (int i = 0; i < N_; ++i) { // iterate over horizon
//     A_con_dense_.block(num_contact_constraints_per_step_*i,nu_*i,num_contact_constraints_per_step_,nu_) = C_step;
//   }
// }

// void QuadrupedMPC::update_weights(const std::vector<Eigen::MatrixXd> &Q, 
//                                const std::vector<Eigen::MatrixXd> &R) {

//   Eigen::MatrixXd Hq(num_state_vars_, num_state_vars_);
//   Hq.setZero();
//   for (int i = 0; i < N_+1; ++i) {
//     Hq.block(i * nx_, i * nx_, nx_, nx_) = Q.at(i);
//   }

//   Eigen::MatrixXd Hu(num_control_vars_, num_control_vars_);
//   Hu.setZero();
//   for (int i = 0; i < N_; ++i) {
//     Hu.block(i*nu_,i*nu_,nu_,nu_) = R.at(i);
//   }
//   H_ = math::block_diag(Hq, Hu);
//   H_f_ = Hq;

//   updated_weights_ = true;
// }

// void QuadrupedMPC::update_dynamics_hip_projected_feet(const Eigen::MatrixXd &ref_traj) {
//   Eigen::MatrixXd foot_positions = Eigen::MatrixXd::Zero(N_,12);

//   Eigen::Vector3d nominal_joint_state;
//   nominal_joint_state << 0, 0.78, 1.57; // Default stand angles

//   for (int i = 0; i < N_; ++i) {
//     for (int j = 0; j < 4; ++j) {
//     Eigen::Vector3d toe_body_pos;
//     this->kinematics_->bodyToFootFK(j, nominal_joint_state, toe_body_pos);
//     foot_positions.block(i,j*3,1,3) = toe_body_pos;
//     }
//   }

//   this->update_dynamics(ref_traj, foot_positions);
// }

// void QuadrupedMPC::update_dynamics(const Eigen::MatrixXd &ref_traj,
//                                 const Eigen::MatrixXd &foot_positions) { // N x 12
//   assert(dt_set_);
//   assert(mass_properties_set_);
//   assert(foot_positions.rows() == N_);
//   assert(foot_positions.cols() == 12);

//   // Create fixed components of dynamics matrices
//   Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(nx_,nx_);
//   Ad.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);
//   Ad.block(6,6,6,6) = Eigen::MatrixXd::Identity(6,6);
//   Ad.block(0,6,3,3) = Eigen::MatrixXd::Identity(3,3)*dt_;

//   Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(nx_,nu_);
//   for (int i = 0; i < 4; ++i) {
//     Bd.block(6,3*i,3,3) = Eigen::MatrixXd::Identity(3,3)/m_*dt_;
//   }
//   Bd(8,12) = -g_*dt_; // gravity acts downwards here

//   // Reset dynamics matrix (not strictly necessary but good practice)
//   A_dyn_dense_ = Eigen::MatrixXd::Zero(num_dyn_constraints_,num_decision_vars_);

//   // Placeholder matrices for dynamics
//   Eigen::MatrixXd Adi(nx_,nx_);
//   Eigen::MatrixXd Bdi(nx_,nu_);
  
  
//   Eigen::VectorXd yaws = ref_traj.row(5);
//   for (int i = 0; i < N_; ++i) {
//     Eigen::AngleAxisd yaw_aa(yaws(i),Eigen::Vector3d::UnitZ());
//     Eigen::Matrix3d Rz = yaw_aa.toRotationMatrix();
//     Eigen::Matrix3d Rzt = Rz.transpose();
//     Eigen::Matrix3d Iw = Rz*Ib_*Rzt;
//     Eigen::Matrix3d Iw_inv = Iw.inverse();

//     // Reset our temporary dynamics matrices
//     Adi = Ad;
//     Bdi = Bd;

//     // Update our linearized angular velocity integration
//     Adi.block(3,9,3,3) = Rzt*dt_;
//     //std::cout << "Rzt: " << std::endl << Rzt << std::endl;

//     // Update our foot positions
//     for (int j = 0; j < 4; ++j) {
//       Eigen::Matrix3d foot_pos_hat;
//       foot_pos_hat << 0, -foot_positions(i,3*j+2),foot_positions(i,3*j+1),
//                       foot_positions(i,3*j+2), 0, -foot_positions(i,3*j+0),
//                       -foot_positions(i,3*j+1), foot_positions(i,3*j+0), 0;
//       Bdi.block(9,3*j,3,3) = Iw_inv * foot_pos_hat * dt_;//Eigen::Matrix3d::Zero();
//     }

//     A_dyn_dense_.block(nx_*i,nx_*i,nx_,nx_) = Adi;
//     A_dyn_dense_.block(nx_*i,nx_*(i+1),nx_,nx_) = -Eigen::MatrixXd::Identity(nx_,nx_);
//     A_dyn_dense_.block(nx_*i,num_state_vars_+nu_*i,nx_,nu_) = Bdi;
//   }
// }

// void QuadrupedMPC::update_contact(const std::vector<std::vector<bool>> contact_schedule,
//                                const double fmin,
//                                const double fmax) {
//   assert(contact_schedule.size() == N_);
//   assert(contact_schedule.front().size() == 4);

//   Eigen::VectorXd lo_contact(num_constraints_per_leg_);
//   lo_contact << -2*mu_*fmax,0,-2*mu_*fmax,0,fmin;
//   Eigen::VectorXd hi_contact(num_constraints_per_leg_);
//   hi_contact << 0,2*mu_*fmax,0,2*mu_*fmax,fmax;

//   b_contact_lo_.setZero();
//   b_contact_hi_.setZero();

//   for (int i = 0; i < N_; ++i) { // iterate over horizon
//     for (int j = 0; j < 4; ++j) { // iterate over legs
//       int row_start = num_contact_constraints_per_step_*i + num_constraints_per_leg_*j;
//       if (contact_schedule.at(i).at(j)) { // ground contact
//          b_contact_lo_.segment(row_start,num_constraints_per_leg_) = lo_contact;
//          b_contact_hi_.segment(row_start,num_constraints_per_leg_) = hi_contact;
//       }
//       else { 
//         // do nothing, bounds have been zeroed out earlier in this function
//       }
//     }
//     b_contact_lo_(num_contact_constraints_per_step_*i+num_contact_constraints_per_step_-1) = 1.0;
//     b_contact_hi_(num_contact_constraints_per_step_*i+num_contact_constraints_per_step_-1) = 1.0; 
//   }
// }

// void QuadrupedMPC::update_state_bounds(const Eigen::VectorXd state_lo,
//                                     const Eigen::VectorXd state_hi) {
//   b_state_lo_ = state_lo.replicate(N_,1);
//   b_state_hi_ = state_hi.replicate(N_,1);
// }

// void QuadrupedMPC::update_control_bounds(const double f_min, const double f_max) {
//   f_min_ = f_min;
//   f_max_ = f_max;
// }

// void QuadrupedMPC::get_cost_function(const Eigen::MatrixXd &ref_traj,
//                                   Eigen::VectorXd &f) {

//   // Construct fx vector
//   Eigen::MatrixXd y(1,num_state_vars_);
//   y.block(0, 0, 1, num_state_vars_) =
//       math::reshape(ref_traj, 1, ref_traj.cols() * ref_traj.rows());

//   //std::cout << "Y size: " << y.rows() << ", " << y.cols() << std::endl;
//   Eigen::MatrixXd fx = -y * H_f_;

//   // Construct fu vector
//   Eigen::MatrixXd fu = Eigen::MatrixXd::Zero(N_ * nu_, 1);

//   // Construct f vector
//   f.resize(num_decision_vars_);
//   f.setZero();
//   f << fx.transpose(), fu;
// }

// //========================================================================================
// void QuadrupedMPC::get_output(const Eigen::MatrixXd &x_out,
//                       Eigen::MatrixXd &opt_traj,
//                       Eigen::MatrixXd &control_traj,
//                       double &f_val) {

//   // Resize and wipe output containers
//   opt_traj.resize(N_+1,nx_);
//   opt_traj.setZero();
//   control_traj.resize(N_,nu_);
//   control_traj.setZero();

//   // Collect optimized control trajectory
//   for (size_t i = 0; i < N_; ++i) {
//     for (size_t j = 0; j < nu_; ++j) {
//       control_traj(i,j) = x_out(num_state_vars_ + i*nu_ + j,0);
//     }
//   }

//   // Collect optimized state trajectory
//   for (size_t i = 0; i < N_ + 1; ++i)
//   {
//     for (size_t j = 0; j < nx_; ++j)
//     {
//       opt_traj(i,j) = x_out(i*nx_+j,0);
//     }
//   }

//   // Get final cost
//   f_val = (0.5*x_out.transpose()*H_*x_out)(0,0) + (f_.transpose() * x_out)(0,0);
// }

// bool QuadrupedMPC::solve(const Eigen::VectorXd &initial_state,
//                          const Eigen::MatrixXd &ref_traj,
//                          Eigen::MatrixXd &x_out) {
//   #ifdef PRINT_TIMING
//     steady_clock::time_point t1 = steady_clock::now();
//   #endif

//   // Update linear component of cost function to reflect new reference traj
//   this->get_cost_function(ref_traj, f_);

//   Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints_,num_decision_vars_);

//   A.block(0,0,num_dyn_constraints_,num_decision_vars_) = A_dyn_dense_;
//   A.block(num_dyn_constraints_,0,num_state_vars_,num_state_vars_) = Eigen::MatrixXd::Identity(num_state_vars_,num_state_vars_);
//   A.block(num_dyn_constraints_+num_state_vars_,num_state_vars_,num_contact_constraints_,num_control_vars_) = A_con_dense_;

//   Eigen::VectorXd lower_bound(num_constraints_);
//   lower_bound << b_dyn_, initial_state, b_state_lo_, b_contact_lo_;

//   Eigen::VectorXd upper_bound(num_constraints_);
//   upper_bound << b_dyn_, initial_state, b_state_hi_, b_contact_hi_;

//   // Setup OsqpEigen solver
//   Eigen::SparseMatrix<double> A_sparse = A.sparseView();

//   // Init solver if not already initialized
//   if (!solver_.isInitialized()) {
//     Eigen::SparseMatrix<double> hessian_sparse = H_.sparseView();
//     solver_.data()->setNumberOfVariables(num_decision_vars_);
//     solver_.data()->setNumberOfConstraints(num_constraints_);
//     solver_.data()->setHessianMatrix(hessian_sparse);
//     solver_.data()->setGradient(f_);
//     solver_.data()->setLinearConstraintsMatrix(A_sparse);
//     solver_.data()->setLowerBound(lower_bound);
//     solver_.data()->setUpperBound(upper_bound);

//     #ifdef PRINT_DEBUG
//       solver_.settings()->setVerbosity(true);
//     #else
//       solver_.settings()->setVerbosity(false);
//     #endif
//     solver_.settings()->setWarmStart(true);
//     solver_.settings()->setCheckTermination(10);
//     solver_.settings()->setScaling(false);
//     solver_.initSolver();

//   }
//   else { // Update components of QP that change from iter to iter

//     solver_.data()->clearLinearConstraintsMatrix();
//     solver_.data()->setLinearConstraintsMatrix(A_sparse);
//     if (updated_weights_) {
//       Eigen::SparseMatrix<double> hessian_sparse = H_.sparseView();
//       solver_.updateHessianMatrix(hessian_sparse);
//       updated_weights_ = false;
//     }

//     solver_.updateBounds(lower_bound,upper_bound);
//     solver_.updateGradient(f_);
//   }

//   // Call solver
//   bool good_solve = solver_.solve();
//   x_out = solver_.getSolution();

//   return good_solve;
//   #ifdef PRINT_TIMING
//     steady_clock::time_point t2 = steady_clock::now();
//     duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
//     std::cout << "QuadrupedMPC::solve completed in "
//               << time_span.count()*1000.0 << " milliseconds"
//               << std::endl;
//   #endif
// }

// bool QuadrupedMPC::computePlan(const Eigen::VectorXd &initial_state, 
//   const Eigen::MatrixXd &ref_traj, const Eigen::MatrixXd &foot_positions,
//   const std::vector<std::vector<bool>> &contact_schedule,
//   Eigen::MatrixXd &state_traj, Eigen::MatrixXd &control_traj){

//   // Pass inputs into solver and solve
//   update_contact(contact_schedule, f_min_, f_max_);
//   update_dynamics(ref_traj,foot_positions);

//   // Perform the solve
//   Eigen::MatrixXd x_out;
//   if (!solve(initial_state, ref_traj, x_out)) {
//     // std::cout << "Failed solve: " << std::endl;
//     // std::cout << "Current state: " << std::endl << initial_state.format(CleanFmt) << std::endl;
//     // std::cout << "Reference trajectory: " << std::endl << ref_traj.format(CleanFmt) << std::endl;
//     // std::cout << "Foot Placements in body frame: " << std::endl << foot_positions.format(CleanFmt) << std::endl;
//     ROS_WARN_THROTTLE(0.1, "Failed OSQP solve!");
//     // throw std::runtime_error("Failed OSQP solve!");
//   }

//   // Get output, remove gravity control
//   double f_val;
//   get_output(x_out, state_traj, control_traj, f_val);
//   control_traj = control_traj.block(0,0,control_traj.rows(), control_traj.cols()-1);
// }