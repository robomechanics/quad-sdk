#ifndef MPCPLUSPLUS_H
#define MPCPLUSPLUS_H

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <quad_utils/quad_kd.h>
#include <assert.h>

//! Implements online MPC for quadrupedal MPC
/*!
   MPCController implements a convex QP approach to legged robot control.
   The robot is treated as a floating body in 3D space and our control authority
   is modeled as a series of ground reaction forces at all four toes. 
*/  
class QuadrupedMPC {

public:

  /**
   * @brief Construct an MPC object
   * @param[in] N Number of steps in horizon, not inclusive of current state
   * @param[in] Nx Number of states in one step
   * @param[in] Nu Number of controls in one step
   */
  QuadrupedMPC(const int N, const int Nx, const int Nu);

  ~QuadrupedMPC() = default;

  /**
   * @brief Update the mass properties of the robot
   * @param[in] m Robot mass in kg
   * @param[in] Ib Robot body inertia in kg m^2
   */
  void setMassProperties(const double m, const Eigen::Matrix3d Ib);

  /**
   * @brief Update our MPC timestep
   * @param[in] dt Timestep in seconds
   */
  void setTimestep(const double dt);

  /**
   * @brief Update our MPC weights on state deviation error and control error
   * @param[in] Q vector of matrices of state costs at each step in the horizon
   * @param[in] R vector of matrices of control costs at each step in the horizon
   */
  void update_weights(const std::vector<Eigen::MatrixXd> &Q,
                      const std::vector<Eigen::MatrixXd> &R);

  /**
   * @brief Update our dynamics (linearized about reference yaw and footsteps)
   * @param[in] ref_traj Matrix holding desired reference trajectory (nx x N+1)
   * @param[in] foot_positions Vector foot positions (fx1 fy1 fz1 fx2 ...) at each tstep
   */
  void update_dynamics(const Eigen::MatrixXd &ref_traj,
                       const Eigen::MatrixXd &foot_positions);

  /**
   * @brief Update our dynamics (linearized about reference yaw and footsteps projected underneath hip)
   * @param[in] ref_traj Matrix holding desired reference trajectory (nx x N+1)
   */
  void update_dynamics_hip_projected_feet(const Eigen::MatrixXd &ref_traj);

   /**
   * @brief Update the footstep contact sequence and normal force bounds
   * @param[in] contact_sequence Vector(N) of vectors(4) holding 
                boolean contact status for each foot at each timesteo
   * @param[in] fmin Minimum allowable normal force in contact phase
   * @param[in] fmax Maximum allowable normal force in contact phase
   */
  void update_contact(const std::vector<std::vector<bool> > contact_sequence,
                      const double fmin,
                      const double fmax);

  /**
   * @brief Update hard constraints on state bounds
   * @param[in] state_lo Vector of minimum allowable values for each state
   * @param[in] state_hi Vector of maximum allowable values for each state
   */
  void update_state_bounds(const Eigen::VectorXd state_lo,
                           const Eigen::VectorXd state_hi);

  /**
   * @brief Update our friction coeffiecient
   * @param[in] mu Friction coefficient used in linear friction cone
   */
  void update_friction(const double mu);

  /**
   * @brief Constructs the quadratic cost function of the form
   * 
   * @param[in] ref_traj Reference trajectory
   * @param[out] f Linear component of cost
   */
  void get_cost_function(const Eigen::MatrixXd &ref_traj, Eigen::VectorXd &f);

  /**
   * @brief Collect first control value and all states and return them.
   * @param[in] x_out optimized decision variable
   * @param[out] opt_traj Optimized state trajectory (Nx x (N+1))
   * @param[out] control_traj Optimized control trajectory (Nu x N)
   * @param[out] f_val Cost function value
   */
  void get_output(const Eigen::MatrixXd &x_out,
                        Eigen::MatrixXd &opt_traj,
                        Eigen::MatrixXd &control_traj,
                        double &f_val);

  /**
   * @brief Collect matrices into specific type for solver and solve
   * @param[in] initial_state Vector with initial state
   * @param[in] ref_traj Matrix holding desired reference trajectory
   * @param[out] x_out Optimized output
   * @return good_solve
   */
  bool solve(const Eigen::VectorXd &initial_state,
             const Eigen::MatrixXd &ref_traj,
             Eigen::MatrixXd &x_out
             );

private:

  /// Number of timesteps in horizon
  const int N_ = 10;

  /// Number of states per step
  const int nx_ = 12;

  /// Number of controls per step
  const int nu_ = 13;

  /// Total number of state variables in qp
  const int num_state_vars_= (N_ + 1) * nx_;

  /// Total number of control variables in qp
  const int num_control_vars_ = N * nu_;

  /// Total number of decision variables in qp
  const int num_decision_vars_ = num_state_vars_ + num_control_vars_;

  /// Number of dynamics constraints
  const int num_dyn_constraints_ = N_ * nx_;

  // Number of contact constraints per leg, per step
  const int num_constraints_per_leg_ = 5;

  /// Number of contact constraint per step
  const int num_contact_constraints_per_step_ = (num_constraints_per_leg_*num_legs_ + 1);

  /// Number of constraints on allowable forces
  const int num_contact_constraints_ = num_contact_constraints_per_step_*N_;

  /// Total number of constraints
  const int num_constraints_ = num_contact_constraints_per_step_*N_;

  /// Flag signaling that we've updated our weights since the last iteration
  bool updated_weights_ = false;

  /// Robot body mass
  double m_;

  /// Robot toe friction coefficient
  double mu_;

  /// Robot inertia matrix in body frame
  Eigen::Matrix3d Ib_;

  /// Flag signaling that mass properties have been set
  bool mass_properties_set_ = false;

  /// Acceleration due to gravity in world -Z
  const double g_ = 9.81;

  /// MPC timestep
  double dt_;

  /// Flag signaling timestep set
  bool dt_set_ = false;

  /// Typedef for state vector
  typedef Eigen::Vector<double,nx_> StateVec;

  /// Typedef for control vector
  typedef Eigen::Vector<double,nu_> ControlVec;

  /// Typedef for state matrix
  typedef Eigen::Matrix<double,nx_,nx_> StateMatrix;

  /// Typedef for control matrix
  typedef Eigen::Matrix<double,nx_,nu_> ControlMatrix;

  /// Typedef for state trajectory
  typedef Eigen::Matrix<double,N_+1,nx_> StateTraj;

  /// Typedef for control trajectory
  typedef Eigen::Matrix<double,N_,nu_> ControlTraj;

  /// Typedef for state Hessian
  typedef Eigen::Matrix<double,num_state_vars_,num_state_vars_> StateHessian;

  /// Typedef for control Hessian
  typedef Eigen::Matrix<double,num_control_vars_,num_control_vars_> ControlHessian;

  /// Typedef for state and control Hessian
  typedef Eigen::Matrix<double,num_state_vars_+num_control_vars_,
    num_state_vars_+num_control_vars_> StateControlHessian;

  /// Quadratic cost matrix
  StateControlHessian H_;

  // Precomputed matrix for linear cost vector construction
  StateHessian H_f_; 

  /// Dynamics constraint matrix
  Eigen::Matrix<double,num_dyn_constraints_,num_decision_vars_> A_dyn_dense_;

  /// Dynamics constraint vector
  Eigen::Vector<double, num_dyn_constraints_> b_dyn_;

  /// Friction and normal force constraint matrix
  Eigen::Matrix<double, num_contact_constraints_, num_control_vars_> A_con_dense_;

  /// Friction and normal force constraint lower bound
  Eigen::Vector<double, num_contact_constraints_> b_contact_lo_;

  /// Friction and normal force constraint upper bound
  Eigen::Vector<double, num_contact_constraints_> b_contact_hi_;

  /// State constraint lower bound
  StateVec b_state_lo_;

  /// State constraint upper bound
  StateVec b_state_hi_;

  Eigen::Vector<double, num_decision_vars_> f_;

  /// Highest possible double value
  const double INF_ = OsqpEigen::INFTY;

  /// OSQP solver instance
  OsqpEigen::Solver solver_;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD>quadKD_;
};


#endif