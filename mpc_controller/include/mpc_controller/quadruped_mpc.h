#ifndef MPCPLUSPLUS_H
#define MPCPLUSPLUS_H

#include "OsqpEigen/OsqpEigen.h"
#include <eigen3/Eigen/Eigen>
#include <spirit_utils/kinematics.h>
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
  int N_;

  /// Number of states per step
  int nx_;

  /// Number of controls per step
  int nu_;

  /// Number of dynamics constraints
  int num_dyn_constraints_;

  /// Number of contact constraint per step
  int num_contact_constraints_per_step_;

  /// Number of contact constraints per leg, per step
  int num_constraints_per_leg_;

  /// Number of constraints on allowable forces
  int num_contact_constraints_;

  /// Total number of constraints
  int num_constraints_;

  /// Total number of state variables in qp
  int num_state_vars_;

  /// Total number of control variables in qp
  int num_control_vars_;

  /// Total number of decision variables in qp
  int num_decision_vars_;

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

  /// Quadratic cost matrix
  Eigen::MatrixXd H_;

  // Precomputed matrix for linear cost vector construction
  Eigen::MatrixXd H_f_; 

  /// Dynamics constraint matrix
  Eigen::MatrixXd A_dyn_dense_;

  /// Dynamics constraint vector
  Eigen::MatrixXd b_dyn_;

  /// Friction and normal force constraint matrix
  Eigen::MatrixXd A_con_dense_;

  /// Friction and normal force constraint lower bound
  Eigen::VectorXd b_contact_lo_;

  /// Friction and normal force constraint upper bound
  Eigen::VectorXd b_contact_hi_;

  /// State constraint lower bound
  Eigen::VectorXd b_state_lo_;

  /// State constraint upper bound
  Eigen::VectorXd b_state_hi_;

  Eigen::VectorXd f_;

  /// Highest possible double value
  const double INF_ = OsqpEigen::INFTY;

  /// OSQP solver instance
  OsqpEigen::Solver solver_;

  /// Spirit Kinematics class
  std::shared_ptr<spirit_utils::SpiritKinematics> kinematics_;
};


#endif