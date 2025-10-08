// Copyright (C) 2005, 2007 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-09

#ifndef __quadNLP_HPP__
#define __quadNLP_HPP__

#include <sys/resource.h>
#include <rbdl/rbdl.h> 
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <IpIpoptData.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "IpTNLP.hpp"
#include "nmpc_controller/gen/eval_g_a1.h"
#include "nmpc_controller/gen/eval_g_go2.h"
#include "nmpc_controller/gen/eval_g_leg_complex.h"
#include "nmpc_controller/gen/eval_g_leg_complex_to_simple.h"
#include "nmpc_controller/gen/eval_g_leg_simple.h"
#include "nmpc_controller/gen/eval_g_leg_simple_to_complex.h"
#include "nmpc_controller/gen/eval_g_spirit.h"
#include "nmpc_controller/gen/eval_hess_g_a1.h"
#include "nmpc_controller/gen/eval_hess_g_go2.h"
#include "nmpc_controller/gen/eval_hess_g_leg_complex.h"
#include "nmpc_controller/gen/eval_hess_g_leg_complex_to_simple.h"
#include "nmpc_controller/gen/eval_hess_g_leg_simple.h"
#include "nmpc_controller/gen/eval_hess_g_leg_simple_to_complex.h"
#include "nmpc_controller/gen/eval_hess_g_spirit.h"
#include "nmpc_controller/gen/eval_jac_g_a1.h"
#include "nmpc_controller/gen/eval_jac_g_go2.h"
#include "nmpc_controller/gen/eval_jac_g_leg_complex.h"
#include "nmpc_controller/gen/eval_jac_g_leg_complex_to_simple.h"
#include "nmpc_controller/gen/eval_jac_g_leg_simple.h"
#include "nmpc_controller/gen/eval_jac_g_leg_simple_to_complex.h"
#include "nmpc_controller/gen/eval_jac_g_spirit.h"
#include "quad_utils/function_timer.hpp"
#include "quad_utils/quad_kd.hpp"
#include <quad_msgs/msg/robot_plan_diagnostics.hpp>

using namespace Ipopt;

enum SystemID {
  SPIRIT,
  A1,
  GO2,
  SIMPLE_TO_SIMPLE,
  SIMPLE_TO_COMPLEX,
  COMPLEX_TO_COMPLEX,
  COMPLEX_TO_SIMPLE
};

enum FunctionID { FUNC, JAC, HESS };

// Struct for storing parameter information
struct NLPConfig {
  int x_dim_complex = 0;
  int u_dim_complex = 0;
  int g_dim_complex = 0;
  int x_dim_cost_complex = 0;
  int u_dim_cost_complex = 0;
  int x_dim_simple = 0;
  int u_dim_simple = 0;
  int g_dim_simple = 0;
  int x_dim_cost_simple = 0;
  int u_dim_cost_simple = 0;
  int x_dim_null = 0;
  int u_dim_null = 0;

  Eigen::VectorXd Q_complex;
  Eigen::VectorXd R_complex;
  Eigen::VectorXd x_min_complex;
  Eigen::VectorXd x_max_complex;
  Eigen::VectorXd x_min_complex_soft;
  Eigen::VectorXd x_max_complex_soft;
  Eigen::VectorXd u_min_complex;
  Eigen::VectorXd u_max_complex;
  Eigen::VectorXd g_min_complex;
  Eigen::VectorXd g_max_complex;
};

// Struct for storing diagnostic information
struct NLPDiagnostics {
  double compute_time, cost;
  int iterations, horizon_length;
  Eigen::VectorXi complexity_schedule;
  Eigen::VectorXd element_times;

  void loadDiagnosticsMsg(quad_msgs::msg::RobotPlanDiagnostics &msg) {
    msg.compute_time = compute_time;
    msg.cost = cost;
    msg.iterations = iterations;
    msg.horizon_length = horizon_length;

    msg.complexity_schedule.resize(complexity_schedule.size());
    msg.element_times.resize(element_times.size());
    for (int i = 0; i < complexity_schedule.size(); i++) {
      msg.complexity_schedule.at(i) = complexity_schedule[i];
      msg.element_times.at(i) = element_times[i];
    }
  }
};

class quadNLP : public TNLP {
 public:
  /// Shared Pointer to Node
  rclcpp::Node::SharedPtr node_;

  /// Robot Namespace
  std::string robot_ns_;

  /// Diagnostics struct for gathering metadata
  NLPDiagnostics diagnostics_;

  /// Config struct for storing meta parameters
  NLPConfig config_;

  /// Default system used for NLP (if not mixed complexity)
  SystemID default_system_;

  // QuadKD object for kinematics calculations
  std::shared_ptr<quad_utils::QuadKD> quadKD_;

  // Horizon length, state dimension, input dimension, and constraints dimension
  int N_, g_relaxed_;

  // Number of states in different components
  const int n_body_ = 12, n_foot_ = 24, n_joints_ = 24, n_tail_ = 4,
            m_body_ = 12, m_foot_ = 24, m_tail_ = 2;

  /// Vectors of state and constraint dimension for each finite element
  Eigen::VectorXi n_vec_, n_slack_vec_, m_vec_, g_vec_, g_slack_vec_,
      n_cost_vec_, m_cost_vec_;

  /// Boolean for whether to apply panic variables for complex constraints
  const bool apply_slack_to_complex_constr_ = true;

  /// Boolean for whether to allow modifications of foot trajectory
  const bool always_constrain_feet_ = false;

  /// Boolean for whether to include the terrain in the foot height constraint
  const bool use_terrain_constraint_ = false;

  /// Boolean for whether to include the terrain in the foot height constraint
  const bool remember_complex_elements_ = true;

  const grid_map::InterpolationMethods interp_type_ =
      grid_map::InterpolationMethods::INTER_LINEAR;

  /// Map for constraint names
  std::vector<std::vector<std::string>> constr_names_;

  /// Number of variables , primal variables, slack variables, and constraints
  int n_vars_, n_vars_primal_, n_vars_slack_, n_constraints_;

  /// Number of state and control variables added in complex model
  int n_null_, m_null_;

  /// Nominal null state variables
  Eigen::VectorXd x_null_nom_;

  /// Declare the number of possible system ids (must match size of SystemID
  /// enum)
  static const int num_sys_id_ = 7;

  /// Declare the number of possible function ids (must match size of FunctionID
  /// enum)
  static const int num_func_id_ = 3;

  // Scale factor for Q and R
  double Q_temporal_factor_, R_temporal_factor_;

  // Feet location from feet to body COM in world frame
  Eigen::MatrixXd foot_pos_body_;

  // Foot locations in world frame
  Eigen::MatrixXd foot_pos_world_;

  // Foot velocities in world frame
  Eigen::MatrixXd foot_vel_world_;

  // Step length
  double dt_;

  // Friction coefficient
  double mu_;

  /// Mass of the platform (set to zero to ignore nominal ff)
  const double mass_ = 13.3;

  /// Mass of the feet (as modeled in casadi)
  const double foot_mass_ = 0.01;

  /// Gravity constant
  const double grav_ = 9.81;

  /// Number of feet
  const int num_feet_ = 4;

  /// Terrain map
  grid_map::GridMap terrain_;

  Eigen::VectorXd g_min_complex_soft_, g_max_complex_soft_;

  // Ground height structure for the height bounds
  Eigen::MatrixXd ground_height_;

  // Initial guess
  Eigen::VectorXd w0_, z_L0_, z_U0_, lambda0_, g0_;

  /// Friction coefficient
  double mu0_;

  /// Whether warm start
  bool warm_start_;

  // State reference for computing cost
  Eigen::MatrixXd x_reference_;

  // Current state
  Eigen::VectorXd x_current_;

  // Feet contact sequence
  Eigen::MatrixXi contact_sequence_;

  // Nonzero entrance number in the constraint jacobian matrix
  int nnz_jac_g_;

  std::vector<int> first_step_idx_jac_g_;

  // Nonzero entrance row and column in the constraint jacobian matrix
  Eigen::VectorXi iRow_jac_g_, jCol_jac_g_;

  // Nonzero entrance number in the pure hessian matrix (exclude the side
  // jacobian part)
  int nnz_h_, nnz_compact_h_;

  // Vector of nnz in each block of the NLP Hessian
  Eigen::VectorXi nnz_step_jac_g_, nnz_step_hess_;

  std::vector<int> first_step_idx_hess_g_;

  // Nonzero entrance row and column in the pure hessian matrix (exclude the
  // side jacobian part)
  Eigen::VectorXi iRow_h_, jCol_h_, iRow_compact_h_, jCol_compact_h_;

  // Penalty on panic variables and constraints
  double panic_weights_, constraint_panic_weights_;

  // Time duration to the next plan index
  double first_element_duration_;

  /// Vector of ids for adaptive model complexity schedule
  Eigen::VectorXi adaptive_complexity_schedule_;

  /// Vector of ids for fixed model complexity schedule
  Eigen::VectorXi fixed_complexity_schedule_;

  /// Vector of indices for relevant quantities
  Eigen::VectorXi fe_idxs_, u_idxs_, x_idxs_, slack_state_var_idxs_,
      slack_constraint_var_idxs_, primal_constraint_idxs_,
      slack_var_constraint_idxs_, dynamic_jac_var_idxs_,
      relaxed_dynamic_jac_var_idxs_, panic_jac_var_idxs_,
      dynamic_hess_var_idxs_, cost_idxs_, relaxed_constraint_idxs_;

  /// Indices of relaxed constraints
  Eigen::ArrayXi relaxed_primal_constraint_idxs_in_element_,
      constraint_panic_weights_vec_;

  /// Vector of system ids
  Eigen::VectorXi sys_id_schedule_;

  /// Matrix of function info
  Eigen::MatrixXi nnz_mat_, nrow_mat_, ncol_mat_, first_step_idx_mat_,
      relaxed_nnz_mat_;

  /// Matrix of function sparsity data
  std::vector<std::vector<Eigen::VectorXi>> iRow_mat_, jCol_mat_;
  std::vector<std::vector<Eigen::VectorXi>> iRow_mat_relaxed_,
      jCol_mat_relaxed_, relaxed_idx_in_full_sparse_;

  /// Vector of nonzero entries for constraint Jacobian and Hessian
  std::vector<casadi_int> nnz_jac_g_vec_, nnz_h_vec_;

  // Maps for casadi functions
  std::vector<std::vector<decltype(eval_g_spirit) *>> eval_vec_;
  std::vector<std::vector<decltype(eval_g_spirit_work) *>> eval_work_vec_;
  std::vector<std::vector<decltype(eval_g_spirit_incref) *>> eval_incref_vec_;
  std::vector<std::vector<decltype(eval_g_spirit_decref) *>> eval_decref_vec_;
  std::vector<std::vector<decltype(eval_g_spirit_checkout) *>>
      eval_checkout_vec_;
  std::vector<std::vector<decltype(eval_g_spirit_release) *>> eval_release_vec_;
  std::vector<std::vector<decltype(eval_g_spirit_sparsity_out) *>>
      eval_sparsity_vec_;

  /** Default constructor */
  /** @brief Construct a new quadNLP object
   * @param[in] default_system Default system type for all finite elements
   * @param[in] N Horizon length
   * @param[in] dt Time duration for each finite element
   * @param[in] mu Friction coefficient
   * @param[in] panic_weights Weight on panic variables in cost function
   * @param[in] constraint_panic_weights Weight on panic variables in constraints
   * @param[in] Q_temporal_factor Temporal scaling factor for state cost
   * @param[in] R_temporal_factor Temporal scaling factor for control cost
   * @param[in] fixed_complexity_schedule Fixed complexity schedule 
   * @param[in] config NLP configuration struct
   * @param[in] node Shared Pointer to ROS Node to publish and subscribe from
   * @param[in] robot_ns Robot Namespace
   */
  quadNLP(SystemID default_system, int N, double dt, double mu,
          double panic_weights, double constraint_panic_weights,
          double Q_temporal_factor, double R_temporal_factor,
          const Eigen::VectorXi &fixed_complexity_schedule,
          const NLPConfig &config, rclcpp::Node::SharedPtr node, std::string robot_ns);

  /**
   * @brief Custom deep copy constructor
   * @param[in] nlp Object to be deep copied
   */
  quadNLP(const quadNLP &nlp);

  /** Default destructor */
  virtual ~quadNLP();

  /**@name Overloaded from TNLP */
  //@{
  /** @brief Method to return some info about the NLP 
   * @param[in] n Number of variables
   * @param[in] m Number of constraints
   * @param[in] nnz_jac_g Number of nonzeros in the constraint Jacobian
   * @param[in] nnz_h_lag Number of nonzeros in the Hessian of the Lagrangian
   * @param[in] index_style The indexing style for the matrices
  */
  virtual bool get_nlp_info(Index &n, Index &m, Index &nnz_jac_g,
                            Index &nnz_h_lag, IndexStyleEnum &index_style);

  /** @brief Method to return the bounds for my problem 
   * @param[in] i Index of the finite element to get bounds for
   * @param[in] x_lb Lower bounds on the state variables
   * @param[in] x_ub Upper bounds on the state variables
   * @param[in] u_lb Lower bounds on the control variables
   * @param[in] u_ub Upper bounds on the control variables
   * @param[in] g_l Lower bounds on the constraint variables
   * @param[in] g_u Upper bounds on the constraint variables
  */
  bool get_bounds_info_single_complex_fe(int i, Eigen::VectorXd &x_lb,
                                         Eigen::VectorXd &x_ub,
                                         Eigen::VectorXd &u_lb,
                                         Eigen::VectorXd &u_ub,
                                         Eigen::VectorXd &g_l,
                                         Eigen::VectorXd &g_u);

  /** @brief Method to return the bounds for my problem 
   * @param[in] i Index of the finite element to get bounds for
   * @param[in] x_l Lower bounds on the state variables
   * @param[in] x_u Upper bounds on the state variables
   * @param[in] m Number of control variables
   * @param[in] g_l Lower bounds on the constraint variables
   * @param[in] g_u Upper bounds on the constraint variables
  */
  virtual bool get_bounds_info(Index n, Number *x_l, Number *x_u, Index m,
                               Number *g_l, Number *g_u);

  /** @brief to return the starting point for the algorithm 
   * @param[in] n Number of variables
   * @param[in] init_x Whether to initialize the primal variables
   * @param[in] x Initial guess for the primal variables
   * @param[in] init_z Whether to initialize the bound multipliers
   * @param[in] z_L Initial guess for the lower bound multipliers
   * @param[in] z_U Initial guess for the upper bound multipliers
   * @param[in] m Number of constraints
   * @param[in] init_lambda Whether to initialize the constraint multipliers
   * @param[in] lambda Initial guess for the constraint multipliers
  */
  virtual bool get_starting_point(Index n, bool init_x, Number *x, bool init_z,
                                  Number *z_L, Number *z_U, Index m,
                                  bool init_lambda, Number *lambda);

  /** @brief Method to return the objective value 
   * @param[in] n Number of variables
   * @param[in] x Current value of the primal variables
   * @param[in] new_x Whether the primal variables have changed since the last
   *        call  
   * @param[in] obj_value Output value of the objective function
  */
  virtual bool eval_f(Index n, const Number *x, bool new_x, Number &obj_value);

  /** @brief Method to return the gradient of the objective 
   * @param[in] n Number of variables
   * @param[in] x Current value of the primal variables
   * @param[in] new_x Whether the primal variables have changed since the last
   *        call  
   * @param[in] grad_f Output value of the gradient of the objective function
  */
  virtual bool eval_grad_f(Index n, const Number *x, bool new_x,
                           Number *grad_f);

  /** @brief Method to return the constraint residual for requested data 
   * @param[in] i Index of the finite element to get constraint residual for
   * @param[in] x0 State at the beginning of the finite element
   * @param[in] u Control input for the finite element
   * @param[in] x1 State at the end of the finite element
  */
  Eigen::VectorXd eval_g_single_complex_fe(int i, const Eigen::VectorXd &x0,
                                           const Eigen::VectorXd &u,
                                           const Eigen::VectorXd &x1);

  /** @brief Method to return the constraint residuals 
   * @param[in] n Number of variables
   * @param[in] x Current value of the primal variables
   * @param[in] new_x Whether the primal variables have changed since the last
   *        call  
   * @param[in] m Number of constraints
   * @param[in] g Output value of the constraint residuals
  */
  virtual bool eval_g(Index n, const Number *x, bool new_x, Index m, Number *g);

  /** @brief Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   * @param[in] n Number of variables
   * @param[in] x Current value of the primal variables
   * @param[in] new_x Whether the primal variables have changed since the last
   *       call
   * @param[in] m Number of constraints
   * @param[in] nele_jac Number of nonzeros in the Jacobian
   * @param[in] iRow Row indices of the nonzeros
   * @param[in] jCol Column indices of the nonzeros
   * @param[in] values Values of the nonzeros (if NULL, return the structure
   */
  virtual bool eval_jac_g(Index n, const Number *x, bool new_x, Index m,
                          Index nele_jac, Index *iRow, Index *jCol,
                          Number *values);

  virtual void compute_nnz_jac_g();

  /** @brief Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   * @param[in] n Number of variables
   * @param[in] x Current value of the primal variables
   * @param[in] new_x Whether the primal variables have changed since the last
   *       call
   * @param[in] obj_factor The scaling factor for the objective function in the
   *       lagrangian
   * @param[in] m Number of constraints
   * @param[in] lambda The multipliers for the constraints in the lagrangian
   * @param[in] new_lambda Whether the multipliers have changed since the last
   *       call
   * @param[in] nele_hess Number of nonzeros in the Hessian
   * @param[in] iRow Row indices of the nonzeros
   * @param[in] jCol Column indices of the nonzeros
   * @param[in] values Values of the nonzeros (if NULL, return the structure
   */
  virtual bool eval_h(Index n, const Number *x, bool new_x, Number obj_factor,
                      Index m, const Number *lambda, bool new_lambda,
                      Index nele_hess, Index *iRow, Index *jCol,
                      Number *values);

  virtual void compute_nnz_h();

  /** @brief This method is called when the algorithm is complete so the TNLP can
   * store/write the solution
   * @param[in] status The solver status
   * @param[in] n Number of variables
   * @param[in] x Final value of the primal variables
   * @param[in] z_L Final value of the lower bound multipliers
   * @param[in] z_U Final value of the upper bound multipliers
   * @param[in] m Number of constraints
   * @param[in] g Final value of the constraint residuals
   * @param[in] lambda Final value of the constraint multipliers
   * @param[in] obj_value Final value of the objective function
   * @param[in] ip_data Pointer to an IpoptData object containing
   *       additional information
   * @param[in] ip_cq Pointer to an IpoptCalculatedQuantities object
   *       containing additional information
   */
  virtual void finalize_solution(SolverReturn status, Index n, const Number *x,
                                 const Number *z_L, const Number *z_U, Index m,
                                 const Number *g, const Number *lambda,
                                 Number obj_value, const IpoptData *ip_data,
                                 IpoptCalculatedQuantities *ip_cq);

  /** @brief 
   * @param[in] nlp_prev Previous nlp object to get initial guess from
   * @param[in] shift_idx Number of indices to shift the initial guess by
   */
  virtual void update_initial_guess(const quadNLP &nlp_prev, int shift_idx);

  /** @brief Method to update the solver with new data 
   * @param[in] initial_state Current state of the robot
   * @param[in] ref_traj Reference trajectory for the horizon
   * @param[in] foot_positions Foot positions in world frame
   * @param[in] contact_schedule Contact schedule for the horizon
   * @param[in] adaptive_complexity_schedule Adaptive complexity schedule for the horizon
   * @param[in] ground_height Ground height for each foot at each finite element
   * @param[in] first_element_duration Time duration to the next plan index
   * @param[in] plan_index_diff If the current solving is duplicated in the same index
   * @param[in] init Whether this is the first time the solver is being initialized
   */
  virtual void update_solver(
      const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
      const Eigen::MatrixXd &foot_positions,
      const std::vector<std::vector<bool>> &contact_schedule,
      const Eigen::VectorXi &adaptive_complexity_schedule,
      const Eigen::VectorXd &ground_height,
      const double &first_element_duration_, int plan_index_diff,
      const bool &init);

  void update_structure();
  
  /** @brief Get the lifted trajectory 
   * @param[out] state_traj_lifted Lifted state trajectory
   * @param[out] control_traj_lifted Lifted control trajectory
   */
  void get_lifted_trajectory(Eigen::MatrixXd &state_traj_lifted,
                             Eigen::MatrixXd &control_traj_lifted);

  /** @brief Get the heuristic trajectory 
   * @param[out] state_traj_heuristic Heuristic state trajectory
   * @param[out] control_traj_heuristic Heuristic control trajectory
   */
  void get_heuristic_trajectory(Eigen::MatrixXd &state_traj_heuristic,
                                Eigen::MatrixXd &control_traj_heuristic);

  // Get the idx-th state variable from decision variable
  template <typename T>
  inline Eigen::Block<T> get_primal_state_var(T &decision_var,
                                              const int &idx) const {
    return decision_var.block(x_idxs_[idx], 0, n_vec_[idx], 1);
  }

  // Get the idx-th body state variable from decision variable
  template <typename T>
  inline Eigen::Block<T> get_primal_body_state_var(T &decision_var,
                                                   const int &idx) const {
    return decision_var.block(x_idxs_[idx], 0, n_body_, 1);
  }

  // Get the idx-th foot state variable from decision variable
  template <typename T>
  inline Eigen::Block<T> get_primal_foot_state_var(T &decision_var,
                                                   const int &idx) const {
    return decision_var.block(x_idxs_[idx] + n_body_, 0, n_foot_, 1);
  }

  // Get the idx-th joint state variable from decision variable
  template <typename T>
  inline Eigen::Block<T> get_primal_joint_state_var(T &decision_var,
                                                    const int &idx) const {
    return decision_var.block(x_idxs_[idx] + n_body_ + n_foot_, 0, n_joints_,
                              1);
  }

  // Get the idx-th control variable from decision variable
  template <typename T>
  inline Eigen::Block<T> get_primal_control_var(T &decision_var,
                                                const int &idx) const {
    return decision_var.block(u_idxs_[idx], 0, m_vec_[idx], 1);
  }

  // Get the idx-th body control variable from decision variable
  template <typename T>
  inline Eigen::Block<T> get_primal_body_control_var(T &decision_var,
                                                     const int &idx) const {
    return decision_var.block(u_idxs_[idx], 0, m_body_, 1);
  }

  // Get the idx-th foot control variable from decision variable
  template <typename T>
  inline Eigen::Block<T> get_primal_foot_control_var(T &decision_var,
                                                     const int &idx) const {
    return decision_var.block(u_idxs_[idx] + m_body_, 0, m_foot_, 1);
  }

  // Get the idx-th panic variable (for (idx+1)-th state variable) from decision
  // variable
  template <typename T>
  inline Eigen::Block<T> get_slack_state_var(T &decision_var,
                                             const int &idx) const {
    return decision_var.block(slack_state_var_idxs_[idx], 0,
                              2 * n_slack_vec_[idx], 1);
  }

  // Get the idx-th panic variable (for (idx+1)-th constraint) from decision
  // variable
  template <typename T>
  inline Eigen::Block<T> get_slack_constraint_var(T &decision_var,
                                                  const int &idx) const {
    return decision_var.block(slack_constraint_var_idxs_[idx], 0,
                              2 * g_slack_vec_[idx], 1);
  }

  // Get the idx-th constraint from constraint values
  template <typename T>
  inline Eigen::Block<T> get_primal_constraint_vals(T &constraint_vals,
                                                    const int &idx) const {
    return constraint_vals.block(primal_constraint_idxs_[idx], 0, g_vec_[idx],
                                 1);
  }

  // Get the idx-th relaxed constraints from constraint values
  template <typename T>
  inline Eigen::Block<T> get_relaxed_primal_constraint_vals(
      T &constraint_vals, const int &idx) const {
    return constraint_vals.block(relaxed_constraint_idxs_[idx], 0,
                                 2 * g_slack_vec_[idx], 1);
  }

  // Get the idx-th panic constraint (for (idx+1)-th state variable) from
  // constraint variable
  template <typename T>
  inline Eigen::Block<T> get_slack_constraint_vals(T &constraint_vals,
                                                   const int &idx) const {
    return constraint_vals.block(slack_var_constraint_idxs_[idx], 0,
                                 2 * n_slack_vec_[idx], 1);
  }

  // Get the idx-th dynamic constraint related decision variable (idx and
  // idx+1-th state and idx-th control)
  template <typename T>
  inline Eigen::Block<T> get_dynamic_var(T &decision_var,
                                         const int &idx) const {
    return decision_var.block(fe_idxs_[idx], 0,
                              n_vec_[idx] + m_vec_[idx] + n_vec_[idx + 1], 1);
  }

  // Get the idx-th dynamic constraint related jacobian nonzero entry
  template <typename T>
  inline Eigen::Block<T> get_dynamic_jac_var(T &jacobian_var,
                                             const int &idx) const {
    return jacobian_var.block(dynamic_jac_var_idxs_[idx], 0,
                              nnz_mat_(sys_id_schedule_[idx], JAC), 1);
  }

  // Get the idx-th dynamic constraint related jacobian nonzero entry
  template <typename T>
  inline Eigen::Block<T> get_relaxed_dynamic_jac_var(T &jacobian_var,
                                                     const int &idx) const {
    return jacobian_var.block(
        relaxed_dynamic_jac_var_idxs_[idx], 0,
        2 * (relaxed_nnz_mat_(sys_id_schedule_[idx], JAC) + g_slack_vec_[idx]),
        1);
  }

  // Get the idx-th panic constraint jacobian (for (idx+1)-th state variable)
  // nonzero entry
  template <typename T>
  inline Eigen::Block<T> get_slack_jac_var(T &jacobian_var,
                                           const int &idx) const {
    return jacobian_var.block(panic_jac_var_idxs_[idx], 0,
                              4 * n_slack_vec_[idx], 1);
  }

  // Get the idx-th dynamic constraint related hessian nonzero entry
  template <typename T>
  inline Eigen::Block<T> get_dynamic_hess_var(T &hessian_var,
                                              const int &idx) const {
    return hessian_var.block(dynamic_hess_var_idxs_[idx], 0,
                             nnz_mat_(sys_id_schedule_[idx], HESS), 1);
  }

  // Get the idx-th state cost hessian nonzero entry
  template <typename T>
  inline Eigen::Block<T> get_state_cost_hess_var(T &hessian_var,
                                                 const int &idx) const {
    return hessian_var.block(cost_idxs_[idx], 0, n_cost_vec_[idx], 1);
  }

  // Get the idx-th control cost hessian nonzero entry
  template <typename T>
  inline Eigen::Block<T> get_control_cost_hess_var(T &hessian_var,
                                                   const int &idx) const {
    return hessian_var.block(cost_idxs_[idx] + n_cost_vec_[idx], 0,
                             m_cost_vec_[idx], 1);
  }

  /**
   * @brief Return the first index of the constraint vector corresponding to the
   * given finite element
   * @param[in] idx Index of requested finite element
   * @return Index in constraint vector corresponding to the beginning of the
   * requested fe
   */
  inline int get_primal_constraint_idx(int idx) const {
    return primal_constraint_idxs_[idx];
  }

  /**
   * @brief Return the first index of the slack constraint vector corresponding
   * to the given finite element
   * @param[in] idx Index of requested finite element
   * @return Index in slack constraint vector corresponding to the beginning of
   * the requested fe
   */
  inline int get_slack_var_constraint_idx(int idx) const {
    return slack_var_constraint_idxs_[idx];
  }

  /**
   * @brief Return the first index of the slack constraint vector corresponding
   * to the given finite element
   * @param[in] idx Index of requested finite element
   * @return Index in slack constraint vector corresponding to the beginning of
   * the requested fe
   */
  inline int get_relaxed_constraint_idx(int idx) const {
    return relaxed_constraint_idxs_[idx];
  }

  /**
   * @brief Return the first index of the decision variable vector corresponding
   * to the given finite element
   * @param[in] idx Index of requested finite element
   * @return Index in decision variable vector corresponding to the beginning of
   * the requested fe
   */
  inline int get_primal_idx(int idx) const { return fe_idxs_[idx]; }

  /**
   * @brief Return the first index of the decision variable vector corresponding
   * to the given finite element's state
   * @param[in] idx Index of requested finite element
   * @return Index in decision variable vector corresponding to the beginning of
   * the state of the requested fe
   */
  inline int get_primal_state_idx(int idx) const { return x_idxs_[idx]; }

  /**
   * @brief Return the first index of the decision variable vector corresponding
   * to the given finite element's control input
   * @param[in] idx Index of requested finite element
   * @return Index in decision variable vector corresponding to the beginning of
   * the control input of the requested fe
   */
  inline int get_primal_control_idx(int idx) const { return u_idxs_[idx]; }

  /**
   * @brief Return the first index of the decision variable vector corresponding
   * to the slack variable for the given finite element's state
   * @param[in] idx Index of requested finite element
   * @return Index in decision variable vector corresponding to the slack
   * variable of the beginning of the state of the requested fe
   */
  inline int get_slack_state_idx(int idx) const {
    return slack_state_var_idxs_[idx];
  }

  /**
   * @brief Return the first index of the decision variable vector corresponding
   * to the slack variable for the given finite element's state
   * @param[in] idx Index of requested finite element
   * @return Index in decision variable vector corresponding to the slack
   * variable of the beginning of the state of the requested fe
   */
  inline int get_slack_constraint_var_idx(int idx) const {
    return slack_constraint_var_idxs_[idx];
  }

  /**
   * @brief Load the casadi function pointers into map member vars
   */
  void loadCasadiFuncs();

  /**
   * @brief Load the constraint names for debugging
   */
  void loadConstraintNames();

  //@}

 private:
  /**@name Methods to block default compiler methods.
   *
   * The compiler automatically generates the following three methods.
   *  Since the default compiler implementation is generally not what
   *  you want (for all but the most simple classes), we usually
   *  put the declarations of these methods in the private section
   *  and never implement them. This prevents the compiler from
   *  implementing an incorrect "default" behavior without us
   *  knowing. (See Scott Meyers book, "Effective C++")
   */
  //@{

  quadNLP &operator=(const quadNLP &);
  //@}
};

#endif
