// Copyright (C) 2005, 2007 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-09

#ifndef __quadNLP_HPP__
#define __quadNLP_HPP__

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <IpIpoptData.hpp>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "IpTNLP.hpp"
#include "nmpc_controller/gen/eval_g_leg.h"
#include "nmpc_controller/gen/eval_g_leg_complex.h"
#include "nmpc_controller/gen/eval_g_leg_complex_to_simple.h"
#include "nmpc_controller/gen/eval_g_leg_simple.h"
#include "nmpc_controller/gen/eval_g_leg_simple_to_complex.h"
#include "nmpc_controller/gen/eval_g_tail.h"
#include "nmpc_controller/gen/eval_hess_g_leg.h"
#include "nmpc_controller/gen/eval_hess_g_leg_complex.h"
#include "nmpc_controller/gen/eval_hess_g_leg_complex_to_simple.h"
#include "nmpc_controller/gen/eval_hess_g_leg_simple.h"
#include "nmpc_controller/gen/eval_hess_g_leg_simple_to_complex.h"
#include "nmpc_controller/gen/eval_hess_g_tail.h"
#include "nmpc_controller/gen/eval_jac_g_leg.h"
#include "nmpc_controller/gen/eval_jac_g_leg_complex.h"
#include "nmpc_controller/gen/eval_jac_g_leg_complex_to_simple.h"
#include "nmpc_controller/gen/eval_jac_g_leg_simple.h"
#include "nmpc_controller/gen/eval_jac_g_leg_simple_to_complex.h"
#include "nmpc_controller/gen/eval_jac_g_tail.h"
#include "quad_utils/function_timer.h"
#include "quad_utils/tail_type.h"

using namespace Ipopt;

enum SystemID {
  LEG,
  TAIL,
  SIMPLE,
  SIMPLE_TO_COMPLEX,
  COMPLEX,
  COMPLEX_TO_SIMPLE
};

enum FunctionID { FUNC, JAC, HESS };

class quadNLP : public TNLP {
 public:
  // Horizon length, state dimension, input dimension, and constraints dimension
  int N_, n_, m_, g_;

  /// State dimension for simple and complex models
  int n_simple_, n_complex_;

  /// Vectors of state and constraint dimension for each finite element
  Eigen::VectorXi n_vec_, n_slack_vec_, g_vec_;

  /// Boolean for whether to apply panic variables for complex states
  const bool apply_slack_to_complex_ = false;

  /// Input dimension for simple and complex models
  int m_simple_, m_complex_;

  /// Constraint dimension for simple and complex models
  int g_simple_, g_complex_;

  /// Number of variables , primal variables, slack variables, and constraints
  int n_vars_, n_vars_primal_, n_vars_slack_, n_constraints_;

  /// Number of state variables added in complex model
  int n_null_;

  /// Nominal null state variables
  Eigen::VectorXd x_null_nom_;

  /// Number of state variables in the initial state
  int n0_;

  /// Declare the number of possible system ids (must match size of SystemID
  /// enum)
  static const int num_sys_id_ = 6;

  /// Declare the number of possible function ids (must match size of FunctionID
  /// enum)
  static const int num_func_id_ = 3;

  int leg_input_start_idx_;

  int type_;

  bool known_leg_input_;

  Eigen::MatrixXd leg_input_;

  // State cost weighting, input cost weighting
  Eigen::VectorXd Q_, R_;

  // Current scale factor for Q and R (may change between iterations)
  Eigen::VectorXd Q_factor_, R_factor_;

  /// Base scale factor for Q and R (does not change between iterations)
  Eigen::VectorXd Q_factor_base_, R_factor_base_;

  // Feet location from feet to body COM in world frame
  Eigen::MatrixXd feet_location_;

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

  /// Gravity constant
  const double grav_ = 9.81;

  // State bounds, input bounds, constraint bounds
  Eigen::VectorXd x_min_, x_max_, u_min_, u_max_, g_min_, g_max_;

  // State bounds, input bounds, constraint bounds
  Eigen::VectorXd x_min_simple_, x_max_simple_, x_min_complex_, x_max_complex_,
      g_min_simple_, g_max_simple_, g_min_complex_, g_max_complex_;

  // Ground height structure for the height bounds
  Eigen::MatrixXd ground_height_;

  // Initial guess
  Eigen::VectorXd w0_, z_L0_, z_U0_, lambda0_, g0_;

  double mu0_;

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

  // Penalty on panic variables
  double panic_weights_;

  // Time duration to the next plan index
  double first_element_duration_;

  // Complexity for the current state
  static const int x0_complexity_ = 0;

  // Complexity for the last state
  static const int xN_complexity_ = 0;

  /// Vector of ids for model complexity schedule
  Eigen::VectorXi complexity_schedule_;

  /// Vector of indices for relevant quantities
  Eigen::VectorXi fe_idxs_, u_idxs_, x_idxs_, slack_idxs_, g_idxs_,
      g_slack_idxs_, dynamic_jac_var_idxs_, panic_jac_var_idxs_,
      dynamic_hess_var_idxs_, cost_idxs_;

  /// Vector of system ids
  Eigen::VectorXi sys_id_schedule_;

  /// Matrix of function info
  Eigen::MatrixXi nnz_mat_, nrow_mat_, ncol_mat_, first_step_idx_mat_;

  /// Matrix of function sparsity data
  std::vector<std::vector<Eigen::VectorXi>> iRow_mat_, jCol_mat_;

  /// Number of complex finite elements in the horizon
  int num_complex_fe_;

  decltype(eval_g_leg_work) *eval_g_work_;
  decltype(eval_g_leg_incref) *eval_g_incref_;
  decltype(eval_g_leg_checkout) *eval_g_checkout_;
  decltype(eval_g_leg) *eval_g_;
  decltype(eval_g_leg_release) *eval_g_release_;
  decltype(eval_g_leg_decref) *eval_g_decref_;

  decltype(eval_hess_g_leg_work) *eval_hess_g_work_;
  decltype(eval_hess_g_leg_incref) *eval_hess_g_incref_;
  decltype(eval_hess_g_leg_checkout) *eval_hess_g_checkout_;
  decltype(eval_hess_g_leg) *eval_hess_g_;
  decltype(eval_hess_g_leg_release) *eval_hess_g_release_;
  decltype(eval_hess_g_leg_decref) *eval_hess_g_decref_;
  decltype(eval_hess_g_leg_sparsity_out) *eval_hess_g_sparsity_out_;

  decltype(eval_jac_g_leg_work) *eval_jac_g_work_;
  decltype(eval_jac_g_leg_incref) *eval_jac_g_incref_;
  decltype(eval_jac_g_leg_checkout) *eval_jac_g_checkout_;
  decltype(eval_jac_g_leg) *eval_jac_g_;
  decltype(eval_jac_g_leg_release) *eval_jac_g_release_;
  decltype(eval_jac_g_leg_decref) *eval_jac_g_decref_;
  decltype(eval_jac_g_leg_sparsity_out) *eval_jac_g_sparsity_out_;

  /// Vector of nonzero entries for constraint Jacobian and Hessian
  std::vector<casadi_int> nnz_jac_g_vec_, nnz_h_vec_;

  // Maps for casadi functions
  std::vector<std::vector<decltype(eval_g_leg) *>> eval_vec_;
  std::vector<std::vector<decltype(eval_g_leg_work) *>> eval_work_vec_;
  std::vector<std::vector<decltype(eval_g_leg_incref) *>> eval_incref_vec_;
  std::vector<std::vector<decltype(eval_g_leg_decref) *>> eval_decref_vec_;
  std::vector<std::vector<decltype(eval_g_leg_checkout) *>> eval_checkout_vec_;
  std::vector<std::vector<decltype(eval_g_leg_release) *>> eval_release_vec_;
  std::vector<std::vector<decltype(eval_g_leg_sparsity_out) *>>
      eval_sparsity_vec_;

  // System type
  int sys_type_;

  /** Default constructor */
  quadNLP(int type, int N, int n, int n_null, int m, double dt, double mu,
          double panic_weights, Eigen::VectorXd Q, Eigen::VectorXd R,
          Eigen::VectorXd Q_factor, Eigen::VectorXd R_factor,
          Eigen::VectorXd x_min, Eigen::VectorXd x_max,
          Eigen::VectorXd x_min_complex, Eigen::VectorXd x_max_complex,
          Eigen::VectorXd u_min, Eigen::VectorXd u_max);

  /**
   * @brief Custom deep copy constructor
   * @param[in] nlp Object to be deep copied
   */
  quadNLP(const quadNLP &nlp);

  /** Default destructor */
  virtual ~quadNLP();

  /**@name Overloaded from TNLP */
  //@{
  /** Method to return some info about the NLP */
  virtual bool get_nlp_info(Index &n, Index &m, Index &nnz_jac_g,
                            Index &nnz_h_lag, IndexStyleEnum &index_style);

  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(Index n, Number *x_l, Number *x_u, Index m,
                               Number *g_l, Number *g_u);

  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(Index n, bool init_x, Number *x, bool init_z,
                                  Number *z_L, Number *z_U, Index m,
                                  bool init_lambda, Number *lambda);

  /** Method to return the objective value */
  virtual bool eval_f(Index n, const Number *x, bool new_x, Number &obj_value);

  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(Index n, const Number *x, bool new_x,
                           Number *grad_f);

  /** Method to return the constraint residuals */
  virtual bool eval_g(Index n, const Number *x, bool new_x, Index m, Number *g);

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  virtual bool eval_jac_g(Index n, const Number *x, bool new_x, Index m,
                          Index nele_jac, Index *iRow, Index *jCol,
                          Number *values);

  virtual void compute_nnz_jac_g();

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  virtual bool eval_h(Index n, const Number *x, bool new_x, Number obj_factor,
                      Index m, const Number *lambda, bool new_lambda,
                      Index nele_hess, Index *iRow, Index *jCol,
                      Number *values);

  virtual void compute_nnz_h();

  /** This method is called when the algorithm is complete so the TNLP can
   * store/write the solution */
  virtual void finalize_solution(SolverReturn status, Index n, const Number *x,
                                 const Number *z_L, const Number *z_U, Index m,
                                 const Number *g, const Number *lambda,
                                 Number obj_value, const IpoptData *ip_data,
                                 IpoptCalculatedQuantities *ip_cq);

  virtual void shift_initial_guess();

  virtual void update_solver(
      const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
      const Eigen::MatrixXd &foot_positions,
      const std::vector<std::vector<bool>> &contact_schedule,
      const Eigen::MatrixXd &state_traj, const Eigen::MatrixXd &control_traj,
      const Eigen::VectorXd &ground_height,
      const double &first_element_duration_, const bool &same_plan_index,
      const bool &init);

  virtual void update_solver(
      const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
      const Eigen::MatrixXd &foot_positions,
      const std::vector<std::vector<bool>> &contact_schedule,
      const Eigen::VectorXd &ground_height,
      const double &first_element_duration_, const bool &same_plan_index,
      const bool &init);

  // Get the idx-th state variable from decision variable
  template <typename T>
  inline Eigen::Block<T> get_state_var(T &decision_var, const int &idx) {
    return decision_var.block(x_idxs_[idx], 0, n_vec_[idx], 1);
  }

  // Get the idx-th control variable from decision variable
  template <typename T>
  inline Eigen::Block<T> get_control_var(T &decision_var, const int &idx) {
    return decision_var.block(u_idxs_[idx], 0, m_, 1);
  }

  // Get the idx-th constraint from constraint variable
  template <typename T>
  inline Eigen::Block<T> get_constraint_var(T &constraint_var, const int &idx) {
    return constraint_var.block(g_idxs_[idx], 0, g_vec_[idx], 1);
  }

  // Get the idx-th panic variable (for (idx+1)-th state variable) from decision
  // variable
  template <typename T>
  inline Eigen::Block<T> get_panic_var(T &decision_var, const int &idx) {
    return decision_var.block(slack_idxs_[idx], 0, 2 * n_slack_vec_[idx], 1);
  }

  // Get the idx-th panic constraint (for (idx+1)-th state variable) from
  // constraint variable
  template <typename T>
  inline Eigen::Block<T> get_panic_constraint_var(T &constraint_var,
                                                  const int &idx) {
    return constraint_var.block(g_slack_idxs_[idx], 0, 2 * n_slack_vec_[idx],
                                1);
  }

  // Get the idx-th dynamic constraint related decision variable (idx and
  // idx+1-th state and idx-th control)
  template <typename T>
  inline Eigen::Block<T> get_dynamic_var(T &decision_var, const int &idx) {
    return decision_var.block(fe_idxs_[idx], 0,
                              n_vec_[idx] + m_ + n_vec_[idx + 1], 1);
  }

  // Get the idx-th dynamic constraint related jacobian nonzero entry
  template <typename T>
  inline Eigen::Block<T> get_dynamic_jac_var(T &jacobian_var, const int &idx) {
    return jacobian_var.block(dynamic_jac_var_idxs_[idx], 0,
                              nnz_mat_(sys_id_schedule_[idx], JAC), 1);
  }

  // Get the idx-th panic constraint jacobian (for (idx+1)-th state variable)
  // nonzero entry
  template <typename T>
  inline Eigen::Block<T> get_panic_jac_var(T &jacobian_var, const int &idx) {
    return jacobian_var.block(panic_jac_var_idxs_[idx], 0,
                              4 * n_slack_vec_[idx], 1);
  }

  // Get the idx-th dynamic constraint related hessian nonzero entry
  template <typename T>
  inline Eigen::Block<T> get_dynamic_hess_var(T &hessian_var, const int &idx) {
    return hessian_var.block(dynamic_hess_var_idxs_[idx], 0,
                             nnz_mat_(sys_id_schedule_[idx], HESS), 1);
  }

  // Get the idx-th state cost hessian nonzero entry
  template <typename T>
  inline Eigen::Block<T> get_state_cost_hess_var(T &hessian_var,
                                                 const int &idx) {
    return hessian_var.block(cost_idxs_[idx], 0, n_, 1);
  }

  // Get the idx-th control cost hessian nonzero entry
  template <typename T>
  inline Eigen::Block<T> get_control_cost_hess_var(T &hessian_var,
                                                   const int &idx) {
    return hessian_var.block(cost_idxs_[idx] + n_, 0, m_, 1);
  }

  void update_complexity_schedule(const Eigen::VectorXi &complexity_schedule);

  // /**
  //  * @brief Return the number of primal variables for this NLP
  //  * @return Number of primal variables
  //  */
  // inline int getNumPrimalVariables() const {
  //   return (m_ * N_ + n_simple_ * (N_ - num_complex_fe_) +
  //           n_complex_ * num_complex_fe_);
  // }

  // /**
  //  * @brief Return the number of slack variables for this NLP
  //  * @return Number of slack variables
  //  */
  // inline int getNumSlackVariables() const { return (2 * n_simple_ * N_); }

  // /**
  //  * @brief Return the number of variables for this NLP
  //  * @return Number of variables
  //  */
  // inline int getNumVariables() const {
  //   return getNumPrimalVariables() + getNumSlackVariables();
  // }

  // /**
  //  * @brief Return the number of constraints in this NLP
  //  * @return Number of constraints
  //  */
  // inline int getNumConstraints() const {
  //   return (g_simple_ * (N_ - num_complex_fe_) + g_complex_ *
  //   num_complex_fe_) +
  //          n_vars_slack_;
  // }

  /**
   * @brief Return the first index of the constraint vector corresponding to the
   * given finite element
   * @param[in] idx Index of requested finite element
   * @return Index in constraint vector corresponding to the beginning of the
   * requested FE
   */
  inline int getPrimalConstraintFEIndex(int idx) const { return g_idxs_[idx]; }

  /**
   * @brief Return the first index of the slack constraint vector corresponding
   * to the given finite element
   * @param[in] idx Index of requested finite element
   * @return Index in slack constraint vector corresponding to the beginning of
   * the requested FE
   */
  inline int getSlackConstraintFEIndex(int idx) const {
    return g_slack_idxs_[idx];
  }

  /**
   * @brief Return the first index of the decision variable vector corresponding
   * to the given finite element
   * @param[in] idx Index of requested finite element
   * @return Index in decision variable vector corresponding to the beginning of
   * the requested FE
   */
  inline int getPrimalFEIndex(int idx) const { return fe_idxs_[idx]; }

  /**
   * @brief Return the first index of the decision variable vector corresponding
   * to the given finite element's control input
   * @param[in] idx Index of requested finite element
   * @return Index in decision variable vector corresponding to the beginning of
   * the control input of the requested FE
   */
  inline int getPrimalControlFEIndex(int idx) const { return u_idxs_[idx]; }

  /**
   * @brief Return the first index of the decision variable vector corresponding
   * to the given finite element's state
   * @param[in] idx Index of requested finite element
   * @return Index in decision variable vector corresponding to the beginning of
   * the state of the requested FE
   */
  inline int getPrimalStateFEIndex(int idx) const { return x_idxs_[idx]; }

  /**
   * @brief Return the first index of the decision variable vector corresponding
   * to the slack variable for the given finite element's state
   * @param[in] idx Index of requested finite element
   * @return Index in decision variable vector corresponding to the slack
   * variable of the beginning of the state of the requested FE
   */
  inline int getSlackStateFEIndex(int idx) const { return slack_idxs_[idx]; }

  /**
   * @brief Load the casadi function pointers into map member vars
   */
  void loadCasadiFuncs();

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
