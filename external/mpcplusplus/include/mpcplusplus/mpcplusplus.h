#ifndef LTI_MPCPLUSPLUS_H
#define LTI_MPCPLUSPLUS_H

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <assert.h>

namespace mpcplusplus {
class LinearMPC {

public:
  LinearMPC(const Eigen::MatrixXd &Ad,
            const Eigen::MatrixXd &Bd, const Eigen::MatrixXd &Q,
            const Eigen::MatrixXd &Qn, const Eigen::MatrixXd &R,
            const Eigen::MatrixXd &state_bounds,
            const Eigen::MatrixXd &control_bounds,
            const int N);

  LinearMPC(const int N, const int Nx, const int Nu);

  ~LinearMPC() = default;

  void update_weights(const Eigen::MatrixXd &Q, 
                      const Eigen::MatrixXd &Qn,
                      const Eigen::MatrixXd &R);

  void update_weights_vector(const std::vector<Eigen::MatrixXd> &Q,
                      const std::vector<Eigen::MatrixXd> &R);

  void update_statespace(const Eigen::MatrixXd &Ad,
                         const Eigen::MatrixXd &Bd);

  void update_statespace_vector(const std::vector<Eigen::MatrixXd> &Ad,
                                const std::vector<Eigen::MatrixXd> &Bd);

  void add_custom_constraint(const Eigen::MatrixXd &constraint,
                             const Eigen::VectorXd &lb,
                             const Eigen::VectorXd &ub);

  /**
   * @brief Constructs the quadratic cost function of the form
   * 
   * @param[in] ref_traj Reference trajectory
   * @param[out] f Linear component of cost
   */
  void get_cost_function(const Eigen::MatrixXd &ref_traj, Eigen::VectorXd &f);

  /**
   * @brief Collect stacked bounds
   * @param[in] initial_state Vector with initial state
   * @param[out] lb Lower bound
   * @param[out] ub Upper bound
   */
  void get_state_control_bounds(const Eigen::VectorXd &initial_state,
                                Eigen::VectorXd &lb, Eigen::VectorXd &ub);

  /**
   * @brief Collect first control value and all states and return them.
   * @param[in] x_out optimized decision variable
   * @param[out] first_control First control input to apply (Nu x 1)
   * @param[out] opt_traj Optimized state trajector (Nx x (N+1))
   */
  void get_output(const Eigen::MatrixXd &x_out,
                        Eigen::MatrixXd &first_control,
                        Eigen::MatrixXd &opt_traj);

  /**
   * @brief Collect matrices into specific type for solver and solve
   * @param[in] initial_state Vector with initial state
   * @param[in] ref_traj Matrix holding desired reference trajectory
   * @param[out] x_out Optimized output
   * @param[out] f_val Cost function value
   */
  void solve(const Eigen::VectorXd &initial_state,
             const Eigen::MatrixXd &ref_traj, Eigen::MatrixXd &x_out,
             double &f_val);

private:

  int m_N;
  int m_Nx;
  int m_Nu;

  int m_Nq;
  int m_Nx_vars;
  int m_Nx_decision;
  int m_Nconst;
  int m_num_control_vars;
  int m_num_state_vars;
  int m_num_decision_vars;

  bool updated_weights_ = false;

  bool updated_statespace_ = false;

  /// Quadratic cost matrix
  Eigen::MatrixXd H_;

  // Precomputed matrix for linear cost vector construction
  Eigen::MatrixXd H_f_; 

  /// Dynamics constraint matrix
  Eigen::MatrixXd A_dyn_dense_;

  /// Dynamics constraint vector
  Eigen::MatrixXd b_dyn_;

  /// Nx x 2 matrix of lower and upper bounds on state variables
  Eigen::MatrixXd m_state_bounds;

  /// Nu x 2 matrix of lower and upper bounds on control variables
  Eigen::MatrixXd m_control_bounds;

  /// OSQP solver instance
  OsqpEigen::Solver solver_;
};

} // mpcplusplus

#endif