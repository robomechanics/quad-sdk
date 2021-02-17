#ifndef MPCPLUSPLUS_H
#define MPCPLUSPLUS_H

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <assert.h>

namespace mpcplusplus {
class LinearMPC {

public:

  LinearMPC(const int N, const int Nx, const int Nu);

  ~LinearMPC() = default;

  void update_weights(const std::vector<Eigen::MatrixXd> &Q,
                      const std::vector<Eigen::MatrixXd> &R);

  void update_dynamics(const std::vector<Eigen::MatrixXd> &Ad,
                       const std::vector<Eigen::MatrixXd> &Bd);

  void update_contact(const std::vector<std::vector<bool> > contact_sequence,
                      const double mu,
                      const double fmin,
                      const double fmax);

  void update_state_bounds(const Eigen::VectorXd state_lo,
                           const Eigen::VectorXd state_hi);

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
   * @param[out] first_control First control input to apply (Nu x 1)
   * @param[out] opt_traj Optimized state trajector (Nx x (N+1))
   */
  void get_output(const Eigen::MatrixXd &x_out,
                        Eigen::VectorXd &first_control,
                        Eigen::MatrixXd &opt_traj,
                        Eigen::MatrixXd &control_traj);

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

  int N_;
  int nx_;
  int nu_;

  int num_dyn_constraints_;
  int num_contact_constraints_;
  int num_constraints_;

  int num_state_vars_;
  int num_control_vars_;
  int num_decision_vars_;

  bool updated_weights_ = false;

  /// Quadratic cost matrix
  Eigen::MatrixXd H_;

  // Precomputed matrix for linear cost vector construction
  Eigen::MatrixXd H_f_; 

  /// Dynamics constraint matrix
  Eigen::MatrixXd A_dyn_dense_;

  /// Dynamics constraint vector
  Eigen::MatrixXd b_dyn_;

  /// Friction constraint matrix
  Eigen::MatrixXd A_con_dense_;

  /// Friction constraint lower bound
  Eigen::VectorXd b_contact_lo_;

  /// Friction constraint upper bound
  Eigen::VectorXd b_contact_hi_;

  /// State constraint lower bound
  Eigen::VectorXd b_state_lo_;

  /// State constraint upper bound
  Eigen::VectorXd b_state_hi_;

  /// Control constraint lower bound
  Eigen::VectorXd b_control_lo_;

  /// Control constraint upper bound
  Eigen::VectorXd b_control_hi_;

  /// Highest possible double value
  const double INF_ = std::numeric_limits<double>::infinity();

  /// OSQP solver instance
  OsqpEigen::Solver solver_;
};

} // mpcplusplus

#endif