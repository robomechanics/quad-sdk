#include "nmpc_controller/quad_nlp.h"

#include <cassert>
#include <iostream>

using namespace Ipopt;

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

// Class constructor
quadNLP::quadNLP(int type, int N, int n, int n_null, int m, double dt,
                 double mu, double panic_weights, Eigen::VectorXd Q,
                 Eigen::VectorXd R, Eigen::VectorXd Q_factor,
                 Eigen::VectorXd R_factor, Eigen::VectorXd x_min,
                 Eigen::VectorXd x_max, Eigen::VectorXd x_min_complex,
                 Eigen::VectorXd x_max_complex, Eigen::VectorXd u_min,
                 Eigen::VectorXd u_max)
// N: prediction steps
// n: states dimension
// m: input dimension
// with_tail: system includes tail or not
// Q: states cost weights
// R: input cost weights
// dt: time step length
// panic_weights: penalty on panic variables
{
  type_ = type;

  N_ = N;
  n_ = n;
  n_null_ = n_null;
  m_ = m;
  g_ = n_ + 16; // states dynamics plus linear friciton cone

  // Load adaptive complexity parameters
  n_simple_ = n;
  n_complex_ = n + n_null_;
  m_simple_ = m;
  g_simple_ = g_;
  n0_ = (x0_complexity_ == 1) ? n_complex_ : n_simple_;

  // Added constraints include all simple constraints plus:
  //    n_null_ constraints for foot position and velocities
  //    n_null_ constraints for pos and neg joint torques
  //    4 constraints for knee position
  g_complex_ = g_ + 2 * n_null_ + 4;

  quad_utils::FunctionTimer timer("loadCasadiFuncs");
  loadCasadiFuncs();
  timer.reportStatistics();
  std::cout << "nnz_mat_\n" << nnz_mat_ << std::endl;

  if (type_ == NONE) {
    // Leg controller
    leg_input_start_idx_ = 0;
    sys_type_ = SIMPLE;

    eval_g_work_ = eval_g_leg_work;
    eval_g_incref_ = eval_g_leg_incref;
    eval_g_checkout_ = eval_g_leg_checkout;
    eval_g_ = eval_g_leg;
    eval_g_release_ = eval_g_leg_release;
    eval_g_decref_ = eval_g_leg_decref;

    eval_hess_g_work_ = eval_hess_g_leg_work;
    eval_hess_g_incref_ = eval_hess_g_leg_incref;
    eval_hess_g_checkout_ = eval_hess_g_leg_checkout;
    eval_hess_g_ = eval_hess_g_leg;
    eval_hess_g_release_ = eval_hess_g_leg_release;
    eval_hess_g_decref_ = eval_hess_g_leg_decref;
    eval_hess_g_sparsity_out_ = eval_hess_g_leg_sparsity_out;

    eval_jac_g_work_ = eval_jac_g_leg_work;
    eval_jac_g_incref_ = eval_jac_g_leg_incref;
    eval_jac_g_checkout_ = eval_jac_g_leg_checkout;
    eval_jac_g_ = eval_jac_g_leg;
    eval_jac_g_release_ = eval_jac_g_leg_release;
    eval_jac_g_decref_ = eval_jac_g_leg_decref;
    eval_jac_g_sparsity_out_ = eval_jac_g_leg_sparsity_out;

  } else {
    // Tail controller
    leg_input_start_idx_ = 2;
    sys_type_ = TAIL;

    eval_g_work_ = eval_g_tail_work;
    eval_g_incref_ = eval_g_tail_incref;
    eval_g_checkout_ = eval_g_tail_checkout;
    eval_g_ = eval_g_tail;
    eval_g_release_ = eval_g_tail_release;
    eval_g_decref_ = eval_g_tail_decref;

    eval_hess_g_work_ = eval_hess_g_tail_work;
    eval_hess_g_incref_ = eval_hess_g_tail_incref;
    eval_hess_g_checkout_ = eval_hess_g_tail_checkout;
    eval_hess_g_ = eval_hess_g_tail;
    eval_hess_g_release_ = eval_hess_g_tail_release;
    eval_hess_g_decref_ = eval_hess_g_tail_decref;
    eval_hess_g_sparsity_out_ = eval_hess_g_tail_sparsity_out;

    eval_jac_g_work_ = eval_jac_g_tail_work;
    eval_jac_g_incref_ = eval_jac_g_tail_incref;
    eval_jac_g_checkout_ = eval_jac_g_tail_checkout;
    eval_jac_g_ = eval_jac_g_tail;
    eval_jac_g_release_ = eval_jac_g_tail_release;
    eval_jac_g_decref_ = eval_jac_g_tail_decref;
    eval_jac_g_sparsity_out_ = eval_jac_g_tail_sparsity_out;
  }

  Q_ = Q;
  R_ = R;
  Q_factor_base_ = Q_factor;
  R_factor_base_ = R_factor;
  Q_factor_ = Q_factor_base_;
  R_factor_ = R_factor_base_;

  panic_weights_ = panic_weights;

  // feet location initialized by nominal position
  feet_location_ = Eigen::MatrixXd(12, N_);
  foot_pos_world_ = Eigen::MatrixXd(12, N_);
  foot_vel_world_ = Eigen::MatrixXd(12, N_).setZero();
  for (int i = 0; i < N_; ++i) {
    feet_location_.block(0, i, 12, 1) << -0.2263, -0.098, 0.3, -0.2263, 0.098,
        0.3, 0.2263, -0.098, 0.3, 0.2263, 0.098, 0.3;
    foot_pos_world_.block(0, i, 12, 1) << -0.2263, -0.098, 0, -0.2263, 0.098, 0,
        0.2263, -0.098, 0, 0.2263, 0.098, 0;
  }

  // Load constant parameters
  dt_ = dt;
  mu_ = mu;

  // Load state bounds
  x_min_ = x_min;
  x_max_ = x_max;
  x_min_simple_ = x_min_;
  x_max_simple_ = x_max_;
  x_min_complex_ = x_min_complex;
  x_max_complex_ = x_max_complex;

  u_min_ = u_min;
  u_max_ = u_max;

  // Initialize simple constraint bounds
  g_min_.resize(g_);
  g_max_.resize(g_);

  // Load dynamics constraint bounds
  g_min_.segment(0, n_).fill(0);
  g_max_.segment(0, n_).fill(0);

  // Load friction cone constraint bounds
  g_min_.segment(n_, 16).fill(-2e19);
  g_max_.segment(n_, 16).fill(0);
  g_min_simple_ = g_min_;
  g_max_simple_ = g_max_;

  // Initialize complex constraint bounds
  g_min_complex_.resize(g_complex_);
  g_max_complex_.resize(g_complex_);
  int current_idx = 0;

  // Load simple constraint bounds
  g_min_complex_.segment(current_idx, g_simple_) = g_min_simple_;
  g_max_complex_.segment(current_idx, g_simple_) = g_max_simple_;
  current_idx += g_simple_;

  // Load foot position and velocity constraint bounds
  g_min_complex_.segment(current_idx, n_null_).fill(0);
  g_max_complex_.segment(current_idx, n_null_).fill(0);
  current_idx += n_null_;

  // Load knee constraint bounds
  g_min_complex_.segment(current_idx, 4).fill(0);
  g_max_complex_.segment(current_idx, 4).fill(10);
  current_idx += 4;

  // Load motor model constraint bounds
  g_min_complex_.segment(current_idx, n_null_).fill(-2e19);
  g_max_complex_.segment(current_idx, n_null_).fill(0);

  // Load ground height data
  ground_height_ = Eigen::MatrixXd(1, N_);
  ground_height_.fill(0);

  num_complex_fe_ = 0;
  complexity_schedule_.setZero(N_);
  // complexity_schedule_.segment<2>(4).fill(1);
  this->update_complexity_schedule(complexity_schedule_);
  std::cout << "n_vec\n" << n_vec_.transpose() << std::endl;
  std::cout << "g_vec\n" << g_vec_.transpose() << std::endl;
  n_vars_ = getNumVariables();
  n_constraints_ = getNumConstraints();

  w0_ = Eigen::VectorXd(n_vars_).setZero();
  z_L0_ = Eigen::VectorXd(n_vars_).Ones(n_vars_, 1);
  z_U0_ = Eigen::VectorXd(n_vars_).Ones(n_vars_, 1);
  lambda0_ = Eigen::VectorXd(n_constraints_).setZero();

  if (m_ >= 12) {
    for (int i = 0; i < N_; ++i) {
      w0_(getPrimalFEIndex(i) + leg_input_start_idx_ + 2) =
          mass_ * grav_ * 0.25;
      w0_(getPrimalFEIndex(i) + leg_input_start_idx_ + 5) =
          mass_ * grav_ * 0.25;
      w0_(getPrimalFEIndex(i) + leg_input_start_idx_ + 8) =
          mass_ * grav_ * 0.25;
      w0_(getPrimalFEIndex(i) + leg_input_start_idx_ + 11) =
          mass_ * grav_ * 0.25;
    }
  }

  x_reference_ = Eigen::MatrixXd(n_, N_);
  x_reference_.fill(0);

  x_current_ = Eigen::VectorXd(n0_);
  x_current_.fill(0);

  contact_sequence_ = Eigen::MatrixXi(4, N_);
  contact_sequence_.fill(1);

  if (type_ == DISTRIBUTED) {
    known_leg_input_ = true;

    leg_input_ = Eigen::MatrixXd(m_ - leg_input_start_idx_, N_);
    leg_input_.setZero();
    if (m_ >= 12) {
      for (int i = 0; i < N_; ++i) {
        leg_input_(2, i) = mass_ * grav_ * 0.25;
        leg_input_(5, i) = mass_ * grav_ * 0.25;
        leg_input_(8, i) = mass_ * grav_ * 0.25;
        leg_input_(11, i) = mass_ * grav_ * 0.25;
      }
    }
  } else {
    known_leg_input_ = false;
  }

  // Initialize the time duration to the next plan index as dt
  first_element_duration_ = dt_;

  if (m_ >= 12) {
    quad_utils::FunctionTimer timer("setup");
    compute_nnz_jac_g();
    compute_nnz_h();
    timer.reportStatistics();
  }
}

// Destructor
quadNLP::~quadNLP() {}

// Returns the size of the problem
bool quadNLP::get_nlp_info(Index &n, Index &m, Index &nnz_jac_g,
                           Index &nnz_h_lag, IndexStyleEnum &index_style) {
  // Decision variables
  n = n_vars_;

  // Constraints
  m = n_constraints_;

  // Nonzero entrance in the constraint jacobian matrix
  nnz_jac_g = nnz_jac_g_;

  // Nonzero entrance in the full hessian matrix
  nnz_h_lag = nnz_compact_h_;

  // use the C style indexing (0-based)
  index_style = TNLP::C_STYLE;

  return true;
}

// Returns the variable bounds
bool quadNLP::get_bounds_info(Index n, Number *x_l, Number *x_u, Index m,
                              Number *g_l, Number *g_u) {
  Eigen::Map<Eigen::VectorXd> x_l_matrix(x_l, n);
  Eigen::Map<Eigen::VectorXd> x_u_matrix(x_u, n);
  Eigen::Map<Eigen::VectorXd> g_l_matrix(g_l, m);
  Eigen::Map<Eigen::VectorXd> g_u_matrix(g_u, m);

  // We have the decision variables ordered as u_0, x_1, ..., u_N-1, x_N
  int curr_var_idx, curr_constr_idx;
  curr_var_idx = curr_constr_idx = 0;
  for (int i = 0; i < N_; ++i) {
    // Inputs bound
    x_l_matrix.segment(curr_var_idx, m_) = u_min_;
    x_u_matrix.segment(curr_var_idx, m_) = u_max_;
    if (known_leg_input_) {
      x_l_matrix.segment(curr_var_idx + leg_input_start_idx_,
                         m_ - leg_input_start_idx_) =
          leg_input_.block(0, i, m_ - leg_input_start_idx_, 1);
      x_u_matrix.segment(curr_var_idx + leg_input_start_idx_,
                         m_ - leg_input_start_idx_) =
          leg_input_.block(0, i, m_ - leg_input_start_idx_, 1);
    }

    // Contact sequence
    for (int j = 0; j < 4; ++j) {
      x_l_matrix.segment(curr_var_idx + leg_input_start_idx_ + 3 * j, 3) =
          (x_l_matrix.segment(curr_var_idx + leg_input_start_idx_ + 3 * j, 3)
               .array() *
           contact_sequence_(j, i))
              .matrix();
      x_u_matrix.segment(curr_var_idx + leg_input_start_idx_ + 3 * j, 3) =
          (x_u_matrix.segment(curr_var_idx + leg_input_start_idx_ + 3 * j, 3)
               .array() *
           contact_sequence_(j, i))
              .matrix();
    }
    curr_var_idx += m_;

    // States bound
    x_l_matrix.segment(curr_var_idx, n_vec_[i]).fill(-2e19);
    x_u_matrix.segment(curr_var_idx, n_vec_[i]).fill(2e19);

    // If this is a complex element, load the bounds (not bounded by slack vars)
    if (n_vec_[i] == n_complex_) {
      x_l_matrix.segment(curr_var_idx + n_simple_, n_null_) =
          x_min_.tail(n_null_);
      x_u_matrix.segment(curr_var_idx + n_simple_, n_null_) =
          x_max_.tail(n_null_);
    }
    curr_var_idx += n_vec_[i];

    // Constraints bound
    g_l_matrix.segment(curr_constr_idx, g_vec_[i]) = g_min_.head(g_vec_[i]);
    g_u_matrix.segment(curr_constr_idx, g_vec_[i]) = g_max_.head(g_vec_[i]);
    curr_constr_idx += g_vec_[i];
  }

  // Panic variable bound
  x_l_matrix.segment(n_vars_primal_, n_vars_slack_).fill(0);
  x_u_matrix.segment(n_vars_primal_, n_vars_slack_).fill(2e19);

  int n_slack = n_;
  for (size_t i = 0; i < N_; i++) {
    // xmin
    g_l_matrix.segment(curr_constr_idx, n_slack) = x_min_.head(n_slack);
    g_l_matrix[curr_constr_idx + 2] = ground_height_(0, i);
    g_u_matrix.segment(curr_constr_idx, n_slack).fill(2e19);
    curr_constr_idx += n_slack;

    // xmax
    g_l_matrix.segment(curr_constr_idx, n_).fill(-2e19);
    g_u_matrix.segment(curr_constr_idx, n_) = x_max_.head(n_slack);
    curr_constr_idx += n_slack;
  }

  if ((curr_var_idx != n_vars_primal_) || (curr_constr_idx != n_constraints_)) {
    throw std::runtime_error("Number of vars and constraints in bounds doesn't "
                             "match the prediction!");
  }

  return true;
}

// Returns the initial point for the problem
bool quadNLP::get_starting_point(Index n, bool init_x, Number *x, bool init_z,
                                 Number *z_L, Number *z_U, Index m,
                                 bool init_lambda, Number *lambda) {
  if (init_x) {
    Eigen::Map<Eigen::VectorXd> w(x, n);
    w = w0_;
  }

  if (init_z) {
    Eigen::Map<Eigen::VectorXd> z_L_matrix(z_L, n);
    Eigen::Map<Eigen::VectorXd> z_U_matrix(z_L, n);
    z_L_matrix = z_L0_;
    z_U_matrix = z_U0_;
  }

  if (init_lambda) {
    Eigen::Map<Eigen::VectorXd> lambda_matrix(lambda, m);
    lambda_matrix = lambda0_;
  }

  return true;
}

// Returns the value of the objective function
bool quadNLP::eval_f(Index n, const Number *x, bool new_x, Number &obj_value) {
  Eigen::Map<const Eigen::VectorXd> w(x, n);

  obj_value = 0;

  int curr_var_idx = 0;
  for (int i = 0; i < N_; ++i) {
    // Compute the number of contacts
    Eigen::VectorXd u_nom(m_);
    u_nom.setZero();
    double num_contacts = contact_sequence_.col(i).sum();

    // If there are some contacts, set the nominal input accordingly
    if (num_contacts > 0) {
      for (int j = 0; j < contact_sequence_.rows(); j++) {
        if (contact_sequence_(j, i)) {
          u_nom[3 * j + 2 + leg_input_start_idx_] =
              mass_ * grav_ / num_contacts;
        }
      }
    }

    Eigen::MatrixXd uk = w.segment(curr_var_idx, m_) - u_nom;
    curr_var_idx += m_;
    Eigen::MatrixXd xk = w.segment(curr_var_idx, n_simple_);
    curr_var_idx += n_vec_[i];

    xk = (xk.array() - x_reference_.block(0, i, n_simple_, 1).array()).matrix();

    Eigen::MatrixXd Q_i = Q_ * Q_factor_(i, 0);
    Eigen::MatrixXd R_i = R_ * R_factor_(i, 0);

    // Scale the cost by time duration
    if (i == 0) {
      Q_i = Q_i * first_element_duration_ / dt_;
    }

    obj_value += (xk.transpose() * Q_i.asDiagonal() * xk / 2 +
                  uk.transpose() * R_i.asDiagonal() * uk / 2)(0, 0);
  }

  Eigen::VectorXd panic = w.segment(n_vars_primal_, n_vars_slack_);
  obj_value += panic_weights_ * panic.sum();

  return true;
}

// Return the gradient of the objective function
bool quadNLP::eval_grad_f(Index n, const Number *x, bool new_x,
                          Number *grad_f) {
  Eigen::Map<const Eigen::VectorXd> w(x, n);

  Eigen::Map<Eigen::VectorXd> grad_f_matrix(grad_f, n);

  int curr_var_idx = 0;
  for (int i = 0; i < N_; ++i) {
    // Compute the number of contacts
    Eigen::VectorXd u_nom(m_);
    u_nom.setZero();
    double num_contacts = contact_sequence_.col(i).sum();

    // If there are some contacts, set the nominal input accordingly
    if (num_contacts > 0) {
      for (int j = 0; j < contact_sequence_.rows(); j++) {
        if (contact_sequence_(j, i)) {
          u_nom[3 * j + 2 + leg_input_start_idx_] =
              mass_ * grav_ / num_contacts;
        }
      }
    }

    Eigen::MatrixXd uk = w.segment(curr_var_idx, m_) - u_nom;
    Eigen::MatrixXd xk = w.segment(curr_var_idx + m_, n_simple_);

    xk = (xk.array() - x_reference_.block(0, i, n_simple_, 1).array()).matrix();

    Eigen::MatrixXd Q_i = Q_ * Q_factor_(i, 0);
    Eigen::MatrixXd R_i = R_ * R_factor_(i, 0);

    // Scale the cost by time duration
    if (i == 0) {
      Q_i = Q_i * first_element_duration_ / dt_;
    }

    grad_f_matrix.segment(curr_var_idx, m_) = R_i.asDiagonal() * uk;
    grad_f_matrix.segment(curr_var_idx + m_, n_simple_) = Q_i.asDiagonal() * xk;
    curr_var_idx += m_ + n_vec_[i];
  }

  grad_f_matrix.segment(n_vars_primal_, n_vars_slack_).fill(panic_weights_);

  return true;
}

// Return the value of the constraints
bool quadNLP::eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) {
  Eigen::Map<const Eigen::VectorXd> w(x, n);
  Eigen::Map<Eigen::VectorXd> g_matrix(g, m);

  int curr_var_idx = 0;
  int curr_constr_idx = 0;
  for (int i = 0; i < N_; ++i) {
    // Select the system ID
    int sys_id = sys_id_schedule_[i];

    // Load the params for this FE
    Eigen::VectorXd pk(14);
    pk[0] = (i == 0) ? first_element_duration_ : dt_;
    pk[1] = mu_;
    pk.segment(2, 12) = feet_location_.col(i);

    // Set up the work function
    casadi_int sz_arg;
    casadi_int sz_res;
    casadi_int sz_iw;
    casadi_int sz_w;
    eval_work_vec_[sys_id][FUNC](&sz_arg, &sz_res, &sz_iw, &sz_w);

    const double *arg[sz_arg];
    double *res[sz_res];
    casadi_int iw[sz_iw];
    double _w[sz_w];

    eval_incref_vec_[sys_id][FUNC]();

    // Declare the temp arg for the first element
    Eigen::VectorXd tmp_arg(n0_ + m_ + n_vec_[i]);
    if (i == 0) {
      tmp_arg.topRows(n0_) = x_current_;
      tmp_arg.bottomRows(m_ + n_vec_[i]) = w.head(m_ + n_vec_[i]);
      arg[0] = tmp_arg.data();
    } else {
      arg[0] = w.segment(curr_var_idx - n_vec_[i - 1],
                         n_vec_[i - 1] + m_ + n_vec_[i])
                   .data();
    }

    arg[1] = pk.data();
    res[0] = g_matrix.segment(curr_constr_idx, g_vec_[i]).data();

    int mem = eval_checkout_vec_[sys_id][FUNC]();
    eval_vec_[sys_id][FUNC](arg, res, iw, _w, mem);
    eval_release_vec_[sys_id][FUNC](mem);
    eval_decref_vec_[sys_id][FUNC]();

    curr_var_idx += m_ + n_vec_[i];
    curr_constr_idx += g_vec_[i];
  }

  int n_slack = n_;
  int curr_var_slack_idx = 0;
  curr_var_idx = m_;
  for (int i = 0; i < N_; ++i) {
    Eigen::VectorXd xk = w.segment(curr_var_idx, n_slack);
    curr_var_idx += m_ + n_vec_[i];
    Eigen::VectorXd panick =
        w.segment(n_vars_primal_ + curr_var_slack_idx, 2 * n_slack);
    curr_var_slack_idx += 2 * n_slack;

    g_matrix.segment(curr_constr_idx, n_) = xk + panick.segment(0, n_slack);
    curr_constr_idx += n_slack;
    g_matrix.segment(curr_constr_idx, n_) =
        xk - panick.segment(n_slack, n_slack);
    curr_constr_idx += n_slack;
  }

  return true;
}

// Return the structure or values of the Jacobian
bool quadNLP::eval_jac_g(Index n, const Number *x, bool new_x, Index m,
                         Index nele_jac, Index *iRow, Index *jCol,
                         Number *values) {
  if (values == NULL) {
    Eigen::Map<Eigen::VectorXi> iRow_matrix(iRow, nele_jac);
    Eigen::Map<Eigen::VectorXi> jCol_matrix(jCol, nele_jac);

    iRow_matrix = iRow_jac_g_;
    jCol_matrix = jCol_jac_g_;
  } else {
    Eigen::Map<const Eigen::VectorXd> w(x, n);

    Eigen::Map<Eigen::VectorXd> values_matrix(values, nele_jac);

    int curr_var_idx = 0;
    int curr_sparse_idx = 0;
    for (int i = 0; i < N_; ++i) {
      // Select the system ID
      int sys_id = sys_id_schedule_[i];

      // Load the params for this FE
      Eigen::VectorXd pk(14);
      pk[0] = (i == 0) ? first_element_duration_ : dt_;
      pk[1] = mu_;
      pk.segment(2, 12) = feet_location_.col(i);

      // Set up the work function
      casadi_int sz_arg;
      casadi_int sz_res;
      casadi_int sz_iw;
      casadi_int sz_w;
      eval_work_vec_[sys_id][JAC](&sz_arg, &sz_res, &sz_iw, &sz_w);

      const double *arg[sz_arg];
      double *res[sz_res];
      casadi_int iw[sz_iw];
      double _w[sz_w];

      eval_incref_vec_[sys_id][JAC]();

      // Declare the temp arg for the first element
      int nnz = nnz_step_jac_g_[i];
      Eigen::VectorXd tmp_arg(n0_ + m_ + n_vec_[i]),
          tmp_res(nnz_step_jac_g_[i]);

      int mem;
      if (i == 0) {
        tmp_arg.head(n0_) = x_current_;
        tmp_arg.tail(m_ + n_vec_[i]) = w.head(m_ + n_vec_[i]);
        arg[0] = tmp_arg.data();
        arg[1] = pk.data();
        res[0] = tmp_res.data();

        mem = eval_checkout_vec_[sys_id][JAC]();
        eval_vec_[sys_id][JAC](arg, res, iw, _w, mem);

        nnz = tmp_res.size() - first_step_idx_mat_(sys_id, JAC);
        values_matrix.head(nnz) = tmp_res.tail(nnz);
      } else {
        arg[0] = w.segment(curr_var_idx - n_vec_[i - 1],
                           n_vec_[i - 1] + m_ + n_vec_[i])
                     .data();
        arg[1] = pk.data();
        res[0] = values_matrix.segment(curr_sparse_idx, nnz).data();

        mem = eval_checkout_vec_[sys_id][JAC]();
        eval_vec_[sys_id][JAC](arg, res, iw, _w, mem);
      }

      eval_release_vec_[sys_id][JAC](mem);
      eval_decref_vec_[sys_id][JAC]();

      curr_var_idx += m_ + n_vec_[i];
      curr_sparse_idx += nnz;
    }

    int n_slack = n_;
    for (size_t i = 0; i < N_; i++) {
      // xmin wrt x
      values_matrix.segment(curr_sparse_idx, n_slack).fill(1);
      curr_sparse_idx += n_slack;

      // xmin wrt panic
      values_matrix.segment(curr_sparse_idx, n_slack).fill(1);
      curr_sparse_idx += n_slack;

      // xmax wrt x
      values_matrix.segment(curr_sparse_idx, n_slack).fill(1);
      curr_sparse_idx += n_slack;

      // xmax wrt panic
      values_matrix.segment(curr_sparse_idx, n_slack).fill(-1);
      curr_sparse_idx += n_slack;
    }

    if (curr_sparse_idx != nele_jac) {
      throw std::runtime_error("Number of nonzero Jacobian entries doesn't "
                               "match the prediction in eval_jac_g!");
    }
  }

  return true;
}

// Return the structure of the Jacobian
void quadNLP::compute_nnz_jac_g() {
  // Initialize nnz
  nnz_jac_g_ = 0;

  // Compute the number of nnz in the Jacobians over the horizon
  for (int i = 0; i < N_; ++i) {
    nnz_jac_g_ += nnz_mat_(sys_id_schedule_[i], JAC);
  }

  // Add slack variables (only on simple model coordinates) and subtract first
  // step vars
  int first_step_idx = first_step_idx_mat_(sys_id_schedule_[0], JAC);
  nnz_jac_g_ = nnz_jac_g_ - first_step_idx + 4 * n_ * N_;
  nnz_step_jac_g_.resize(N_);

  // Size the NLP constraint Jacobian sparsity structure
  iRow_jac_g_ = Eigen::VectorXi(nnz_jac_g_);
  jCol_jac_g_ = Eigen::VectorXi(nnz_jac_g_);

  int curr_sparse_idx = 0;
  int curr_constr_idx = 0;
  int curr_var_idx = 0;
  for (int i = 0; i < N_; ++i) {
    int sys_id = sys_id_schedule_[i];
    nnz_step_jac_g_[i] = nnz_mat_(sys_id, JAC);

    if (i == 0) {
      int first_step_size = iRow_mat_[sys_id][JAC].size() - first_step_idx;
      iRow_jac_g_.head(first_step_size) =
          iRow_mat_[sys_id][JAC].tail(first_step_size);
      jCol_jac_g_.head(first_step_size) =
          (jCol_mat_[sys_id][JAC].tail(first_step_size).array() - n_).matrix();
      curr_sparse_idx += first_step_size;
    } else {
      // Update the number of nonzeros in this block
      int nnz = nnz_mat_(sys_id, JAC);

      // Each step we shift g constraints
      iRow_jac_g_.segment(curr_sparse_idx, nnz) =
          (iRow_mat_[sys_id][JAC].array() + curr_constr_idx).matrix();

      // Each step we shift n states and m inputs
      jCol_jac_g_.segment(curr_sparse_idx, nnz) =
          (jCol_mat_[sys_id][JAC].array() + curr_var_idx - n_).matrix();

      curr_sparse_idx += nnz;
    }
    curr_constr_idx += g_vec_[i];
    curr_var_idx += m_ + n_vec_[i];
  }

  for (size_t i = 0; i < N_; i++) {
    // Get index of the primal state vars for this finite element and define num
    // of slack variables
    int primal_state_fe_idx = getPrimalStateFEIndex(i);
    int n_slack = n_;

    // xmin wrt x
    iRow_jac_g_.segment(curr_sparse_idx, n_slack) = Eigen::ArrayXi::LinSpaced(
        n_slack, curr_constr_idx, curr_constr_idx + n_slack - 1);
    jCol_jac_g_.segment(curr_sparse_idx, n_slack) = Eigen::ArrayXi::LinSpaced(
        n_slack, primal_state_fe_idx, primal_state_fe_idx + n_slack - 1);
    curr_sparse_idx += n_slack;

    // xmin wrt panic
    iRow_jac_g_.segment(curr_sparse_idx, n_slack) = Eigen::ArrayXi::LinSpaced(
        n_slack, curr_constr_idx, curr_constr_idx + n_slack - 1);
    jCol_jac_g_.segment(curr_sparse_idx, n_slack) = Eigen::ArrayXi::LinSpaced(
        n_slack, curr_var_idx, curr_var_idx + n_slack - 1);
    curr_sparse_idx += n_slack;
    curr_constr_idx += n_slack;
    curr_var_idx += n_slack;

    // xmax wrt x
    iRow_jac_g_.segment(curr_sparse_idx, n_slack) = Eigen::ArrayXi::LinSpaced(
        n_slack, curr_constr_idx, curr_constr_idx + n_slack - 1);
    jCol_jac_g_.segment(curr_sparse_idx, n_slack) = Eigen::ArrayXi::LinSpaced(
        n_slack, primal_state_fe_idx, primal_state_fe_idx + n_slack - 1);
    curr_sparse_idx += n_slack;

    // xmax wrt panic
    iRow_jac_g_.segment(curr_sparse_idx, n_slack) = Eigen::ArrayXi::LinSpaced(
        n_slack, curr_constr_idx, curr_constr_idx + n_slack - 1);
    jCol_jac_g_.segment(curr_sparse_idx, n_slack) = Eigen::ArrayXi::LinSpaced(
        n_slack, curr_var_idx, curr_var_idx + n_slack - 1);
    curr_sparse_idx += n_slack;
    curr_constr_idx += n_slack;
    curr_var_idx += n_slack;
  }

  if ((curr_sparse_idx != nnz_jac_g_) || (curr_var_idx != n_vars_) ||
      (curr_constr_idx != n_constraints_)) {
    throw std::runtime_error("Number of nonzero Jacobian entries doesn't match "
                             "the prediction in compute_nnz_jac_g!");
  }
}

// Return the structure or values of the Hessian
bool quadNLP::eval_h(Index n, const Number *x, bool new_x, Number obj_factor,
                     Index m, const Number *lambda, bool new_lambda,
                     Index nele_hess, Index *iRow, Index *jCol,
                     Number *values) {
  if (values == NULL) {
    Eigen::Map<Eigen::VectorXi> iRow_matrix(iRow, nele_hess);
    Eigen::Map<Eigen::VectorXi> jCol_matrix(jCol, nele_hess);

    iRow_matrix = iRow_compact_h_;
    jCol_matrix = jCol_compact_h_;
  } else {
    Eigen::Map<const Eigen::VectorXd> w(x, n);

    Eigen::Map<Eigen::VectorXd> values_compact_matrix(values, nele_hess);
    Eigen::Map<const Eigen::VectorXd> lambda_matrix(lambda, m);
    Eigen::VectorXd values_matrix(nnz_h_);

    int curr_sparse_idx = 0;
    int curr_var_idx = 0;
    int curr_constr_idx = 0;
    for (int i = 0; i < N_; ++i) {
      // Select the system ID
      int sys_id = sys_id_schedule_[i];

      // Load the params for this FE
      Eigen::VectorXd pk(14);
      pk[0] = (i == 0) ? first_element_duration_ : dt_;
      pk[1] = mu_;
      pk.segment(2, 12) = feet_location_.col(i);

      // Set up the work function
      casadi_int sz_arg;
      casadi_int sz_res;
      casadi_int sz_iw;
      casadi_int sz_w;
      eval_work_vec_[sys_id][HESS](&sz_arg, &sz_res, &sz_iw, &sz_w);

      const double *arg[sz_arg];
      double *res[sz_res];
      casadi_int iw[sz_iw];
      double _w[sz_w];

      eval_incref_vec_[sys_id][HESS]();
      int mem;

      // Declare the temp arg for the first element
      int nnz = nnz_step_hess_[i];

      Eigen::VectorXd tmp_arg(n0_ + m_ + n_vec_[i]), tmp_res(nnz);
      if (i == 0) {
        tmp_arg.head(n0_) = x_current_;
        tmp_arg.tail(m_ + n_vec_[i]) = w.head(m_ + n_vec_[i]);

        arg[0] = tmp_arg.data();
        arg[1] = lambda_matrix.segment(curr_constr_idx, g_vec_[i]).data();
        arg[2] = pk.data();
        res[0] = tmp_res.data();

        mem = eval_checkout_vec_[sys_id][HESS]();
        eval_vec_[sys_id][HESS](arg, res, iw, _w, mem);

        nnz = tmp_res.size() - first_step_idx_mat_(sys_id, HESS);
        values_matrix.segment(curr_sparse_idx, nnz) = tmp_res.tail(nnz);
      } else {
        arg[0] = w.segment(curr_var_idx - n_vec_[i - 1],
                           n_vec_[i - 1] + m_ + n_vec_[i])
                     .data();
        arg[1] = lambda_matrix.segment(curr_constr_idx, g_vec_[i]).data();
        arg[2] = pk.data();
        res[0] = values_matrix.segment(curr_sparse_idx, nnz).data();

        mem = eval_checkout_vec_[sys_id][HESS]();
        eval_vec_[sys_id][HESS](arg, res, iw, _w, mem);
      }

      // Update the iterators
      curr_sparse_idx += nnz;
      curr_var_idx += m_ + n_vec_[i];
      curr_constr_idx += g_vec_[i];

      // Release memory for function
      eval_release_vec_[sys_id][HESS](mem);
      eval_decref_vec_[sys_id][HESS]();

      // Initialize Q and R weights
      Eigen::MatrixXd Q_i = Q_ * Q_factor_(i, 0);
      Eigen::MatrixXd R_i = R_ * R_factor_(i, 0);

      // Scale the cost by time duration
      if (i == 0) {
        Q_i = Q_i * first_element_duration_ / dt_;
      }

      // Update the values for the cost
      values_matrix.segment(curr_sparse_idx, m_) =
          (obj_factor * R_i.array()).matrix();
      curr_sparse_idx += m_;
      values_matrix.segment(curr_sparse_idx, n_vec_[i]) =
          (obj_factor * Q_i.array()).matrix();
      curr_sparse_idx += n_vec_[i];
    }

    if (curr_var_idx != n_vars_primal_) {
      throw std::runtime_error(
          "Number of Hessian vars doesn't match the number of primals!");
    }

    if (curr_sparse_idx != nnz_h_) {
      throw std::runtime_error("Number of Hessian nonzero entries doesn't "
                               "match the prediction in eval_h!");
    }

    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(nnz_h_);
    for (int i = 0; i < nnz_h_; ++i) {
      tripletList.push_back(
          Eigen::Triplet<double>(iRow_h_[i], jCol_h_[i], values_matrix[i]));
    }
    Eigen::SparseMatrix<double> hess(n_vars_primal_, n_vars_primal_);
    hess.setFromTriplets(tripletList.begin(), tripletList.end());

    int idx_hess = 0;
    for (int k = 0; k < hess.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(hess, k); it; ++it) {
        values_compact_matrix[idx_hess] = it.value();
        idx_hess += 1;
      }
    }
  }

  return true;
}

// Return the structure of the Hessian
void quadNLP::compute_nnz_h() {
  // Initialize nnz
  nnz_h_ = 0;

  // Compute the number of nnz in the Hessian over the horizon
  for (int i = 0; i < N_; ++i) {
    nnz_h_ += nnz_mat_(sys_id_schedule_[i], HESS);
  }

  // Add cost variables (only on simple model coordinates) and subtract first
  // step vars
  int first_step_idx = first_step_idx_mat_(sys_id_schedule_[0], HESS);
  nnz_h_ = nnz_h_ - first_step_idx + N_ * (n_simple_ + m_);

  // Size the NLP constraint Hessian sparsity structure
  iRow_h_ = Eigen::VectorXi(nnz_h_);
  jCol_h_ = Eigen::VectorXi(nnz_h_);

  // Hessian from cost
  Eigen::ArrayXi iRow_cost =
      Eigen::ArrayXi::LinSpaced(n_simple_ + m_, 0, n_simple_ + m_ - 1);
  Eigen::ArrayXi jCol_cost =
      Eigen::ArrayXi::LinSpaced(n_simple_ + m_, 0, n_simple_ + m_ - 1);

  iRow_h_ = Eigen::VectorXi(nnz_h_);
  jCol_h_ = Eigen::VectorXi(nnz_h_);
  nnz_step_hess_.resize(N_);

  int curr_sparse_idx = 0;
  int curr_var_idx = 0;
  for (int i = 0; i < N_; ++i) {
    int sys_id = sys_id_schedule_[i];
    nnz_step_hess_[i] = nnz_mat_(sys_id, HESS);

    if (i == 0) {
      int first_step_size = iRow_mat_[sys_id][HESS].size() - first_step_idx;
      iRow_h_.head(first_step_size) =
          (iRow_mat_[sys_id][HESS].tail(first_step_size).array() - n_simple_)
              .matrix();
      jCol_h_.head(first_step_size) =
          (jCol_mat_[sys_id][HESS].tail(first_step_size).array() - n_simple_)
              .matrix();
      curr_sparse_idx += first_step_size;
    } else {
      // Update the number of nonzeros in this block
      int nnz = nnz_mat_(sys_id_schedule_[i], HESS);

      // Each step we shift n states and m inputs
      iRow_h_.segment(curr_sparse_idx, nnz) =
          (iRow_mat_[sys_id][HESS].array() + curr_var_idx - n_).matrix();
      jCol_h_.segment(curr_sparse_idx, nnz) =
          (jCol_mat_[sys_id][HESS].array() + curr_var_idx - n_).matrix();

      curr_sparse_idx += nnz;
    }
    // Hessian from cost
    iRow_h_.segment(curr_sparse_idx, n_simple_ + m_) =
        (iRow_cost + curr_var_idx).matrix();
    jCol_h_.segment(curr_sparse_idx, n_simple_ + m_) =
        (jCol_cost + curr_var_idx).matrix();

    curr_sparse_idx += (n_simple_ + m_);
    curr_var_idx += n_vec_[i] + m_;
  }

  if ((curr_sparse_idx != nnz_h_) || (curr_var_idx != n_vars_primal_)) {
    throw std::runtime_error(
        "Number of Hessian nonzero entries doesn't match the prediction!");
  }

  std::vector<Eigen::Triplet<double>> tripletList;
  tripletList.reserve(nnz_h_);
  for (int i = 0; i < nnz_h_; ++i) {
    tripletList.push_back(Eigen::Triplet<double>(iRow_h_[i], jCol_h_[i], 1));
  }
  Eigen::SparseMatrix<double> hess(n_vars_primal_, n_vars_primal_);
  hess.setFromTriplets(tripletList.begin(), tripletList.end());

  // We eliminate the overlap nonzero entrances here to get the exact nonzero
  // number
  nnz_compact_h_ = hess.nonZeros();
  iRow_compact_h_ = Eigen::VectorXi(nnz_compact_h_);
  jCol_compact_h_ = Eigen::VectorXi(nnz_compact_h_);

  int idx_hess = 0;

  for (int k = 0; k < hess.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(hess, k); it; ++it) {
      iRow_compact_h_[idx_hess] = it.row();
      jCol_compact_h_[idx_hess] = it.col();
      idx_hess += 1;
    }
  }
}

// Return the optimization results
void quadNLP::finalize_solution(SolverReturn status, Index n, const Number *x,
                                const Number *z_L, const Number *z_U, Index m,
                                const Number *g, const Number *lambda,
                                Number obj_value, const IpoptData *ip_data,
                                IpoptCalculatedQuantities *ip_cq) {
  Eigen::Map<const Eigen::VectorXd> w(x, n);
  w0_ = w;

  Eigen::Map<const Eigen::VectorXd> z_L_matrix(z_L, n);
  Eigen::Map<const Eigen::VectorXd> z_U_matrix(z_L, n);
  z_L0_ = z_L_matrix;
  z_U0_ = z_U_matrix;

  Eigen::Map<const Eigen::VectorXd> lambda_matrix(lambda, m);
  lambda0_ = lambda_matrix;
}

void quadNLP::shift_initial_guess() {
  // Shift decision variables for 1 time step
  int curr_var_idx = 0;
  int next_var_idx = m_ + n_vec_[0];
  int curr_constr_idx = 0;
  int curr_slack_idx = n_vars_primal_;
  int n_slack = n_;
  for (size_t i = 0; i < N_ - 1; i++) {
    w0_.segment(curr_var_idx, m_ + n_vec_[i]) =
        w0_.segment(next_var_idx, m_ + n_vec_[i + 1]);
    z_L0_.segment(curr_var_idx, m_ + n_vec_[i]) =
        z_L0_.segment(next_var_idx, m_ + n_vec_[i + 1]);
    z_U0_.segment(curr_var_idx, m_ + n_vec_[i]) =
        z_U0_.segment(next_var_idx, m_ + n_vec_[i + 1]);
    lambda0_.segment(curr_constr_idx, g_vec_[i]) =
        lambda0_.segment(curr_constr_idx + g_vec_[i], g_vec_[i + 1]);
    curr_var_idx = next_var_idx;
    next_var_idx += m_ + n_vec_[i];
    curr_constr_idx += g_vec_[i];

    w0_.segment(curr_slack_idx, 2 * n_slack) =
        w0_.segment(curr_slack_idx + 2 * n_slack, 2 * n_slack);
    z_L0_.segment(curr_slack_idx, 2 * n_slack) =
        z_L0_.segment(curr_slack_idx + 2 * n_slack, 2 * n_slack);
    z_U0_.segment(curr_slack_idx, 2 * n_slack) =
        z_U0_.segment(curr_slack_idx + 2 * n_slack, 2 * n_slack);
    lambda0_.segment(curr_constr_idx, 2 * n_slack) =
        lambda0_.segment(curr_constr_idx + 2 * n_slack, 2 * n_slack);
    curr_slack_idx += 2 * n_slack;
    curr_constr_idx += 2 * n_slack;
  }

  // New contact
  if (!(contact_sequence_.col(N_ - 1) - contact_sequence_.col(N_ - 2))
           .isMuchSmallerThan(1e-3)) {
    // There's a dual pair
    if ((Eigen::MatrixXi::Ones(4, 1) - (contact_sequence_.col(N_ - 1)) -
         contact_sequence_.col(N_ - 2))
            .isMuchSmallerThan(1e-3)) {
      Eigen::MatrixXd trans = Eigen::MatrixXd::Zero(12, 12);
      trans.block(0, 3, 3, 3).diagonal() << 1, 1, 1;
      trans.block(3, 0, 3, 3).diagonal() << 1, 1, 1;
      trans.block(6, 9, 3, 3).diagonal() << 1, 1, 1;
      trans.block(9, 6, 3, 3).diagonal() << 1, 1, 1;

      w0_.block((N_ - 1) * (n_ + m_) + leg_input_start_idx_, 0,
                m_ - leg_input_start_idx_, 1) =
          trans * w0_.block((N_ - 7) * (n_ + m_) + leg_input_start_idx_, 0,
                            m_ - leg_input_start_idx_, 1);
      z_L0_.block((N_ - 1) * (n_ + m_) + leg_input_start_idx_, 0,
                  m_ - leg_input_start_idx_, 1) =
          trans * z_L0_.block((N_ - 7) * (n_ + m_) + leg_input_start_idx_, 0,
                              m_ - leg_input_start_idx_, 1);
      z_U0_.block((N_ - 1) * (n_ + m_) + leg_input_start_idx_, 0,
                  m_ - leg_input_start_idx_, 1) =
          trans * z_U0_.block((N_ - 7) * (n_ + m_) + leg_input_start_idx_, 0,
                              m_ - leg_input_start_idx_, 1);
    }
    // New contact mode
    else {
      w0_.block((N_ - 1) * (n_ + m_) + leg_input_start_idx_, 0,
                m_ - leg_input_start_idx_, 1)
          .fill(0);
      z_L0_
          .block((N_ - 1) * (n_ + m_) + leg_input_start_idx_, 0,
                 m_ - leg_input_start_idx_, 1)
          .fill(1);
      z_U0_
          .block((N_ - 1) * (n_ + m_) + leg_input_start_idx_, 0,
                 m_ - leg_input_start_idx_, 1)
          .fill(1);

      // If there're legs on the ground, we distribute gravity evenly
      if (contact_sequence_.col(N_ - 1).sum() > 0.5) {
        double grf = 11.51 * 9.81 / contact_sequence_.col(N_ - 1).sum();
        for (size_t i = 0; i < 4; i++) {
          if (contact_sequence_(i, N_ - 1) == 1) {
            w0_((N_ - 1) * (n_ + m_) + leg_input_start_idx_ + 3 * i + 2, 0) =
                grf;
          }
        }
      }
    }
  }
}

void quadNLP::update_solver(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule) {
  // Update foot positions
  // Local planner has row as N horizon and col as position
  feet_location_ = -foot_positions.transpose();

  // Update contact sequence
  // Local planner has outer as N and inner as boolean contact
  for (size_t i = 0; i < contact_schedule.size(); i++) {
    for (size_t j = 0; j < contact_schedule.front().size(); j++) {
      if (bool(contact_schedule.at(i).at(j))) {
        contact_sequence_(j, i) = 1;
      } else {
        contact_sequence_(j, i) = 0;
      }
    }
  }

  // Update initial states
  x_current_ = initial_state.transpose();

  // Update reference trajectory
  // Local planner has row as N+1 horizon and col as states
  x_reference_ = ref_traj.transpose();

  // Reset the state weighting factors
  Q_factor_ = Q_factor_base_;
  R_factor_ = R_factor_base_;
}

void quadNLP::update_solver(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::MatrixXd &state_traj, const Eigen::MatrixXd &control_traj) {
  this->update_solver(initial_state, ref_traj, foot_positions,
                      contact_schedule);

  leg_input_ = control_traj.transpose();

  known_leg_input_ = true;
}

void quadNLP::update_solver(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXd &ground_height) {
  this->update_solver(initial_state, ref_traj, foot_positions,
                      contact_schedule);

  ground_height_ = ground_height.transpose();
}

void quadNLP::update_solver(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXd &ground_height,
    const double &first_element_duration) {
  first_element_duration_ = first_element_duration;

  this->update_solver(initial_state, ref_traj, foot_positions, contact_schedule,
                      ground_height);
}

void quadNLP::update_complexity_schedule(
    const Eigen::VectorXi &complexity_schedule) {
  // Update the complexity schedule
  this->complexity_schedule_ = complexity_schedule;

  // Resize vectors appropriately
  N_ = complexity_schedule.size();
  sys_id_schedule_.resize(N_);
  g_vec_.resize(N_);
  n_vec_.resize(N_);
  fe_idxs_.resize(N_);
  u_idxs_.resize(N_);
  x_idxs_.resize(N_);
  slack_idxs_.resize(N_);
  g_idxs_.resize(N_);
  g_slack_idxs_.resize(N_);
  jac_sparse_idxs_.resize(N_);
  hess_sparse_idxs_.resize(N_);

  // Initialize indexing veriables
  num_complex_fe_ = 0;
  int curr_var_idx = 0;
  int curr_constr_idx = 0;
  int curr_j_sparse_idx = 0;
  int curr_h_sparse_idx = 0;

  // Loop through the horizon
  for (int i = 0; i < N_; i++) {

    // Update the number of complex elements
    num_complex_fe_ += complexity_schedule[i];

    // Update the complexity of the prior finite element
    int prior_complexity =
        (i > 0) ? complexity_schedule[i - 1] : x0_complexity_;

    // Determine the system to assign to this finite element
    if (complexity_schedule[i] == 0) {
      sys_id_schedule_[i] = (prior_complexity == 0) ? LEG : COMPLEX_TO_SIMPLE;
    } else {
      sys_id_schedule_[i] =
          (prior_complexity == 1) ? COMPLEX : SIMPLE_TO_COMPLEX;
    }

    // Update the number of state vars and constraints for this FE
    n_vec_[i] = (complexity_schedule[i] == 1) ? n_complex_ : n_simple_;
    g_vec_[i] = nrow_mat_(sys_id_schedule_[i], FUNC);

    // Update the indices for the finite element, control, and state variables
    fe_idxs_[i] = curr_var_idx;
    u_idxs_[i] = curr_var_idx;
    curr_var_idx += m_;
    x_idxs_[i] = curr_var_idx;
    curr_var_idx += n_vec_[i];

    // Update the indices for the constraints
    g_idxs_[i] = curr_constr_idx;
    curr_constr_idx += g_vec_[i];

    // Compute the offsets for the first step
    int j_first_step_idx =
        (i == 0) ? first_step_idx_mat_(sys_id_schedule_[i], JAC) : 0;
    int h_first_step_idx =
        (i == 0) ? first_step_idx_mat_(sys_id_schedule_[i], HESS) : 0;

    // Update the indices for the sparsity patterns
    jac_sparse_idxs_[i] = curr_j_sparse_idx;
    curr_j_sparse_idx += nnz_mat_(sys_id_schedule_[i], JAC) - j_first_step_idx;
    hess_sparse_idxs_[i] = curr_h_sparse_idx;
    curr_h_sparse_idx += nnz_mat_(sys_id_schedule_[i], HESS) - h_first_step_idx;
  }

  // Update the total number of primal variables
  n_vars_primal_ = curr_var_idx;

  // Update the slack variable indices
  int n_slack = n_simple_;
  for (int i = 0; i < N_; i++) {
    slack_idxs_[i] = curr_var_idx;
    curr_var_idx += 2 * n_slack;

    g_slack_idxs_[i] = curr_constr_idx;
    curr_constr_idx += 2 * n_slack;
  }

  // Update the total number of vars and slack vars
  n_vars_ = curr_var_idx;
  n_vars_slack_ = n_vars_ - n_vars_primal_;

  // Check the number of primal and slack vars for correctness
  if (n_vars_primal_ != (m_ * N_ + n_simple_ * (N_ - num_complex_fe_) +
                         n_complex_ * num_complex_fe_)) {
    throw std::runtime_error("Number of primal vars is inconsistent");
  } else if (n_vars_slack_ != (2 * n_slack * N_)) {
    throw std::runtime_error("Number of slack vars is inconsistent");
  }

  // Update the number of constraints
  n_constraints_ = curr_constr_idx;

  // Check the number of constraints
  if (n_constraints_ !=
      ((g_simple_ * (N_ - num_complex_fe_) + g_complex_ * num_complex_fe_) +
       n_vars_slack_)) {
    throw std::runtime_error("Number of constraints is inconsistent");
  }
}