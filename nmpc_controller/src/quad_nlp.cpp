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
                 Eigen::VectorXd u_max,
                 Eigen::VectorXi fixed_complexity_schedule)
// N: prediction steps
// n: states dimension
// m: input dimension
// with_tail: system includes tail or not
// Q: states cost weights
// R: input cost weights
// dt: time step length
// panic_weights: penalty on panic variables
{  // NOLINT
  type_ = type;

  N_ = N;
  n_ = n;
  n_null_ = n_null;
  m_ = m;
  g_ = n_ + 16;  // states dynamics plus linear friciton cone

  // Load adaptive complexity parameters
  n_simple_ = n;
  n_complex_ = n + n_null_;
  m_simple_ = m;
  g_simple_ = g_;

  // Added constraints include all simple constraints plus:
  //    n_null_ constraints for foot position and velocities
  //    n_null_ constraints for pos and neg joint torques
  //    4 constraints for knee position
  g_complex_ = g_ + 2 * n_null_ + 4;

  loadCasadiFuncs();

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
  foot_pos_world_ = Eigen::MatrixXd(N_, 12);
  foot_vel_world_ = Eigen::MatrixXd(N_, 12).setZero();
  for (int i = 0; i < N_; ++i) {
    feet_location_.block(0, i, 12, 1) << -0.2263, -0.098, 0.3, -0.2263, 0.098,
        0.3, 0.2263, -0.098, 0.3, 0.2263, 0.098, 0.3;
    foot_pos_world_.row(i) << -0.2263, -0.098, 0, -0.2263, 0.098, 0, 0.2263,
        -0.098, 0, 0.2263, 0.098, 0;
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

  double abad_nom = -0.248694;
  double hip_nom = 0.760144;
  double knee_nom = 1.52029;
  x_null_nom_.resize(n_null_);
  x_null_nom_ << abad_nom, hip_nom, knee_nom, abad_nom, hip_nom, knee_nom,
      abad_nom, hip_nom, knee_nom, abad_nom, hip_nom, knee_nom, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0;

  u_min_ = u_min;
  u_max_ = u_max;

  // Initialize simple constraint bounds
  g_min_.resize(g_);
  g_max_.resize(g_);

  // Load dynamics constraint bounds
  g_min_.segment(0, n_simple_).fill(0);
  g_max_.segment(0, n_simple_).fill(0);

  // Load friction cone constraint bounds
  g_min_.segment(n_simple_, 16).fill(-2e19);
  g_max_.segment(n_simple_, 16).fill(0);
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
  // TODO(jcnorby) : Add these back in
  g_min_complex_.segment(current_idx, n_null_).fill(0);
  g_max_complex_.segment(current_idx, n_null_).fill(0);
  // g_min_complex_.segment(current_idx, n_null_).fill(-2e19);
  // g_max_complex_.segment(current_idx, n_null_).fill(2e19);
  current_idx += n_null_;

  // Load knee constraint bounds
  g_min_complex_.segment(current_idx, 4).fill(0);
  g_max_complex_.segment(current_idx, 4).fill(10);
  // g_min_complex_.segment(current_idx, 4).fill(-2e19);
  // g_max_complex_.segment(current_idx, 4).fill(2e19);
  current_idx += 4;

  // Load motor model constraint bounds
  g_min_complex_.segment(current_idx, n_null_).fill(-2e19);
  g_max_complex_.segment(current_idx, n_null_).fill(0);
  // g_max_complex_.segment(current_idx, n_null_).fill(2e19);

  this->fixed_complexity_schedule_ = fixed_complexity_schedule;
  this->adaptive_complexity_schedule_.setZero(N_ + 1);
  this->update_structure();

  w0_ = Eigen::VectorXd(n_vars_).setZero();
  z_L0_ = Eigen::VectorXd(n_vars_).Ones(n_vars_, 1);
  z_U0_ = Eigen::VectorXd(n_vars_).Ones(n_vars_, 1);
  lambda0_ = Eigen::VectorXd(n_constraints_).setZero();
  g0_ = Eigen::VectorXd(n_constraints_).setZero();

  mu0_ = 1e-1;
  warm_start_ = false;

  for (size_t i = 0; i < N_; i++) {
    for (size_t j = 0; j < 4; j++) {
      get_control_var(w0_, i)(leg_input_start_idx_ + 2 + j * 3, 0) =
          mass_ * grav_ / 4;
    }
  }

  x_reference_ = Eigen::MatrixXd(n_simple_, N_ + 1);
  x_reference_.fill(0);

  ground_height_ = Eigen::VectorXd(N_ + 1);
  ground_height_.fill(-2e19);

  x_current_ = Eigen::VectorXd(n_vec_[0]);
  x_current_.fill(0);

  contact_sequence_ = Eigen::MatrixXi(4, N_);
  contact_sequence_.fill(1);

  if (type_ == DISTRIBUTED) {
    known_leg_input_ = true;

    leg_input_ = Eigen::MatrixXd(m_ - leg_input_start_idx_, N_);
    leg_input_.setZero();
    for (size_t i = 0; i < N_; i++) {
      for (size_t j = 0; j < 4; j++) {
        leg_input_(j * 3 + 2, i) = mass_ * grav_ / 4;
      }
    }
  } else {
    known_leg_input_ = false;
  }

  // Initialize the time duration to the next plan index as dt
  first_element_duration_ = dt_;

  compute_nnz_jac_g();
  compute_nnz_h();
}

quadNLP::quadNLP(const quadNLP &nlp) {
  w0_ = nlp.w0_;
  z_L0_ = nlp.z_L0_;
  z_U0_ = nlp.z_U0_;
  lambda0_ = nlp.lambda0_;
  g0_ = nlp.g0_;
  n_vec_ = nlp.n_vec_;
  g_vec_ = nlp.g_vec_;
  fe_idxs_ = nlp.fe_idxs_;
  x_idxs_ = nlp.x_idxs_;
  u_idxs_ = nlp.u_idxs_;
  m_ = nlp.m_;
  g_idxs_ = nlp.g_idxs_;
  slack_idxs_ = nlp.slack_idxs_;
  g_slack_idxs_ = nlp.g_slack_idxs_;
  n_slack_vec_ = nlp.n_slack_vec_;
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

  // States bound
  get_state_var(x_l_matrix, 0) = x_current_;
  get_state_var(x_u_matrix, 0) = x_current_;

  for (int i = 0; i < N_; ++i) {
    // Inputs bound
    get_control_var(x_l_matrix, i) = u_min_;
    get_control_var(x_u_matrix, i) = u_max_;

    if (known_leg_input_) {
      get_control_var(x_l_matrix, i)
          .segment(leg_input_start_idx_, m_ - leg_input_start_idx_) =
          leg_input_.block(0, i, m_ - leg_input_start_idx_, 1);
      get_control_var(x_u_matrix, i)
          .segment(leg_input_start_idx_, m_ - leg_input_start_idx_) =
          leg_input_.block(0, i, m_ - leg_input_start_idx_, 1);
    }

    // Contact sequence
    for (int j = 0; j < 4; ++j) {
      get_control_var(x_l_matrix, i).segment(leg_input_start_idx_ + 3 * j, 3) =
          (get_control_var(x_l_matrix, i)
               .segment(leg_input_start_idx_ + 3 * j, 3)
               .array() *
           contact_sequence_(j, i))
              .matrix();
      get_control_var(x_u_matrix, i).segment(leg_input_start_idx_ + 3 * j, 3) =
          (get_control_var(x_u_matrix, i)
               .segment(leg_input_start_idx_ + 3 * j, 3)
               .array() *
           contact_sequence_(j, i))
              .matrix();
    }

    // States bound
    get_state_var(x_l_matrix, i + 1).fill(-2e19);
    get_state_var(x_u_matrix, i + 1).fill(2e19);

    // Add bounds if not covered by panic variables
    if (n_vec_[i + 1] > n_slack_vec_[i]) {
      get_state_var(x_l_matrix, i + 1).tail(n_null_) =
          x_min_complex_.tail(n_null_);
      get_state_var(x_u_matrix, i + 1).tail(n_null_) =
          x_max_complex_.tail(n_null_);
    }

    // Constraints bound
    get_constraint_var(g_l_matrix, i) = g_min_complex_.head(g_vec_[i]);
    get_constraint_var(g_u_matrix, i) = g_max_complex_.head(g_vec_[i]);

    // Panic variable bound
    get_panic_var(x_l_matrix, i).fill(0);
    get_panic_var(x_u_matrix, i).fill(2e19);
  }

  for (size_t i = 0; i < N_; i++) {
    // xmin
    get_panic_constraint_var(g_l_matrix, i).head(n_slack_vec_[i]) =
        x_min_complex_.head(n_slack_vec_[i]);
    get_panic_constraint_var(g_l_matrix, i)(2, 0) = ground_height_(0, i);

    // get_panic_constraint_var(g_l_matrix, i)(2, 0) = 0;
    get_panic_constraint_var(g_u_matrix, i).head(n_slack_vec_[i]).fill(2e19);

    // xmax
    get_panic_constraint_var(g_l_matrix, i).tail(n_slack_vec_[i]).fill(-2e19);
    get_panic_constraint_var(g_u_matrix, i).tail(n_slack_vec_[i]) =
        x_max_complex_.head(n_slack_vec_[i]);
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
    Eigen::Map<Eigen::VectorXd> z_U_matrix(z_U, n);
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

  for (int i = 0; i < N_; ++i) {
    // Compute the number of contacts
    Eigen::VectorXd u_nom(m_);
    u_nom.setZero();
    double num_contacts = contact_sequence_.col(i).sum();

    // If there are some contacts, set the nominal input accordingly
    if (num_contacts > 0) {
      for (int j = 0; j < contact_sequence_.rows(); j++) {
        if (contact_sequence_(j, i) == 1) {
          u_nom[3 * j + 2 + leg_input_start_idx_] =
              mass_ * grav_ / num_contacts;
        }
      }
    }

    Eigen::MatrixXd uk = get_control_var(w, i) - u_nom;
    Eigen::MatrixXd xk = get_state_var(w, i + 1).head(n_simple_) -
                         x_reference_.block(0, i + 1, n_simple_, 1);

    Eigen::MatrixXd Q_i = Q_ * Q_factor_(i, 0);
    Eigen::MatrixXd R_i = R_ * R_factor_(i, 0);

    // Scale the cost by time duration
    if (i == 0) {
      Q_i = Q_i * first_element_duration_ / dt_;
      R_i = R_i * first_element_duration_ / dt_;
    }

    obj_value += (xk.transpose() * Q_i.asDiagonal() * xk / 2 +
                  uk.transpose() * R_i.asDiagonal() * uk / 2)(0, 0);
    obj_value += panic_weights_ * get_panic_var(w, i).sum();
  }

  return true;
}

// Return the gradient of the objective function
bool quadNLP::eval_grad_f(Index n, const Number *x, bool new_x,
                          Number *grad_f) {
  Eigen::Map<const Eigen::VectorXd> w(x, n);

  Eigen::Map<Eigen::VectorXd> grad_f_matrix(grad_f, n);
  grad_f_matrix.setZero();

  get_state_var(grad_f_matrix, 0).fill(0);

  for (int i = 0; i < N_; ++i) {
    // Compute the number of contacts
    Eigen::VectorXd u_nom(m_);
    u_nom.setZero();
    double num_contacts = contact_sequence_.col(i).sum();

    // If there are some contacts, set the nominal input accordingly
    if (num_contacts > 0) {
      for (int j = 0; j < contact_sequence_.rows(); j++) {
        if (contact_sequence_(j, i) == 1) {
          u_nom[3 * j + 2 + leg_input_start_idx_] =
              mass_ * grav_ / num_contacts;
        }
      }
    }

    Eigen::MatrixXd uk = get_control_var(w, i) - u_nom;
    Eigen::MatrixXd xk = get_state_var(w, i + 1).head(n_simple_) -
                         x_reference_.block(0, i + 1, n_simple_, 1);

    Eigen::MatrixXd Q_i = Q_ * Q_factor_(i, 0);
    Eigen::MatrixXd R_i = R_ * R_factor_(i, 0);

    // Scale the cost by time duration
    if (i == 0) {
      Q_i = Q_i * first_element_duration_ / dt_;
      R_i = R_i * first_element_duration_ / dt_;
    }

    get_control_var(grad_f_matrix, i) = R_i.asDiagonal() * uk;
    get_state_var(grad_f_matrix, i + 1).head(n_simple_) = Q_i.asDiagonal() * xk;
    get_panic_var(grad_f_matrix, i).fill(panic_weights_);
  }

  return true;
}

// Return the value of the constraints
bool quadNLP::eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) {
  Eigen::Map<const Eigen::VectorXd> w(x, n);
  Eigen::Map<Eigen::VectorXd> g_matrix(g, m);
  g_matrix.setZero();

  // std::cout << "g_matrix.sum() before = " << g_matrix.sum() << std::endl;
  // std::cout << "n = " << n << std::endl;
  // std::cout << "m = " << m << std::endl;

  for (int i = 0; i < N_; ++i) {
    // Select the system ID
    int sys_id = sys_id_schedule_[i];

    // Load the params for this FE
    Eigen::VectorXd pk(26);
    pk[0] = (i == 0) ? first_element_duration_ : dt_;
    pk[1] = mu_;
    // pk.segment(2, 12) = feet_location_.block(0, i, 12, 1);
    pk.segment(2, 12) = foot_pos_world_.row(i);
    pk.segment(14, 12) = foot_vel_world_.row(i);

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

    arg[0] = get_dynamic_var(w, i).data();

    arg[1] = pk.data();
    res[0] = get_constraint_var(g_matrix, i).data();

    int mem = eval_checkout_vec_[sys_id][FUNC]();
    eval_vec_[sys_id][FUNC](arg, res, iw, _w, mem);
    eval_release_vec_[sys_id][FUNC](mem);
    eval_decref_vec_[sys_id][FUNC]();

    // if (complexity_schedule_[i + 1] == 1) {
    //   std::cout << "\nget_dynamic_var(w, i) = "
    //             << get_dynamic_var(w, i).transpose() << std::endl;
    //   std::cout << "EOM constraint = "
    //             << get_constraint_var(g_matrix, i).segment(0, 12).transpose()
    //             << std::endl;
    //   std::cout << "Friction constraint = "
    //             << get_constraint_var(g_matrix, i).segment(12,
    //             16).transpose()
    //             << std::endl;
    //   std::cout << "Foot position constraint = "
    //             << get_constraint_var(g_matrix, i).segment(28,
    //             12).transpose()
    //             << std::endl;
    //   std::cout << "Foot velocity constraint = "
    //             << get_constraint_var(g_matrix, i).segment(40,
    //             12).transpose()
    //             << std::endl;
    //   std::cout << "Knee height constraint = "
    //             << get_constraint_var(g_matrix, i).segment(52, 4).transpose()
    //             << std::endl;
    //   std::cout << "Motor model pos constraint = "
    //             << get_constraint_var(g_matrix, i).segment(56,
    //             12).transpose()
    //             << std::endl;
    //   std::cout << "Motor model neg constraint = "
    //             << get_constraint_var(g_matrix, i).segment(68,
    //             12).transpose()
    //             << std::endl
    //             << std::endl;

    //   Eigen::Vector3d body_pos = get_state_var(w, i + 1).segment(0, 3);
    //   Eigen::Vector3d body_rpy = get_state_var(w, i + 1).segment(3, 6);
    //   Eigen::Vector3d joint_state = get_state_var(w, i + 1).segment(12, 3);
    //   Eigen::Vector3d foot_pos_world;
    //   quad_utils::QuadKD quad_kd;
    //   quad_kd.worldToFootFKWorldFrame(0, body_pos, body_rpy, joint_state,
    //                                   foot_pos_world);
    //   std::cout << "body_pos = " << body_pos.transpose() << std::endl;
    //   std::cout << "body_rpy = " << body_rpy.transpose() << std::endl;
    //   std::cout << "joint_state = " << joint_state.transpose() << std::endl;
    //   std::cout << "foot_pos_world = " << foot_pos_world.transpose()
    //             << std::endl;
    //   std::cout << "foot_pos_world target = " << pk.segment(2, 3).transpose()
    //             << std::endl;
    //   std::cout << "constraint val = "
    //             << (foot_pos_world - pk.segment(2, 3)).transpose() <<
    //             std::endl;
    // }
  }

  for (int i = 0; i < N_; ++i) {
    Eigen::VectorXd xk = get_state_var(w, i + 1).head(n_slack_vec_[i]);
    Eigen::VectorXd panick = get_panic_var(w, i);

    get_panic_constraint_var(g_matrix, i).head(n_slack_vec_[i]) =
        xk + panick.head(n_slack_vec_[i]);
    get_panic_constraint_var(g_matrix, i).tail(n_slack_vec_[i]) =
        xk - panick.tail(n_slack_vec_[i]);

    // std::cout << "i = " << i << std::endl;
    // std::cout << "n_slack_vec_[i] = " << n_slack_vec_[i] << std::endl;
    // std::cout << "get_state_var(w, i + 1) = "
    //           << get_state_var(w, i + 1).transpose() << std::endl;
    // std::cout << "xk = " << xk.transpose() << std::endl;
    // std::cout << "panick = " << panick.transpose() << std::endl;
    // std::cout << "get_panic_constraint_var(g_matrix, i) = "
    //           << get_panic_constraint_var(g_matrix, i).transpose() <<
    //           std::endl;
  }

  // std::cout << "g_matrix max = " << g_matrix.maxCoeff() << std::endl;
  // std::cout << "g_matrix min = " << g_matrix.minCoeff() << std::endl;

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
    values_matrix.setZero();

    for (int i = 0; i < N_; ++i) {
      // Select the system ID
      int sys_id = sys_id_schedule_[i];

      // Load the params for this FE
      Eigen::VectorXd pk(26);
      pk[0] = (i == 0) ? first_element_duration_ : dt_;
      pk[1] = mu_;
      // pk.segment(2, 12) = feet_location_.block(0, i, 12, 1);
      pk.segment(2, 12) = foot_pos_world_.row(i);
      pk.segment(14, 12) = foot_vel_world_.row(i);

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

      int mem;
      arg[0] = get_dynamic_var(w, i).data();
      arg[1] = pk.data();
      res[0] = get_dynamic_jac_var(values_matrix, i).data();

      mem = eval_checkout_vec_[sys_id][JAC]();
      eval_vec_[sys_id][JAC](arg, res, iw, _w, mem);

      eval_release_vec_[sys_id][JAC](mem);
      eval_decref_vec_[sys_id][JAC]();
    }

    for (size_t i = 0; i < N_; i++) {
      // xmin wrt x
      get_panic_jac_var(values_matrix, i).segment(0, n_slack_vec_[i]).fill(1);

      // xmin wrt panic
      get_panic_jac_var(values_matrix, i)
          .segment(n_slack_vec_[i], n_slack_vec_[i])
          .fill(1);

      // xmax wrt x
      get_panic_jac_var(values_matrix, i)
          .block(2 * n_slack_vec_[i], 0, n_slack_vec_[i], 1)
          .fill(1);

      // xmax wrt panic
      get_panic_jac_var(values_matrix, i)
          .block(3 * n_slack_vec_[i], 0, n_slack_vec_[i], 1)
          .fill(-1);
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

  // Add slack variables
  nnz_jac_g_ += 2 * n_vars_slack_;
  nnz_step_jac_g_.resize(N_);

  // Size the NLP constraint Jacobian sparsity structure
  iRow_jac_g_ = Eigen::VectorXi(nnz_jac_g_);
  jCol_jac_g_ = Eigen::VectorXi(nnz_jac_g_);

  for (int i = 0; i < N_; ++i) {
    int sys_id = sys_id_schedule_[i];
    nnz_step_jac_g_[i] = nnz_mat_(sys_id, JAC);
    // Each step we shift g constraints
    get_dynamic_jac_var(iRow_jac_g_, i) =
        (iRow_mat_[sys_id][JAC].array() + getPrimalConstraintFEIndex(i))
            .matrix();

    // Each step we shift n states and m inputs
    get_dynamic_jac_var(jCol_jac_g_, i) =
        (jCol_mat_[sys_id][JAC].array() + getPrimalFEIndex(i)).matrix();
  }

  for (size_t i = 0; i < N_; i++) {
    // Declare number of slack variables for this element
    int n_slack = n_slack_vec_[i];

    Eigen::ArrayXi g_xmin_idx =
        Eigen::ArrayXi::LinSpaced(n_slack, getSlackConstraintFEIndex(i),
                                  getSlackConstraintFEIndex(i) + n_slack - 1);
    Eigen::ArrayXi g_xmax_idx = Eigen::ArrayXi::LinSpaced(
        n_slack, getSlackConstraintFEIndex(i) + n_slack,
        getSlackConstraintFEIndex(i) + 2 * n_slack - 1);
    Eigen::ArrayXi x_idx =
        Eigen::ArrayXi::LinSpaced(n_slack, getPrimalStateFEIndex(i + 1),
                                  getPrimalStateFEIndex(i + 1) + n_slack - 1);
    Eigen::ArrayXi panic_xmin_idx =
        Eigen::ArrayXi::LinSpaced(n_slack, getSlackStateFEIndex(i),
                                  getSlackStateFEIndex(i) + n_slack - 1);
    Eigen::ArrayXi panic_xmax_idx =
        Eigen::ArrayXi::LinSpaced(n_slack, getSlackStateFEIndex(i) + n_slack,
                                  getSlackStateFEIndex(i) + 2 * n_slack - 1);

    // xmin wrt x
    get_panic_jac_var(iRow_jac_g_, i).segment(0, n_slack) = g_xmin_idx;
    get_panic_jac_var(jCol_jac_g_, i).segment(0, n_slack) = x_idx;

    // xmin wrt panic
    get_panic_jac_var(iRow_jac_g_, i).segment(n_slack, n_slack) = g_xmin_idx;
    get_panic_jac_var(jCol_jac_g_, i).segment(n_slack, n_slack) =
        panic_xmin_idx;

    // xmax wrt x
    get_panic_jac_var(iRow_jac_g_, i).segment(2 * n_slack, n_slack) =
        g_xmax_idx;
    get_panic_jac_var(jCol_jac_g_, i).segment(2 * n_slack, n_slack) = x_idx;

    // xmax wrt panic
    get_panic_jac_var(iRow_jac_g_, i).segment(3 * n_slack, n_slack) =
        g_xmax_idx;
    get_panic_jac_var(jCol_jac_g_, i).segment(3 * n_slack, n_slack) =
        panic_xmax_idx;
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
    values_compact_matrix.setZero();
    Eigen::Map<const Eigen::VectorXd> lambda_matrix(lambda, m);
    Eigen::VectorXd values_matrix(nnz_h_);
    values_matrix.setZero();

    for (int i = 0; i < N_; ++i) {
      // Select the system ID
      int sys_id = sys_id_schedule_[i];

      // Load the params for this FE
      Eigen::VectorXd pk(26);
      pk[0] = (i == 0) ? first_element_duration_ : dt_;
      pk[1] = mu_;
      // pk.segment(2, 12) = feet_location_.block(0, i, 12, 1);
      pk.segment(2, 12) = foot_pos_world_.row(i);
      pk.segment(14, 12) = foot_vel_world_.row(i);

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
      arg[0] = get_dynamic_var(w, i).data();
      arg[1] = get_constraint_var(lambda_matrix, i).data();
      arg[2] = pk.data();
      res[0] = get_dynamic_hess_var(values_matrix, i).data();

      mem = eval_checkout_vec_[sys_id][HESS]();
      eval_vec_[sys_id][HESS](arg, res, iw, _w, mem);

      // Release memory for function
      eval_release_vec_[sys_id][HESS](mem);
      eval_decref_vec_[sys_id][HESS]();
    }
    // Initialize Q and R weights
    // Hessian from cost
    for (size_t i = 0; i < N_; i++) {
      Eigen::MatrixXd Q_i = Q_ * Q_factor_(i, 0);
      Eigen::MatrixXd R_i = R_ * R_factor_(i, 0);

      // Scale the cost by time duration
      if (i == 0) {
        Q_i = Q_i * first_element_duration_ / dt_;
        R_i = R_i * first_element_duration_ / dt_;
      }

      get_control_cost_hess_var(values_matrix, i) =
          (obj_factor * R_i.array()).matrix();
      get_state_cost_hess_var(values_matrix, i) =
          (obj_factor * Q_i.array()).matrix();
    }

    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(nnz_h_);
    for (int i = 0; i < nnz_h_; ++i) {
      tripletList.push_back(
          Eigen::Triplet<double>(iRow_h_[i], jCol_h_[i], values_matrix[i]));
    }
    Eigen::SparseMatrix<double> hess(n, n);
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

  // Add cost variables (only on simple model coordinates)
  nnz_h_ = nnz_h_ + N_ * (n_simple_ + m_);

  iRow_h_ = Eigen::VectorXi(nnz_h_);
  jCol_h_ = Eigen::VectorXi(nnz_h_);
  nnz_step_hess_.resize(N_);

  for (int i = 0; i < N_; ++i) {
    int sys_id = sys_id_schedule_[i];
    nnz_step_hess_[i] = nnz_mat_(sys_id, HESS);
    // Each step we shift n states and m inputs
    get_dynamic_hess_var(iRow_h_, i) =
        (iRow_mat_[sys_id][HESS].array() + getPrimalFEIndex(i)).matrix();
    get_dynamic_hess_var(jCol_h_, i) =
        (jCol_mat_[sys_id][HESS].array() + getPrimalFEIndex(i)).matrix();
  }

  // Hessian from cost
  for (size_t i = 0; i < N_; i++) {
    Eigen::ArrayXi iRow_control_cost = Eigen::ArrayXi::LinSpaced(
        m_, getPrimalControlFEIndex(i), getPrimalControlFEIndex(i) + m_ - 1);
    Eigen::ArrayXi jCol_control_cost = iRow_control_cost;
    Eigen::ArrayXi iRow_state_cost =
        Eigen::ArrayXi::LinSpaced(n_simple_, getPrimalStateFEIndex(i + 1),
                                  getPrimalStateFEIndex(i + 1) + n_simple_ - 1);
    Eigen::ArrayXi jCol_state_cost = iRow_state_cost;

    get_control_cost_hess_var(iRow_h_, i) = iRow_control_cost.matrix();
    get_control_cost_hess_var(jCol_h_, i) = jCol_control_cost.matrix();

    get_state_cost_hess_var(iRow_h_, i) = iRow_state_cost.matrix();
    get_state_cost_hess_var(jCol_h_, i) = jCol_state_cost.matrix();
  }

  std::vector<Eigen::Triplet<double>> tripletList;
  tripletList.reserve(nnz_h_);
  for (int i = 0; i < nnz_h_; ++i) {
    tripletList.push_back(Eigen::Triplet<double>(iRow_h_[i], jCol_h_[i], 1));
  }
  Eigen::SparseMatrix<double> hess(n_vars_, n_vars_);
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
  Eigen::Map<const Eigen::VectorXd> z_U_matrix(z_U, n);
  z_L0_ = z_L_matrix;
  z_U0_ = z_U_matrix;

  Eigen::Map<const Eigen::VectorXd> lambda_matrix(lambda, m);
  lambda0_ = lambda_matrix;

  Eigen::Map<const Eigen::VectorXd> g_matrix(g, m);
  g0_ = g_matrix;

  mu0_ = ip_data->curr_mu();
}

void quadNLP::update_initial_guess(const quadNLP &nlp_prev, int shift_idx) {
  // Shift decision variables for 1 time step
  // quad_utils::FunctionTimer timer("shift_initial_guess");

  // if (shift_idx > 0) {
  //   std::cout << "Updating initial guess because index shifted" << std::endl;
  // } else {
  //   std::cout << "Updating initial guess because structure changed"
  //             << std::endl;
  // }

  // std::cout << "w0_ before = \n" << w0_ << std::endl;
  // std::cout << "lambda0_ before = \n" << lambda0_ << std::endl;

  // Update the initial state
  get_state_var(w0_, 0) = x_current_;
  get_state_var(z_L0_, 0).fill(0);
  get_state_var(z_U0_, 0).fill(0);

  for (int i = 0; i < N_ - 1; i++) {
    int i_prev = std::min(i + shift_idx, N_ - 1);

    int n_shared = std::min(n_vec_[i + 1], nlp_prev.n_vec_[i_prev + 1]);
    int n_slack_shared =
        std::min(n_slack_vec_[i], nlp_prev.n_slack_vec_[i_prev]);
    int g_shared = std::min(g_vec_[i], nlp_prev.g_vec_[i_prev]);

    // if (n_vec_[i + 1] > n_shared) {
    //   std::cout << "Lifting state i+1 = " << i + 1 << std::endl;
    // }

    // Update state and control for w0
    get_state_var(w0_, i + 1).head(n_shared) =
        nlp_prev.get_state_var(nlp_prev.w0_, i_prev + 1).head(n_shared);
    get_control_var(w0_, i) = nlp_prev.get_control_var(nlp_prev.w0_, i_prev);

    // Update state and control for z_L0_
    get_state_var(z_L0_, i + 1).head(n_shared) =
        nlp_prev.get_state_var(nlp_prev.z_L0_, i_prev + 1).head(n_shared);
    get_control_var(z_L0_, i) =
        nlp_prev.get_control_var(nlp_prev.z_L0_, i_prev);

    // Update state and control for z_U0_
    get_state_var(z_U0_, i + 1).head(n_shared) =
        nlp_prev.get_state_var(nlp_prev.z_U0_, i_prev + 1).head(n_shared);
    get_control_var(z_U0_, i) =
        nlp_prev.get_control_var(nlp_prev.z_U0_, i_prev);

    // Udate constraint variables
    get_constraint_var(lambda0_, i).head(g_shared) =
        nlp_prev.get_constraint_var(nlp_prev.lambda0_, i_prev).head(g_shared);

    // Update panic variables
    get_panic_var(w0_, i).segment(0, n_slack_shared) =
        nlp_prev.get_panic_var(nlp_prev.w0_, i_prev)
            .head(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_panic_var(w0_, i).segment(n_slack_vec_[i], n_slack_shared) =
        nlp_prev.get_panic_var(nlp_prev.w0_, i_prev)
            .tail(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);

    get_panic_var(z_L0_, i).segment(0, n_slack_shared) =
        nlp_prev.get_panic_var(nlp_prev.z_L0_, i_prev)
            .head(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_panic_var(z_L0_, i).segment(n_slack_vec_[i], n_slack_shared) =
        nlp_prev.get_panic_var(nlp_prev.z_L0_, i_prev)
            .tail(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);

    get_panic_var(z_U0_, i).segment(0, n_slack_shared) =
        nlp_prev.get_panic_var(nlp_prev.z_U0_, i_prev)
            .head(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_panic_var(z_U0_, i).segment(n_slack_vec_[i], n_slack_shared) =
        nlp_prev.get_panic_var(nlp_prev.z_U0_, i_prev)
            .tail(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);

    get_panic_constraint_var(lambda0_, i).segment(0, n_slack_shared) =
        nlp_prev.get_panic_constraint_var(nlp_prev.lambda0_, i_prev)
            .head(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_panic_constraint_var(lambda0_, i)
        .segment(n_slack_vec_[i], n_slack_shared) =
        nlp_prev.get_panic_constraint_var(nlp_prev.lambda0_, i_prev)
            .tail(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);

    // If this element is newly lifted, update with nominal otherwise leave
    // unchanged (may be one timestep old)
    if (n_vec_[i] > n_shared && n_vec_[i] > nlp_prev.n_vec_[i]) {
      std::cout << "New complexity information!!!!! i = " << i << std::endl;

      get_state_var(w0_, i + 1).tail(n_null_) = x_null_nom_;
      get_state_var(z_L0_, i + 1).tail(n_null_).fill(1);
      get_state_var(z_U0_, i + 1).tail(n_null_).fill(1);
      get_constraint_var(lambda0_, i).tail(g_vec_[i] - g_shared).fill(1000);

      // Update panic variables if they also differ
      if (n_slack_vec_[i] > n_slack_shared &&
          n_slack_vec_[i] > nlp_prev.n_slack_vec_[i]) {
        get_panic_var(w0_, i).head(n_slack_vec_[i]).tail(n_null_).fill(0);
        get_panic_var(w0_, i).tail(n_slack_vec_[i]).tail(n_null_).fill(0);
        get_panic_var(z_L0_, i).head(n_slack_vec_[i]).tail(n_null_).fill(1);
        get_panic_var(z_L0_, i).tail(n_slack_vec_[i]).tail(n_null_).fill(1);
        get_panic_var(z_U0_, i).head(n_slack_vec_[i]).tail(n_null_).fill(1);
        get_panic_var(z_U0_, i).tail(n_slack_vec_[i]).tail(n_null_).fill(1);
        get_panic_constraint_var(lambda0_, i)
            .head(n_slack_vec_[i])
            .tail(n_null_)
            .fill(1000);
      }
    }
  }

  //   if (n_vec_[i] > nlp_prev.n_vec_[i_prev]) {
  //     // std::cout << "adding state information at i = " << i << std::endl;

  //     // Update state and control for w0
  //     get_state_var(w0_, i).head(nlp_prev.n_vec_[i_prev]) =
  //         nlp_prev.get_state_var(nlp_prev.w0_, i_prev);
  //     // get_state_var(w0_, i).tail(n_vec_[i] - nlp_prev.n_vec_[i_prev]) =
  //     //     x_null_nom_;
  //     get_control_var(w0_, i) = nlp_prev.get_control_var(nlp_prev.w0_,
  //     i_prev);

  //     // std::cout << "get_state_var(w0_, i)\n"
  //     //           << get_state_var(w0_, i) << std::endl
  //     //           << std::endl;
  //     // std::cout << "get_control_var(w0_, i)\n"
  //     //           << get_control_var(w0_, i) << std::endl
  //     //           << std::endl;

  //     // Update state and control for z_L0_
  //     get_state_var(z_L0_, i).head(nlp_prev.n_vec_[i_prev]) =
  //         nlp_prev.get_state_var(nlp_prev.z_L0_, i_prev);
  //     // get_state_var(z_L0_, i).tail(n_vec_[i] - nlp_prev.n_vec_[i +
  //     // 1]).fill(1);
  //     get_control_var(z_L0_, i) =
  //         nlp_prev.get_control_var(nlp_prev.z_L0_, i_prev);

  //     // std::cout << "get_state_var(z_L0_, i)\n"
  //     //           << get_state_var(z_L0_, i) << std::endl
  //     //           << std::endl;
  //     // std::cout << "get_control_var(z_L0_, i)\n"
  //     //           << get_control_var(z_L0_, i) << std::endl
  //     //           << std::endl;

  //     // Update state and control for z_U0_
  //     get_state_var(z_U0_, i).head(nlp_prev.n_vec_[i_prev]) =
  //         nlp_prev.get_state_var(nlp_prev.z_U0_, i_prev);
  //     // get_state_var(z_U0_, i).tail(n_vec_[i] - nlp_prev.n_vec_[i +
  //     // 1]).fill(1);
  //     get_control_var(z_U0_, i) =
  //         nlp_prev.get_control_var(nlp_prev.z_U0_, i_prev);

  //     // std::cout << "get_state_var(z_U0_, i)\n"
  //     //           << get_state_var(z_U0_, i) << std::endl
  //     //           << std::endl;
  //     // std::cout << "get_control_var(z_U0_, i)\n"
  //     //           << get_control_var(z_U0_, i) << std::endl
  //     //           << std::endl;

  //     // Update constraint variables
  //     get_constraint_var(lambda0_, i - 1).head(nlp_prev.g_vec_[i]) =
  //         nlp_prev.get_constraint_var(nlp_prev.lambda0_, i);
  //     // get_constraint_var(lambda0_, i - 1)
  //     //     .tail(g_vec_[i - 1] - nlp_prev.g_vec_[i])
  //     //     .fill(1000);

  //     // std::cout << "get_constraint_var(lambda0_, i-1)\n"
  //     //           << get_constraint_var(lambda0_, i - 1) << std::endl
  //     //           << std::endl;
  //     // std::cout << "g_vec_[i-1] = " << g_vec_[i - 1] << std::endl;
  //     // std::cout << "nlp_prev.g_vec_[i] = " << nlp_prev.g_vec_[i] <<
  //     // std::endl;

  //     // Update panic variables
  //     get_panic_var(w0_, i)
  //         .head(n_slack_vec_[i])
  //         .head(nlp_prev.n_slack_vec_[i_prev]) =
  //         nlp_prev.get_panic_var(nlp_prev.w0_, i_prev)
  //             .head(nlp_prev.n_slack_vec_[i_prev]);
  //     get_panic_var(w0_, i)
  //         .tail(n_slack_vec_[i])
  //         .head(nlp_prev.n_slack_vec_[i_prev]) =
  //         nlp_prev.get_panic_var(nlp_prev.w0_, i_prev)
  //             .tail(nlp_prev.n_slack_vec_[i_prev]);

  //     // std::cout << "get_panic_var(w0_, i)\n"
  //     //           << get_panic_var(w0_, i) << std::endl
  //     //           << std::endl;

  //     get_panic_var(z_L0_, i)
  //         .head(n_slack_vec_[i])
  //         .head(nlp_prev.n_slack_vec_[i_prev]) =
  //         nlp_prev.get_panic_var(nlp_prev.z_L0_, i_prev)
  //             .head(nlp_prev.n_slack_vec_[i_prev]);
  //     get_panic_var(z_L0_, i)
  //         .tail(n_slack_vec_[i])
  //         .head(nlp_prev.n_slack_vec_[i_prev]) =
  //         nlp_prev.get_panic_var(nlp_prev.z_L0_, i_prev)
  //             .tail(nlp_prev.n_slack_vec_[i_prev]);

  //     // std::cout << "get_panic_var(z_L0_, i)\n"
  //     //           << get_panic_var(z_L0_, i) << std::endl
  //     //           << std::endl;

  //     get_panic_var(z_U0_, i)
  //         .head(n_slack_vec_[i])
  //         .head(nlp_prev.n_slack_vec_[i_prev]) =
  //         nlp_prev.get_panic_var(nlp_prev.z_U0_, i_prev)
  //             .head(nlp_prev.n_slack_vec_[i_prev]);
  //     get_panic_var(z_U0_, i)
  //         .tail(n_slack_vec_[i])
  //         .head(nlp_prev.n_slack_vec_[i_prev]) =
  //         nlp_prev.get_panic_var(nlp_prev.z_U0_, i_prev)
  //             .tail(nlp_prev.n_slack_vec_[i_prev]);

  //     // std::cout << "get_panic_var(z_U0_, i)\n"
  //     //           << get_panic_var(z_U0_, i) << std::endl
  //     //           << std::endl;

  //     get_panic_constraint_var(lambda0_, i)
  //         .head(n_slack_vec_[i])
  //         .head(nlp_prev.n_slack_vec_[i_prev]) =
  //         nlp_prev.get_panic_constraint_var(nlp_prev.lambda0_, i_prev)
  //             .head(nlp_prev.n_slack_vec_[i_prev]);
  //     get_panic_constraint_var(lambda0_, i)
  //         .tail(n_slack_vec_[i])
  //         .head(nlp_prev.n_slack_vec_[i_prev]) =
  //         nlp_prev.get_panic_constraint_var(nlp_prev.lambda0_, i_prev)
  //             .tail(nlp_prev.n_slack_vec_[i_prev]);

  //     get_panic_constraint_var(lambda0_, i).head(n_slack_vec_[i]) =
  //         nlp_prev.get_panic_constraint_var(nlp_prev.lambda0_, i_prev)
  //             .head(nlp_prev.n_slack_vec_[i])
  //             .head(n_slack_vec_[i]);
  //     get_panic_constraint_var(lambda0_, i).tail(n_slack_vec_[i]) =
  //         nlp_prev.get_panic_constraint_var(nlp_prev.lambda0_, i_prev)
  //             .tail(nlp_prev.n_slack_vec_[i])
  //             .head(n_slack_vec_[i]);

  //     // std::cout << "get_panic_constraint_var(lambda0_, i)\n"
  //     //           << get_panic_constraint_var(lambda0_, i) << std::endl
  //     //           << std::endl;
  //   } else {
  //     // Update state and control for w0
  //     get_state_var(w0_, i) =
  //         nlp_prev.get_state_var(nlp_prev.w0_, i_prev).head(n_vec_[i]);
  //     get_control_var(w0_, i) = nlp_prev.get_control_var(nlp_prev.w0_,
  //     i_prev);

  //     // Update state and control for z_L0_
  //     get_state_var(z_L0_, i) =
  //         nlp_prev.get_state_var(nlp_prev.z_L0_, i_prev).head(n_vec_[i]);
  //     get_control_var(z_L0_, i) =
  //         nlp_prev.get_control_var(nlp_prev.z_L0_, i_prev);

  //     // Update state and control for z_U0_
  //     get_state_var(z_U0_, i) =
  //         nlp_prev.get_state_var(nlp_prev.z_U0_, i_prev).head(n_vec_[i]);
  //     get_control_var(z_U0_, i) =
  //         nlp_prev.get_control_var(nlp_prev.z_U0_, i_prev);

  //     // Udate constraint variables
  //     get_constraint_var(lambda0_, i).head(g_vec_[i]) =
  //         nlp_prev.get_constraint_var(nlp_prev.lambda0_, i_prev)
  //             .head(g_vec_[i]);

  //     // Update panic variables
  //     get_panic_var(w0_, i).head(n_slack_vec_[i]) =
  //         nlp_prev.get_panic_var(nlp_prev.w0_, i_prev)
  //             .head(nlp_prev.n_slack_vec_[i])
  //             .head(n_slack_vec_[i]);
  //     get_panic_var(w0_, i).tail(n_slack_vec_[i]) =
  //         nlp_prev.get_panic_var(nlp_prev.w0_, i_prev)
  //             .tail(nlp_prev.n_slack_vec_[i])
  //             .head(n_slack_vec_[i]);

  //     get_panic_var(z_L0_, i).head(n_slack_vec_[i]) =
  //         nlp_prev.get_panic_var(nlp_prev.z_L0_, i_prev)
  //             .head(nlp_prev.n_slack_vec_[i])
  //             .head(n_slack_vec_[i]);
  //     get_panic_var(z_L0_, i).tail(n_slack_vec_[i]) =
  //         nlp_prev.get_panic_var(nlp_prev.z_L0_, i_prev)
  //             .tail(nlp_prev.n_slack_vec_[i])
  //             .head(n_slack_vec_[i]);

  //     get_panic_var(z_U0_, i).head(n_slack_vec_[i]) =
  //         nlp_prev.get_panic_var(nlp_prev.z_U0_, i_prev)
  //             .head(nlp_prev.n_slack_vec_[i])
  //             .head(n_slack_vec_[i]);
  //     get_panic_var(z_U0_, i).tail(n_slack_vec_[i]) =
  //         nlp_prev.get_panic_var(nlp_prev.z_U0_, i_prev)
  //             .tail(nlp_prev.n_slack_vec_[i])
  //             .head(n_slack_vec_[i]);

  //     get_panic_constraint_var(lambda0_, i).head(n_slack_vec_[i]) =
  //         nlp_prev.get_panic_constraint_var(nlp_prev.lambda0_, i_prev)
  //             .head(nlp_prev.n_slack_vec_[i])
  //             .head(n_slack_vec_[i]);
  //     get_panic_constraint_var(lambda0_, i).tail(n_slack_vec_[i]) =
  //         nlp_prev.get_panic_constraint_var(nlp_prev.lambda0_, i_prev)
  //             .tail(nlp_prev.n_slack_vec_[i])
  //             .head(n_slack_vec_[i]);
  //   }
  // }

  // std::cout << "w0_ = \n" << w0_ << std::endl;
  // std::cout << "lambda0_ = \n" << lambda0_ << std::endl;
  // std::cout << "done adding state information" << std::endl;

  // timer.reportStatistics();

  // New contact
  if ((contact_sequence_.col(N_ - 1) - contact_sequence_.col(N_ - 2)).norm() >
      1e-3) {
    // There's a dual pair
    if ((Eigen::MatrixXi::Ones(4, 1) - (contact_sequence_.col(N_ - 1)) -
         contact_sequence_.col(N_ - 2))
            .norm() < 1e-3) {
      Eigen::MatrixXd trans = Eigen::MatrixXd::Zero(12, 12);
      trans.block(0, 3, 3, 3).diagonal() << 1, 1, 1;
      trans.block(3, 0, 3, 3).diagonal() << 1, 1, 1;
      trans.block(6, 9, 3, 3).diagonal() << 1, 1, 1;
      trans.block(9, 6, 3, 3).diagonal() << 1, 1, 1;

      w0_.segment(getPrimalControlFEIndex(N_ - 1) + leg_input_start_idx_,
                  m_ - leg_input_start_idx_) =
          trans *
          w0_.segment(getPrimalControlFEIndex(N_ - 7) + leg_input_start_idx_,
                      m_ - leg_input_start_idx_);
      z_L0_.segment(getPrimalControlFEIndex(N_ - 1) + leg_input_start_idx_,
                    m_ - leg_input_start_idx_) =
          trans *
          z_L0_.segment(getPrimalControlFEIndex(N_ - 7) + leg_input_start_idx_,
                        m_ - leg_input_start_idx_);
      z_U0_.segment(getPrimalControlFEIndex(N_ - 1) + leg_input_start_idx_,
                    m_ - leg_input_start_idx_) =
          trans *
          z_U0_.segment(getPrimalControlFEIndex(N_ - 7) + leg_input_start_idx_,
                        m_ - leg_input_start_idx_);
    } else {  // New contact mode
      w0_.segment(getPrimalControlFEIndex(N_ - 1) + leg_input_start_idx_,
                  m_ - leg_input_start_idx_)
          .fill(0);
      z_L0_
          .segment(getPrimalControlFEIndex(N_ - 1) + leg_input_start_idx_,
                   m_ - leg_input_start_idx_)
          .fill(1);
      z_U0_
          .segment(getPrimalControlFEIndex(N_ - 1) + leg_input_start_idx_,
                   m_ - leg_input_start_idx_)
          .fill(1);

      // Compute the number of contacts
      double num_contacts = contact_sequence_.col(N_ - 1).sum();

      // If there are some contacts, set the nominal input accordingly
      if (num_contacts > 0) {
        for (size_t i = 0; i < 4; i++) {
          if (contact_sequence_(i, N_ - 1) == 1) {
            w0_(getPrimalControlFEIndex(N_ - 1) + leg_input_start_idx_ + 3 * i +
                    2,
                0) = mass_ * grav_ / num_contacts;
          }
        }
      }
    }
  }
}

void quadNLP::update_solver(
    // TODO(jcnorby) update this to account for complexity
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXi &adaptive_complexity_schedule,
    const Eigen::VectorXd &ground_height, const double &first_element_duration,
    const bool &same_plan_index, const bool &init) {
  // Update foot positions
  // Local planner has row as N horizon and col as position
  feet_location_ = -foot_positions.transpose();

  // Copy the previous contact sequence and nlp data for comparison
  Eigen::MatrixXi contact_sequence_prev = contact_sequence_;
  quadNLP nlp_prev = *this;

  // Update contact sequence
  // Local planner has outer as N and inner as boolean contact
  for (size_t i = 0; i < contact_schedule.size(); i++) {
    for (size_t j = 0; j < contact_schedule.front().size(); j++) {
      if (contact_schedule.at(i).at(j)) {
        contact_sequence_(j, i) = 1;
      } else {
        contact_sequence_(j, i) = 0;
      }
    }
  }

  for (size_t i = 0; i < N_; i++) {
    int idx;

    if (same_plan_index) {
      idx = i;
    } else {
      if (i == N_ - 1) {
        continue;
      }

      idx = i + 1;
    }

    if ((contact_sequence_prev.col(idx) - contact_sequence_.col(i))
            .cwiseAbs()
            .sum() > 1e-3) {
      // Contact change unexpectedly, update the warmstart info
      get_control_var(w0_, idx)
          .segment(leg_input_start_idx_, m_ - leg_input_start_idx_)
          .fill(0);
      get_control_var(z_L0_, idx)
          .segment(leg_input_start_idx_, m_ - leg_input_start_idx_)
          .fill(1);
      get_control_var(z_U0_, idx)
          .segment(leg_input_start_idx_, m_ - leg_input_start_idx_)
          .fill(1);

      // Compute the number of contacts
      double num_contacts = contact_sequence_.col(i).sum();

      // If there are some contacts, set the nominal input accordingly
      if (num_contacts > 0) {
        for (size_t j = 0; j < 4; j++) {
          if (contact_sequence_(j, i) == 1) {
            get_control_var(w0_, idx)(leg_input_start_idx_ + 3 * j + 2, 0) =
                mass_ * grav_ / num_contacts;
          }
        }
      }

      mu0_ = 1e-1;
      warm_start_ = false;
    }
  }

  // If the complexity schedule has changed, update the problem structure
  bool new_structure =
      adaptive_complexity_schedule != this->adaptive_complexity_schedule_;

  if (new_structure) {
    this->adaptive_complexity_schedule_ = adaptive_complexity_schedule;
    this->update_structure();
  }

  // Update initial state
  x_current_ = initial_state.transpose().head(n_vec_[0]);

  // Update reference trajectory
  // Local planner has row as N+1 horizon and col as states
  x_reference_ = ref_traj.transpose();

  // Update the first finite element length
  first_element_duration_ = first_element_duration;

  // Update ground height
  ground_height_ = ground_height.transpose();

  if (init) {
    w0_ = Eigen::VectorXd(n_vars_).setZero();
    z_L0_ = Eigen::VectorXd(n_vars_).Ones(n_vars_);
    z_U0_ = Eigen::VectorXd(n_vars_).Ones(n_vars_);
    lambda0_ = Eigen::VectorXd(n_constraints_);
    lambda0_.fill(1000);

    // Initialize current state
    get_state_var(w0_, 0) = x_current_;

    // Initialize future state predictions and controls
    for (size_t i = 0; i < N_; i++) {
      get_state_var(w0_, i + 1).head(n_simple_) = x_reference_.col(i + 1);
      if (n_vec_[i + 1] == n_complex_) {
        get_state_var(w0_, i + 1).tail(n_null_) = x_null_nom_;
      }

      double num_contacts = contact_sequence_.col(i).sum();
      if (num_contacts > 0) {
        for (size_t j = 0; j < 4; j++) {
          if (contact_schedule.at(i).at(j)) {
            get_control_var(w0_, i)(leg_input_start_idx_ + 2 + j * 3, 0) =
                mass_ * grav_ / num_contacts;
          }
        }
      }
    }
  } else {
    // Determine how much to shift the initial guess
    int shift_idx = (same_plan_index) ? 0 : 1;

    // Update the initial guess if something has changed
    if (!same_plan_index || new_structure) {
      update_initial_guess(nlp_prev, shift_idx);
    }
  }

  // Reset the state weighting factors
  Q_factor_ = Q_factor_base_;
  R_factor_ = R_factor_base_;
}

void quadNLP::update_solver(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::MatrixXd &state_traj, const Eigen::MatrixXd &control_traj,
    const Eigen::VectorXd &ground_height, const double &first_element_duration,
    const bool &same_plan_index, const bool &init) {
  update_solver(initial_state, ref_traj, foot_positions, contact_schedule,
                adaptive_complexity_schedule_, ground_height,
                first_element_duration, same_plan_index, init);

  // Update known leg input
  leg_input_ = control_traj.transpose();
  for (size_t i = 0; i < N_; i++) {
    get_control_var(w0_, i).tail(12) = leg_input_.col(i);
  }

  // Update known leg input flag
  known_leg_input_ = true;

  if (init) {
    for (size_t i = 0; i < N_; i++) {
      w0_.block(getPrimalStateFEIndex(i), 0, 6, 1) =
          state_traj.row(i).transpose().segment(0, 6);
      w0_.block(getPrimalStateFEIndex(i) + 8, 0, 6, 1) =
          state_traj.row(i).transpose().segment(6, 6);
    }
  }
}

void quadNLP::update_structure() {
  // Update the complexity schedule
  quad_utils::FunctionTimer timer(__FUNCTION__);

  std::cout << "in update_structure" << std::endl;

  // Determine the (possibly new) size of the horizon
  N_ = std::min(adaptive_complexity_schedule_.size(),
                fixed_complexity_schedule_.size()) -
       1;

  // Update the complexity schedule by anding with fixed schedule
  Eigen::VectorXi complexity_schedule =
      adaptive_complexity_schedule_.head(N_ + 1).cwiseMax(
          fixed_complexity_schedule_.head(N_ + 1));

  // Resize vectors appropriately
  sys_id_schedule_.resize(N_);
  g_vec_.resize(N_);
  n_vec_.resize(N_ + 1);
  n_slack_vec_.resize(N_);
  fe_idxs_.resize(N_);
  u_idxs_.resize(N_);
  x_idxs_.resize(N_ + 1);
  slack_idxs_.resize(N_);
  g_idxs_.resize(N_);
  g_slack_idxs_.resize(N_);
  dynamic_jac_var_idxs_.resize(N_);
  panic_jac_var_idxs_.resize(N_);
  dynamic_hess_var_idxs_.resize(N_);
  cost_idxs_.resize(N_);

  // Initialize indexing veriables
  num_complex_fe_ = 0;
  int curr_var_idx = 0;
  int curr_constr_idx = 0;
  int dynamic_jac_var_idx = 0;
  int dynamic_hess_var_idx = 0;

  // Loop through the horizon
  for (int i = 0; i < N_; i++) {
    // Update the number of complex elements
    num_complex_fe_ += complexity_schedule[i];

    // Determine the system to assign to this finite element
    if (complexity_schedule[i] == 0) {
      sys_id_schedule_[i] =
          (complexity_schedule[i + 1] == 0) ? SIMPLE : SIMPLE_TO_COMPLEX;
    } else {
      sys_id_schedule_[i] =
          (complexity_schedule[i + 1] == 1) ? COMPLEX : COMPLEX_TO_SIMPLE;
    }

    // Update the number of state vars and constraints for this FE
    n_vec_[i] = (complexity_schedule[i] == 1) ? n_complex_ : n_simple_;

    n_slack_vec_[i] =
        (complexity_schedule[i + 1] == 1 && apply_slack_to_complex_)
            ? n_complex_
            : n_simple_;
    g_vec_[i] = nrow_mat_(sys_id_schedule_[i], FUNC);

    // Update the indices for the finite element, control, and state variables
    fe_idxs_[i] = curr_var_idx;
    x_idxs_[i] = curr_var_idx;
    curr_var_idx += n_vec_[i];
    u_idxs_[i] = curr_var_idx;
    curr_var_idx += m_;

    // Update the indices for the constraints
    g_idxs_[i] = curr_constr_idx;
    curr_constr_idx += g_vec_[i];

    // Update the indices for the sparsity patterns
    dynamic_jac_var_idxs_[i] = dynamic_jac_var_idx;
    dynamic_jac_var_idx += nnz_mat_(sys_id_schedule_[i], JAC);
    dynamic_hess_var_idxs_[i] = dynamic_hess_var_idx;
    dynamic_hess_var_idx += nnz_mat_(sys_id_schedule_[i], HESS);
  }

  num_complex_fe_ += complexity_schedule[N_];
  n_vec_[N_] = (complexity_schedule[N_] == 1) ? n_complex_ : n_simple_;
  x_idxs_[N_] = curr_var_idx;
  curr_var_idx += n_vec_[N_];

  if (n_vec_[0] != x_current_.size()) {
    ROS_ERROR_THROTTLE(0.1, "n_vec_[0] != x_current_.size()!");
  }

  // Update the total number of primal variables
  n_vars_primal_ = curr_var_idx;

  // Update the slack variable indices
  for (int i = 0; i < N_; i++) {
    int n_slack = n_slack_vec_[i];
    slack_idxs_[i] = curr_var_idx;
    curr_var_idx += 2 * n_slack;

    g_slack_idxs_[i] = curr_constr_idx;
    curr_constr_idx += 2 * n_slack;

    panic_jac_var_idxs_[i] = dynamic_jac_var_idx;
    dynamic_jac_var_idx += 4 * n_slack;

    cost_idxs_[i] = dynamic_hess_var_idx;
    dynamic_hess_var_idx += n_simple_ + m_;
  }

  // Update the total number of vars and slack vars
  n_vars_ = curr_var_idx;
  n_vars_slack_ = n_vars_ - n_vars_primal_;

  // Check the number of primal and slack vars for correctness
  if (n_vars_primal_ != (n_vec_.sum() + m_ * N_)) {
    std::cout << "n_vec_ = " << n_vec_.transpose() << std::endl;
    std::cout << "n_vars_primal_ = " << n_vars_primal_ << std::endl;
    std::cout << "n_vec_.sum() + m_ * N_ = " << n_vec_.sum() + m_ * N_
              << std::endl;
    throw std::runtime_error("Number of primal vars is inconsistent");
  } else if (n_vars_slack_ != (2 * n_slack_vec_.sum())) {
    std::cout << "n_vars_slack_ = " << n_vars_slack_ << std::endl;
    std::cout << "2 * n_slack_vec_.sum() = " << (2 * n_slack_vec_.sum())
              << std::endl;
    throw std::runtime_error("Number of slack vars is inconsistent");
  }

  // Update the number of constraints
  n_constraints_ = curr_constr_idx;

  // Check the number of constraints
  if (n_constraints_ != (g_vec_.sum() + n_vars_slack_)) {
    throw std::runtime_error("Number of constraints is inconsistent");
  }

  compute_nnz_jac_g();
  compute_nnz_h();

  // std::cout << "cmplx_sch = " << complexity_schedule.transpose() <<
  // std::endl; std::cout << "sys_id_sch = " << sys_id_schedule_.transpose() <<
  // std::endl; std::cout << "n_vec_ =    " << n_vec_.transpose() << std::endl;
  // std::cout << "n_slack_vec_ = " << n_slack_vec_.transpose() << std::endl;
  // std::cout << "g_vec_ =      " << g_vec_.transpose() << std::endl;
  // std::cout << "g_vec_.sum() = " << g_vec_.sum() << std::endl;
  // std::cout << "g_vec_.sum() + n_vars_slack_ = " << g_vec_.sum() +
  // n_vars_slack_
  //           << std::endl;

  // std::cout << "n_constraints_ = " << n_constraints_ << std::endl;
  // std::cout << "n_vars_ = " << n_vars_ << std::endl;
  // std::cout << "n_vars_primal_ = " << n_vars_primal_ << std::endl;
  // std::cout << "n_vars_slack_ = " << n_vars_slack_ << std::endl;
}
