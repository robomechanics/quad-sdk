#include "nmpc_controller/quad_nlp.h"

#include <cassert>
#include <iostream>

using namespace Ipopt;

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

// Class constructor
quadNLP::quadNLP(
    int N, double dt, double mu, double panic_weights,
    double constraint_panic_weights, double Q_temporal_factor,
    double R_temporal_factor, int n_simple, int n_complex, int m_simple,
    int m_complex, int g_simple, int g_complex, int n_cost_simple,
    int n_cost_complex, int m_cost_simple, int m_cost_complex,
    const Eigen::VectorXd &Q_complex, const Eigen::VectorXd &R_complex,
    const Eigen::VectorXd &x_min_complex, const Eigen::VectorXd &x_max_complex,
    const Eigen::VectorXd &u_min_complex, const Eigen::VectorXd &u_max_complex,
    const Eigen::VectorXd &g_min_complex, const Eigen::VectorXd &g_max_complex,
    const Eigen::VectorXi &fixed_complexity_schedule) {
  // Load constant parameters
  dt_ = dt;
  mu_ = mu;
  N_ = N;
  Q_temporal_factor_ = Q_temporal_factor;
  R_temporal_factor_ = R_temporal_factor;
  panic_weights_ = panic_weights;
  constraint_panic_weights_ = constraint_panic_weights;

  // Load dimension parameters
  n_simple_ = n_simple;
  n_complex_ = n_complex;
  n_null_ = n_complex - n_simple;
  m_simple_ = m_simple;
  m_complex_ = m_complex;
  g_simple_ = g_simple;
  g_complex_ = g_complex;
  n_cost_simple_ = n_cost_simple;
  n_cost_complex_ = n_cost_complex;
  m_cost_simple_ = m_cost_simple;
  m_cost_complex_ = m_cost_complex;

  Q_complex_ = Q_complex;
  R_complex_ = R_complex;

  // feet location initialized by nominal position
  foot_pos_body_ = Eigen::MatrixXd(N_, 12);
  foot_pos_world_ = Eigen::MatrixXd(N_, 12);
  foot_vel_world_ = Eigen::MatrixXd(N_, 12).setZero();
  for (int i = 0; i < N_; ++i) {
    foot_pos_body_.row(i) << -0.2263, -0.098, -0.3, -0.2263, 0.098, -0.3,
        0.2263, -0.098, -0.3, 0.2263, 0.098, -0.3;
    foot_pos_world_.row(i) << -0.2263, -0.098, 0, -0.2263, 0.098, 0, 0.2263,
        -0.098, 0, 0.2263, 0.098, 0;
  }

  // Load state and control bounds
  x_min_complex_ = x_min_complex;
  x_max_complex_ = x_max_complex;
  u_min_complex_ = u_min_complex;
  u_max_complex_ = u_max_complex;
  g_min_complex_ = g_min_complex;
  g_max_complex_ = g_max_complex;
  x_min_simple_ = x_min_complex.head(n_simple);
  x_max_simple_ = x_max_complex.head(n_simple);
  u_min_simple_ = u_min_complex.head(m_simple);
  u_max_simple_ = u_max_complex.head(m_simple);
  g_min_simple_ = g_min_complex.head(g_simple);
  g_max_simple_ = g_max_complex.head(g_simple);

  double abad_nom = 0.248694;
  double hip_nom = 0.760144;
  double knee_nom = 1.52029;
  x_null_nom_.resize(n_joints_);
  x_null_nom_ << -abad_nom, hip_nom, knee_nom, -abad_nom, hip_nom, knee_nom,
      abad_nom, hip_nom, knee_nom, abad_nom, hip_nom, knee_nom, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0;

  // Define which constraints will be relaxed
  g_relaxed_ = num_feet_;
  int g_relaxed_idx = 80;
  g_min_complex_soft_.resize(g_relaxed_);
  g_max_complex_soft_.resize(g_relaxed_);
  constraint_panic_weights_vec_.resize(g_relaxed_);

  // Start relaxed constraints here
  relaxed_primal_constraint_idxs_in_element_ = Eigen::ArrayXi::LinSpaced(
      g_relaxed_, g_relaxed_idx, g_relaxed_idx + g_relaxed_ - 1);

  // Load knee constraint bounds
  g_min_complex_soft_.fill(-2e19);
  g_max_complex_soft_.fill(-0.05);

  loadCasadiFuncs();
  loadConstraintNames();

  this->fixed_complexity_schedule_ = fixed_complexity_schedule;
  this->adaptive_complexity_schedule_.setZero(N_);
  this->update_structure();

  w0_ = Eigen::VectorXd(n_vars_).setZero();
  z_L0_ = Eigen::VectorXd(n_vars_).Ones(n_vars_, 1);
  z_U0_ = Eigen::VectorXd(n_vars_).Ones(n_vars_, 1);
  lambda0_ = Eigen::VectorXd(n_constraints_).setZero();
  g0_ = Eigen::VectorXd(n_constraints_).setZero();

  mu0_ = 1e-1;
  warm_start_ = false;

  for (size_t i = 0; i < N_ - 1; i++) {
    for (size_t j = 0; j < 4; j++) {
      get_primal_control_var(w0_, i)(2 + j * 3, 0) = mass_ * grav_ * 0.25;
    }
  }

  x_reference_ = Eigen::MatrixXd(n_body_, N_);
  x_reference_.fill(0);

  ground_height_ = Eigen::VectorXd(N_);
  ground_height_.fill(-2e19);

  x_current_ = Eigen::VectorXd(n_vec_[0]);
  x_current_.fill(0);

  contact_sequence_ = Eigen::MatrixXi(4, N_);
  contact_sequence_.fill(1);

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
  m_vec_ = nlp.m_vec_;
  g_vec_ = nlp.g_vec_;
  n_cost_vec_ = nlp.n_cost_vec_;
  m_cost_vec_ = nlp.m_cost_vec_;
  fe_idxs_ = nlp.fe_idxs_;
  x_idxs_ = nlp.x_idxs_;
  u_idxs_ = nlp.u_idxs_;
  primal_constraint_idxs_ = nlp.primal_constraint_idxs_;
  slack_state_var_idxs_ = nlp.slack_state_var_idxs_;
  slack_constraint_var_idxs_ = nlp.slack_constraint_var_idxs_;
  slack_var_constraint_idxs_ = nlp.slack_var_constraint_idxs_;
  relaxed_constraint_idxs_ = nlp.relaxed_constraint_idxs_;
  relaxed_dynamic_jac_var_idxs_ = nlp.relaxed_dynamic_jac_var_idxs_;
  g_slack_vec_ = nlp.g_slack_vec_;
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

  for (int i = 0; i < N_ - 1; ++i) {
    // Inputs bound
    get_primal_control_var(x_l_matrix, i) = u_min_complex_.head(m_vec_[i]);
    get_primal_control_var(x_u_matrix, i) = u_max_complex_.head(m_vec_[i]);

    // States bound
    get_primal_state_var(x_l_matrix, i + 1).fill(-2e19);
    get_primal_state_var(x_u_matrix, i + 1).fill(2e19);

    // Contact sequence
    for (int j = 0; j < num_feet_; ++j) {
      get_primal_body_control_var(x_l_matrix, i).segment(3 * j, 3) =
          (get_primal_body_control_var(x_l_matrix, i)
               .segment(3 * j, 3)
               .array() *
           contact_sequence_(j, i))
              .matrix();
      get_primal_body_control_var(x_u_matrix, i).segment(3 * j, 3) =
          (get_primal_body_control_var(x_u_matrix, i)
               .segment(3 * j, 3)
               .array() *
           contact_sequence_(j, i))
              .matrix();
    }

    // Add bounds if not covered by panic variables - JN Changes
    if (n_vec_[i + 1] > n_simple_) {
      get_primal_state_var(x_l_matrix, i + 1).tail(n_null_) =
          x_min_complex_.tail(n_null_);
      get_primal_state_var(x_u_matrix, i + 1).tail(n_null_) =
          x_max_complex_.tail(n_null_);
    }

    // Constraints bound - leave to enforce hard constraints
    get_primal_constraint_vals(g_l_matrix, i) = g_min_complex_.head(g_vec_[i]);
    get_primal_constraint_vals(g_u_matrix, i) = g_max_complex_.head(g_vec_[i]);

    bool modify_foot_constraints = (sys_id_schedule_[i] == COMPLEX ||
                                    sys_id_schedule_[i] == COMPLEX_TO_SIMPLE);

    if (modify_foot_constraints) {
      for (int j = 0; j < num_feet_; ++j) {
        bool is_stance = contact_sequence_(j, i + 1) == 1;
        bool is_liftoff =
            (contact_sequence_(j, i + 1) == 0 && contact_sequence_(j, i) == 1);
        bool is_touchdown =
            (contact_sequence_(j, i + 1) == 1 && contact_sequence_(j, i) == 0);
        bool in_flight_interior =
            contact_sequence_(j, std::min(i + 2, N_ - 1)) == 0 &&
            contact_sequence_(j, std::max(i - 1, 0)) == 0;

        bool constrain_feet = !allow_foot_traj_modification ||
                              sys_id_schedule_[i] == COMPLEX_TO_SIMPLE ||
                              is_stance || is_liftoff;

        bool remove_foot_dynamics =
            !allow_foot_traj_modification || contact_sequence_(j, i) == 1;

        bool relax_vel_constraint =
            constrain_feet &&
            (is_touchdown || sys_id_schedule_[i] == COMPLEX_TO_SIMPLE);

        bool add_terrain_height_constraint =
            use_terrain_constraint && !constrain_feet && in_flight_interior;

        if (remove_foot_dynamics) {
          int g_foot_eom_idx = n_body_ + 16;
          get_primal_constraint_vals(g_l_matrix, i)
              .segment(g_foot_eom_idx + 3 * j, 3)
              .fill(-2e19);
          get_primal_constraint_vals(g_u_matrix, i)
              .segment(g_foot_eom_idx + 3 * j, 3)
              .fill(2e19);
          get_primal_constraint_vals(g_l_matrix, i)
              .segment(g_foot_eom_idx + n_foot_ / 2 + 3 * j, 3)
              .fill(-2e19);
          get_primal_constraint_vals(g_u_matrix, i)
              .segment(g_foot_eom_idx + n_foot_ / 2 + 3 * j, 3)
              .fill(2e19);
          get_primal_foot_control_var(x_l_matrix, i).segment(3 * j, 3).fill(0);
          get_primal_foot_control_var(x_u_matrix, i).segment(3 * j, 3).fill(0);
        }

        if (constrain_feet) {
          get_primal_foot_state_var(x_l_matrix, i + 1).segment(3 * j, 3) =
              foot_pos_world_.block<1, 3>(i + 1, 3 * j);
          get_primal_foot_state_var(x_u_matrix, i + 1).segment(3 * j, 3) =
              foot_pos_world_.block<1, 3>(i + 1, 3 * j);
          if (!relax_vel_constraint) {
            get_primal_foot_state_var(x_l_matrix, i + 1)
                .segment(3 * j + n_foot_ / 2, 3) =
                foot_vel_world_.block<1, 3>(i + 1, 3 * j);
            get_primal_foot_state_var(x_u_matrix, i + 1)
                .segment(3 * j + n_foot_ / 2, 3) =
                foot_vel_world_.block<1, 3>(i + 1, 3 * j);
          }
        }

        if (add_terrain_height_constraint) {
          int g_foot_height_idx = n_body_ + 16 + n_foot_;
          get_primal_constraint_vals(g_u_matrix, i)(g_foot_height_idx + j, 0) =
              -0.02;
        }
      }
    }

    // Panic variable bound
    get_slack_state_var(x_l_matrix, i).fill(0);
    get_slack_state_var(x_u_matrix, i).fill(2e19);
    // get_slack_state_var(x_u_matrix, i).head(n_simple_).fill(0);

    // Relaxed constraints slack variables
    if (g_slack_vec_[i] > 0) {
      get_slack_constraint_var(x_l_matrix, i).fill(0);
      get_slack_constraint_var(x_u_matrix, i).fill(2e19);
    }
  }

  // Current state bound
  get_primal_state_var(x_l_matrix, 0) = x_current_;
  get_primal_state_var(x_u_matrix, 0) = x_current_;

  for (size_t i = 0; i < N_ - 1; i++) {
    // xmin
    get_slack_constraint_vals(g_l_matrix, i).head(n_slack_vec_[i]) =
        x_min_complex_.head(n_slack_vec_[i]);
    get_slack_constraint_vals(g_l_matrix, i)(2, 0) = ground_height_(0, i);
    get_slack_constraint_vals(g_u_matrix, i).head(n_slack_vec_[i]).fill(2e19);

    // xmax
    get_slack_constraint_vals(g_l_matrix, i).tail(n_slack_vec_[i]).fill(-2e19);
    get_slack_constraint_vals(g_u_matrix, i).tail(n_slack_vec_[i]) =
        x_max_complex_.head(n_slack_vec_[i]);

    // Relaxed constraints bound
    if (g_slack_vec_[i] > 0) {
      // Remove bounds on relaxed constraints
      get_primal_constraint_vals(g_l_matrix, i)
          .segment(relaxed_primal_constraint_idxs_in_element_(0),
                   g_slack_vec_[i])
          .fill(-2e19);
      get_primal_constraint_vals(g_u_matrix, i)
          .segment(relaxed_primal_constraint_idxs_in_element_(0),
                   g_slack_vec_[i])
          .fill(2e19);

      // Add relaxed minimum constraint bounds
      get_relaxed_primal_constraint_vals(g_l_matrix, i).head(g_slack_vec_[i]) =
          g_min_complex_soft_;
      get_relaxed_primal_constraint_vals(g_u_matrix, i)
          .head(g_slack_vec_[i])
          .fill(2e19);

      // Add relaxed maximum constraint bounds
      get_relaxed_primal_constraint_vals(g_l_matrix, i)
          .tail(g_slack_vec_[i])
          .fill(-2e19);
      get_relaxed_primal_constraint_vals(g_u_matrix, i).tail(g_slack_vec_[i]) =
          g_max_complex_soft_;
    }

    // if (g_vec_[i] > g_simple_) {
    //   std::cout << "i = " << i << std::endl;
    //   std::cout << "contact mode now  = "
    //             << contact_sequence_.col(i).transpose() << std::endl;
    //   std::cout << "contact mode next = "
    //             << contact_sequence_.col(i + 1).transpose() << std::endl;

    //   std::cout << "foot state lb =\n"
    //             << get_primal_foot_state_var(x_l_matrix, i + 1) << std::endl;
    //   std::cout << "foot state ub =\n"
    //             << get_primal_foot_state_var(x_u_matrix, i + 1) << std::endl;

    //   std::cout << "foot dyn lb =\n"
    //             << get_primal_constraint_vals(g_l_matrix, i).segment(28, 24)
    //             << std::endl;
    //   std::cout << "foot dyn ub =\n"
    //             << get_primal_constraint_vals(g_u_matrix, i).segment(28, 24)
    //             << std::endl;

    //   if (g_vec_[i] > 52) {
    //     std::cout << "foot height lb =\n"
    //               << get_primal_constraint_vals(g_l_matrix, i).segment(52, 4)
    //               << std::endl;
    //     std::cout << "foot height ub =\n"
    //               << get_primal_constraint_vals(g_u_matrix, i).segment(52, 4)
    //               << std::endl;

    //     std::cout << "fk lb =\n"
    //               << get_primal_constraint_vals(g_l_matrix, i).segment(56,
    //               24)
    //               << std::endl;
    //     std::cout << "fk ub =\n"
    //               << get_primal_constraint_vals(g_u_matrix, i).segment(56,
    //               24)
    //               << std::endl;

    //     std::cout << "knee height lb =\n"
    //               << get_primal_constraint_vals(g_l_matrix, i).segment(80, 4)
    //               << std::endl;
    //     std::cout << "knee height ub =\n"
    //               << get_primal_constraint_vals(g_u_matrix, i).segment(80, 4)
    //               << std::endl;

    //     std::cout << "mm lb =\n"
    //               << get_primal_constraint_vals(g_l_matrix, i).segment(84,
    //               24)
    //               << std::endl;
    //     std::cout << "mm ub =\n"
    //               << get_primal_constraint_vals(g_u_matrix, i).segment(84,
    //               24)
    //               << std::endl;
    //   }
    // }
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

  for (int i = 0; i < N_ - 1; ++i) {
    // Compute the number of contacts
    Eigen::VectorXd u_nom(m_cost_vec_[i]);
    u_nom.setZero();
    double num_contacts = contact_sequence_.col(i).sum();

    // If there are some contacts, set the nominal input accordingly
    if (num_contacts > 0) {
      for (int j = 0; j < contact_sequence_.rows(); j++) {
        if (contact_sequence_(j, i) == 1) {
          u_nom[3 * j + 2] = mass_ * grav_ / num_contacts;
        }
      }
    }

    Eigen::VectorXd x_nom(n_cost_vec_[i]);
    x_nom.head(n_body_) = x_reference_.col(i + 1);
    if (n_cost_vec_[i] > n_body_) {
      x_nom.segment(n_body_, n_foot_ / 2) = foot_pos_world_.row(i + 1);
      x_nom.segment(n_body_ + n_foot_ / 2, n_foot_ / 2) =
          foot_vel_world_.row(i + 1);
    }

    Eigen::VectorXd uk =
        get_primal_control_var(w, i).head(m_cost_vec_[i]) - u_nom;
    Eigen::VectorXd xk =
        get_primal_state_var(w, i + 1).head(n_cost_vec_[i]) - x_nom;

    Eigen::VectorXd Q_i =
        (Q_complex_ * std::pow(Q_temporal_factor_, i)).head(n_cost_vec_[i]);
    Eigen::VectorXd R_i =
        (R_complex_ * std::pow(R_temporal_factor_, i)).head(m_cost_vec_[i]);

    // Scale the cost by time duration
    if (i == 0) {
      Q_i = Q_i * first_element_duration_ / dt_;
      R_i = R_i * first_element_duration_ / dt_;
    }

    obj_value += (xk.transpose() * Q_i.asDiagonal() * xk / 2 +
                  uk.transpose() * R_i.asDiagonal() * uk / 2)(0, 0);
    obj_value +=
        panic_weights_ * get_slack_state_var(w, i).sum() +
        constraint_panic_weights_ * get_slack_constraint_var(w, i).sum();
  }
  // std::cout << "Stop and check cost" << std::endl;
  // throw std::runtime_error("Stop");
  return true;
}

// Return the gradient of the objective function
bool quadNLP::eval_grad_f(Index n, const Number *x, bool new_x,
                          Number *grad_f) {
  Eigen::Map<const Eigen::VectorXd> w(x, n);

  Eigen::Map<Eigen::VectorXd> grad_f_matrix(grad_f, n);
  grad_f_matrix.setZero();

  get_primal_state_var(grad_f_matrix, 0).fill(0);

  for (int i = 0; i < N_ - 1; ++i) {
    // Compute the number of contacts
    Eigen::VectorXd u_nom(m_cost_vec_[i]);
    u_nom.setZero();
    double num_contacts = contact_sequence_.col(i).sum();

    // If there are some contacts, set the nominal input accordingly
    if (num_contacts > 0) {
      for (int j = 0; j < contact_sequence_.rows(); j++) {
        if (contact_sequence_(j, i) == 1) {
          u_nom[3 * j + 2] = mass_ * grav_ / num_contacts;
        }
      }
    }

    Eigen::VectorXd x_nom(n_cost_vec_[i]);
    x_nom.head(n_body_) = x_reference_.col(i + 1);
    if (n_cost_vec_[i] > n_body_) {
      x_nom.segment(n_body_, n_foot_ / 2) = foot_pos_world_.row(i + 1);
      x_nom.segment(n_body_ + n_foot_ / 2, n_foot_ / 2) =
          foot_vel_world_.row(i + 1);
    }

    Eigen::VectorXd uk =
        get_primal_control_var(w, i).head(m_cost_vec_[i]) - u_nom;
    Eigen::VectorXd xk =
        get_primal_state_var(w, i + 1).head(n_cost_vec_[i]) - x_nom;

    Eigen::VectorXd Q_i =
        (Q_complex_ * std::pow(Q_temporal_factor_, i)).head(n_cost_vec_[i]);
    Eigen::VectorXd R_i =
        (R_complex_ * std::pow(R_temporal_factor_, i)).head(m_cost_vec_[i]);

    // Scale the cost by time duration
    if (i == 0) {
      Q_i = Q_i * first_element_duration_ / dt_;
      R_i = R_i * first_element_duration_ / dt_;
    }

    get_primal_control_var(grad_f_matrix, i) = R_i.asDiagonal() * uk;
    get_primal_state_var(grad_f_matrix, i + 1).head(n_simple_) =
        Q_i.asDiagonal() * xk;
    get_slack_state_var(grad_f_matrix, i).fill(panic_weights_);

    get_slack_constraint_var(grad_f_matrix, i).fill(constraint_panic_weights_);
  }

  return true;
}

Eigen::VectorXd quadNLP::eval_g_single_fe(int sys_id, double dt,
                                          const Eigen::VectorXd &x0,
                                          const Eigen::VectorXd &u,
                                          const Eigen::VectorXd &x1,
                                          const Eigen::VectorXd &params) {
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

  Eigen::VectorXd dynamic_var(x0.size() + u.size() + x1.size()),
      pk(2 + params.size());

  if (dynamic_var.size() != ncol_mat_(sys_id, JAC)) {
    printf("Expected %d vars, got %d instead.\n", ncol_mat_(sys_id, JAC),
           (int)dynamic_var.size());
    throw std::runtime_error(
        "Number of decision variables does not match that expected by "
        "constraints");
  }

  dynamic_var << x0, u, x1;
  pk << dt, mu_, params;

  arg[0] = dynamic_var.data();
  arg[1] = pk.data();

  Eigen::VectorXd constr_vars(nrow_mat_(sys_id, FUNC));
  res[0] = constr_vars.data();

  int mem = eval_checkout_vec_[sys_id][FUNC]();
  eval_vec_[sys_id][FUNC](arg, res, iw, _w, mem);
  eval_release_vec_[sys_id][FUNC](mem);
  eval_decref_vec_[sys_id][FUNC]();

  return constr_vars;
}

// Return the value of the constraints
bool quadNLP::eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) {
  Eigen::Map<const Eigen::VectorXd> w(x, n);
  Eigen::Map<Eigen::VectorXd> g_matrix(g, m);
  g_matrix.setZero();

  for (int i = 0; i < N_ - 1; ++i) {
    // Select the system ID
    int sys_id = sys_id_schedule_[i];

    // Load the params for this fe
    Eigen::VectorXd pk(54);
    pk[0] = (i == 0) ? first_element_duration_ : dt_;
    pk[1] = mu_;
    pk.segment(2, 12) = foot_pos_body_.row(i + 1);
    pk.segment(14, 12) = foot_pos_world_.row(i + 1);
    pk.segment(26, 12) = foot_vel_world_.row(i + 1);
    if (use_terrain_constraint) {
      for (int j = 0; j < num_feet_; j++) {
        Eigen::Vector3d foot_pos =
            get_primal_foot_state_var(w, i + 1).segment(3 * j, 3);

        pk(38 + j) = terrain_.atPosition("z_inpainted", foot_pos.head<2>(),
                                         interp_type_);
        pk(42 + 3 * j) = terrain_.atPosition("normal_vectors_x",
                                             foot_pos.head<2>(), interp_type_);
        pk(42 + 3 * j) = terrain_.atPosition("normal_vectors_y",
                                             foot_pos.head<2>(), interp_type_);
        pk(42 + 3 * j) = terrain_.atPosition("normal_vectors_z",
                                             foot_pos.head<2>(), interp_type_);
      }
    } else {
      pk.segment(38, 4).fill(0);
      pk.segment(42, 12).fill(1);
    }

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
    res[0] = get_primal_constraint_vals(g_matrix, i).data();

    int mem = eval_checkout_vec_[sys_id][FUNC]();
    eval_vec_[sys_id][FUNC](arg, res, iw, _w, mem);
    eval_release_vec_[sys_id][FUNC](mem);
    eval_decref_vec_[sys_id][FUNC]();
  }

  // Evaluate relaxed state and constraint bounds
  for (int i = 0; i < N_ - 1; ++i) {
    Eigen::VectorXd xk = get_primal_state_var(w, i + 1).head(n_slack_vec_[i]);
    Eigen::VectorXd panick = get_slack_state_var(w, i);

    get_slack_constraint_vals(g_matrix, i).head(n_slack_vec_[i]) =
        xk + panick.head(n_slack_vec_[i]);
    get_slack_constraint_vals(g_matrix, i).tail(n_slack_vec_[i]) =
        xk - panick.tail(n_slack_vec_[i]);

    if (g_slack_vec_[i] > 0) {
      Eigen::VectorXd gk =
          get_primal_constraint_vals(g_matrix, i)
              .segment(relaxed_primal_constraint_idxs_in_element_[0],
                       g_slack_vec_[i]);
      Eigen::VectorXd gk_panic = get_slack_constraint_var(w, i);

      get_relaxed_primal_constraint_vals(g_matrix, i).head(g_slack_vec_[i]) =
          gk + gk_panic.head(g_slack_vec_[i]);

      get_relaxed_primal_constraint_vals(g_matrix, i).tail(g_slack_vec_[i]) =
          gk - gk_panic.tail(g_slack_vec_[i]);
    }
  }

  // std::cout << "g_matrix = \n" << g_matrix << std::endl;

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

    for (int i = 0; i < N_ - 1; ++i) {
      // Select the system ID
      int sys_id = sys_id_schedule_[i];

      // Load the params for this fe
      Eigen::VectorXd pk(54);
      pk[0] = (i == 0) ? first_element_duration_ : dt_;
      pk[1] = mu_;
      pk.segment(2, 12) = foot_pos_body_.row(i + 1);
      pk.segment(14, 12) = foot_pos_world_.row(i + 1);
      pk.segment(26, 12) = foot_vel_world_.row(i + 1);
      if (use_terrain_constraint) {
        for (int j = 0; j < num_feet_; j++) {
          Eigen::Vector3d foot_pos =
              get_primal_foot_state_var(w, i + 1).segment(3 * j, 3);

          pk(38 + j) = terrain_.atPosition("z_inpainted", foot_pos.head<2>(),
                                           interp_type_);
          pk(42 + 3 * j) = terrain_.atPosition(
              "normal_vectors_x", foot_pos.head<2>(), interp_type_);
          pk(42 + 3 * j) = terrain_.atPosition(
              "normal_vectors_y", foot_pos.head<2>(), interp_type_);
          pk(42 + 3 * j) = terrain_.atPosition(
              "normal_vectors_z", foot_pos.head<2>(), interp_type_);
        }
      } else {
        pk.segment(38, 4).fill(0);
        pk.segment(42, 12).fill(1);
      }

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

    for (size_t i = 0; i < N_ - 1; i++) {
      int sys_id = sys_id_schedule_[i];
      int g_slack = g_slack_vec_[i];

      // xmin wrt x
      get_slack_jac_var(values_matrix, i).segment(0, n_slack_vec_[i]).fill(1);

      // xmin wrt panic
      get_slack_jac_var(values_matrix, i)
          .segment(n_slack_vec_[i], n_slack_vec_[i])
          .fill(1);

      // xmax wrt x
      get_slack_jac_var(values_matrix, i)
          .segment(2 * n_slack_vec_[i], n_slack_vec_[i])
          .fill(1);

      // xmax wrt panic
      get_slack_jac_var(values_matrix, i)
          .segment(3 * n_slack_vec_[i], n_slack_vec_[i])
          .fill(-1);

      if (g_slack > 0) {
        int nnz_relaxed = relaxed_nnz_mat_(sys_id, JAC);

        // g_min wrt x
        for (int j = 0; j < nnz_relaxed; j++) {
          get_relaxed_dynamic_jac_var(values_matrix, i)
              .segment(0, nnz_relaxed)(j, 0) = get_dynamic_jac_var(
              values_matrix, i)(relaxed_idx_in_full_sparse_[sys_id][JAC][j], 0);
        }

        // g_min wrt slack
        get_relaxed_dynamic_jac_var(values_matrix, i)
            .segment(nnz_relaxed, g_slack)
            .fill(1);

        // g_max wrt x
        for (int j = 0; j < nnz_relaxed; j++) {
          get_relaxed_dynamic_jac_var(values_matrix, i)
              .segment(nnz_relaxed + g_slack, nnz_relaxed)(j, 0) =
              get_dynamic_jac_var(values_matrix, i)(
                  relaxed_idx_in_full_sparse_[sys_id][JAC][j], 0);
        }

        // g_max wrt slack
        get_relaxed_dynamic_jac_var(values_matrix, i)
            .segment(2 * nnz_relaxed + g_slack, g_slack)
            .fill(-1);
      }
    }
  }

  return true;
}

// Return the structure of the Jacobian
void quadNLP::compute_nnz_jac_g() {
  // Initialize nnz
  nnz_jac_g_ = 0;

  // Compute the number of nnz in the Jacobians over the horizon
  for (int i = 0; i < N_ - 1; ++i) {
    nnz_jac_g_ += nnz_mat_(sys_id_schedule_[i], JAC);
    nnz_jac_g_ += 4 * n_slack_vec_[i];
    if (g_slack_vec_[i] > 0) {
      nnz_jac_g_ += 2 * relaxed_nnz_mat_(sys_id_schedule_[i], JAC);
      nnz_jac_g_ += 2 * g_slack_vec_[i];
    }
  }

  // Add slack variables
  nnz_step_jac_g_.resize(N_ - 1);

  // Size the NLP constraint Jacobian sparsity structure
  iRow_jac_g_ = Eigen::VectorXi(nnz_jac_g_);
  jCol_jac_g_ = Eigen::VectorXi(nnz_jac_g_);

  for (int i = 0; i < N_ - 1; ++i) {
    int sys_id = sys_id_schedule_[i];
    nnz_step_jac_g_[i] = nnz_mat_(sys_id, JAC);
    // Each step we shift g constraints
    get_dynamic_jac_var(iRow_jac_g_, i) =
        (iRow_mat_[sys_id][JAC].array() + get_primal_constraint_idx(i))
            .matrix();

    // Each step we shift n states and m inputs
    get_dynamic_jac_var(jCol_jac_g_, i) =
        (jCol_mat_[sys_id][JAC].array() + get_primal_idx(i)).matrix();
  }

  for (size_t i = 0; i < N_ - 1; i++) {
    // Declare number of slack variables and relaxed constraints for this
    // element
    int n_slack = n_slack_vec_[i];
    int g_slack = g_slack_vec_[i];
    int sys_id = sys_id_schedule_[i];

    Eigen::ArrayXi slack_constraint_min_idx = Eigen::ArrayXi::LinSpaced(
        n_slack, get_slack_var_constraint_idx(i),
        get_slack_var_constraint_idx(i) + n_slack - 1);

    Eigen::ArrayXi slack_constraint_max_idx = Eigen::ArrayXi::LinSpaced(
        n_slack, get_slack_var_constraint_idx(i) + n_slack,
        get_slack_var_constraint_idx(i) + 2 * n_slack - 1);

    Eigen::ArrayXi x_idx =
        Eigen::ArrayXi::LinSpaced(n_slack, get_primal_state_idx(i + 1),
                                  get_primal_state_idx(i + 1) + n_slack - 1);

    Eigen::ArrayXi slack_var_min_idx = Eigen::ArrayXi::LinSpaced(
        n_slack, get_slack_state_idx(i), get_slack_state_idx(i) + n_slack - 1);

    Eigen::ArrayXi slack_var_max_idx =
        Eigen::ArrayXi::LinSpaced(n_slack, get_slack_state_idx(i) + n_slack,
                                  get_slack_state_idx(i) + 2 * n_slack - 1);

    // xmin wrt x
    get_slack_jac_var(iRow_jac_g_, i).segment(0, n_slack) =
        slack_constraint_min_idx;
    get_slack_jac_var(jCol_jac_g_, i).segment(0, n_slack) = x_idx;

    // xmin wrt panic
    get_slack_jac_var(iRow_jac_g_, i).segment(n_slack, n_slack) =
        slack_constraint_min_idx;
    get_slack_jac_var(jCol_jac_g_, i).segment(n_slack, n_slack) =
        slack_var_min_idx;

    // xmax wrt x
    get_slack_jac_var(iRow_jac_g_, i).segment(2 * n_slack, n_slack) =
        slack_constraint_max_idx;
    get_slack_jac_var(jCol_jac_g_, i).segment(2 * n_slack, n_slack) = x_idx;

    // xmax wrt panic
    get_slack_jac_var(iRow_jac_g_, i).segment(3 * n_slack, n_slack) =
        slack_constraint_max_idx;
    get_slack_jac_var(jCol_jac_g_, i).segment(3 * n_slack, n_slack) =
        slack_var_max_idx;

    // Get indices for relaxed constraints
    int nnz_relaxed = relaxed_nnz_mat_(sys_id, JAC);

    // Each step we shift by g_slack constraints (iRow_mat_relaxed_ must be
    // relative to the relaxed constraint Jacobian)
    if (g_slack > 0) {
      // Indices of relaxed constraint min wrt dynamic vars x (repeated)
      Eigen::ArrayXi g_min_idx_sparse =
          (iRow_mat_relaxed_[sys_id][JAC].array() +
           get_relaxed_constraint_idx(i))
              .matrix();
      // Indices of relaxed constraint min wrt slack vars (unique)
      Eigen::ArrayXi g_min_idx = Eigen::ArrayXi::LinSpaced(
          g_slack, get_relaxed_constraint_idx(i),
          get_relaxed_constraint_idx(i) + g_slack - 1);
      // Indices of relaxed constraint max wrt dynamic vars x (repeated)
      Eigen::ArrayXi g_max_idx_sparse =
          (iRow_mat_relaxed_[sys_id][JAC].array() +
           get_relaxed_constraint_idx(i) + g_slack)
              .matrix();
      // Indices of relaxed constraint max wrt slack vars (unique)
      Eigen::ArrayXi g_max_idx = Eigen::ArrayXi::LinSpaced(
          g_slack, get_relaxed_constraint_idx(i) + g_slack,
          get_relaxed_constraint_idx(i) + 2 * g_slack - 1);

      // Indices of dynamic vars
      Eigen::ArrayXi g_var_idx =
          (jCol_mat_relaxed_[sys_id][JAC].array() + get_primal_idx(i)).matrix();

      // Indices of slack vars for constraint min
      Eigen::ArrayXi g_slack_var_min_idx = Eigen::ArrayXi::LinSpaced(
          g_slack, get_slack_constraint_var_idx(i),
          get_slack_constraint_var_idx(i) + g_slack - 1);
      // Indices of slack vars for constraint max
      Eigen::ArrayXi g_slack_var_max_idx = Eigen::ArrayXi::LinSpaced(
          g_slack, get_slack_constraint_var_idx(i) + g_slack,
          get_slack_constraint_var_idx(i) + 2 * g_slack - 1);

      // g_min wrt x, row = constraint, column = var
      get_relaxed_dynamic_jac_var(iRow_jac_g_, i).segment(0, nnz_relaxed) =
          g_min_idx_sparse;
      get_relaxed_dynamic_jac_var(jCol_jac_g_, i).segment(0, nnz_relaxed) =
          g_var_idx;

      // g_min wrt slack
      get_relaxed_dynamic_jac_var(iRow_jac_g_, i)
          .segment(nnz_relaxed, g_slack) = g_min_idx;
      get_relaxed_dynamic_jac_var(jCol_jac_g_, i)
          .segment(nnz_relaxed, g_slack) = g_slack_var_min_idx;

      // g_max wrt x, row = constraint, column = var
      get_relaxed_dynamic_jac_var(iRow_jac_g_, i)
          .segment(nnz_relaxed + g_slack, nnz_relaxed) = g_max_idx_sparse;
      get_relaxed_dynamic_jac_var(jCol_jac_g_, i)
          .segment(nnz_relaxed + g_slack, nnz_relaxed) = g_var_idx;

      // g_max wrt slack
      get_relaxed_dynamic_jac_var(iRow_jac_g_, i)
          .segment(2 * nnz_relaxed + g_slack, g_slack) = g_max_idx;
      get_relaxed_dynamic_jac_var(jCol_jac_g_, i)
          .segment(2 * nnz_relaxed + g_slack, g_slack) = g_slack_var_max_idx;
    }
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

    for (int i = 0; i < N_ - 1; ++i) {
      // Select the system ID
      int sys_id = sys_id_schedule_[i];

      // Load the params for this fe
      Eigen::VectorXd pk(54);
      pk[0] = (i == 0) ? first_element_duration_ : dt_;
      pk[1] = mu_;
      pk.segment(2, 12) = foot_pos_body_.row(i + 1);
      pk.segment(14, 12) = foot_pos_world_.row(i + 1);
      pk.segment(26, 12) = foot_vel_world_.row(i + 1);
      if (use_terrain_constraint) {
        for (int j = 0; j < num_feet_; j++) {
          Eigen::Vector3d foot_pos =
              get_primal_foot_state_var(w, i + 1).segment(3 * j, 3);

          pk(38 + j) = terrain_.atPosition("z_inpainted", foot_pos.head<2>(),
                                           interp_type_);
          pk(42 + 3 * j) = terrain_.atPosition(
              "normal_vectors_x", foot_pos.head<2>(), interp_type_);
          pk(42 + 3 * j) = terrain_.atPosition(
              "normal_vectors_y", foot_pos.head<2>(), interp_type_);
          pk(42 + 3 * j) = terrain_.atPosition(
              "normal_vectors_z", foot_pos.head<2>(), interp_type_);
        }
      } else {
        pk.segment(38, 4).fill(0);
        pk.segment(42, 12).fill(1);
      }

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
      arg[1] = get_primal_constraint_vals(lambda_matrix, i).data();
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
    for (size_t i = 0; i < N_ - 1; i++) {
      Eigen::VectorXd Q_i =
          (Q_complex_ * std::pow(Q_temporal_factor_, i)).head(n_cost_vec_[i]);
      Eigen::VectorXd R_i =
          (R_complex_ * std::pow(R_temporal_factor_, i)).head(m_cost_vec_[i]);

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
  for (int i = 0; i < N_ - 1; ++i) {
    nnz_h_ += nnz_mat_(sys_id_schedule_[i], HESS);
  }

  // Add cost variables
  nnz_h_ = nnz_h_ + n_cost_vec_.sum() + m_cost_vec_.sum();

  iRow_h_ = Eigen::VectorXi(nnz_h_);
  jCol_h_ = Eigen::VectorXi(nnz_h_);
  nnz_step_hess_.resize(N_ - 1);

  for (int i = 0; i < N_ - 1; ++i) {
    int sys_id = sys_id_schedule_[i];
    nnz_step_hess_[i] = nnz_mat_(sys_id, HESS);
    // Each step we shift n states and m inputs
    get_dynamic_hess_var(iRow_h_, i) =
        (iRow_mat_[sys_id][HESS].array() + get_primal_idx(i)).matrix();
    get_dynamic_hess_var(jCol_h_, i) =
        (jCol_mat_[sys_id][HESS].array() + get_primal_idx(i)).matrix();
  }

  // Hessian from cost
  for (size_t i = 0; i < N_ - 1; i++) {
    Eigen::ArrayXi iRow_control_cost = Eigen::ArrayXi::LinSpaced(
        m_cost_vec_[i], get_primal_control_idx(i),
        get_primal_control_idx(i) + m_cost_vec_[i] - 1);
    Eigen::ArrayXi jCol_control_cost = iRow_control_cost;
    Eigen::ArrayXi iRow_state_cost = Eigen::ArrayXi::LinSpaced(
        n_cost_vec_[i], get_primal_state_idx(i + 1),
        get_primal_state_idx(i + 1) + n_cost_vec_[i] - 1);
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

  w0_.conservativeResize(n_vars_);
  z_L0_.conservativeResize(n_vars_);
  z_U0_.conservativeResize(n_vars_);
  lambda0_.conservativeResize(n_constraints_);

  int i_prev, i_prev_constr, n_shared, n_slack_shared, g_shared, g_slack_shared;
  // Update the initial state
  get_primal_state_var(w0_, 0) = x_current_;
  get_primal_state_var(z_L0_, 0).fill(0);
  get_primal_state_var(z_U0_, 0).fill(0);

  for (int i = 0; i < N_ - 1; i++) {
    i_prev = std::min(i + shift_idx, N_ - 2);

    n_shared = std::min(n_vec_[i + 1], nlp_prev.n_vec_[i_prev + 1]);
    n_slack_shared = std::min(n_slack_vec_[i], nlp_prev.n_slack_vec_[i_prev]);
    g_slack_shared = std::min(g_slack_vec_[i], nlp_prev.g_slack_vec_[i_prev]);
    g_shared = std::min(g_vec_[i], nlp_prev.g_vec_[i_prev]);

    // Update state and control for w0
    get_primal_state_var(w0_, i + 1).head(n_shared) =
        nlp_prev.get_primal_state_var(nlp_prev.w0_, i_prev + 1).head(n_shared);
    get_primal_control_var(w0_, i) =
        nlp_prev.get_primal_control_var(nlp_prev.w0_, i_prev);

    // Update state and control for z_L0_
    get_primal_state_var(z_L0_, i + 1).head(n_shared) =
        nlp_prev.get_primal_state_var(nlp_prev.z_L0_, i_prev + 1)
            .head(n_shared);
    get_primal_control_var(z_L0_, i) =
        nlp_prev.get_primal_control_var(nlp_prev.z_L0_, i_prev);

    // Update state and control for z_U0_
    get_primal_state_var(z_U0_, i + 1).head(n_shared) =
        nlp_prev.get_primal_state_var(nlp_prev.z_U0_, i_prev + 1)
            .head(n_shared);
    get_primal_control_var(z_U0_, i) =
        nlp_prev.get_primal_control_var(nlp_prev.z_U0_, i_prev);

    // Update constraint variables
    get_primal_constraint_vals(lambda0_, i).head(g_shared) =
        nlp_prev.get_primal_constraint_vals(nlp_prev.lambda0_, i_prev)
            .head(g_shared);

    // Update panic variables
    get_slack_state_var(w0_, i).segment(0, n_slack_shared) =
        nlp_prev.get_slack_state_var(nlp_prev.w0_, i_prev)
            .head(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_slack_state_var(w0_, i).segment(n_slack_vec_[i], n_slack_shared) =
        nlp_prev.get_slack_state_var(nlp_prev.w0_, i_prev)
            .tail(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_slack_constraint_var(w0_, i).head(2 * g_slack_shared) =
        nlp_prev.get_slack_constraint_var(nlp_prev.w0_, i_prev)
            .head(2 * g_slack_shared);

    get_slack_state_var(z_L0_, i).segment(0, n_slack_shared) =
        nlp_prev.get_slack_state_var(nlp_prev.z_L0_, i_prev)
            .head(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_slack_state_var(z_L0_, i).segment(n_slack_vec_[i], n_slack_shared) =
        nlp_prev.get_slack_state_var(nlp_prev.z_L0_, i_prev)
            .tail(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_slack_constraint_var(z_L0_, i).head(2 * g_slack_shared) =
        nlp_prev.get_slack_constraint_var(nlp_prev.z_L0_, i_prev)
            .head(2 * g_slack_shared);

    get_slack_state_var(z_U0_, i).segment(0, n_slack_shared) =
        nlp_prev.get_slack_state_var(nlp_prev.z_U0_, i_prev)
            .head(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_slack_state_var(z_U0_, i).segment(n_slack_vec_[i], n_slack_shared) =
        nlp_prev.get_slack_state_var(nlp_prev.z_U0_, i_prev)
            .tail(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_slack_constraint_var(z_U0_, i).head(2 * g_slack_shared) =
        nlp_prev.get_slack_constraint_var(nlp_prev.z_U0_, i_prev)
            .head(2 * g_slack_shared);

    get_slack_constraint_vals(lambda0_, i).segment(0, n_slack_shared) =
        nlp_prev.get_slack_constraint_vals(nlp_prev.lambda0_, i_prev)
            .head(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_slack_constraint_vals(lambda0_, i)
        .segment(n_slack_vec_[i], n_slack_shared) =
        nlp_prev.get_slack_constraint_vals(nlp_prev.lambda0_, i_prev)
            .tail(nlp_prev.n_slack_vec_[i_prev])
            .head(n_slack_shared);
    get_relaxed_primal_constraint_vals(lambda0_, i).head(2 * g_slack_shared) =
        nlp_prev.get_relaxed_primal_constraint_vals(nlp_prev.lambda0_, i_prev)
            .head(2 * g_slack_shared);

    // If this element is newly lifted, update with nominal otherwise leave
    // unchanged (may be one timestep old)
    if (n_vec_[i + 1] > nlp_prev.n_vec_[i + 1]) {
      std::cout << "Complexity at i = " << i
                << " increased, disabling warm start" << std::endl;
      warm_start_ = false;
      // mu0_ = 1e-1;

      if (n_vec_[i + 1] > n_shared) {
        std::cout << "No null data from prev solve, using nominal" << std::endl;
        get_primal_state_var(w0_, i + 1).tail(n_null_) = x_null_nom_;
        get_primal_state_var(z_L0_, i + 1).tail(n_null_).fill(1);
        get_primal_state_var(z_U0_, i + 1).tail(n_null_).fill(1);

        // Update panic variables if they also differ
        if (i < N_ - 1) {
          get_primal_constraint_vals(lambda0_, i)
              .tail(g_vec_[i] - g_shared)
              .fill(1000);

          if (n_slack_vec_[i] > n_slack_shared &&
              n_slack_vec_[i] > nlp_prev.n_slack_vec_[i]) {
            get_slack_state_var(w0_, i)
                .head(n_slack_vec_[i])
                .tail(n_null_)
                .fill(0);
            get_slack_state_var(w0_, i)
                .tail(n_slack_vec_[i])
                .tail(n_null_)
                .fill(0);
            get_slack_constraint_var(w0_, i).fill(0);
            get_slack_state_var(z_L0_, i)
                .head(n_slack_vec_[i])
                .tail(n_null_)
                .fill(1);
            get_slack_state_var(z_L0_, i)
                .tail(n_slack_vec_[i])
                .tail(n_null_)
                .fill(1);
            get_slack_constraint_var(z_L0_, i).fill(1);
            get_slack_state_var(z_U0_, i)
                .head(n_slack_vec_[i])
                .tail(n_null_)
                .fill(1);
            get_slack_state_var(z_U0_, i)
                .tail(n_slack_vec_[i])
                .tail(n_null_)
                .fill(1);
            get_slack_constraint_var(z_U0_, i).fill(1);
            get_slack_constraint_vals(lambda0_, i)
                .head(n_slack_vec_[i])
                .tail(n_null_)
                .fill(1000);
            get_relaxed_primal_constraint_vals(lambda0_, i).fill(1000);
          }
        }
      }
    }
  }
  // New contact
  double contact_sequence_diff =
      (contact_sequence_.col(N_ - 2) - contact_sequence_.col(N_ - 3)).norm();

  if (contact_sequence_diff > 1e-3) {  // New contact mode
    w0_.segment(get_primal_control_idx(N_ - 2), m_body_).fill(0);
    z_L0_.segment(get_primal_control_idx(N_ - 2), m_body_).fill(0);
    z_U0_.segment(get_primal_control_idx(N_ - 2), m_body_).fill(0);

    // Compute the number of contacts
    double num_contacts = contact_sequence_.col(N_ - 2).sum();

    // If there are some contacts, set the nominal input accordingly
    if (num_contacts > 0) {
      for (size_t i = 0; i < 4; i++) {
        if (contact_sequence_(i, N_ - 2) == 1) {
          w0_(get_primal_control_idx(N_ - 2) + 3 * i + 2, 0) =
              mass_ * grav_ / num_contacts;
        }
      }
    }
  }

  if (w0_.size() != n_vars_) {
    throw std::runtime_error("w0_.size() != n_vars_!");
  } else if (lambda0_.size() != n_constraints_) {
    throw std::runtime_error("lambda0_.size() != n_constraints_");
  }
  // Update the last state

  if (n_vec_[N_ - 1] > n_simple_) {
    get_primal_foot_state_var(w0_, N_ - 1).head(n_foot_ / 2) =
        foot_pos_world_.row(N_ - 1);
    get_primal_foot_state_var(w0_, N_ - 1).tail(n_foot_ / 2) =
        foot_vel_world_.row(N_ - 1);
  }
}

void quadNLP::update_solver(
    const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::VectorXi &adaptive_complexity_schedule,
    const Eigen::VectorXd &ground_height, const double &first_element_duration,
    const bool &same_plan_index, const bool &init) {
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

  for (size_t i = 0; i < N_ - 1; i++) {
    int idx;

    if (same_plan_index) {
      idx = i;
    } else {
      if (i == N_ - 2) {
        continue;
      }

      idx = i + 1;
    }

    if ((contact_sequence_prev.col(idx) - contact_sequence_.col(i))
            .cwiseAbs()
            .sum() > 1e-3) {
      // Contact change unexpectedly, update the warmstart info
      get_primal_body_control_var(w0_, idx).fill(0);
      get_primal_body_control_var(z_L0_, idx).fill(1);
      get_primal_body_control_var(z_U0_, idx).fill(1);

      // Compute the number of contactscontact_sequence_

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
  x_current_.segment(0, n_body_) = initial_state.head(n_body_);
  if (n_vec_[0] > n_simple_) {
    x_current_.segment(n_body_, n_foot_ / 2) = foot_pos_world_.row(0);
    x_current_.segment(n_body_ + n_foot_ / 2, n_foot_ / 2) =
        foot_vel_world_.row(0);
    x_current_.tail(n_joints_) = initial_state.tail(n_joints_);
  }

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
    get_primal_body_state_var(w0_, 0) = x_current_.head(n_body_);
    if (n_vec_[0] > n_simple_) {
      get_primal_foot_state_var(w0_, 0).head(n_foot_ / 2) =
          foot_pos_world_.row(0);
      get_primal_foot_state_var(w0_, 0).tail(n_foot_ / 2) =
          foot_vel_world_.row(0);
      get_primal_joint_state_var(w0_, 0) = x_current_.tail(n_joints_);
    }

    // Initialize future state predictions and controls
    for (size_t i = 0; i < N_ - 1; i++) {
      get_primal_body_state_var(w0_, i + 1) = x_reference_.col(i + 1);
      if (n_vec_[i + 1] == n_complex_) {
        get_primal_foot_state_var(w0_, i + 1).head(n_foot_ / 2) =
            foot_pos_world_.row(i + 1);
        get_primal_foot_state_var(w0_, i + 1).tail(n_foot_ / 2) =
            foot_vel_world_.row(i + 1);
        get_primal_state_var(w0_, i + 1).tail(n_joints_) = x_null_nom_;
      }

      double num_contacts = contact_sequence_.col(i).sum();
      if (num_contacts > 0) {
        for (size_t j = 0; j < 4; j++) {
          if (contact_schedule.at(i).at(j)) {
            get_primal_control_var(w0_, i)(2 + j * 3, 0) =
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
}

void quadNLP::update_structure() {
  // Update the complexity schedule
  quad_utils::FunctionTimer timer(__FUNCTION__);

  // Determine the (possibly new) size of the horizon
  N_ = std::min(adaptive_complexity_schedule_.size(),
                fixed_complexity_schedule_.size());

  // Update the complexity schedule by anding with fixed schedule
  Eigen::VectorXi complexity_schedule =
      adaptive_complexity_schedule_.head(N_).cwiseMax(
          fixed_complexity_schedule_.head(N_));

  // Resize vectors appropriately
  sys_id_schedule_.resize(N_ - 1);
  n_vec_.resize(N_);
  m_vec_.resize(N_ - 1);
  n_cost_vec_.resize(N_ - 1);
  m_cost_vec_.resize(N_ - 1);
  n_slack_vec_.resize(N_ - 1);
  g_vec_.resize(N_ - 1);
  g_slack_vec_.resize(N_ - 1);
  fe_idxs_.resize(N_ - 1);
  x_idxs_.resize(N_);
  u_idxs_.resize(N_ - 1);
  slack_state_var_idxs_.resize(N_ - 1);
  slack_constraint_var_idxs_.resize(N_ - 1);
  primal_constraint_idxs_.resize(N_ - 1);
  slack_var_constraint_idxs_.resize(N_ - 1);
  relaxed_constraint_idxs_.resize(N_ - 1);
  dynamic_jac_var_idxs_.resize(N_ - 1);
  panic_jac_var_idxs_.resize(N_ - 1);
  relaxed_dynamic_jac_var_idxs_.resize(N_ - 1);
  dynamic_hess_var_idxs_.resize(N_ - 1);
  cost_idxs_.resize(N_ - 1);

  // Initialize indexing veriables
  int curr_var_idx = 0;
  int curr_constr_idx = 0;
  int curr_jac_var_idx = 0;
  int curr_hess_var_idx = 0;

  // Loop through the horizon
  for (int i = 0; i < N_ - 1; i++) {
    // Determine the system to assign to this finite element
    if (complexity_schedule[i] == 0) {
      sys_id_schedule_[i] =
          (complexity_schedule[i + 1] == 0) ? SIMPLE : SIMPLE_TO_COMPLEX;
    } else {
      sys_id_schedule_[i] =
          (complexity_schedule[i + 1] == 1) ? COMPLEX : COMPLEX_TO_SIMPLE;
    }

    // Update the number of state vars and constraints for this fe
    n_vec_[i] = (complexity_schedule[i] == 1) ? n_complex_ : n_simple_;
    m_vec_[i] = (complexity_schedule[i] == 1) ? m_complex_ : m_simple_;
    n_cost_vec_[i] =
        (complexity_schedule[i + 1] == 1) ? n_cost_complex_ : n_cost_simple_;
    m_cost_vec_[i] =
        (complexity_schedule[i] == 1) ? m_cost_complex_ : m_cost_simple_;
    n_slack_vec_[i] =
        (complexity_schedule[i + 1] == 1) ? n_complex_ : n_simple_;

    g_vec_[i] = nrow_mat_(sys_id_schedule_[i], FUNC);
    g_slack_vec_[i] =
        (complexity_schedule[i + 1] == 1 && apply_slack_to_complex_constr_)
            ? relaxed_primal_constraint_idxs_in_element_.size()
            : 0;

    // Update the indices for the finite element, control, and state variables
    fe_idxs_[i] = curr_var_idx;
    x_idxs_[i] = curr_var_idx;
    curr_var_idx += n_vec_[i];
    u_idxs_[i] = curr_var_idx;
    curr_var_idx += m_vec_[i];

    // Update the indices for the constraints
    primal_constraint_idxs_[i] = curr_constr_idx;
    curr_constr_idx += g_vec_[i];

    // Update the indices for the sparsity patterns
    dynamic_jac_var_idxs_[i] = curr_jac_var_idx;
    curr_jac_var_idx += nnz_mat_(sys_id_schedule_[i], JAC);
    dynamic_hess_var_idxs_[i] = curr_hess_var_idx;
    curr_hess_var_idx += nnz_mat_(sys_id_schedule_[i], HESS);
  }

  n_vec_[N_ - 1] = (complexity_schedule[N_ - 1] == 1) ? n_complex_ : n_simple_;
  x_idxs_[N_ - 1] = curr_var_idx;
  curr_var_idx += n_vec_[N_ - 1];

  // Update the total number of primal variables
  n_vars_primal_ = curr_var_idx;

  // Update the slack variable indices
  for (int i = 0; i < N_ - 1; i++) {
    int n_slack = n_slack_vec_[i];

    // Log the index of first slack var in finite element and update the
    // current variable pointer with the number of state slack vars in this fe
    // (xmin, xmax)
    slack_state_var_idxs_[i] = curr_var_idx;
    curr_var_idx += 2 * n_slack;

    // Log the index of first slack constraint in finite element and update
    // current constraint pointer with total number of additional slack
    // constraints (only for state bounds since constraints are already added)
    slack_var_constraint_idxs_[i] = curr_constr_idx;
    curr_constr_idx += 2 * n_slack;

    // Log the index of first nonzero entry of the Jacobian corresponding to
    panic_jac_var_idxs_[i] = curr_jac_var_idx;
    curr_jac_var_idx += 4 * n_slack;

    cost_idxs_[i] = curr_hess_var_idx;
    curr_hess_var_idx += n_cost_vec_[i] + m_cost_vec_[i];
  }

  // Add the relaxed jacobian information at the end
  for (int i = 0; i < N_ - 1; i++) {
    int g_slack = g_slack_vec_[i];

    relaxed_constraint_idxs_[i] = curr_constr_idx;

    slack_constraint_var_idxs_[i] = curr_var_idx;

    relaxed_dynamic_jac_var_idxs_[i] = curr_jac_var_idx;

    if (g_slack > 0) {
      curr_constr_idx += 2 * g_slack;
      curr_var_idx += 2 * g_slack;
      curr_jac_var_idx +=
          2 * (relaxed_nnz_mat_(sys_id_schedule_[i], JAC) + g_slack);
    }
  }

  // Update the total number of vars and slack vars
  n_vars_ = curr_var_idx;
  n_vars_slack_ = n_vars_ - n_vars_primal_;

  // Check the number of primal and slack vars for correctness
  if (n_vars_primal_ != (n_vec_.sum() + m_vec_.sum())) {
    std::cout << "n_vec_ = " << n_vec_.transpose() << std::endl;
    std::cout << "m_vec_ = " << m_vec_.transpose() << std::endl;
    std::cout << "n_vars_primal_ = " << n_vars_primal_ << std::endl;
    std::cout << "n_vec_.sum() + m_vec_.sum() = " << n_vec_.sum() + m_vec_.sum()
              << std::endl;
    throw std::runtime_error("Number of primal vars is inconsistent");
  } else if (n_vars_slack_ !=
             (2 * n_slack_vec_.sum() + 2 * g_slack_vec_.sum())) {
    std::cout << "n_vars_slack_ = " << n_vars_slack_ << std::endl;
    std::cout << "2 * n_slack_vec_.sum() = " << (2 * n_slack_vec_.sum())
              << std::endl;
    std::cout << "2 * g_slack_vec_.sum() = " << (2 * g_slack_vec_.sum())
              << std::endl;
    throw std::runtime_error("Number of slack vars is inconsistent");
  }

  // Update the number of constraints
  n_constraints_ = curr_constr_idx;

  // Check the number of constraints
  if (n_constraints_ !=
      (g_vec_.sum() + 2 * n_slack_vec_.sum() + 2 * g_slack_vec_.sum())) {
    throw std::runtime_error("Number of constraints is inconsistent");
  }

  // Uncomment to check relaxed constraint nnz
  // std::cout << "iRow_mat_relaxed_[sys_id][JAC] = "
  //           << iRow_mat_relaxed_[COMPLEX][JAC] << std::endl;
  // std::cout << "jCol_mat_relaxed_[sys_id][JAC] = "
  //           << jCol_mat_relaxed_[COMPLEX][JAC] << std::endl;
  // std::cout << "iRow_mat_relaxed_[sys_id][JAC].size() = "
  //           << iRow_mat_relaxed_[COMPLEX][JAC].size() << std::endl;
  // std::cout << "jCol_mat_relaxed_[sys_id][JAC].size() = "
  //           << jCol_mat_relaxed_[COMPLEX][JAC].size() << std::endl;
  std::cout << "nnz_mat_ = " << nnz_mat_ << std::endl;

  compute_nnz_jac_g();
  compute_nnz_h();

  // Uncomment to check NLP variables
  // std::cout << "complexity_schedule.size() = " << complexity_schedule.size()
  //           << std::endl;
  // std::cout << "N_ = " << N_ << std::endl;
  std::cout << "cmplx_sch = " << complexity_schedule.transpose() << std::endl;
  std::cout << "sys_id_sch = " << sys_id_schedule_.transpose() << std::endl;
  std::cout << "n_vec_ =    " << n_vec_.transpose() << std::endl;
  std::cout << "m_vec_ =    " << m_vec_.transpose() << std::endl;
  std::cout << "n_cost_vec_ =    " << n_cost_vec_.transpose() << std::endl;
  std::cout << "m_cost_vec_ =    " << m_cost_vec_.transpose() << std::endl;
  std::cout << "n_slack_vec_ = " << n_slack_vec_.transpose() << std::endl;
  std::cout << "g_vec_ =      " << g_vec_.transpose() << std::endl;
  // std::cout << "g_vec_.sum() = " << g_vec_.sum() << std::endl;
  // std::cout << "g_slack_vec_ = " << g_slack_vec_.transpose() << std::endl;
  // std::cout << "g_vec_.sum() + n_vars_slack_ = " << g_vec_.sum() +
  // n_vars_slack_
  //           << std::endl;

  // std::cout << "n_constraints_ = " << n_constraints_ << std::endl;
  // std::cout << "n_vars_ = " << n_vars_ << std::endl;
  // std::cout << "n_vars_primal_ = " << n_vars_primal_ << std::endl;
  // std::cout << "n_vars_slack_ = " << n_vars_slack_ << std::endl;
}
