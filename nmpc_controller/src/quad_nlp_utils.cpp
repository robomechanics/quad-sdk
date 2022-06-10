#include "nmpc_controller/quad_nlp.h"

using namespace Ipopt;

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

// Load casadi functions
void quadNLP::loadCasadiFuncs() {
  eval_vec_.resize(num_sys_id_);
  eval_work_vec_.resize(num_sys_id_);
  eval_incref_vec_.resize(num_sys_id_);
  eval_decref_vec_.resize(num_sys_id_);
  eval_checkout_vec_.resize(num_sys_id_);
  eval_release_vec_.resize(num_sys_id_);
  eval_sparsity_vec_.resize(num_sys_id_);

  for (int i = 0; i < num_sys_id_; i++) {
    eval_vec_[i].resize(num_func_id_);
    eval_work_vec_[i].resize(num_func_id_);
    eval_incref_vec_[i].resize(num_func_id_);
    eval_decref_vec_[i].resize(num_func_id_);
    eval_checkout_vec_[i].resize(num_func_id_);
    eval_release_vec_[i].resize(num_func_id_);
    eval_sparsity_vec_[i].resize(num_func_id_);
  }

  // Load basic leg controller functions for the spirit platform
  eval_vec_[SPIRIT][FUNC] = eval_g_spirit;
  eval_vec_[SPIRIT][JAC] = eval_jac_g_spirit;
  eval_vec_[SPIRIT][HESS] = eval_hess_g_spirit;
  eval_work_vec_[SPIRIT][FUNC] = eval_g_spirit_work;
  eval_work_vec_[SPIRIT][JAC] = eval_jac_g_spirit_work;
  eval_work_vec_[SPIRIT][HESS] = eval_hess_g_spirit_work;
  eval_incref_vec_[SPIRIT][FUNC] = eval_g_spirit_incref;
  eval_incref_vec_[SPIRIT][JAC] = eval_jac_g_spirit_incref;
  eval_incref_vec_[SPIRIT][HESS] = eval_hess_g_spirit_incref;
  eval_decref_vec_[SPIRIT][FUNC] = eval_g_spirit_decref;
  eval_decref_vec_[SPIRIT][JAC] = eval_jac_g_spirit_decref;
  eval_decref_vec_[SPIRIT][HESS] = eval_hess_g_spirit_decref;
  eval_checkout_vec_[SPIRIT][FUNC] = eval_g_spirit_checkout;
  eval_checkout_vec_[SPIRIT][JAC] = eval_jac_g_spirit_checkout;
  eval_checkout_vec_[SPIRIT][HESS] = eval_hess_g_spirit_checkout;
  eval_release_vec_[SPIRIT][FUNC] = eval_g_spirit_release;
  eval_release_vec_[SPIRIT][JAC] = eval_jac_g_spirit_release;
  eval_release_vec_[SPIRIT][HESS] = eval_hess_g_spirit_release;
  eval_sparsity_vec_[SPIRIT][FUNC] = eval_g_spirit_sparsity_out;
  eval_sparsity_vec_[SPIRIT][JAC] = eval_jac_g_spirit_sparsity_out;
  eval_sparsity_vec_[SPIRIT][HESS] = eval_hess_g_spirit_sparsity_out;

  // Load basic leg controller functions for the A1 platform
  eval_vec_[A1][FUNC] = eval_g_a1;
  eval_vec_[A1][JAC] = eval_jac_g_a1;
  eval_vec_[A1][HESS] = eval_hess_g_a1;
  eval_work_vec_[A1][FUNC] = eval_g_a1_work;
  eval_work_vec_[A1][JAC] = eval_jac_g_a1_work;
  eval_work_vec_[A1][HESS] = eval_hess_g_a1_work;
  eval_incref_vec_[A1][FUNC] = eval_g_a1_incref;
  eval_incref_vec_[A1][JAC] = eval_jac_g_a1_incref;
  eval_incref_vec_[A1][HESS] = eval_hess_g_a1_incref;
  eval_decref_vec_[A1][FUNC] = eval_g_a1_decref;
  eval_decref_vec_[A1][JAC] = eval_jac_g_a1_decref;
  eval_decref_vec_[A1][HESS] = eval_hess_g_a1_decref;
  eval_checkout_vec_[A1][FUNC] = eval_g_a1_checkout;
  eval_checkout_vec_[A1][JAC] = eval_jac_g_a1_checkout;
  eval_checkout_vec_[A1][HESS] = eval_hess_g_a1_checkout;
  eval_release_vec_[A1][FUNC] = eval_g_a1_release;
  eval_release_vec_[A1][JAC] = eval_jac_g_a1_release;
  eval_release_vec_[A1][HESS] = eval_hess_g_a1_release;
  eval_sparsity_vec_[A1][FUNC] = eval_g_a1_sparsity_out;
  eval_sparsity_vec_[A1][JAC] = eval_jac_g_a1_sparsity_out;
  eval_sparsity_vec_[A1][HESS] = eval_hess_g_a1_sparsity_out;

  // Load simple to simple functions - for adaptive complexity
  eval_vec_[SIMPLE_TO_SIMPLE][FUNC] = eval_g_leg_simple;
  eval_vec_[SIMPLE_TO_SIMPLE][JAC] = eval_jac_g_leg_simple;
  eval_vec_[SIMPLE_TO_SIMPLE][HESS] = eval_hess_g_leg_simple;
  eval_work_vec_[SIMPLE_TO_SIMPLE][FUNC] = eval_g_leg_simple_work;
  eval_work_vec_[SIMPLE_TO_SIMPLE][JAC] = eval_jac_g_leg_simple_work;
  eval_work_vec_[SIMPLE_TO_SIMPLE][HESS] = eval_hess_g_leg_simple_work;
  eval_incref_vec_[SIMPLE_TO_SIMPLE][FUNC] = eval_g_leg_simple_incref;
  eval_incref_vec_[SIMPLE_TO_SIMPLE][JAC] = eval_jac_g_leg_simple_incref;
  eval_incref_vec_[SIMPLE_TO_SIMPLE][HESS] = eval_hess_g_leg_simple_incref;
  eval_decref_vec_[SIMPLE_TO_SIMPLE][FUNC] = eval_g_leg_simple_decref;
  eval_decref_vec_[SIMPLE_TO_SIMPLE][JAC] = eval_jac_g_leg_simple_decref;
  eval_decref_vec_[SIMPLE_TO_SIMPLE][HESS] = eval_hess_g_leg_simple_decref;
  eval_checkout_vec_[SIMPLE_TO_SIMPLE][FUNC] = eval_g_leg_simple_checkout;
  eval_checkout_vec_[SIMPLE_TO_SIMPLE][JAC] = eval_jac_g_leg_simple_checkout;
  eval_checkout_vec_[SIMPLE_TO_SIMPLE][HESS] = eval_hess_g_leg_simple_checkout;
  eval_release_vec_[SIMPLE_TO_SIMPLE][FUNC] = eval_g_leg_simple_release;
  eval_release_vec_[SIMPLE_TO_SIMPLE][JAC] = eval_jac_g_leg_simple_release;
  eval_release_vec_[SIMPLE_TO_SIMPLE][HESS] = eval_hess_g_leg_simple_release;
  eval_sparsity_vec_[SIMPLE_TO_SIMPLE][FUNC] = eval_g_leg_simple_sparsity_out;
  eval_sparsity_vec_[SIMPLE_TO_SIMPLE][JAC] =
      eval_jac_g_leg_simple_sparsity_out;
  eval_sparsity_vec_[SIMPLE_TO_SIMPLE][HESS] =
      eval_hess_g_leg_simple_sparsity_out;

  // Load complex to complex functions - for adaptive complexity
  eval_vec_[COMPLEX_TO_COMPLEX][FUNC] = eval_g_leg_complex;
  eval_vec_[COMPLEX_TO_COMPLEX][JAC] = eval_jac_g_leg_complex;
  eval_vec_[COMPLEX_TO_COMPLEX][HESS] = eval_hess_g_leg_complex;
  eval_work_vec_[COMPLEX_TO_COMPLEX][FUNC] = eval_g_leg_complex_work;
  eval_work_vec_[COMPLEX_TO_COMPLEX][JAC] = eval_jac_g_leg_complex_work;
  eval_work_vec_[COMPLEX_TO_COMPLEX][HESS] = eval_hess_g_leg_complex_work;
  eval_incref_vec_[COMPLEX_TO_COMPLEX][FUNC] = eval_g_leg_complex_incref;
  eval_incref_vec_[COMPLEX_TO_COMPLEX][JAC] = eval_jac_g_leg_complex_incref;
  eval_incref_vec_[COMPLEX_TO_COMPLEX][HESS] = eval_hess_g_leg_complex_incref;
  eval_decref_vec_[COMPLEX_TO_COMPLEX][FUNC] = eval_g_leg_complex_decref;
  eval_decref_vec_[COMPLEX_TO_COMPLEX][JAC] = eval_jac_g_leg_complex_decref;
  eval_decref_vec_[COMPLEX_TO_COMPLEX][HESS] = eval_hess_g_leg_complex_decref;
  eval_checkout_vec_[COMPLEX_TO_COMPLEX][FUNC] = eval_g_leg_complex_checkout;
  eval_checkout_vec_[COMPLEX_TO_COMPLEX][JAC] = eval_jac_g_leg_complex_checkout;
  eval_checkout_vec_[COMPLEX_TO_COMPLEX][HESS] =
      eval_hess_g_leg_complex_checkout;
  eval_release_vec_[COMPLEX_TO_COMPLEX][FUNC] = eval_g_leg_complex_release;
  eval_release_vec_[COMPLEX_TO_COMPLEX][JAC] = eval_jac_g_leg_complex_release;
  eval_release_vec_[COMPLEX_TO_COMPLEX][HESS] = eval_hess_g_leg_complex_release;
  eval_sparsity_vec_[COMPLEX_TO_COMPLEX][FUNC] =
      eval_g_leg_complex_sparsity_out;
  eval_sparsity_vec_[COMPLEX_TO_COMPLEX][JAC] =
      eval_jac_g_leg_complex_sparsity_out;
  eval_sparsity_vec_[COMPLEX_TO_COMPLEX][HESS] =
      eval_hess_g_leg_complex_sparsity_out;

  // Load simple to complex functions - for adaptive complexity
  eval_vec_[SIMPLE_TO_COMPLEX][FUNC] = eval_g_leg_simple_to_complex;
  eval_vec_[SIMPLE_TO_COMPLEX][JAC] = eval_jac_g_leg_simple_to_complex;
  eval_vec_[SIMPLE_TO_COMPLEX][HESS] = eval_hess_g_leg_simple_to_complex;
  eval_work_vec_[SIMPLE_TO_COMPLEX][FUNC] = eval_g_leg_simple_to_complex_work;
  eval_work_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_work;
  eval_work_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_work;
  eval_incref_vec_[SIMPLE_TO_COMPLEX][FUNC] =
      eval_g_leg_simple_to_complex_incref;
  eval_incref_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_incref;
  eval_incref_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_incref;
  eval_decref_vec_[SIMPLE_TO_COMPLEX][FUNC] =
      eval_g_leg_simple_to_complex_decref;
  eval_decref_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_decref;
  eval_decref_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_decref;
  eval_checkout_vec_[SIMPLE_TO_COMPLEX][FUNC] =
      eval_g_leg_simple_to_complex_checkout;
  eval_checkout_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_checkout;
  eval_checkout_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_checkout;
  eval_release_vec_[SIMPLE_TO_COMPLEX][FUNC] =
      eval_g_leg_simple_to_complex_release;
  eval_release_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_release;
  eval_release_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_release;
  eval_sparsity_vec_[SIMPLE_TO_COMPLEX][FUNC] =
      eval_g_leg_simple_to_complex_sparsity_out;
  eval_sparsity_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_sparsity_out;
  eval_sparsity_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_sparsity_out;

  // Load complex to simple functions - for adaptive complexity
  eval_vec_[COMPLEX_TO_SIMPLE][FUNC] = eval_g_leg_complex_to_simple;
  eval_vec_[COMPLEX_TO_SIMPLE][JAC] = eval_jac_g_leg_complex_to_simple;
  eval_vec_[COMPLEX_TO_SIMPLE][HESS] = eval_hess_g_leg_complex_to_simple;
  eval_work_vec_[COMPLEX_TO_SIMPLE][FUNC] = eval_g_leg_complex_to_simple_work;
  eval_work_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_work;
  eval_work_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_work;
  eval_incref_vec_[COMPLEX_TO_SIMPLE][FUNC] =
      eval_g_leg_complex_to_simple_incref;
  eval_incref_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_incref;
  eval_incref_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_incref;
  eval_decref_vec_[COMPLEX_TO_SIMPLE][FUNC] =
      eval_g_leg_complex_to_simple_decref;
  eval_decref_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_decref;
  eval_decref_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_decref;
  eval_checkout_vec_[COMPLEX_TO_SIMPLE][FUNC] =
      eval_g_leg_complex_to_simple_checkout;
  eval_checkout_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_checkout;
  eval_checkout_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_checkout;
  eval_release_vec_[COMPLEX_TO_SIMPLE][FUNC] =
      eval_g_leg_complex_to_simple_release;
  eval_release_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_release;
  eval_release_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_release;
  eval_sparsity_vec_[COMPLEX_TO_SIMPLE][FUNC] =
      eval_g_leg_complex_to_simple_sparsity_out;
  eval_sparsity_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_sparsity_out;
  eval_sparsity_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_sparsity_out;

  // Resize containers
  nnz_mat_.resize(num_sys_id_, num_func_id_);
  relaxed_nnz_mat_.resize(num_sys_id_, num_func_id_);
  nrow_mat_.resize(num_sys_id_, num_func_id_);
  ncol_mat_.resize(num_sys_id_, num_func_id_);
  iRow_mat_.resize(num_sys_id_);
  jCol_mat_.resize(num_sys_id_);
  iRow_mat_relaxed_.resize(num_sys_id_);
  jCol_mat_relaxed_.resize(num_sys_id_);
  relaxed_idx_in_full_sparse_.resize(num_sys_id_);

  // Iterate through all functions and compute metadata
  for (int sys_id = 0; sys_id < num_sys_id_; sys_id++) {
    // Resize vector data
    iRow_mat_[sys_id].resize(num_func_id_);
    jCol_mat_[sys_id].resize(num_func_id_);
    iRow_mat_relaxed_[sys_id].resize(num_func_id_);
    jCol_mat_relaxed_[sys_id].resize(num_func_id_);
    relaxed_idx_in_full_sparse_[sys_id].resize(num_func_id_);

    for (int func_id = 0; func_id < num_func_id_; func_id++) {
      // Extract data
      const casadi_int* sp_i;
      sp_i = eval_sparsity_vec_[sys_id][func_id](0);
      casadi_int nrow = *sp_i++;
      casadi_int ncol = *sp_i++;
      const casadi_int* colind = sp_i;
      const casadi_int* row = sp_i + ncol + 1;
      casadi_int nnz = sp_i[ncol];

      nnz_mat_(sys_id, func_id) = nnz;
      nrow_mat_(sys_id, func_id) = nrow;
      ncol_mat_(sys_id, func_id) = ncol;
      iRow_mat_[sys_id][func_id].resize(nnz);
      jCol_mat_[sys_id][func_id].resize(nnz);
      iRow_mat_relaxed_[sys_id][func_id].resize(nnz);
      jCol_mat_relaxed_[sys_id][func_id].resize(nnz);
      relaxed_idx_in_full_sparse_[sys_id][func_id].resize(nnz);

      int idx = 0;
      int idx_relaxed = 0;
      for (int i = 0; i < ncol; ++i) {
        for (int j = colind[i]; j < colind[i + 1]; ++j) {
          iRow_mat_[sys_id][func_id](idx) = row[j];
          jCol_mat_[sys_id][func_id](idx) = i;

          if ((func_id == JAC) &&
              (row[j] >= relaxed_primal_constraint_idxs_in_element_(0)) &&
              (row[j] <=
               relaxed_primal_constraint_idxs_in_element_[g_relaxed_ - 1])) {
            iRow_mat_relaxed_[sys_id][func_id](idx_relaxed) =
                row[j] - relaxed_primal_constraint_idxs_in_element_(0);
            jCol_mat_relaxed_[sys_id][func_id](idx_relaxed) = i;
            relaxed_idx_in_full_sparse_[sys_id][func_id](idx_relaxed) = idx;
            idx_relaxed++;
          }

          idx += 1;
        }
      }

      iRow_mat_relaxed_[sys_id][func_id].conservativeResize(idx_relaxed);
      jCol_mat_relaxed_[sys_id][func_id].conservativeResize(idx_relaxed);
      relaxed_idx_in_full_sparse_[sys_id][func_id].conservativeResize(
          idx_relaxed);
      relaxed_nnz_mat_(sys_id, func_id) = idx_relaxed;
    }
  }
}

void quadNLP::loadConstraintNames() {
  // Clear the name map
  constr_names_.clear();
  constr_names_.resize(num_sys_id_);

  std::vector<std::string> constr_names;
  constr_names.resize(nrow_mat_(COMPLEX_TO_COMPLEX, FUNC));

  // Define number of variables to make looping easier
  int n_simple = 12;
  int n_null = 24;
  int n_complex = n_simple + n_null;

  int curr_constr = 0;
  for (int i = 0; i < n_simple; i++) {
    constr_names[curr_constr] = "eom_state_" + std::to_string(i);
    curr_constr++;
  }

  int num_feet = 4;
  for (int i = 0; i < num_feet; i++) {
    constr_names[curr_constr] = "friction_x_pos_foot_" + std::to_string(i);
    curr_constr++;

    constr_names[curr_constr] = "friction_x_neg_foot_" + std::to_string(i);
    curr_constr++;

    constr_names[curr_constr] = "friction_y_pos_foot_" + std::to_string(i);
    curr_constr++;

    constr_names[curr_constr] = "friction_y_neg_foot_" + std::to_string(i);
    curr_constr++;
  }

  for (int i = 0; i < num_feet; i++) {
    constr_names[curr_constr] = "eom_x_pos_foot_" + std::to_string(i);
    curr_constr++;

    constr_names[curr_constr] = "eom_y_pos_foot_" + std::to_string(i);
    curr_constr++;

    constr_names[curr_constr] = "eom_z_pos_foot_" + std::to_string(i);
    curr_constr++;
  }

  for (int i = 0; i < num_feet; i++) {
    constr_names[curr_constr] = "eom_dx_pos_foot_" + std::to_string(i);
    curr_constr++;

    constr_names[curr_constr] = "eom_dy_pos_foot_" + std::to_string(i);
    curr_constr++;

    constr_names[curr_constr] = "eom_dz_pos_foot_" + std::to_string(i);
    curr_constr++;
  }

  for (int i = 0; i < num_feet; i++) {
    constr_names[curr_constr] = "foot_height_leg_" + std::to_string(i);
    curr_constr++;
  }

  for (int i = 0; i < num_feet; i++) {
    constr_names[curr_constr] = "knee_height_leg_" + std::to_string(i);
    curr_constr++;
  }

  for (int i = 0; i < num_feet; i++) {
    constr_names[curr_constr] = "fk_pos_x_foot_" + std::to_string(i);
    curr_constr++;

    constr_names[curr_constr] = "fk_pos_y_foot_" + std::to_string(i);
    curr_constr++;

    constr_names[curr_constr] = "fk_pos_z_foot_" + std::to_string(i);
    curr_constr++;
  }

  for (int i = 0; i < num_feet; i++) {
    constr_names[curr_constr] = "fk_vel_x_foot_" + std::to_string(i);
    curr_constr++;

    constr_names[curr_constr] = "fk_vel_y_foot_" + std::to_string(i);
    curr_constr++;

    constr_names[curr_constr] = "fk_vel_z_foot_" + std::to_string(i);
    curr_constr++;
  }

  for (int i = 0; i < n_null / 2; i++) {
    constr_names[curr_constr] = "motor_model_pos_joint_" + std::to_string(i);
    curr_constr++;
  }

  for (int i = 0; i < n_null / 2; i++) {
    constr_names[curr_constr] = "motor_model_neg_joint_" + std::to_string(i);
    curr_constr++;
  }

  // Load the correct slice of the constraint names into the veector
  for (int i = 0; i < num_sys_id_; i++) {
    int num_constr = nrow_mat_(i, FUNC);
    constr_names_[i] = std::vector<std::string>(
        constr_names.begin(), constr_names.begin() + num_constr);
  }
}
