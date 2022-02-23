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

  // Load function map
  eval_vec_[LEG][FUNC] = eval_g_leg;
  eval_vec_[LEG][JAC] = eval_jac_g_leg;
  eval_vec_[LEG][HESS] = eval_hess_g_leg;
  eval_vec_[TAIL][FUNC] = eval_g_tail;
  eval_vec_[TAIL][JAC] = eval_jac_g_tail;
  eval_vec_[TAIL][HESS] = eval_hess_g_tail;
  eval_vec_[SIMPLE][FUNC] = eval_g_leg_simple;
  eval_vec_[SIMPLE][JAC] = eval_jac_g_leg_simple;
  eval_vec_[SIMPLE][HESS] = eval_hess_g_leg_simple;
  eval_vec_[COMPLEX][FUNC] = eval_g_leg_complex;
  eval_vec_[COMPLEX][JAC] = eval_jac_g_leg_complex;
  eval_vec_[COMPLEX][HESS] = eval_hess_g_leg_complex;
  eval_vec_[SIMPLE_TO_COMPLEX][FUNC] = eval_g_leg_simple_to_complex;
  eval_vec_[SIMPLE_TO_COMPLEX][JAC] = eval_jac_g_leg_simple_to_complex;
  eval_vec_[SIMPLE_TO_COMPLEX][HESS] = eval_hess_g_leg_simple_to_complex;
  eval_vec_[COMPLEX_TO_SIMPLE][FUNC] = eval_g_leg_complex_to_simple;
  eval_vec_[COMPLEX_TO_SIMPLE][JAC] = eval_jac_g_leg_complex_to_simple;
  eval_vec_[COMPLEX_TO_SIMPLE][HESS] = eval_hess_g_leg_complex_to_simple;

  // Load work map
  eval_work_vec_[LEG][FUNC] = eval_g_leg_work;
  eval_work_vec_[LEG][JAC] = eval_jac_g_leg_work;
  eval_work_vec_[LEG][HESS] = eval_hess_g_leg_work;
  eval_work_vec_[TAIL][FUNC] = eval_g_tail_work;
  eval_work_vec_[TAIL][JAC] = eval_jac_g_tail_work;
  eval_work_vec_[TAIL][HESS] = eval_hess_g_tail_work;
  eval_work_vec_[SIMPLE][FUNC] = eval_g_leg_simple_work;
  eval_work_vec_[SIMPLE][JAC] = eval_jac_g_leg_simple_work;
  eval_work_vec_[SIMPLE][HESS] = eval_hess_g_leg_simple_work;
  eval_work_vec_[COMPLEX][FUNC] = eval_g_leg_complex_work;
  eval_work_vec_[COMPLEX][JAC] = eval_jac_g_leg_complex_work;
  eval_work_vec_[COMPLEX][HESS] = eval_hess_g_leg_complex_work;
  eval_work_vec_[SIMPLE_TO_COMPLEX][FUNC] = eval_g_leg_simple_to_complex_work;
  eval_work_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_work;
  eval_work_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_work;
  eval_work_vec_[COMPLEX_TO_SIMPLE][FUNC] = eval_g_leg_complex_to_simple_work;
  eval_work_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_work;
  eval_work_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_work;

  // Load incref map
  eval_incref_vec_[LEG][FUNC] = eval_g_leg_incref;
  eval_incref_vec_[LEG][JAC] = eval_jac_g_leg_incref;
  eval_incref_vec_[LEG][HESS] = eval_hess_g_leg_incref;
  eval_incref_vec_[TAIL][FUNC] = eval_g_tail_incref;
  eval_incref_vec_[TAIL][JAC] = eval_jac_g_tail_incref;
  eval_incref_vec_[TAIL][HESS] = eval_hess_g_tail_incref;
  eval_incref_vec_[SIMPLE][FUNC] = eval_g_leg_simple_incref;
  eval_incref_vec_[SIMPLE][JAC] = eval_jac_g_leg_simple_incref;
  eval_incref_vec_[SIMPLE][HESS] = eval_hess_g_leg_simple_incref;
  eval_incref_vec_[COMPLEX][FUNC] = eval_g_leg_complex_incref;
  eval_incref_vec_[COMPLEX][JAC] = eval_jac_g_leg_complex_incref;
  eval_incref_vec_[COMPLEX][HESS] = eval_hess_g_leg_complex_incref;
  eval_incref_vec_[SIMPLE_TO_COMPLEX][FUNC] =
      eval_g_leg_simple_to_complex_incref;
  eval_incref_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_incref;
  eval_incref_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_incref;
  eval_incref_vec_[COMPLEX_TO_SIMPLE][FUNC] =
      eval_g_leg_complex_to_simple_incref;
  eval_incref_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_incref;
  eval_incref_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_incref;

  // Load decref map
  eval_decref_vec_[LEG][FUNC] = eval_g_leg_decref;
  eval_decref_vec_[LEG][JAC] = eval_jac_g_leg_decref;
  eval_decref_vec_[LEG][HESS] = eval_hess_g_leg_decref;
  eval_decref_vec_[TAIL][FUNC] = eval_g_tail_decref;
  eval_decref_vec_[TAIL][JAC] = eval_jac_g_tail_decref;
  eval_decref_vec_[TAIL][HESS] = eval_hess_g_tail_decref;
  eval_decref_vec_[SIMPLE][FUNC] = eval_g_leg_simple_decref;
  eval_decref_vec_[SIMPLE][JAC] = eval_jac_g_leg_simple_decref;
  eval_decref_vec_[SIMPLE][HESS] = eval_hess_g_leg_simple_decref;
  eval_decref_vec_[COMPLEX][FUNC] = eval_g_leg_complex_decref;
  eval_decref_vec_[COMPLEX][JAC] = eval_jac_g_leg_complex_decref;
  eval_decref_vec_[COMPLEX][HESS] = eval_hess_g_leg_complex_decref;
  eval_decref_vec_[SIMPLE_TO_COMPLEX][FUNC] =
      eval_g_leg_simple_to_complex_decref;
  eval_decref_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_decref;
  eval_decref_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_decref;
  eval_decref_vec_[COMPLEX_TO_SIMPLE][FUNC] =
      eval_g_leg_complex_to_simple_decref;
  eval_decref_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_decref;
  eval_decref_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_decref;

  // Load checkout map
  eval_checkout_vec_[LEG][FUNC] = eval_g_leg_checkout;
  eval_checkout_vec_[LEG][JAC] = eval_jac_g_leg_checkout;
  eval_checkout_vec_[LEG][HESS] = eval_hess_g_leg_checkout;
  eval_checkout_vec_[TAIL][FUNC] = eval_g_tail_checkout;
  eval_checkout_vec_[TAIL][JAC] = eval_jac_g_tail_checkout;
  eval_checkout_vec_[TAIL][HESS] = eval_hess_g_tail_checkout;
  eval_checkout_vec_[SIMPLE][FUNC] = eval_g_leg_simple_checkout;
  eval_checkout_vec_[SIMPLE][JAC] = eval_jac_g_leg_simple_checkout;
  eval_checkout_vec_[SIMPLE][HESS] = eval_hess_g_leg_simple_checkout;
  eval_checkout_vec_[COMPLEX][FUNC] = eval_g_leg_complex_checkout;
  eval_checkout_vec_[COMPLEX][JAC] = eval_jac_g_leg_complex_checkout;
  eval_checkout_vec_[COMPLEX][HESS] = eval_hess_g_leg_complex_checkout;
  eval_checkout_vec_[SIMPLE_TO_COMPLEX][FUNC] =
      eval_g_leg_simple_to_complex_checkout;
  eval_checkout_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_checkout;
  eval_checkout_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_checkout;
  eval_checkout_vec_[COMPLEX_TO_SIMPLE][FUNC] =
      eval_g_leg_complex_to_simple_checkout;
  eval_checkout_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_checkout;
  eval_checkout_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_checkout;

  // Load release map
  eval_release_vec_[LEG][FUNC] = eval_g_leg_release;
  eval_release_vec_[LEG][JAC] = eval_jac_g_leg_release;
  eval_release_vec_[LEG][HESS] = eval_hess_g_leg_release;
  eval_release_vec_[TAIL][FUNC] = eval_g_tail_release;
  eval_release_vec_[TAIL][JAC] = eval_jac_g_tail_release;
  eval_release_vec_[TAIL][HESS] = eval_hess_g_tail_release;
  eval_release_vec_[SIMPLE][FUNC] = eval_g_leg_simple_release;
  eval_release_vec_[SIMPLE][JAC] = eval_jac_g_leg_release;
  eval_release_vec_[SIMPLE][HESS] = eval_hess_g_leg_simple_release;
  eval_release_vec_[COMPLEX][FUNC] = eval_g_leg_complex_release;
  eval_release_vec_[COMPLEX][JAC] = eval_jac_g_leg_complex_release;
  eval_release_vec_[COMPLEX][HESS] = eval_hess_g_leg_complex_release;
  eval_release_vec_[SIMPLE_TO_COMPLEX][FUNC] =
      eval_g_leg_simple_to_complex_release;
  eval_release_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_release;
  eval_release_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_release;
  eval_release_vec_[COMPLEX_TO_SIMPLE][FUNC] =
      eval_g_leg_complex_to_simple_release;
  eval_release_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_release;
  eval_release_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_release;

  // Load sparsity map
  eval_sparsity_vec_[LEG][FUNC] = eval_g_leg_sparsity_out;
  eval_sparsity_vec_[LEG][JAC] = eval_jac_g_leg_sparsity_out;
  eval_sparsity_vec_[LEG][HESS] = eval_hess_g_leg_sparsity_out;
  eval_sparsity_vec_[TAIL][FUNC] = eval_g_tail_sparsity_out;
  eval_sparsity_vec_[TAIL][JAC] = eval_jac_g_tail_sparsity_out;
  eval_sparsity_vec_[TAIL][HESS] = eval_hess_g_tail_sparsity_out;
  eval_sparsity_vec_[SIMPLE][FUNC] = eval_g_leg_simple_sparsity_out;
  eval_sparsity_vec_[SIMPLE][JAC] = eval_jac_g_leg_simple_sparsity_out;
  eval_sparsity_vec_[SIMPLE][HESS] = eval_hess_g_leg_simple_sparsity_out;
  eval_sparsity_vec_[COMPLEX][FUNC] = eval_g_leg_complex_sparsity_out;
  eval_sparsity_vec_[COMPLEX][JAC] = eval_jac_g_leg_complex_sparsity_out;
  eval_sparsity_vec_[COMPLEX][HESS] = eval_hess_g_leg_complex_sparsity_out;
  eval_sparsity_vec_[SIMPLE_TO_COMPLEX][FUNC] =
      eval_g_leg_simple_to_complex_sparsity_out;
  eval_sparsity_vec_[SIMPLE_TO_COMPLEX][JAC] =
      eval_jac_g_leg_simple_to_complex_sparsity_out;
  eval_sparsity_vec_[SIMPLE_TO_COMPLEX][HESS] =
      eval_hess_g_leg_simple_to_complex_sparsity_out;
  eval_sparsity_vec_[COMPLEX_TO_SIMPLE][FUNC] =
      eval_g_leg_complex_to_simple_sparsity_out;
  eval_sparsity_vec_[COMPLEX_TO_SIMPLE][JAC] =
      eval_jac_g_leg_complex_to_simple_sparsity_out;
  eval_sparsity_vec_[COMPLEX_TO_SIMPLE][HESS] =
      eval_hess_g_leg_complex_to_simple_sparsity_out;

  // Resize containers
  nnz_mat_.resize(num_sys_id_, num_func_id_);
  nrow_mat_.resize(num_sys_id_, num_func_id_);
  ncol_mat_.resize(num_sys_id_, num_func_id_);
  iRow_mat_.resize(num_sys_id_);
  jCol_mat_.resize(num_sys_id_);

  // Iterate through all functions and compute metadata
  for (int sys_id = 0; sys_id < num_sys_id_; sys_id++) {
    // Resize vector data
    iRow_mat_[sys_id].resize(num_func_id_);
    jCol_mat_[sys_id].resize(num_func_id_);

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

      int idx = 0;
      for (int i = 0; i < ncol; ++i) {
        for (int j = colind[i]; j < colind[i + 1]; ++j) {
          iRow_mat_[sys_id][func_id](idx) = row[j];
          jCol_mat_[sys_id][func_id](idx) = i;

          idx += 1;
        }
      }
    }
  }
}
