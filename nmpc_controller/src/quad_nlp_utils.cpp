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
  eval_vec_[SPIRIT][FUNC] = eval_g_spirit;
  eval_vec_[SPIRIT][JAC] = eval_jac_g_spirit;
  eval_vec_[SPIRIT][HESS] = eval_hess_g_spirit;
  eval_vec_[A1][FUNC] = eval_g_a1;
  eval_vec_[A1][JAC] = eval_jac_g_a1;
  eval_vec_[A1][HESS] = eval_hess_g_a1;

  // Load work map
  eval_work_vec_[SPIRIT][FUNC] = eval_g_spirit_work;
  eval_work_vec_[SPIRIT][JAC] = eval_jac_g_spirit_work;
  eval_work_vec_[SPIRIT][HESS] = eval_hess_g_spirit_work;
  eval_work_vec_[A1][FUNC] = eval_g_a1_work;
  eval_work_vec_[A1][JAC] = eval_jac_g_a1_work;
  eval_work_vec_[A1][HESS] = eval_hess_g_a1_work;

  // Load incref map
  eval_incref_vec_[SPIRIT][FUNC] = eval_g_spirit_incref;
  eval_incref_vec_[SPIRIT][JAC] = eval_jac_g_spirit_incref;
  eval_incref_vec_[SPIRIT][HESS] = eval_hess_g_spirit_incref;
  eval_incref_vec_[A1][FUNC] = eval_g_a1_incref;
  eval_incref_vec_[A1][JAC] = eval_jac_g_a1_incref;
  eval_incref_vec_[A1][HESS] = eval_hess_g_a1_incref;

  // Load decref map
  eval_decref_vec_[SPIRIT][FUNC] = eval_g_spirit_decref;
  eval_decref_vec_[SPIRIT][JAC] = eval_jac_g_spirit_decref;
  eval_decref_vec_[SPIRIT][HESS] = eval_hess_g_spirit_decref;
  eval_decref_vec_[A1][FUNC] = eval_g_a1_decref;
  eval_decref_vec_[A1][JAC] = eval_jac_g_a1_decref;
  eval_decref_vec_[A1][HESS] = eval_hess_g_a1_decref;

  // Load checkout map
  eval_checkout_vec_[SPIRIT][FUNC] = eval_g_spirit_checkout;
  eval_checkout_vec_[SPIRIT][JAC] = eval_jac_g_spirit_checkout;
  eval_checkout_vec_[SPIRIT][HESS] = eval_hess_g_spirit_checkout;
  eval_checkout_vec_[A1][FUNC] = eval_g_a1_checkout;
  eval_checkout_vec_[A1][JAC] = eval_jac_g_a1_checkout;
  eval_checkout_vec_[A1][HESS] = eval_hess_g_a1_checkout;

  // Load release map
  eval_release_vec_[SPIRIT][FUNC] = eval_g_spirit_release;
  eval_release_vec_[SPIRIT][JAC] = eval_jac_g_spirit_release;
  eval_release_vec_[SPIRIT][HESS] = eval_hess_g_spirit_release;
  eval_release_vec_[A1][FUNC] = eval_g_a1_release;
  eval_release_vec_[A1][JAC] = eval_jac_g_a1_release;
  eval_release_vec_[A1][HESS] = eval_hess_g_a1_release;

  // Load sparsity map
  eval_sparsity_vec_[SPIRIT][FUNC] = eval_g_spirit_sparsity_out;
  eval_sparsity_vec_[SPIRIT][JAC] = eval_jac_g_spirit_sparsity_out;
  eval_sparsity_vec_[SPIRIT][HESS] = eval_hess_g_spirit_sparsity_out;
  eval_sparsity_vec_[A1][FUNC] = eval_g_a1_sparsity_out;
  eval_sparsity_vec_[A1][JAC] = eval_jac_g_a1_sparsity_out;
  eval_sparsity_vec_[A1][HESS] = eval_hess_g_a1_sparsity_out;
}
