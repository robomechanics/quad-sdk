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

  // Load work map
  eval_work_vec_[LEG][FUNC] = eval_g_leg_work;
  eval_work_vec_[LEG][JAC] = eval_jac_g_leg_work;
  eval_work_vec_[LEG][HESS] = eval_hess_g_leg_work;
  eval_work_vec_[TAIL][FUNC] = eval_g_tail_work;
  eval_work_vec_[TAIL][JAC] = eval_jac_g_tail_work;
  eval_work_vec_[TAIL][HESS] = eval_hess_g_tail_work;

  // Load incref map
  eval_incref_vec_[LEG][FUNC] = eval_g_leg_incref;
  eval_incref_vec_[LEG][JAC] = eval_jac_g_leg_incref;
  eval_incref_vec_[LEG][HESS] = eval_hess_g_leg_incref;
  eval_incref_vec_[TAIL][FUNC] = eval_g_tail_incref;
  eval_incref_vec_[TAIL][JAC] = eval_jac_g_tail_incref;
  eval_incref_vec_[TAIL][HESS] = eval_hess_g_tail_incref;

  // Load decref map
  eval_decref_vec_[LEG][FUNC] = eval_g_leg_decref;
  eval_decref_vec_[LEG][JAC] = eval_jac_g_leg_decref;
  eval_decref_vec_[LEG][HESS] = eval_hess_g_leg_decref;
  eval_decref_vec_[TAIL][FUNC] = eval_g_tail_decref;
  eval_decref_vec_[TAIL][JAC] = eval_jac_g_tail_decref;
  eval_decref_vec_[TAIL][HESS] = eval_hess_g_tail_decref;

  // Load checkout map
  eval_checkout_vec_[LEG][FUNC] = eval_g_leg_checkout;
  eval_checkout_vec_[LEG][JAC] = eval_jac_g_leg_checkout;
  eval_checkout_vec_[LEG][HESS] = eval_hess_g_leg_checkout;
  eval_checkout_vec_[TAIL][FUNC] = eval_g_tail_checkout;
  eval_checkout_vec_[TAIL][JAC] = eval_jac_g_tail_checkout;
  eval_checkout_vec_[TAIL][HESS] = eval_hess_g_tail_checkout;

  // Load release map
  eval_release_vec_[LEG][FUNC] = eval_g_leg_release;
  eval_release_vec_[LEG][JAC] = eval_jac_g_leg_release;
  eval_release_vec_[LEG][HESS] = eval_hess_g_leg_release;
  eval_release_vec_[TAIL][FUNC] = eval_g_tail_release;
  eval_release_vec_[TAIL][JAC] = eval_jac_g_tail_release;
  eval_release_vec_[TAIL][HESS] = eval_hess_g_tail_release;

  // Load sparsity map
  eval_sparsity_vec_[LEG][FUNC] = eval_g_leg_sparsity_out;
  eval_sparsity_vec_[LEG][JAC] = eval_jac_g_leg_sparsity_out;
  eval_sparsity_vec_[LEG][HESS] = eval_hess_g_leg_sparsity_out;
  eval_sparsity_vec_[TAIL][FUNC] = eval_g_tail_sparsity_out;
  eval_sparsity_vec_[TAIL][JAC] = eval_jac_g_tail_sparsity_out;
  eval_sparsity_vec_[TAIL][HESS] = eval_hess_g_tail_sparsity_out;
}
