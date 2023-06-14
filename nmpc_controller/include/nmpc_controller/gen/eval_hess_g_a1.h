/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

extern "C" int eval_hess_g_a1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int eval_hess_g_a1_alloc_mem(void);
extern "C" int eval_hess_g_a1_init_mem(int mem);
extern "C" void eval_hess_g_a1_free_mem(int mem);
extern "C" int eval_hess_g_a1_checkout(void);
extern "C" void eval_hess_g_a1_release(int mem);
extern "C" void eval_hess_g_a1_incref(void);
extern "C" void eval_hess_g_a1_decref(void);
extern "C" casadi_int eval_hess_g_a1_n_out(void);
extern "C" casadi_int eval_hess_g_a1_n_in(void);
extern "C" casadi_real eval_hess_g_a1_default_in(casadi_int i);
extern "C" const char* eval_hess_g_a1_name_in(casadi_int i);
extern "C" const char* eval_hess_g_a1_name_out(casadi_int i);
extern "C" const casadi_int* eval_hess_g_a1_sparsity_in(casadi_int i);
extern "C" const casadi_int* eval_hess_g_a1_sparsity_out(casadi_int i);
extern "C" int eval_hess_g_a1_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
