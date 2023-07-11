/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

extern "C" int eval_hess_g_new_platform(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int eval_hess_g_new_platform_alloc_mem(void);
extern "C" int eval_hess_g_new_platform_init_mem(int mem);
extern "C" void eval_hess_g_new_platform_free_mem(int mem);
extern "C" int eval_hess_g_new_platform_checkout(void);
extern "C" void eval_hess_g_new_platform_release(int mem);
extern "C" void eval_hess_g_new_platform_incref(void);
extern "C" void eval_hess_g_new_platform_decref(void);
extern "C" casadi_int eval_hess_g_new_platform_n_in(void);
extern "C" casadi_int eval_hess_g_new_platform_n_out(void);
extern "C" casadi_real eval_hess_g_new_platform_default_in(casadi_int i);
extern "C" const char* eval_hess_g_new_platform_name_in(casadi_int i);
extern "C" const char* eval_hess_g_new_platform_name_out(casadi_int i);
extern "C" const casadi_int* eval_hess_g_new_platform_sparsity_in(casadi_int i);
extern "C" const casadi_int* eval_hess_g_new_platform_sparsity_out(casadi_int i);
extern "C" int eval_hess_g_new_platform_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define eval_hess_g_new_platform_SZ_ARG 3
#define eval_hess_g_new_platform_SZ_RES 1
#define eval_hess_g_new_platform_SZ_IW 0
#define eval_hess_g_new_platform_SZ_W 83
