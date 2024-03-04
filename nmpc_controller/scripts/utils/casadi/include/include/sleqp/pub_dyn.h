#ifndef SLEQP_PUB_DYN_H
#define SLEQP_PUB_DYN_H

#include "sleqp/pub_func.h"

/**
 * @file pub_dyn.h
 * @brief Definition of dynamic functions.
 *
 * @defgroup dynamic Dynamic nonlinear functions
 *
 * Dynamic functions extend the evaluation of the objective and constraints
 * to include limited accuracy. Specifically, the original objective
 * \f$ f(x) \f$ is replaced by a counterpart \f$ f(x; \epsilon) \f$
 * with an accuracy limited by \f$ \epsilon \f$  satisfying that
 *
 * \f[ | f(x; \epsilon) - f(x) | \leq \epsilon. \f]
 *
 * It is assumed that the value of \f$ \epsilon \f$ can be chosen freely,
 * trading off computational complexity against accuracy.
 * The same should be true for the constraints \f$ c(x) \f$.
 * Consequently, the combined function, given by
 * by
 *
 * \f[
 * \Pi(x, w^{f}, w^{c}) := w^f \cdot f(x)
 *   + \sum_{i=1}^{m} w^{c}_i \cdot
 * \left( \max(c_i(x) - u_i, 0) + \max(l_i - c_i(x), 0) \right)
 * \f]
 *
 * is subject to a limited accuracy (where \f$ w^{f} > 0 \f$
 * and \f$ w^{c} \in \R^{m}_{> 0} \f$ ). Specifically,
 * \f$ \Pi(x, w^{f}, w^{c}) \f$ is replaced by
 * \f$ \Pi(x, w^{f}, w^{c}; \epsilon^{f}, \epsilon^{c}) \f$
 * given by
 *
 * \f[
 * \Pi(x, w^{f}, w^{c}; \epsilon^{f}, \epsilon^{c})
 *  := w^f \cdot f(x; \epsilon^{f})
 *   + \sum_{i=1}^{m} w^{c}_i \cdot
 * \left( \max(c_i(x; \epsilon^{c}_i) - u_i, 0)
 *      + \max(l_i - c_i(x; \epsilon^{c}_i), 0) \right) \f]
 *
 * where \f$ \epsilon^{f} > 0 \f$
 * and \f$ \epsilon^{c} \in \R^{m}_{> 0} \f$.
 **/

/**
 * Evaluates the dynamic function at the current input vector. Given
 * the current error bound \f$ \epsilon > 0 \f$ and weights
 * \f$ w^{f} > 0 \f$ and \f$ w^{c} \in \R^{m}_{> 0} \f$,
 * the evaluation should satisfy
 *
 * \f[ \hat{\epsilon} := |\Pi(x, w^{f}, w^{c})
 *    - \Pi(x, w^{f}, w^{c}; \epsilon^{f}, \epsilon^{c} )| \leq \epsilon.
 * \f]
 *
 * The choice of the values of \f$ \epsilon^{f} \f$ and \f$ \epsilon^{c} \f$
 * (the distribution of the error with respect to the objective and constraints)
 * are up to the user.
 *
 * @param[in]     func        The function
 * @param[out]    obj_val     The function value \f$ f(x) \f$
 * @param[out]    cons_val    The value of the constraint function \f$ c(x) \f$
 * @param[out]    error       The actual error \f$ \hat{\epsilon} \f$
 * @param[in,out] func_data   The function data
 **/
typedef SLEQP_RETCODE (*SLEQP_DYN_FUNC_EVAL)(SleqpFunc* func,
                                             double* obj_val,
                                             SleqpVec* cons_val,
                                             double* error,
                                             void* func_data);

/**
 * Sets the error bound \f$ \epsilon \f$ for subsequent evaluations
 *
 * @param[in]     func            The function
 * @param[in]     error_bound     The error bound \f$ \epsilon \f$
 * @param[in,out] func_data       The function data
 **/
typedef SLEQP_RETCODE (*SLEQP_DYN_FUNC_SET_ERROR_BOUND)(SleqpFunc* func,
                                                        double error_bound,
                                                        void* func_data);

/**
 * Sets the objective weight \f$ w^{f} \f$ for subsequent evaluations
 *
 * @param[in]     func            The function
 * @param[in]     obj_weight      The objective weight \f$ w^{f} \f$
 * @param[in,out] func_data       The function data
 **/
typedef SLEQP_RETCODE (*SLEQP_DYN_FUNC_SET_OBJ_WEIGHT)(SleqpFunc* func,
                                                       double obj_weight,
                                                       void* func_data);

/**
 * Sets the constraint weights \epsilon^{c} for subsequent evaluations
 *
 * @param[in]     func            The function
 * @param[in]     cons_weights    The constraint weights \f$ \epsilon^{c} \f$
 * @param[in,out] func_data       The function data
 **/
typedef SLEQP_RETCODE (*SLEQP_DYN_FUNC_SET_CONS_WEIGHTS)(
  SleqpFunc* func,
  const double* cons_weights,
  void* func_data);

typedef struct
{
  SLEQP_FUNC_SET set_value;
  SLEQP_FUNC_NONZEROS nonzeros;
  SLEQP_DYN_FUNC_SET_ERROR_BOUND set_error_bound;
  SLEQP_DYN_FUNC_SET_OBJ_WEIGHT set_obj_weight;
  SLEQP_DYN_FUNC_SET_CONS_WEIGHTS set_cons_weights;
  SLEQP_DYN_FUNC_EVAL eval;
  SLEQP_FUNC_OBJ_GRAD obj_grad;
  SLEQP_FUNC_CONS_JAC cons_jac;
  SLEQP_FUNC_HESS_PROD hess_prod;
  SLEQP_FUNC_FREE func_free;
} SleqpDynFuncCallbacks;

/**
 * Creates a new dynamic function
 * @param[out] fstar            A pointer to the function to be created
 * @param[in]  callbacks        The dynamic function callbacks
 * @param[in]  num_variables    The number of variables
 * @param[in]  num_constraints  The number of constraints
 * @param[in]  func_data        The function data
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_dyn_func_create(SleqpFunc** fstar,
                      SleqpDynFuncCallbacks* callbacks,
                      int num_variables,
                      int num_constraints,
                      void* func_data);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_dyn_func_set_callbacks(SleqpFunc* func, SleqpDynFuncCallbacks* callbacks);

#endif /* SLEQP_PUB_DYN_H */
