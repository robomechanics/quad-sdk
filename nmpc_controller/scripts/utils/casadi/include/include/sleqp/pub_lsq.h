#ifndef SLEQP_PUB_LSQ_H
#define SLEQP_PUB_LSQ_H

/**
 * @file pub_lsq.h
 * @brief Defintion of least squares functions.
 **/

#include "sleqp/export.h"
#include "sleqp/pub_iterate.h"
#include "sleqp/pub_problem.h"
#include "sleqp/sparse/pub_vec.h"

/**
 * @defgroup least_squares Nonlinear least-squares problems
 *
 * A least-squares function is defined in terms of residuals
 * \f$ r : \R^n \to \R^k \f$
 *
 * yielding the objective
 *
 * \f[
 *     f(x) := \frac{1}{2} \|r(x)\|^2
 * \f]
 *
 * subjecting to the usual constraints. During the optimization, the
 * Hessian being used is not that of the original Lagrangean, but
 * instead the Levenberg-Marquardt approximation of the Hessian of the
 * objective \f$ f \f$, given as
 *
 * \f[
 * H_f(x) \approx J^{T}(x) \cdot J(x).
 * \f]
 *
 * Optionally, the user can provide a regularization
 * factor \f$ \lmfact > 0 \f$, which will add a term
 * of \f$ \lmfact \cdot I_{n} \f$ to the Hessian.
 **/

/**
 * Queries the number of nonzeros of the function at the
 * current primal point.
 *
 * @param[in]     func            The function
 * @param[out]    residual_nnz    The number of nonzeros of the residuals
 *\f$r(x)\f$
 * @param[out]    jac_fwd_nnz     The number of nonzeros of
 *\f$J_r(x)d\f$
 * @param[out]    jac_adj_nnz     The number of nonzeros of
 *\f$d^{T}J_r(x)\f$
 * @param[out]    cons_val_nnz    The number of nonzeros of the constraint
 *function \f$ c(x) \f$
 * @param[out]    cons_jac_nnz    The number of nonzeros of the constraint
 *Jacobian \f$ J_c(x) \f$
 * @param[out]    cons_jac_nnz    The number of nonzeros of Hessian products
 **/
typedef SLEQP_RETCODE (*SLEQP_LSQ_NONZEROS)(SleqpFunc* func,
                                            int* residual_nnz,
                                            int* jac_fwd_nnz,
                                            int* jac_adj_nnz,
                                            int* cons_val_nnz,
                                            int* cons_jac_nnz,
                                            void* func_data);

/**
 * Evaluates the residuals \f$ r(x) \f$ at the current primal point.
 *
 * @param[in]     func              The function
 * @param[out]    residual          The resulting residual
 * @param[in,out] func_data         The function data
 *
 */
typedef SLEQP_RETCODE (*SLEQP_LSQ_RESIDUALS)(SleqpFunc* func,
                                             SleqpVec* residual,
                                             void* func_data);

/**
 * Evaluates the forward product of the Jacobian of the residuals at the
 * current primal point \f$ J_r(x) \f$ with a direction
 * \f$ d \in \R^n \f$.
 *
 * @param[in]     func              The function
 * @param[in]     forward_direction The direction \f$ d \f$
 * @param[out]    product           The resulting product
 * @param[in,out] func_data         The function data
 *
 */
typedef SLEQP_RETCODE (*SLEQP_LSQ_JAC_FORWARD)(
  SleqpFunc* func,
  const SleqpVec* forward_direction,
  SleqpVec* product,
  void* func_data);

/**
 * Evaluates the adjoint product of the Jacobian of the residuals at the
 * current primal point \f$ J_r(x) \f$ with a
 * direction \f$ d \in \R^k \f$.
 *
 * @param[in]     func              The function
 * @param[in]     adjoint_direction The direction \f$ d \f$
 * @param[out]    product           The resulting product
 * @param[in,out] func_data         The function data
 *
 */
typedef SLEQP_RETCODE (*SLEQP_LSQ_JAC_ADJOINT)(
  SleqpFunc* func,
  const SleqpVec* adjoint_direction,
  SleqpVec* product,
  void* func_data);

typedef struct
{
  SLEQP_FUNC_SET set_value;
  SLEQP_LSQ_NONZEROS lsq_nonzeros;
  SLEQP_LSQ_RESIDUALS lsq_residuals;
  SLEQP_LSQ_JAC_FORWARD lsq_jac_forward;
  SLEQP_LSQ_JAC_ADJOINT lsq_jac_adjoint;
  SLEQP_FUNC_CONS_VAL cons_val;
  SLEQP_FUNC_CONS_JAC cons_jac;
  SLEQP_FUNC_FREE func_free;
} SleqpLSQCallbacks;

/**
 * Creates a least-squares function.
 *
 * @param[out] fstar           A pointer to the function to be created
 * @param[in]  callbacks       Required callbacks
 * @param[in]  num_variables   The number \f$ n \f$ of variables
 * @param[in]  num_constraints The number \f$ m \f$ of constraints
 * @param[in]  num_residuals   The number \f$ k \f$ of residuals
 * @param[in]  lm_factor       The regularization factor \f$ \lmfact \f$
 * @param[in]  params          Numerical parameters
 * @param[in]  func_data       User-provided function data
 *
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_lsq_func_create(SleqpFunc** fstar,
                      SleqpLSQCallbacks* callbacks,
                      int num_variables,
                      int num_constraints,
                      int num_residuals,
                      double lm_factor,
                      SleqpSettings* settings,
                      void* func_data);

/**
 * Sets the callbacks of this LSQ function to the specified ones
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_lsq_func_set_callbacks(SleqpFunc* func, SleqpLSQCallbacks* callbacks);

/**
 * Sets the regularization factor \f$ \lmfact \f$
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_lsq_func_set_lm_factor(SleqpFunc* func, double lm_factor);

#endif /* SLEQP_PUB_LSQ_H */
