#ifndef PUB_FUNC_H
#define PUB_FUNC_H

/**
 * @file pub_func.h
 * @brief Definition of nonlinear functions.
 **/

#include "sleqp/pub_hess_struct.h"
#include "sparse/pub_mat.h"
#include "sparse/pub_vec.h"

/**
 * @defgroup function Nonlinear functions
 * @{
 *
 * A function is given by an objective
 * \f$ f : \R^n \to \R \f$
 * and constraints
 * \f$ c : \R^n \to \R^{m} \f$.
 *
 * The functions are assumed to be twice continuously
 * differentiable. Function, gradient, and Hessian evaluations are
 * supposed to be provided by the user in the form of callbacks.
 **/

typedef struct SleqpFunc SleqpFunc;

/**
 * The reason for setting the primal point
 **/
typedef enum
{
  /** No reason **/
  SLEQP_VALUE_REASON_NONE,
  /** Initial step **/
  SLEQP_VALUE_REASON_INIT,
  /** Checking derivatives **/
  SLEQP_VALUE_REASON_CHECKING_DERIV,
  /** Accepted trial step **/
  SLEQP_VALUE_REASON_ACCEPTED_ITERATE,
  /** New trial step **/
  SLEQP_VALUE_REASON_TRYING_ITERATE,
  /** New SOC trial step **/
  SLEQP_VALUE_REASON_TRYING_SOC_ITERATE,
  /** Rejected trial step **/
  SLEQP_VALUE_REASON_REJECTED_ITERATE,
} SLEQP_VALUE_REASON;

/**
 * Type of the function
 **/
typedef enum
{
  /** Regular function **/
  SLEQP_FUNC_TYPE_REGULAR,
  /** Least-squares function @see least_squares **/
  SLEQP_FUNC_TYPE_LSQ,
  /** Dynamic function @see dynamic **/
  SLEQP_FUNC_TYPE_DYNAMIC
} SLEQP_FUNC_TYPE;

/**
 * Sets the current primal point
 *
 * @param[in]     func            The function
 * @param[in]     value           The value
 * @param[in]     reason          The reason for setting \f$ x \f$
 * @param[out]    reject          Whether to manually reject the step
 * @param[in,out] func_data       The function data
 **/
typedef SLEQP_RETCODE (*SLEQP_FUNC_SET)(SleqpFunc* func,
                                        SleqpVec* value,
                                        SLEQP_VALUE_REASON reason,
                                        bool* reject,
                                        void* func_data);

/**
 * Queries the number of nonzeros of the function at the
 * current primal point
 *
 * @param[in]     func            The function
 * @param[out]    obj_grad_nnz    The number of nonzeros of the objective
 *gradient \f$ \nabla f(x) \f$
 * @param[out]    cons_val_nnz    The number of nonzeros of the constraint
 *function \f$ c(x) \f$
 * @param[out]    cons_jac_nnz    The number of nonzeros of the constraint
 *Jacobian \f$ J_c(x) \f$
 * @param[out]    cons_jac_nnz    The number of nonzeros of Hessian products
 * \f$ \nabla_{xx} L(x, \lambda) \f$
 * @param[in,out] func_data       The function data
 **/
typedef SLEQP_RETCODE (*SLEQP_FUNC_NONZEROS)(SleqpFunc* func,
                                             int* obj_grad_nnz,
                                             int* cons_val_nnz,
                                             int* cons_jac_nnz,
                                             int* hess_prod_nnz,
                                             void* func_data);

/**
 * Evaluates the objective \f$ f \f$ at the current primal point
 *
 * @param[in]     func            The function
 * @param[out]    obj_val         The objective value \f$ f(x) \f$
 * @param[in,out] func_data       The function data
 **/
typedef SLEQP_RETCODE (*SLEQP_FUNC_OBJ_VAL)(SleqpFunc* func,
                                            double* obj_val,
                                            void* func_data);

/**
 * Evaluates the objective gradient \f$ \nabla f \f$ at the current
 * primal point
 *
 * @param[in]     func            The function
 * @param[out]    obj_grad        The objective gradient \f$ \nabla f(x) \f$
 * @param[in,out] func_data       The function data
 **/
typedef SLEQP_RETCODE (*SLEQP_FUNC_OBJ_GRAD)(SleqpFunc* func,
                                             SleqpVec* obj_grad,
                                             void* func_data);

/**
 * Evaluates the constraints \f$ c \f$ at the current primal point
 *
 * @param[in]     func            The function
 * @param[out]    cons_val        The value of the constraint function \f$ c(x)
 *\f$
 * @param[in,out] func_data       The function data
 **/
typedef SLEQP_RETCODE (*SLEQP_FUNC_CONS_VAL)(SleqpFunc* func,
                                             SleqpVec* cons_val,
                                             void* func_data);

/**
 * Evaluates the constraing Jacobian \f$ J_c \f$ at the current primal point
 *
 * @param[in]     func            The function
 * @param[out]    cons_jac        The constraint Jacobian \f$ J_c(x) \f$
 * @param[in,out] func_data       The function data
 **/
typedef SLEQP_RETCODE (*SLEQP_FUNC_CONS_JAC)(SleqpFunc* func,
                                             SleqpMat* cons_jac,
                                             void* func_data);

/**
 * Evaluates the product of the Hessian of the Lagrangian function.
 * The Lagrangian function is given by:
 *
 * \f[
 * L(x, \lambda) := f(x) + \langle \lambda, c(x) \rangle
 * \f]
 *
 * The product with a direction \f$ d \f$ is then:
 * \f[
 * \nabla_{xx} L(x, \lambda) d
 * = \left( \nabla_{xx} f(x) d
 *   + \sum_{i=1}^{m} \lambda_i  \nabla_{xx} c_i(x) d \right)
 * \f]
 *
 * @param[in]     func              The function
 * @param[in]     direction         The direction \f$ d \f$
 * @param[in]     cons_duals        The values \f$ \lambda \f$
 * @param[out]    product           The resulting product
 * @param[in,out] func_data         The function data
 *
 **/
typedef SLEQP_RETCODE (*SLEQP_FUNC_HESS_PROD)(SleqpFunc* func,
                                              const SleqpVec* direction,
                                              const SleqpVec* cons_duals,
                                              SleqpVec* product,
                                              void* func_data);

/**
 * Cleans up any allocated memory stored in the function data.
 *
 * @param[in,out] func_data  The function data
 *
 **/
typedef SLEQP_RETCODE (*SLEQP_FUNC_FREE)(void* func_data);

typedef struct
{
  SLEQP_FUNC_SET set_value;
  SLEQP_FUNC_NONZEROS nonzeros;
  SLEQP_FUNC_OBJ_VAL obj_val;
  SLEQP_FUNC_OBJ_GRAD obj_grad;
  SLEQP_FUNC_CONS_VAL cons_val;
  SLEQP_FUNC_CONS_JAC cons_jac;
  SLEQP_FUNC_HESS_PROD hess_prod;
  SLEQP_FUNC_FREE func_free;
} SleqpFuncCallbacks;

/**
 * Creates a new function
 *
 * @param[out] fstar            A pointer to the function to be created
 * @param[in]  callbacks        A callback to the function callbacks
 * @param[in]  num_variables    The number of variables
 * @param[in]  num_constraints  The number of constraints
 * @param[in]  func_data        User-provided function data
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_func_create(SleqpFunc** fstar,
                  SleqpFuncCallbacks* callbacks,
                  int num_variables,
                  int num_constraints,
                  void* func_data);

/**
 * Returns the number of variables \f$ n \f$.
 **/
SLEQP_EXPORT int
sleqp_func_num_vars(const SleqpFunc* func);

/**
 * Returns the number of constraints \f$ m \f$.
 **/
SLEQP_EXPORT int
sleqp_func_num_cons(const SleqpFunc* func);

/**
 * Sets the callbacks of this function to the specified ones
 *
 * @param[in]     func            The function
 * @param[in]     callbacks       The new callbacks
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_func_set_callbacks(SleqpFunc* func, SleqpFuncCallbacks* callbacks);

/**
 * Returns the Hessian structure of this function
 *
 * @param[in]     func            The function
 *
 **/
SLEQP_EXPORT
SleqpHessStruct*
sleqp_func_hess_struct(SleqpFunc* func);

/**
 * Returns the function data associated with the given function.
 **/
SLEQP_EXPORT void*
sleqp_func_get_data(SleqpFunc* func);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_func_capture(SleqpFunc* func);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_func_release(SleqpFunc** fstar);

/**
 * @}
 **/

#endif /* PUB_FUNC_H */
