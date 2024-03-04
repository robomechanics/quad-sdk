#ifndef SLEQP_PUB_ITERATE_H
#define SLEQP_PUB_ITERATE_H

/**
 * @file pub_iterate.h
 * @brief Definition of iterate.
 **/

#include "sleqp/export.h"
#include "sleqp/pub_problem.h"
#include "sleqp/pub_working_set.h"

#include "sparse/pub_mat.h"
#include "sparse/pub_vec.h"

typedef struct SleqpIterate SleqpIterate;

/**
 * Create a new iterate
 *
 * @param[in,out] star    A pointer to the newly created iterate
 * @param[in] problem     The underlying problem
 * @param[in] x           The point of the iterate
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_iterate_create(SleqpIterate** star,
                     SleqpProblem* problem,
                     const SleqpVec* x);

/**
 * The current point. Has dimension = num_variables.
 **/
SLEQP_EXPORT SleqpVec*
sleqp_iterate_primal(const SleqpIterate* iterate);

/**
 * The current function value
 **/
SLEQP_EXPORT double
sleqp_iterate_obj_val(const SleqpIterate* iterate);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_iterate_set_obj_val(SleqpIterate* iterate, double value);

/**
 * The current function gradient. Has dimension = num_variables.
 **/
SLEQP_EXPORT SleqpVec*
sleqp_iterate_obj_grad(const SleqpIterate* iterate);

/**
 * The current constraint values. Has dimension = num_constraints.
 **/
SLEQP_EXPORT SleqpVec*
sleqp_iterate_cons_val(const SleqpIterate* iterate);

/**
 * The Jacobian of the constraitns at the current iterate.
 * Has num_constraints many rows, num_variables many columns.
 */
SLEQP_EXPORT SleqpMat*
sleqp_iterate_cons_jac(const SleqpIterate* iterate);

/**
 * The current working set.
 **/
SLEQP_EXPORT SleqpWorkingSet*
sleqp_iterate_working_set(const SleqpIterate* iterate);

/**
 * The dual values of the constraints. Has dimension = num_constraints.
 */
SLEQP_EXPORT SleqpVec*
sleqp_iterate_cons_dual(const SleqpIterate* iterate);

/**
 * The dual values of the variable bounds. Has dimension = num_variables.
 */
SLEQP_EXPORT SleqpVec*
sleqp_iterate_vars_dual(const SleqpIterate* iterate);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_iterate_capture(SleqpIterate* iterate);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_iterate_release(SleqpIterate** star);

#endif /* SLEQP_PUB_ITERATE_H */
