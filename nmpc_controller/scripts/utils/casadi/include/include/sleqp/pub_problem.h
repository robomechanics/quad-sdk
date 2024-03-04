#ifndef SLEQP_PUB_PROBLEM_H
#define SLEQP_PUB_PROBLEM_H

#include "sleqp/export.h"

#include "sleqp/pub_func.h"
#include "sleqp/pub_settings.h"
#include "sparse/pub_mat.h"
#include "sparse/pub_vec.h"

/**
 * @file pub_problem.h
 * @brief Definition of the programming problem.
 **/

/**
 * @defgroup problem Nonlinear programming problems
 *
 * An NLP is given as
 *
 * \f[
 * \begin{aligned}
 * \min \: & f(x)                       \\
 * \st \: & l \leq c(x) \leq u \\
 * & l_x \leq x \leq u_x
 * \end{aligned}
 * \f]
 *
 * where \f$ f : \R^{n} \to \R \f$,
 * \f$ c : \R^{n} \to \R^{m} \f$
 * are functions, \f$ l, u \in \R^{m}, l \leq u \f$
 * are the constraint bounds, and
 * \f$ l_x, u_x \in \R^{n}, l_x \leq u_x \f$
 * are the variable bounds.
 *
 * The constraints \f$ c\f$ can optionally be split up into general
 * nonlinear constraints
 * \f$ c_{\nonlin} : \R^{n} \to \R^{m_{\nonlin}} \f$
 * and linear coefficients \f$ A \in \R^{m_{\lin} \times n}\f$.
 * Consequently, the lower and upper bounds \f$ l, u \f$ are split up
 * into \f$ l_{\nonlin}, u_{\nonlin} \in \R^{m_{\nonlin}} \f$ and
 * \f$ l_{\lin}, u_{\lin} \in \R^{m_{\lin}} \f$
 * The constraints \f$ c \f$ are then given as
 *
 * \f[
 *   c(x) = \left( c_{\nonlin}(x), A \cdot x \right).
 * \f]
 *
 * @see function
 *
 * @{
 **/

typedef struct SleqpProblem SleqpProblem;

/**
 * Creates a new problem without linear coefficients.
 *
 * @param[in] func        The function associated with the problem
 * @param[in] var_lb      The lower variable bounds \f$ l_x \f$
 * @param[in] var_ub      The upper variable bounds \f$ u_x \f$
 * @param[in] general_lb  The lower bounds \f$ l \f$ on the constraints
 * @param[in] general_ub  The upper bounds \f$ u \f$ on the constraints
 * @param[in] settings    Settings (`NULL` for default settings)
 *
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_problem_create_simple(SleqpProblem** star,
                            SleqpFunc* func,
                            const SleqpVec* var_lb,
                            const SleqpVec* var_ub,
                            const SleqpVec* general_lb,
                            const SleqpVec* general_ub,
                            SleqpSettings* settings);

/**
 * Creates a new problem with linear coefficients.
 *
 * @param[in] func        The function associated with the problem
 * @param[in] var_lb      The lower variable bounds \f$ l_x \f$
 * @param[in] var_ub      The upper variable bounds \f$ u_x \f$
 * @param[in] general_lb  The lower bounds \f$ l_{\nonlin} \f$ on the
 *constraints
 * @param[in] general_ub  The upper bounds \f$ u_{\nonlin} \f$ on the
 *constraints
 * @param[in] linear_coeffs The linear coefficient matrix \f$ A \f$
 * @param[in] linear_lb  The lower bounds \f$ l_{\lin} \f$ on the
 *constraints
 * @param[in] linear_ub  The upper bounds \f$ u_{\lin} \f$ on the
 *constraints
 * @param[in] settings    Settings (`NULL` for default settings)
 *
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_problem_create(SleqpProblem** star,
                     SleqpFunc* func,
                     const SleqpVec* var_lb,
                     const SleqpVec* var_ub,
                     const SleqpVec* genereal_lb,
                     const SleqpVec* genereal_ub,
                     const SleqpMat* linear_coeffs,
                     const SleqpVec* linear_lb,
                     const SleqpVec* linear_ub,
                     SleqpSettings* settings);

/**
 * Returns the total number \f$ m \f$ of constraints (both general and
 * linear) of the problem.
 **/
SLEQP_EXPORT int
sleqp_problem_num_cons(const SleqpProblem* problem);

/**
 * Returns the total number \f$ m_{\lin} \f$ of linear constraints of
 * the problem.
 **/
SLEQP_EXPORT int
sleqp_problem_num_lin_cons(const SleqpProblem* problem);

/**
 * Returns the total number \f$ m_{\nonlin} \f$ of general constraints
 * of the problem.
 **/
SLEQP_EXPORT int
sleqp_problem_num_gen_cons(const SleqpProblem* problem);

/**
 * Returns the function, composed of the objective \f$ f \f$ and the
 * general constraints \f$ c_{\nonlin} \f$ associated with the
 * problem.
 **/
SLEQP_EXPORT SleqpFunc*
sleqp_problem_func(SleqpProblem* problem);

/**
 * Returns the number \f$ n \f$ of variables of the problem.
 **/
SLEQP_EXPORT int
sleqp_problem_num_vars(const SleqpProblem* problem);

/**
 * Returns the lower bounds \f$ l_x \f$ of the variables
 * with respect to the problem.
 **/
SLEQP_EXPORT SleqpVec*
sleqp_problem_vars_lb(SleqpProblem* problem);

/**
 * Returns the lower bounds \f$ u_x \f$ of the variables
 * with respect to the problem.
 **/
SLEQP_EXPORT SleqpVec*
sleqp_problem_vars_ub(SleqpProblem* problem);

/**
 * Returns the lower bounds \f$ l_{\nonlin} \f$ of the general
 * constraints \f$ c_{\nonlin} \f$ of the problem.
 **/
SLEQP_EXPORT SleqpVec*
sleqp_problem_general_lb(SleqpProblem* problem);

/**
 * Returns the upper bounds \f$ l_{\nonlin} \f$ of the general
 * constraints \f$ c_{\nonlin} \f$ of the problem.
 **/
SLEQP_EXPORT SleqpVec*
sleqp_problem_general_ub(SleqpProblem* problem);

/**
 * Returns the linear coefficient matrix \f$ A \f$ of the problem.
 **/
SLEQP_EXPORT SleqpMat*
sleqp_problem_linear_coeffs(SleqpProblem* problem);

/**
 * Returns the upper bounds \f$ l_{\lin} \f$ of the linear
 * constraints of the problem.
 **/
SLEQP_EXPORT SleqpVec*
sleqp_problem_linear_lb(SleqpProblem* problem);

/**
 * Returns the upper bounds \f$ u_{\lin} \f$ of the linear
 * constraints of the problem.
 **/
SLEQP_EXPORT SleqpVec*
sleqp_problem_linear_ub(SleqpProblem* problem);

/**
 * Returns the lower bounds \f$ l \f$ of the
 * constraints \f$ c \f$ of the problem.
 **/
SLEQP_EXPORT SleqpVec*
sleqp_problem_cons_lb(SleqpProblem* problem);

/**
 * Returns the upper bounds \f$ u \f$ of the
 * constraints \f$ c \f$ of the problem.
 **/
SLEQP_EXPORT SleqpVec*
sleqp_problem_cons_ub(SleqpProblem* problem);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_problem_capture(SleqpProblem* problem);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_problem_release(SleqpProblem** star);

/**
 * @}
 **/

#endif /* SLEQP_PUB_PROBLEM_H */
