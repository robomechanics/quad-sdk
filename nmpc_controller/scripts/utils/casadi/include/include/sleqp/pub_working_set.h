#ifndef SLEQP_PUB_WORKING_SET_H
#define SLEQP_PUB_WORKING_SET_H

/**
 * @file pub_working_set.h
 * @brief Definition of working sets.
 **/

/**
 * @defgroup working_set Working sets
 *
 * The working set \f$ \workingSet \f$ is a subset of at most
 * \f$ n \f$ variables / constraints which are active
 * with respect to the primal point \f$ x \in R^n \f$
 * and a primal direction \f$ d \in \R^n \f$. The
 * set of active variables is given as
 *
 * \f[
 *     \{ j \in [n] \mid x_j + d_j \in \{(l_x)_j, (u_x)_j\} \},
 * \f]
 *
 * where a variable \f$ j \f$ can be active at its
 * lower bound \f$ l_j \f$, it upper bound \f$ u_j \f$,
 * or at both bounds (if their values coincide).
 * Activity with respect to constraints is defined
 * analogously.
 *
 * In term of the constraints, each variable
 * \f$ j \f$ corresponds to the row
 *
 * \f[
 * (l_x)_j \leq \langle e_j, x \rangle \leq (u_x)_j,
 * \f]
 *
 * whereas each constraint \f$ i \in [m] \f$
 * has the associated row
 *
 * \f[
 * l_i \leq c_i(x) + \langle \nabla c_i(x), x \rangle \leq u_i,
 * \f]
 *
 * The rows corresponding of variables / constraints
 * in \f$ \workingSet \f$
 * are guaranteed to be linearly independent.
 *
 **/

#include "sleqp/export.h"

#include "sleqp/pub_problem.h"

typedef struct SleqpWorkingSet SleqpWorkingSet;

/**
 * Creates a new working set corresponding to the given problem
 *
 * @param[out] star            A pointer to the working set to be created
 * @param[int] problem         The underlying problem
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_working_set_create(SleqpWorkingSet** star, SleqpProblem* problem);

/**
 * Returns the state of the given variable with respect to the given working set
 *
 * @param[in]  working_set           The working set
 * @param[in]  index                 The variable index
 **/
SLEQP_EXPORT SLEQP_ACTIVE_STATE
sleqp_working_set_var_state(const SleqpWorkingSet* working_set, int index);

/**
 * Returns the state of the given constraint with respect to the given working
 * set
 *
 * @param[in]  working_set           The working set
 * @param[in]  index                 The constraint index
 **/
SLEQP_EXPORT SLEQP_ACTIVE_STATE
sleqp_working_set_cons_state(const SleqpWorkingSet* working_set, int index);

/**
 * Returns an array containing the states of all variables with respect to the
 * given working set
 **/
SLEQP_EXPORT const SLEQP_ACTIVE_STATE*
sleqp_working_set_var_states(const SleqpWorkingSet* working_set);

/**
 * Returns an array containing the states of all constraints with respect to the
 * given working set
 **/
SLEQP_EXPORT const SLEQP_ACTIVE_STATE*
sleqp_working_set_cons_states(const SleqpWorkingSet* working_set);

/**
 * Returns the problem underling the given working set
 *
 * @param[in]  working_set           The working set
 **/
SLEQP_EXPORT SleqpProblem*
sleqp_working_set_problem(const SleqpWorkingSet* working_set);

/**
 * Returns the number of variables contained in the given working set
 *
 * @param[in]  working_set           The working set
 **/
SLEQP_EXPORT int
sleqp_working_set_num_active_vars(const SleqpWorkingSet* working_set);

/**
 * Returns the number of constraints contained in the given working set
 *
 * @param[in]  working_set           The working set
 **/
SLEQP_EXPORT int
sleqp_working_set_num_active_cons(const SleqpWorkingSet* working_set);

/**
 * Returns the size of the given set, i.e. the number of contained variables
 *plus the number of contained constraints
 *
 * @param[in]  working_set           The working set
 **/
SLEQP_EXPORT int
sleqp_working_set_size(const SleqpWorkingSet* working_set);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_working_set_capture(SleqpWorkingSet* working_set);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_working_set_release(SleqpWorkingSet** star);

#endif /* SLEQP_PUB_WORKING_SET_H */
