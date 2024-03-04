#ifndef SLEQP_PUB_SOLVER_H
#define SLEQP_PUB_SOLVER_H

/**
 * @file pub_solver.h
 * @brief Definition of the solver structure.
 **/

#include "sleqp/export.h"
#include "sleqp/pub_iterate.h"
#include "sleqp/pub_problem.h"
#include "sleqp/pub_scale.h"
#include "sleqp/pub_settings.h"

typedef struct SleqpSolver SleqpSolver;

typedef SLEQP_RETCODE (*SLEQP_ACCEPTED_ITERATE)(SleqpSolver* solver,
                                                SleqpIterate* iterate,
                                                void* callback_data);

typedef SLEQP_RETCODE (*SLEQP_PERFORMED_ITERATION)(SleqpSolver* solver,
                                                   void* callback_data);

typedef SLEQP_RETCODE (*SLEQP_FINISHED)(SleqpSolver* solver,
                                        SleqpIterate* iterate,
                                        void* callback_data);

typedef struct SleqpSolver SleqpSolver;

/**
 * Creates a solver
 *
 * @param[out] star            A pointer to the solver to be created
 * @param[in]  problem         The underlying problem
 * @param[in]  settings         The solver settings
 * @param[in]  primal          The initial solution
 * @param[in]  scaling_data    The scaling to be used (may be `NULL`)
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_create(SleqpSolver** star,
                    SleqpProblem* problem,
                    SleqpVec* primal,
                    SleqpScaling* scaling_data);

/**
 * Solves the problem by performing iteration starting from the current solution
 *
 * @param[in]  solver           The solver
 * @param[in]  num_iterations   The number of iterations to be performed, or
 *@ref SLEQP_NONE
 * @param[in]  time_limit       A time limit in seconds, or @ref SLEQP_NONE
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_solve(SleqpSolver* solver,
                   int max_num_iterations,
                   double time_limit);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_real_state(const SleqpSolver* solver,
                        SLEQP_SOLVER_STATE_REAL state,
                        double* value);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_int_state(const SleqpSolver* solver,
                       SLEQP_SOLVER_STATE_INT state,
                       int* value);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_vec_state(const SleqpSolver* solver,
                       SLEQP_SOLVER_STATE_VEC value,
                       SleqpVec* result);

SLEQP_EXPORT
const char*
sleqp_solver_info(const SleqpSolver* solver);

/**
 * Returns the status of the last call to @ref sleqp_solver_solve
 *
 * @param[in]  solver           The solver
 *
 **/
SLEQP_EXPORT SLEQP_STATUS
sleqp_solver_status(const SleqpSolver* solver);

/**
 * Resets the solvers internal state
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_reset(SleqpSolver* solver);

/**
 * Aborts the solver after the next iteration. To be used from callback
 *functions
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_abort(SleqpSolver* solver);

/**
 * Returns the current iterate of the solver
 *
 * @param[in]  solver           The solver
 * @param[out] iterate          A pointer to the current iterate
 *
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_solution(SleqpSolver* solver, SleqpIterate** iterate);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_violated_constraints(SleqpSolver* solver,
                                  SleqpIterate* iterate,
                                  int* violated_constraints,
                                  int* num_violated_constraints);

/**
 * Returns the number of iterations performed during the last call to @ref
 *sleqp_solver_solve
 *
 * @param[in]  solver           The solver
 *
 **/
SLEQP_EXPORT
int
sleqp_solver_iterations(const SleqpSolver* solver);

/**
 * Returns the number of seconds elapsed during the last call to @ref
 *sleqp_solver_solve
 *
 * @param[in]  solver           The solver
 *
 **/
SLEQP_EXPORT
double
sleqp_solver_elapsed_seconds(const SleqpSolver* solver);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_add_callback(SleqpSolver* solver,
                          SLEQP_SOLVER_EVENT solver_event,
                          void* callback_func,
                          void* callback_data);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_remove_callback(SleqpSolver* solver,
                             SLEQP_SOLVER_EVENT solver_event,
                             void* callback_func,
                             void* callback_data);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_capture(SleqpSolver* solver);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_solver_release(SleqpSolver** star);

#endif /* SLEQP_PUB_SOLVER_H */
