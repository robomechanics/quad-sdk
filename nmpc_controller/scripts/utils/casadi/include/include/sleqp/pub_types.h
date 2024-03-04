#ifndef SLEQP_PUB_TYPES_H
#define SLEQP_PUB_TYPES_H

/**
 * @file pub_types.h
 * @brief Definition of basic types.
 **/

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "sleqp/pub_log.h"

#ifdef SLEQP_HAVE_ATTRIBUTE_WARN_UNUSED_RESULT
#define SLEQP_WARNUNUSED __attribute__((warn_unused_result))
#else
#define SLEQP_WARNUNUSED
#endif

#ifdef SLEQP_HAVE_ATTRIBUTE_NODISCARD
#define SLEQP_NODISCARD(x) [[nodiscard(x)]]
#else
#define SLEQP_NODISCARD(x)
#endif

typedef enum SLEQP_NODISCARD("error indicator")
{
  SLEQP_ERROR      = -1,
  SLEQP_OKAY       = 0,
  SLEQP_ABORT_TIME = 1,
} SLEQP_RETCODE;

typedef enum
{
  SLEQP_FAILED_ASSERTION,
  SLEQP_NOMEM,
  SLEQP_INTERNAL_ERROR,
  SLEQP_FUNC_EVAL_ERROR,
  SLEQP_CALLBACK_ERROR,
  SLEQP_MATH_ERROR,
  SLEQP_INVALID_DERIV,
  SLEQP_ILLEGAL_ARGUMENT
} SLEQP_ERROR_TYPE;

/**
 * The state of a variable or constraint with respect
 * to an active set
 **/
typedef enum
{
  /** The variable / constraint is inactive **/
  SLEQP_INACTIVE = 0,
  /** The variable / constraint is active at its lower bound **/
  SLEQP_ACTIVE_LOWER = (1 << 0),
  /** The variable / constraint is active at its upper bound **/
  SLEQP_ACTIVE_UPPER = (1 << 1),
  /** The variable / constraint is active at both of its bounds **/
  SLEQP_ACTIVE_BOTH = (SLEQP_ACTIVE_LOWER | SLEQP_ACTIVE_UPPER),
} SLEQP_ACTIVE_STATE;

typedef enum
{
  SLEQP_STATUS_UNKNOWN = 0,
  SLEQP_STATUS_RUNNING,
  SLEQP_STATUS_OPTIMAL,
  SLEQP_STATUS_INFEASIBLE,
  SLEQP_STATUS_UNBOUNDED,
  SLEQP_STATUS_ABORT_DEADPOINT,
  SLEQP_STATUS_ABORT_ITER,
  SLEQP_STATUS_ABORT_MANUAL,
  SLEQP_STATUS_ABORT_TIME
} SLEQP_STATUS;

#define SLEQP_CALL(x)                                                          \
  do                                                                           \
  {                                                                            \
    const SLEQP_RETCODE _status = (x);                                         \
    if (_status < SLEQP_OKAY)                                                  \
    {                                                                          \
      sleqp_log_error("Error in function %s", __PRETTY_FUNCTION__);            \
      return _status;                                                          \
    }                                                                          \
    else if (_status != SLEQP_OKAY)                                            \
    {                                                                          \
      return _status;                                                          \
    }                                                                          \
  } while (0)

typedef enum
{
  SLEQP_DERIV_CHECK_SKIP       = 0,
  SLEQP_DERIV_CHECK_FIRST_OBJ  = (1 << 0),
  SLEQP_DERIV_CHECK_FIRST_CONS = (1 << 1),
  SLEQP_DERIV_CHECK_FIRST
    = (SLEQP_DERIV_CHECK_FIRST_OBJ | SLEQP_DERIV_CHECK_FIRST_CONS),
  SLEQP_DERIV_CHECK_SECOND_SIMPLE = (1 << 2),
  SLEQP_DERIV_CHECK_SECOND_OBJ    = (1 << 3),
  SLEQP_DERIV_CHECK_SECOND_CONS   = (1 << 4),
  SLEQP_DERIV_CHECK_SECOND_EXHAUSTIVE
    = (SLEQP_DERIV_CHECK_SECOND_OBJ | SLEQP_DERIV_CHECK_SECOND_CONS),
} SLEQP_DERIV_CHECK;

typedef enum
{
  SLEQP_HESS_EVAL_EXACT = 0,
  SLEQP_HESS_EVAL_SR1,
  SLEQP_HESS_EVAL_SIMPLE_BFGS,
  SLEQP_HESS_EVAL_DAMPED_BFGS,
} SLEQP_HESS_EVAL;

typedef enum
{
  SLEQP_BFGS_SIZING_NONE = 0,    // No sizing
  SLEQP_BFGS_SIZING_CENTERED_OL, // Centered Oren–Luenberger
} SLEQP_BFGS_SIZING;

typedef enum
{
  SLEQP_STEPTYPE_NONE = 0,
  SLEQP_STEPTYPE_ACCEPTED,
  SLEQP_STEPTYPE_ACCEPTED_FULL,
  SLEQP_STEPTYPE_ACCEPTED_SOC,
  SLEQP_STEPTYPE_REJECTED
} SLEQP_STEPTYPE;

typedef enum
{
  SLEQP_DUAL_ESTIMATION_TYPE_LP,
  SLEQP_DUAL_ESTIMATION_TYPE_LSQ,
  SLEQP_DUAL_ESTIMATION_TYPE_MIXED,
} SLEQP_DUAL_ESTIMATION_TYPE;

typedef enum
{
  SLEQP_TR_SOLVER_TRLIB = 0,
  SLEQP_TR_SOLVER_CG,
  SLEQP_TR_SOLVER_LSQR,
  SLEQP_TR_SOLVER_AUTO
} SLEQP_TR_SOLVER;

typedef enum
{
  SLEQP_POLISHING_NONE = 0,
  SLEQP_POLISHING_ZERO_DUAL,
  SLEQP_POLISHING_INACTIVE
} SLEQP_POLISHING_TYPE;

typedef enum
{
  SLEQP_PARAMETRIC_CAUCHY_DISABLED = 0,
  SLEQP_PARAMETRIC_CAUCHY_COARSE,
  SLEQP_PARAMETRIC_CAUCHY_FINE
} SLEQP_PARAMETRIC_CAUCHY;

typedef enum
{
  SLEQP_INITIAL_TR_CHOICE_NARROW,
  SLEQP_INITIAL_TR_CHOICE_WIDE
} SLEQP_INITIAL_TR_CHOICE;

typedef enum
{
  SLEQP_LINESEARCH_EXACT,
  SLEQP_LINESEARCH_APPROX,
} SLEQP_LINESEARCH;

typedef enum
{
  SLEQP_SOLVER_EVENT_ACCEPTED_ITERATE = 0,
  SLEQP_SOLVER_EVENT_PERFORMED_ITERATION,
  SLEQP_SOLVER_EVENT_FINISHED,
  SLEQP_SOLVER_NUM_EVENTS
} SLEQP_SOLVER_EVENT;

typedef enum
{
  SLEQP_PREPROCESSING_RESULT_SUCCESS,
  SLEQP_PREPROCESSING_RESULT_FAILURE,
  SLEQP_PREPROCESSING_RESULT_INFEASIBLE
} SLEQP_PREPROCESSING_RESULT;

typedef enum
{
  SLEQP_STEP_RULE_DIRECT,
  SLEQP_STEP_RULE_WINDOW,
  SLEQP_STEP_RULE_MINSTEP
} SLEQP_STEP_RULE;

typedef enum
{
  SLEQP_AUG_JAC_AUTO,
  SLEQP_AUG_JAC_STANDARD,
  SLEQP_AUG_JAC_REDUCED,
  SLEQP_AUG_JAC_DIRECT
} SLEQP_AUG_JAC_METHOD;

typedef enum
{
  SLEQP_SOLVER_STATE_REAL_TRUST_RADIUS,
  SLEQP_SOLVER_STATE_REAL_LP_TRUST_RADIUS,
  SLEQP_SOLVER_STATE_REAL_SCALED_OBJ_VAL,
  SLEQP_SOLVER_STATE_REAL_SCALED_MERIT_VAL,
  SLEQP_SOLVER_STATE_REAL_SCALED_FEAS_RES,
  SLEQP_SOLVER_STATE_REAL_SCALED_STAT_RES,
  SLEQP_SOLVER_STATE_REAL_SCALED_SLACK_RES,
  SLEQP_SOLVER_STATE_REAL_PENALTY_PARAM,
  SLEQP_SOLVER_STATE_REAL_MIN_RAYLEIGH,
  SLEQP_SOLVER_STATE_REAL_MAX_RAYLEIGH,
} SLEQP_SOLVER_STATE_REAL;

typedef enum
{
  SLEQP_SOLVER_STATE_INT_LAST_STEP_ON_BDRY,
  SLEQP_SOLVER_STATE_INT_ITERATION,
  SLEQP_SOLVER_STATE_INT_LAST_STEP_TYPE,
} SLEQP_SOLVER_STATE_INT;

typedef enum
{
  SLEQP_SOLVER_STATE_VEC_SCALED_STAT_RESIDUALS,
  SLEQP_SOLVER_STATE_VEC_SCALED_FEAS_RESIDUALS,
  SLEQP_SOLVER_STATE_VEC_SCALED_CONS_SLACK_RESIDUALS,
  SLEQP_SOLVER_STATE_VEC_SCALED_VAR_SLACK_RESIDUALS,
} SLEQP_SOLVER_STATE_VEC;

/**None value to be used in place of integer parameters **/
#define SLEQP_NONE (-1)

#endif /* SLEQP_PUB_TYPES_H */
