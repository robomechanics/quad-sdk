#ifndef SLEQP_PUB_SETTINGS_H
#define SLEQP_PUB_SETTINGS_H

/**
 * @file pub_settings.h
 * @brief Definition of settings.
 **/

#include "sleqp/export.h"
#include "sleqp/pub_types.h"

typedef enum
{
  SLEQP_SETTINGS_ENUM_DERIV_CHECK = 0,
  SLEQP_SETTINGS_ENUM_HESS_EVAL,
  SLEQP_SETTINGS_ENUM_DUAL_ESTIMATION_TYPE,
  SLEQP_SETTINGS_ENUM_FLOAT_WARNING_FLAGS,
  SLEQP_SETTINGS_ENUM_FLOAT_ERROR_FLAGS,
  SLEQP_SETTINGS_ENUM_BFGS_SIZING,
  SLEQP_SETTINGS_ENUM_TR_SOLVER,
  SLEQP_SETTINGS_ENUM_POLISHING_TYPE,
  SLEQP_SETTINGS_ENUM_STEP_RULE,
  SLEQP_SETTINGS_ENUM_LINESEARCH,
  SLEQP_SETTINGS_ENUM_PARAMETRIC_CAUCHY,
  SLEQP_SETTINGS_ENUM_INITIAL_TR_CHOICE,
  SLEQP_SETTINGS_ENUM_AUG_JAC_METHOD,
  SLEQP_NUM_ENUM_SETTINGS
} SLEQP_SETTINGS_ENUM;

typedef enum
{
  SLEQP_SETTINGS_INT_NUM_QUASI_NEWTON_ITERATES = 0,
  SLEQP_SETTINGS_INT_MAX_NEWTON_ITERATIONS,
  SLEQP_SETTINGS_INT_NUM_THREADS,
  SLEQP_NUM_INT_SETTINGS
} SLEQP_SETTINGS_INT;

typedef enum
{
  SLEQP_SETTINGS_BOOL_PERFORM_NEWTON_STEP = 0,
  SLEQP_SETTINGS_BOOL_GLOBAL_PENALTY_RESETS,
  SLEQP_SETTINGS_BOOL_PERFORM_SOC,
  SLEQP_SETTINGS_BOOL_USE_QUADRATIC_MODEL,
  SLEQP_SETTINGS_BOOL_ALWAYS_WARM_START_LP,
  SLEQP_SETTINGS_BOOL_ENABLE_RESTORATION_PHASE,
  SLEQP_SETTINGS_BOOL_ENABLE_PREPROCESSOR,
  SLEQP_SETTINGS_BOOL_LP_RESOLVES,
  SLEQP_NUM_BOOL_SETTINGS
} SLEQP_SETTINGS_BOOL;

typedef enum
{
  SLEQP_SETTINGS_REAL_ZERO_EPS = 0,
  SLEQP_SETTINGS_REAL_EPS,
  SLEQP_SETTINGS_REAL_OBJ_LOWER,
  SLEQP_SETTINGS_REAL_DERIV_PERTURBATION,
  SLEQP_SETTINGS_REAL_DERIV_TOL,
  SLEQP_SETTINGS_REAL_CAUCHY_TAU,
  SLEQP_SETTINGS_REAL_CAUCHY_ETA,
  SLEQP_SETTINGS_REAL_LINESEARCH_TAU,
  SLEQP_SETTINGS_REAL_LINESEARCH_ETA,
  SLEQP_SETTINGS_REAL_LINESEARCH_CUTOFF,
  SLEQP_SETTINGS_REAL_FEAS_TOL,
  SLEQP_SETTINGS_REAL_SLACK_TOL,
  SLEQP_SETTINGS_REAL_STAT_TOL,
  SLEQP_SETTINGS_REAL_ACCEPTED_REDUCTION,
  SLEQP_SETTINGS_REAL_DEADPOINT_BOUND,
  SLEQP_NUM_REAL_SETTINGS
} SLEQP_SETTINGS_REAL;

typedef struct SleqpSettings SleqpSettings;

SLEQP_EXPORT const char*
sleqp_settings_enum_name(SLEQP_SETTINGS_ENUM settings);

SLEQP_EXPORT const char*
sleqp_settings_enum_desc(SLEQP_SETTINGS_ENUM settings);

SLEQP_EXPORT const char*
sleqp_settings_int_name(SLEQP_SETTINGS_INT settings);

SLEQP_EXPORT const char*
sleqp_settings_int_desc(SLEQP_SETTINGS_INT settings);

SLEQP_EXPORT const char*
sleqp_settings_bool_name(SLEQP_SETTINGS_BOOL settings);

SLEQP_EXPORT const char*
sleqp_settings_bool_desc(SLEQP_SETTINGS_BOOL settings);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_settings_create(SleqpSettings** star);

SLEQP_EXPORT const char*
sleqp_settings_real_name(SLEQP_SETTINGS_REAL param);

SLEQP_EXPORT const char*
sleqp_settings_real_desc(SLEQP_SETTINGS_REAL param);

SLEQP_EXPORT double
sleqp_settings_real_value(const SleqpSettings* settings,
                          SLEQP_SETTINGS_REAL param);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_settings_set_real_value(SleqpSettings* setting,
                              SLEQP_SETTINGS_REAL param,
                              double value);

SLEQP_EXPORT
int
sleqp_settings_enum_value(const SleqpSettings* settings,
                          SLEQP_SETTINGS_ENUM option);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_settings_set_enum_value(SleqpSettings* settings,
                              SLEQP_SETTINGS_ENUM option,
                              int value);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_settings_set_enum_value_from_string(SleqpSettings* settings,
                                          SLEQP_SETTINGS_ENUM option,
                                          const char* value);

SLEQP_EXPORT
int
sleqp_settings_int_value(const SleqpSettings* settings,
                         SLEQP_SETTINGS_INT option);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_settings_set_int_value(SleqpSettings* settings,
                             SLEQP_SETTINGS_INT option,
                             int value);

SLEQP_EXPORT
bool
sleqp_settings_bool_value(const SleqpSettings* settings,
                          SLEQP_SETTINGS_BOOL option);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_settings_set_bool_value(SleqpSettings* settings,
                              SLEQP_SETTINGS_BOOL option,
                              bool value);

/**
 * Read settings from file on the disk and
 * update relevant parts of the given settings
 *
 * @param[in,out] settings           Current settings
 * @param[in]     settings_filename  Name of settings file
 *
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_settings_read_file(SleqpSettings* settings,
                         const char* settings_filename);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_settings_capture(SleqpSettings* settings);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_settings_release(SleqpSettings** star);

#endif /* SLEQP_PUB_SETTINGS_H */
