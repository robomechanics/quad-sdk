#ifndef SLEQP_DEFS_H
#define SLEQP_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

  typedef enum
  {
    SLEQP_SOLVER_GUROBI, SLEQP_SOLVER_HIGHS, SLEQP_SOLVER_SOPLEX
  } SLEQP_LP_SOLVERS;

#define SLEQP_VERSION "1.0.1"

/* #undef SLEQP_DEBUG */
#define SLEQP_FORMAT_CODES
/* #undef SLEQP_ENABLE_NUM_ASSERTS */
#define SLEQP_HAVE_ATTRIBUTE_WARN_UNUSED_RESULT
/* #undef SLEQP_HAVE_ATTRIBUTE_NODISCARD */
#define SLEQP_HAVE_ATTRIBUTE_FORMAT
/* #undef SLEQP_HAVE_QR_FACT */

#ifdef SLEQP_HAVE_ATTRIBUTE_FORMAT
#define SLEQP_FORMAT_PRINTF(index, first)                                      \
  __attribute__((__format__(__printf__, index, first)))
#else
#define SLEQP_FORMAT_PRINTF(index, first)
#endif

#define SLEQP_VERSION_MAJOR 1
#define SLEQP_VERSION_MINOR 0
#define SLEQP_VERSION_PATCH 1

#define SLEQP_TRLIB_VERSION "0.4"

#define SLEQP_GIT_BRANCH "patch-1"
#define SLEQP_GIT_COMMIT_HASH "e9a4553"

#define SLEQP_LONG_VERSION "1.0.1 [patch-1-e9a4553]"

#define SLEQP_LP_SOLVER SLEQP_LP_SOLVER_HIGHS
#define SLEQP_LP_SOLVER_NAME "HiGHS"
#define SLEQP_LP_SOLVER_VERSION "1.6.0"

#define SLEQP_LP_SOLVER_HIGHS_NAME "HiGHS"
#define SLEQP_LP_SOLVER_HIGHS_VERSION "1.6.0"

#define SLEQP_FACT_NAME "MUMPS"
#define SLEQP_FACT_VERSION "5.4.1"

#define SLEQP_FACT_MUMPS_NAME "MUMPS"
#define SLEQP_FACT_MUMPS_VERSION "5.4.1"

#ifdef __cplusplus
}
#endif

#endif /* SLEQP_DEFS_H */
