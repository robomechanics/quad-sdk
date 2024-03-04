#ifndef SPRAL_SSMFE_H
#define SPRAL_SSMFE_H

#include <stdbool.h>

// Define complex types that work with both C and C++
#include "spral_complex.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * Derived types
 ************************************/

struct spral_ssmfe_rcid {
   int job;
   int nx;
   int jx;
   int kx;
   int ny;
   int jy;
   int ky;
   int i;
   int j;
   int k;
   double alpha;
   double beta;
   double *x;
   double *y;
   char unused[80]; // Allow for future expansion
};

struct spral_ssmfe_rciz {
   int job;
   int nx;
   int jx;
   int kx;
   int ny;
   int jy;
   int ky;
   int i;
   int j;
   int k;
   spral_double_complex alpha;
   spral_double_complex beta;
   spral_double_complex *x;
   spral_double_complex *y;
   char unused[80]; // Allow for future expansion
};

struct spral_ssmfe_core_options {
   int array_base; // Not in Fortran type
   double cf_max;
   int err_est;
   int extra_left;
   int extra_right;
   double min_gap;
   bool minAprod;
   bool minBprod;
   char unused[80]; // Allow for future expansion
};

struct spral_ssmfe_options {
   int array_base;
   int print_level;
   int unit_error;
   int unit_warning;
   int unit_diagnostic;
   int max_iterations;
   int user_x;
   int err_est;
   double abs_tol_lambda;
   double rel_tol_lambda;
   double abs_tol_residual;
   double rel_tol_residual;
   double tol_x;
   double left_gap;
   double right_gap;
   int extra_left;
   int extra_right;
   int max_left;
   int max_right;
   bool minAprod;
   bool minBprod;
};

struct spral_ssmfe_inform {
   int flag;
   int stat;
   int non_converged;
   int iteration;
   int left;
   int right;
   int *converged;
   double next_left;
   double next_right;
   double *residual_norms;
   double *err_lambda;
   double *err_X;
   char unused[80]; // Allow for future expansion
};

/************************************
 * SSMFE subroutines 
 ************************************/

/* Initialize options to defaults */
void spral_ssmfe_default_options(struct spral_ssmfe_options *options);
/* Leftmost real eigenpairs of std eigenvalue problem */
void spral_ssmfe_standard_double(struct spral_ssmfe_rcid *rci, int left,
      int mep, double *lambda, int n, double *x, int ldx,
      void **keep, const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Leftmost complex eigenpairs of std eigenvalue problem */
void spral_ssmfe_standard_double_complex(struct spral_ssmfe_rciz *rci,
      int left, int mep, double *lambda, int n, spral_double_complex *x,
      int ldx, void **keep, const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Real eigenpairs around shift for std eigenvalue problem */
void spral_ssmfe_standard_shift_double(struct spral_ssmfe_rcid *rci,
      double sigma, int left, int right, int mep, double *lambda, int n,
      double *x, int ldx, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Complex eigenpairs around shift for std eigenvalue problem */
void spral_ssmfe_standard_shift_double_complex(
      struct spral_ssmfe_rciz *rci, double sigma, int left, int right, int mep,
      double *lambda, int n, spral_double_complex *x, int ldx, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Leftmost real eigenpairs of generalized eigenvalue problem */
void spral_ssmfe_generalized_double(struct spral_ssmfe_rcid *rci,
      int left, int mep, double *lambda, int n, double *x, int ldx,
      void **keep, const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Leftmost complex eigenpairs of generalized eigenvalue problem */
void spral_ssmfe_generalized_double_complex(struct spral_ssmfe_rciz *rci,
      int left, int mep, double *lambda, int n, spral_double_complex *x,
      int ldx, void **keep, const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Real eigenpairs around shift for generalized eigenvalue problem */
void spral_ssmfe_generalized_shift_double(struct spral_ssmfe_rcid *rci,
      double sigma, int left, int right, int mep, double *lambda, int n,
      double *x, int ldx, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Complex eigenpairs around shift for generalized eigenvalue problem */
void spral_ssmfe_generalized_shift_double_complex(
      struct spral_ssmfe_rciz *rci, double sigma, int left, int right, int mep,
      double *lambda, int n, spral_double_complex *x, int ldx, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Real eigenpairs around shift for buckling problem */
void spral_ssmfe_buckling_double(struct spral_ssmfe_rcid *rci,
      double sigma, int left, int right, int mep, double *lambda, int n,
      double *x, int ldx, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Complex eigenpairs around shift for buckling problem */
void spral_ssmfe_buckling_double_complex(struct spral_ssmfe_rciz *rci,
      double sigma, int left, int right, int mep, double *lambda, int n,
      spral_double_complex *x, int ldx, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Free memory (real) */
void spral_ssmfe_free_double(void **keep, struct spral_ssmfe_inform *inform);
/* Free memory (complex) */
void spral_ssmfe_free_double_complex(void **keep, struct spral_ssmfe_inform *inform);

/************************************
 * SSMFE_EXPERT subroutines (additional to those shared with SSMFE)
 ************************************/

/* Leftmost real eigenpairs of std eigenvalue problem */
void spral_ssmfe_expert_standard_double(struct spral_ssmfe_rcid *rci, int left,
      int mep, double *lambda, int m, double *rr, int *ind,
      void **keep, const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Leftmost complex eigenpairs of std eigenvalue problem */
void spral_ssmfe_expert_standard_double_complex(struct spral_ssmfe_rciz *rci,
      int left, int mep, double *lambda, int m, spral_double_complex *rr,
      int *ind, void **keep, const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Real eigenpairs around shift for std eigenvalue problem */
void spral_ssmfe_expert_standard_shift_double(struct spral_ssmfe_rcid *rci,
      double sigma, int left, int right, int mep, double *lambda, int m,
      double *rr, int *ind, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Complex eigenpairs around shift for std eigenvalue problem */
void spral_ssmfe_expert_standard_shift_double_complex(
      struct spral_ssmfe_rciz *rci, double sigma, int left, int right, int mep,
      double *lambda, int m, spral_double_complex *rr, int *ind, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Leftmost real eigenpairs of generalized eigenvalue problem */
void spral_ssmfe_expert_generalized_double(struct spral_ssmfe_rcid *rci,
      int left, int mep, double *lambda, int m, double *rr, int *ind,
      void **keep, const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Leftmost complex eigenpairs of generalized eigenvalue problem */
void spral_ssmfe_expert_generalized_double_complex(struct spral_ssmfe_rciz *rci,
      int left, int mep, double *lambda, int m, spral_double_complex *rr,
      int *ind, void **keep, const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Real eigenpairs around shift for generalized eigenvalue problem */
void spral_ssmfe_expert_generalized_shift_double(struct spral_ssmfe_rcid *rci,
      double sigma, int left, int right, int mep, double *lambda, int m,
      double *rr, int *ind, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Complex eigenpairs around shift for generalized eigenvalue problem */
void spral_ssmfe_expert_generalized_shift_double_complex(
      struct spral_ssmfe_rciz *rci, double sigma, int left, int right, int mep,
      double *lambda, int m, spral_double_complex *rr, int *ind, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Real eigenpairs around shift for buckling problem */
void spral_ssmfe_expert_buckling_double(struct spral_ssmfe_rcid *rci,
      double sigma, int left, int right, int mep, double *lambda, int m,
      double *rr, int *ind, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Complex eigenpairs around shift for buckling problem */
void spral_ssmfe_expert_buckling_double_complex(struct spral_ssmfe_rciz *rci,
      double sigma, int left, int right, int mep, double *lambda, int m,
      spral_double_complex *rr, int *ind, void **keep,
      const struct spral_ssmfe_options *options,
      struct spral_ssmfe_inform *inform);
/* Free memory */
void spral_ssmfe_expert_free(void **keep, struct spral_ssmfe_inform *inform);

/************************************
 * SSMFE_CORE subroutines 
 ************************************/

/* Initialize options to defaults */
void spral_ssmfe_core_default_options(struct spral_ssmfe_core_options *options);
/* Core driver routine for left and right values of (real) eigenproblems */
void spral_ssmfe_double(struct spral_ssmfe_rcid *rci, int problem,
      int left, int right, int m, double *lambda, double *rr, int *ind,
      void **keep, const struct spral_ssmfe_core_options *options,
      struct spral_ssmfe_inform *inform);
/* Core driver routine for left and right values of (complex) eigenproblems */
void spral_ssmfe_double_complex(struct spral_ssmfe_rciz *rci, int problem,
      int left, int right, int m, double *lambda, spral_double_complex *rr,
      int *ind, void **keep, const struct spral_ssmfe_core_options *options,
      struct spral_ssmfe_inform *inform);
/* Core driver routine for largest values of (real) eigenproblems */
void spral_ssmfe_largest_double(struct spral_ssmfe_rcid *rci, int problem,
      int nep, int m, double *lambda, double *rr, int *ind,
      void **keep, const struct spral_ssmfe_core_options *options,
      struct spral_ssmfe_inform *inform);
/* Core driver routine for largest values of (complex) eigenproblems */
void spral_ssmfe_largest_double_complex(struct spral_ssmfe_rciz *rci,
      int problem, int nep, int m, double *lambda, spral_double_complex *rr,
      int *ind, void **keep, const struct spral_ssmfe_core_options *options,
      struct spral_ssmfe_inform *inform);
/* Free memory */
void spral_ssmfe_core_free(void **keep, struct spral_ssmfe_inform *inform);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // SPRAL_SSMFE_H
