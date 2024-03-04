#ifndef SPRAL_SCALING_H
#define SPRAL_SCALING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/************************************
 * Derived types
 ************************************/

struct spral_scaling_auction_options {
   int array_base; // Not in Fortran type
   int max_iterations;
   int max_unchanged[3];
   float min_proportion[3];
   float eps_initial;
   char unused[80]; // Allow for future expansion
};
struct spral_scaling_auction_inform {
   int flag;
   int stat;
   int matched;
   int iterations;
   int unmatchable;
   char unused[80]; // Allow for future expansion
};

struct spral_scaling_equilib_options {
   int array_base; // Not in Fortran type
   int max_iterations;
   float tol;
   char unused[80]; // Allow for future expansion
};
struct spral_scaling_equilib_inform {
   int flag;
   int stat;
   int iterations;
   char unused[80]; // Allow for future expansion
};

struct spral_scaling_hungarian_options {
   int array_base; // Not in Fortran type
   bool scale_if_singular;
   char unused[80]; // Allow for future expansion
};
struct spral_scaling_hungarian_inform {
   int flag;
   int stat;
   int matched;
   char unused[80]; // Allow for future expansion
};

/************************************
 * Default setting subroutines
 ************************************/

/* Set default values for auction_options */
void spral_scaling_auction_default_options(
      struct spral_scaling_auction_options *options);
/* Set default values for equilib_options */
void spral_scaling_equilib_default_options(
      struct spral_scaling_equilib_options *options);
/* Set default values for hungarian_options */
void spral_scaling_hungarian_default_options(
      struct spral_scaling_hungarian_options *options);

/************************************
 * Symmetric subroutines
 ************************************/

/* Scale a symmetric matrix using auction algorithm */
void spral_scaling_auction_sym(int n, const int *ptr, const int *row,
      const double *val, double *scaling, int *match,
      const struct spral_scaling_auction_options *options,
      struct spral_scaling_auction_inform *inform);
void spral_scaling_auction_sym_long(int n, const int64_t *ptr, const int *row,
      const double *val, double *scaling, int *match,
      const struct spral_scaling_auction_options *options,
      struct spral_scaling_auction_inform *inform);
/* Scale a symmetric matrix using norm equilibriation algorithm */
void spral_scaling_equilib_sym(int n, const int *ptr, const int *row,
      const double *val, double *scaling,
      const struct spral_scaling_equilib_options *options,
      struct spral_scaling_equilib_inform *inform);
void spral_scaling_equilib_sym_long(int n, const int64_t *ptr, const int *row,
      const double *val, double *scaling,
      const struct spral_scaling_equilib_options *options,
      struct spral_scaling_equilib_inform *inform);
/* Scale a symmetric matrix using Hungarian algorithm */
void spral_scaling_hungarian_sym(int n, const int *ptr, const int *row,
      const double *val, double *scaling, int *match,
      const struct spral_scaling_hungarian_options *options,
      struct spral_scaling_hungarian_inform *inform);
void spral_scaling_hungarian_sym_long(int n, const int64_t *ptr, const int *row,
      const double *val, double *scaling, int *match,
      const struct spral_scaling_hungarian_options *options,
      struct spral_scaling_hungarian_inform *inform);

/************************************
 * Unsymmetric subroutines
 ************************************/

/* Scale a symmetric matrix using auction algorithm */
void spral_scaling_auction_unsym(int m, int n, const int *ptr,
      const int *row, const double *val, double *rscaling, double *cscaling,
      int *match, const struct spral_scaling_auction_options *options,
      struct spral_scaling_auction_inform *inform);
void spral_scaling_auction_unsym_long(int m, int n, const int64_t *ptr,
      const int *row, const double *val, double *rscaling, double *cscaling,
      int *match, const struct spral_scaling_auction_options *options,
      struct spral_scaling_auction_inform *inform);
/* Scale a symmetric matrix using norm equilibriation algorithm */
void spral_scaling_equilib_unsym(int m, int n, const int *ptr,
      const int *row, const double *val, double *rscaling, double *cscaling,
      const struct spral_scaling_equilib_options *options,
      struct spral_scaling_equilib_inform *inform);
void spral_scaling_equilib_unsym_long(int m, int n, const int64_t *ptr,
      const int *row, const double *val, double *rscaling, double *cscaling,
      const struct spral_scaling_equilib_options *options,
      struct spral_scaling_equilib_inform *inform);
/* Scale a symmetric matrix using Hungarian algorithm */
void spral_scaling_hungarian_unsym(int m, int n, const int *ptr,
      const int *row, const double *val, double *rscaling, double *cscaling,
      int *match, const struct spral_scaling_hungarian_options *options,
      struct spral_scaling_hungarian_inform *inform);
void spral_scaling_hungarian_unsym_long(int m, int n, const int64_t *ptr,
      const int *row, const double *val, double *rscaling, double *cscaling,
      int *match, const struct spral_scaling_hungarian_options *options,
      struct spral_scaling_hungarian_inform *inform);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // SPRAL_SCALING_H
