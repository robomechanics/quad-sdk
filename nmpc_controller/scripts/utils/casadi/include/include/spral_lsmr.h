#ifndef SPRAL_LSMR_H
#define SPRAL_LSMR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/************************************
 * Derived types
 ************************************/
struct spral_lsmr_options {
   double atol;
   double btol;
   double conlim;
   int ctest;
   int itnlim;
   int itn_test;
   int localSize;
   int print_freq_head;
   int print_freq_itn;
   int unit_diagnostics;
   int unit_error;
};

struct spral_lsmr_inform {
   int flag;
   int itn;
   int stat;
   double normb;
   double normAP;
   double condAP;
   double normr;
   double normAPr;
   double normy;
};

/************************************
 * Subroutines 
 ************************************/

/* Initalise options structure to defaults */
void spral_lsmr_default_options(struct spral_lsmr_options *options);
/* Solve a least squares problem */
int spral_lsmr_solve(int *action, int m, int n, double u[], double v[],
   double y[], void **keep, struct spral_lsmr_options const *options,
   struct spral_lsmr_inform *inform, double *damp);
/* Free memory after solution */
int spral_lsmr_free(void **keep);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // SPRAL_LSMR_H
