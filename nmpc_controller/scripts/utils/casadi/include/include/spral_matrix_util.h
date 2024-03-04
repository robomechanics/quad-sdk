#include <stdint.h>

#ifndef SPRAL_MATRIX_UTIL_H
#define SPRAL_MATRIX_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Note: At present, interface is only partially defined! */

enum spral_matrix_type {
   // User doesn't wish to specify, use default behaviour
   SPRAL_MATRIX_UNSPECIFIED=0,
   // Rectangular matrix, m!=n
   SPRAL_MATRIX_REAL_RECT=1,        SPRAL_MATRIX_CPLX_RECT=-1,
   // Square unsymmetric matrix m==n
   SPRAL_MATRIX_REAL_UNSYM=2,       SPRAL_MATRIX_CPLX_UNSYM=-2,
   // Symmetric/Hermitian positive-definite matrix
   SPRAL_MATRIX_REAL_SYM_PSDEF=3,   SPRAL_MATRIX_CPLX_HERM_PSDEF=-3,
   // Symmetric/Hermitian indefinite matrix
   SPRAL_MATRIX_REAL_SYM_INDEF=4,   SPRAL_MATRIX_CPLX_HERM_INDEF=-4,
   // Complex symmetric matrix
                                    SPRAL_MATRIX_CPLX_SYM=-5,
   // Skew-symmetric matrix
   SPRAL_MATRIX_REAL_SKEW=6,        SPRAL_MATRIX_CPLX_SKEW=-6
};

void spral_half_to_full_i64d(int n, int64_t *ptr, int *row, double *a, int base);
void spral_print_matrix(int lines, enum spral_matrix_type matrix_type, int m,
      int n, const int *ptr, const int *row, const double *val, int base);
void spral_print_matrix_i64d(int lines, enum spral_matrix_type matrix_type,
      int m, int n, const int64_t *ptr, const int *row, const double *val,
      int base);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // SPRAL_MATRIX_UTIL_H
