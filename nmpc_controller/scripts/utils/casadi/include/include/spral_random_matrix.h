#ifndef SPRAL_RANDOM_MATRIX_H
#define SPRAL_RANDOM_MATRIX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "spral_matrix_util.h"

#define SPRAL_RANDOM_MATRIX_FINDEX        1
#define SPRAL_RANDOM_MATRIX_NONSINGULAR   2
#define SPRAL_RANDOM_MATRIX_SORT          4

/* Generate an m x n random matrix with nnz non-zero entries */
int spral_random_matrix_generate(int *state, enum spral_matrix_type matrix_type,
      int m, int n, int nnz, int *ptr, int *row, double *val, int flags);
/* Generate an m x n random matrix with nnz non-zero entries (nnz,ptr int64_t) */
int spral_random_matrix_generate_long(int *state,
      enum spral_matrix_type matrix_type, int m, int n, int64_t nnz, int64_t *ptr,
      int *row, double *val, int flags);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // SPRAL_RANDOM_MATRIX_H
