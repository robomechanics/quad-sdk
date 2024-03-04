#ifndef SLEQP_PUB_MAT_H
#define SLEQP_PUB_MAT_H

/**
 * @file pub_mat.h
 * @brief Definition of sparse matrices.
 **/

#include "sleqp/export.h"
#include "sleqp/pub_types.h"
#include "sleqp/sparse/pub_vec.h"

/**
 * A sparse matrix data structure.
 * So far the data is stored in CSC format.
 * Specifically:
 *
 * \f$ A(i, j) = data[k]\f$ iff \f$ rows[k] = i \f$ and \f$ cols[j] <= k <
 *cols[j + 1] \f$
 *
 * for \f$ k = 0, \ldots, nnz - 1 \f$
 *
 **/
typedef struct SleqpMat SleqpMat;

/**
 * Creates a new sparse matrix with a specified number of nonzeros
 *
 * @param[in] matrix     A pointer to the matrix to be created
 * @param[in] num_rows   The desired number of rows
 * @param[in] num_cols   The desired number of columns
 * @param[in] nnz_max    The desired number of nonzeros
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_create(SleqpMat** matrix, int num_rows, int num_cols, int nnz_max);

/**
 * Reserves a number of nonzeros for the given matrix
 *
 * @param[in] matrix   The matrix
 * @param[in] nnz      The desired number of nonzeros
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_reserve(SleqpMat* matrix, int nnz);

/**
 * Resizes the given matrix
 *
 * @note If the matrix is non-empty, decreasing the size can leave the matrix in
 *an inconsistent state
 *
 * @param[in] matrix     The matrix
 * @param[in] num_rows   The desired number of rows
 * @param[in] num_cols   The desired number of columns
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_resize(SleqpMat* matrix, int num_rows, int num_cols);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_scale(SleqpMat* matrix, double scale);

/**
 * Returns the number of columns of the given matrix
 **/
SLEQP_EXPORT int
sleqp_mat_num_cols(const SleqpMat* matrix);

/**
 * Returns the number of rows of the given matrix
 **/
SLEQP_EXPORT int
sleqp_mat_num_rows(const SleqpMat* matrix);

/**
 * Returns the number of nonzeros of the given matrix
 **/
SLEQP_EXPORT int
sleqp_mat_nnz(const SleqpMat* matrix);

/**
 * Returns the maximum number of nonzeros of the given matrix
 **/
SLEQP_EXPORT int
sleqp_mat_nnz_max(const SleqpMat* matrix);

/**
 * Sets the number of nonzeros of the given matrix
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_set_nnz(SleqpMat* matrix, int nnz);

/**
 * Returns whether the given matrix is rectangular
 **/
SLEQP_EXPORT bool
sleqp_mat_is_quadratic(const SleqpMat* matrix);

/**
 * Returns a pointer to the values of the matrix
 **/
SLEQP_EXPORT double*
sleqp_mat_data(const SleqpMat* matrix);

/**
 * Returns a pointer to the columns of the given matrix
 **/
SLEQP_EXPORT int*
sleqp_mat_cols(const SleqpMat* matrix);

/**
 * Returns a pointer to the rows of the given matrix
 **/
SLEQP_EXPORT int*
sleqp_mat_rows(const SleqpMat* matrix);

/**
 * Pushes a new entry to the matrix. Fails if the matrix is at capacity
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_push(SleqpMat* matrix, int row, int col, double value);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_push_vec(SleqpMat* matrix, int col, SleqpVec* vec);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_push_col(SleqpMat* matrix, int col);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_pop_col(SleqpMat* matrix, int col);

/**
 * Copies the given matrix
 *
 * @param[in]     source     The source matrix
 * @param[in,out] target     The target matrix
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_copy(const SleqpMat* source, SleqpMat* target);

/**
 * Increases the reference count of the given matrix
 */
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_capture(SleqpMat* matrix);

/**
 * Decreases the reference count of the given matrix, freeing it
 * if the reference count reaches count
 */
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_mat_release(SleqpMat** star);

#endif /* SLEQP_PUB_MAT_H */
