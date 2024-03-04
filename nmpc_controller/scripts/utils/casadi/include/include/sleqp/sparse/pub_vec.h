#ifndef SLEQP_PUB_VEC_H
#define SLEQP_PUB_VEC_H

/**
 * @file pub_vec.h
 * @brief Definition of vectors.
 **/

#include "sleqp/export.h"
#include "sleqp/pub_types.h"

/**
 * A sparse vector data structure. Indices
 * are stored in an ascending fashion.
 **/
typedef struct SleqpVec
{
  double* data;
  int* indices;

  int dim;
  int nnz;
  int nnz_max;

} SleqpVec;

/**
 * Creates a new vector. Data and indices are set to
 * have size of nnz_max.
 *
 * @param[in]  vec     A pointer to the vector to be created
 * @param[in]  dim     The desired dimension of the vector
 * @param[in]  nnz_max The desired amount of nonzeros of the vector
 *
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_create(SleqpVec** vec, int dim, int nnz_max);

/**
 * Creates a new vector without allocating memory
 * for non-zero entries
 *
 * @param[in]  vec     A pointer to the vector to be created
 * @param[in]  dim     The desired dimension of the vector
 *
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_create_empty(SleqpVec** vec, int dim);

/**
 * Creates a new vector, allocating memory sufficient
 * for `dim` non-zero entries
 *
 * @param[in]  vec     A pointer to the vector to be created
 * @param[in]  dim     The desired dimension of the vector
 *
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_create_full(SleqpVec** vec, int dim);

/**
 * Pushes a new entry on top of a vector. The new
 * entry is assumed to have a larger index than the existing ones.
 *
 * @param[in,out] vec    A pointer to the vector
 * @param[in]     idx    The index of the new entry
 * @param[in]     value  The value of the new entry
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_push(SleqpVec* vec, int idx, double value);

/**
 * Creates the entries of a vector from a dense
 * vector. The vector will reserve an appropriate
 * number of entries, the dimension will be changed to
 * match that of the dense vector.
 *
 * @param[in,out] vec         A pointer to the vector
 * @param[in]     values      A vector of values
 * @param[in]     dim         The dimension of the values input
 * @param[in]     zero_eps    The numerical tolerance
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_set_from_raw(SleqpVec* vec,
                       const double* values,
                       int dim,
                       double zero_eps);

/**
 * Fills the given vector with the given value
 * by setting all of its element to have that precise value
 *
 * @param[in,out] vec         A pointer to the vector
 * @param[in]     value       The given value
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_fill(SleqpVec* vec, double value);

/**
 * Writes the content of this vector into an array. The
 * array is assumed to have a size of at least the dimension
 * of the given vector
 *
 * @param[in] vec     A pointer to the vector
 * @param[in] values  A pointer to the output array
 *
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_to_raw(const SleqpVec* vec, double* values);

/**
 * Copies one vector to another
 *
 * @param[in] source     A pointer to the copy source
 * @param[out] target    A pointer to the copy target
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_copy(const SleqpVec* source, SleqpVec* target);

/**
 * Clears the given vector, discarding all entries while
 * keeping the dimension constant
 *
 * @param[in,out] vec     A pointer to the vector
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_clear(SleqpVec* vec);

/**
 * Reserves space for additional nonzeros
 *
 * @param[in,out] vec     A pointer to the vector
 * @param[in]     nnz_max The number of nonzeros
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_reserve(SleqpVec* vec, int nnz);

/**
 * Resizes the vector to the given dimension, discarding
 * entries if necessary
 *
 * @param[in,out] vec     A pointer to the vector
 * @param[in]     dim     The new dimension
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_resize(SleqpVec* vec, int dim);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_concat(const SleqpVec* first,
                 const SleqpVec* second,
                 SleqpVec* result);

/**
 * Returns whether all entries of the given vector are equal
 * up to the given tolerance
 *
 * @param[in]  first     A pointer to the first vector
 * @param[in]  second    A pointer to the second vector
 * @param[in]  eps       The desred tolerance
 *
 * @sa sleqp_is_eq(double x, double y, double eps)
 **/
SLEQP_EXPORT bool
sleqp_vec_eq(const SleqpVec* first, const SleqpVec* second, double eps);

/**
 * Computes the dot product of two vectors
 *
 * @param[in]  first     A pointer to the first vector
 * @param[in]  second    A pointer to the second vector
 * @param[out] product   A pointer to the result
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_dot(const SleqpVec* first, const SleqpVec* second, double* product);

/**
 * Scales the vector by a factor
 *
 * @param[in,out] vector   A pointer to the vector
 * @param[in]     factor   The factor
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_scale(SleqpVec* vector, const double factor);

/**
 * Computes the sum of two vectors
 *
 * @param[in]  first         A pointer to the first vector
 * @param[in]  second        A pointer to the second vector
 * @param[out] result        A pointer to the result
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_add(const SleqpVec* first,
              const SleqpVec* second,
              const double eps,
              SleqpVec* result);

/**
 * Computes the weighted sum of two vectors
 *
 * @param[in]  first         A pointer to the first vector
 * @param[in]  second        A pointer to the second vector
 * @param[in]  first_factor  A factor for the first vector
 * @param[in]  second_factor A factor for the second vector
 * @param[out] result        A pointer to the result
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_add_scaled(const SleqpVec* first,
                     const SleqpVec* second,
                     const double first_factor,
                     const double second_factor,
                     const double eps,
                     SleqpVec* result);

/**
 * Fills all entries of the vector with the specified value
 *
 * @param[in,out] vector   A pointer to the vector
 * @param[in]      value   The value
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_fill(SleqpVec* vec, double value);

/**
 * Returns the 2-norm of the given vector
 *
 * @param[in] vector   A pointer to the vector
 **/
SLEQP_EXPORT double
sleqp_vec_norm(const SleqpVec* vec);

/**
 * Returns the 1-norm of the given vector
 *
 * @param[in] vector   A pointer to the vector
 **/
SLEQP_EXPORT double
sleqp_vec_one_norm(const SleqpVec* vec);

/**
 * Returns the squared 2-norm of the given vector
 *
 * @param[in] vector   A pointer to the vector
 **/
SLEQP_EXPORT double
sleqp_vec_norm_sq(const SleqpVec* vec);

/**
 * Returns the oo-norm of the given vector
 *
 * @param[in] vector   A pointer to the vector
 **/
SLEQP_EXPORT double
sleqp_vec_inf_norm(const SleqpVec* vec);

/**
 * Returns a pointer to the entry of the given vector at
 * the given index, or `NULL` if the entry is not present.
 *
 * @param[in] vector   A pointer to the vector
 * @param[in] index    The desired index
 **/
SLEQP_EXPORT double*
sleqp_vec_at(const SleqpVec* vec, int index);

/**
 * Returns the value of the given vector at the given index
 *
 * @param[in] vector   A pointer to the vector
 * @param[in] index    The desired index
 **/
SLEQP_EXPORT double
sleqp_vec_value_at(const SleqpVec* vec, int index);

/**
 * Returns whether this vector is boxed, i.e., \f$ lb \leq x \leq ub \f$
 * for all components.
 *
 * @param[in] x    A pointer to the vector
 * @param[in] lb   A pointer to the lower bound vector
 * @param[in] ub   A pointer to the upper bound vector
 *
 SLEQP_EXPORT * @sa sleqp_vec_clip
**/
SLEQP_EXPORT bool
sleqp_vec_is_boxed(const SleqpVec* x, const SleqpVec* lb, const SleqpVec* ub);

/**
 * Clips this vector to the specified lower and upper bounds, storing
 * the result in the pointer to a new vector, which will be boxed
 * w.r.t. `lb` and `ub`
 *
 * @param[in]  x     A pointer to the vector
 * @param[in]  lb    A pointer to the lower bound vector
 * @param[in]  ub    A pointer to the upper bound vector
 * @param[out] xclip A pointer to the clipped vector
 *
 SLEQP_EXPORT * @sa sleqp_vec_is_boxed
**/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_clip(const SleqpVec* x,
               const SleqpVec* lb,
               const SleqpVec* ub,
               const double eps,
               SleqpVec* xclip);

/**
 * Prints this vector to the given file
 *
 * @param[in]  vec     A pointer to the vector
 * @param[in]  output  A pointer to an output `FILE*`
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_fprintf(const SleqpVec* vec, FILE* output);

/**
 * Returns whether the given vector is *valid*, i.e., whether
 * - `nnz` non-negative and less than or equal to `nnz_max`
 * - all indices are non-negative and less than or equal to `dim`
 * - the entries are ordered according to their indices
 *
 **/
SLEQP_EXPORT bool
sleqp_vec_is_valid(const SleqpVec* vec);

/**
 * Returns whether the entries of the given vector are finite with respect to
 *  \ref sleqp_is_finite(double)
 **/
SLEQP_EXPORT bool
sleqp_vec_is_finite(const SleqpVec* vec);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_vec_free(SleqpVec** vec);

#endif /* SLEQP_PUB_VEC_H */
