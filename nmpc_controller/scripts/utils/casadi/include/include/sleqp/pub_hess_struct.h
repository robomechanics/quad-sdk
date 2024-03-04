#ifndef SLEQP_PUB_HESS_STRUCT_H
#define SLEQP_PUB_HESS_STRUCT_H

#include "sleqp/export.h"
#include "sleqp/pub_types.h"

/**
 * @file pub_hess_struct.h
 * @brief Definition of problem block structure.
 **/

/**
 *
 * Hessian structure of the Lagrangian
 *
 * \f[ L(x, \lambda, \mu) = f(x) + \langle \lambda, c \rangle + \langle 1, \mu
 *\rangle \f].
 *
 * The Hessian \f$ H_L \f$ is assumed to consist of a number of \f$k\f$ blocks,
 * given in terms of indices \f$ 1 = j_1 < j_2 < \ldots < j_{k+1} \leq n \f$.
 * All non-zero entries \f$ (i, j) \f$ must satisfy that
 * \f$ j_l \leq i, j < j_{l + 1} \f$ for some \f$ l = 1, \ldots, k \f$.
 *
 * The default is to assume one block of size \f$ n \f$, implying
 * no particular structure of the Hessian.
 *
 * If \f$ j_{k+1} < n \f$, then the range from \f$ j_{k+1} \f$ to \f$ n \f$
 * must only contain zero entries, i.e., combinations of linear constraints
 * and variables.
 *
 **/
typedef struct SleqpHessStruct SleqpHessStruct;

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_hess_struct_create(SleqpHessStruct** star, int dimension, bool empty);

/**
 * Returns the number \f$ k \f$ of blocks.
 *
 * @param[in]  hessian_struct  The Hessian structure
 * @returns                    The number \f$ k \f$ of blocks
 **/
SLEQP_EXPORT int
sleqp_hess_struct_num_blocks(const SleqpHessStruct* hessian_struct);

/**
 * Returns the \f$ l \f$-th block of the Hessian
 *
 * @param[in]  hessian_struct  The Hessian structure
 * @param[in]  block           The index \f$ l \f$ of the block
 * @param[out] begin           The 0-based index \f$ j_l \f$
 * @param[out] end             The 0-based index \f$ j_{l+1} \f$
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_hess_struct_block_range(const SleqpHessStruct* hessian_struct,
                              int block,
                              int* begin,
                              int* end);

/**
 * Pushes a new block into the Hessian
 *
 * @param[in]  hessian_struct  The Hessian structure
 * @param[out] end             The 0-based index \f$ j_{l+1} \f$
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_hess_struct_push_block(SleqpHessStruct* hessian_struct, int end);

/**
 * Clears the Hessian structure, i.e., sets \f$ k = 0 \f$
 *
 * @param[in]  hessian_struct  The Hessian structure
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_hess_struct_clear(SleqpHessStruct* hessian_struct);

/**
 * Returns the linear range
 *
 * @param[in]   hessian_struct  The Hessian structure
 * @param[out]  begin           The value \f$ j_{k + 1} \f$
 * @param[out]  end             The value \f$ n \f$
 *
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_hess_struct_lin_range(const SleqpHessStruct* hessian_struct,
                            int* begin,
                            int* end);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_hess_struct_copy(const SleqpHessStruct* source, SleqpHessStruct* target);

/**
 * Prints the Hessian structure
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_hess_struct_fprintf(SleqpHessStruct* hessian_struct, FILE* output);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_hess_struct_capture(SleqpHessStruct* hessian_struct);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_hess_struct_release(SleqpHessStruct** star);

#endif /* SLEQP_PUB_HESS_STRUCT_H */
