/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Pantelis Sopasakis (https://alphaville.github.io),
 *                    Krina Menounou (https://www.linkedin.com/in/krinamenounou), 
 *                    Panagiotis Patrinos (http://homes.esat.kuleuven.be/~ppatrino)
 * Copyright (c) 2012 Brendan O'Donoghue (bodonoghue85@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */
#ifndef SCS_LINALG_H_GUARD
#define SCS_LINALG_H_GUARD

#ifdef __cplusplus
extern "C" {
#endif

#include "scs.h"
#include <math.h>

    /**
     * \brief Computes the optimal workspace size for ::scs_svdls
     * 
     * @param m     number of rows of matrix A
     * @param n     number of columns of matrix A
     * @return      optimal workspace size
     * 
     * \note To use this function, you need to compile with USE_LAPACK=1 (recommended).
     */
    scs_int scs_svd_workspace_size(
            scs_int m,
            scs_int n
            );

    /**
     * \brief Compute the optimal size of workspace for ::scs_qrls.
     * 
     * @param m rows of A
     * @param n columns of A
     * 
     * @return optimal size of workspace
     * 
     * \note To use this function, you need to compile with USE_LAPACK=1 (recommended).
     */
    scs_int scs_qr_workspace_size(
            scs_int m,
            scs_int n
            );

    /**
     * \brief Solves a least squares problem using the QR factorization
     * 
     * @param m rows of A
     * @param n columns of A
     * @param A On entry, matrix A (column-packed). On exit, if \f$m\geq n\f$, A is 
     *          overwritten by details of its QR factorization as returned by lapack's
     *          DGEQRF; otherwise, A is overwritten by details of its LQ factorization 
     *          as returned by DGELQF.
     * @param b On entry: vector b, On exit: solution
     * @param wspace workspace
     * @param wsize workspace size
     * @return status code (0: success)
     * 
     * @see ::scs_qr_workspace_size
     * 
     * \note This is a wrapper for lapack's ?gels
     * 
     * \warning It is assumed that matrix \f$A\f$ has full rank. If not, 
     * use ::scs_svdls.
     * 
     * \note To use this function, you need to compile with USE_LAPACK=1 (recommended).
     */
    scs_int scs_qrls(
            scs_int m,
            scs_int n,
            scs_float * RESTRICT A,
            scs_float * RESTRICT b,
            scs_float * RESTRICT wspace,
            scs_int wsize
            );

    /**
     * \brief Solves a least squares problem using the SVD factorization
     * 
     * Solves the least squares problem \f$\mathrm{Minimize}\|b-Ax\|^2\f$ where 
     * \f$A\in\mathbb{R}^{m\times n}\f$ and \f$b\in\mathbb{R}^{m}\f$.
     * 
     * @param m                 number of rows of matrix A
     * @param n                 number of columns of matrix A
     * @param A                 On entry, matrix A. On exit, the first min(m,n) 
     *                          rows of A are overwritten with its right singular 
     *                          vectors, stored row-wise.
     * @param b                 On entry, vector b, On exit, solution
     * @param wspace            workspace
     * @param wsize             size of the workspace (its size is returned by
     *                          #scs_svd_workspace_size)
     * @param rcond             rcond is used to determine the effective rank of A. 
     *                          singular values \f$ \sigma_i \leq \mathrm{rcond} \cdot \sigma_1\f$ 
     *                          are treated as zero.
     * @param singular_values   this function computes the singular values of \f$A\f$
     * @param rank              the effective rank of matrix \f$A\f$, that is, the number 
     *                          of singular values which are greater than 
     *                          \f$\mathrm{rcond} \cdot \sigma_1\f$.
     * @return status (0: success)
     * 
     * \note This is a wrapper for lapack's ?gelss.
     * 
     * \note To use this function, you need to compile with USE_LAPACK=1 (recommended).
     */
    scs_int scs_svdls(
            scs_int m,
            scs_int n,
            scs_float * RESTRICT A,
            scs_float * RESTRICT b,
            scs_float * RESTRICT wspace,
            scs_int wsize,
            scs_float rcond,
            scs_float * RESTRICT singular_values,
            scs_int * RESTRICT rank
            );


    /**
     * Performs the operation
     * \f[
     *  x \leftarrow b\cdot a,
     * \f]
     * where <code>a</code> is a vector and <code>b</code> is a scalar.
     * 
     * @param x
     * @param a
     * @param b
     * @param len
     * 
     * \note with loop unrolling for speed
     */
    void scs_set_as_scaled_array(
            scs_float * RESTRICT x,
            const scs_float * RESTRICT a,
            const scs_float b,
            scs_int len);

    /**
     * Performs the operation
     * \f[
     *   a \leftarrow b\cdot a
     * \f]
     * @param a vector \f$a\f$
     * @param b vector \f$b\f$
     * @param len length of vectors
     * 
     */
    void scs_scale_array(
            scs_float * RESTRICT a,
            const scs_float b,
            scs_int len);

    /**
     * Computes the inner product of two vectors, that is
     * \f[
     *  \langle x, y \rangle = x'y = \sum_{i=1}^{\mathrm{len}}x_i y_i.
     * \f]
     * @param x vector \f$x\f$
     * @param y vector \f$y\f$
     * @param len length of vectors
     * @return  inner product of \f$x\f$ with \f$y\f$
     * 
     */
    scs_float scs_inner_product(
            const scs_float * RESTRICT x,
            const scs_float * RESTRICT y,
            scs_int len);

    /**
     * Returns the square Euclidean norm of a vector \f$v\f$.
     * @param v vector \f$v\f$
     * @param len length of vector
     * @return norm of vector \f$v\f$
     * 
     * \note uses ::scs_inner_product
     */
    scs_float scs_norm_squared(
            const scs_float * RESTRICT v,
            scs_int len);

    /**
     * Returns the Euclidean norm of a vector.
     * @param v
     * @param len
     * @return 
     */
    scs_float scs_norm(
            const scs_float * RESTRICT v,
            scs_int len);

    /**
     * Returns the infinity norm of a vector.
     * @param a
     * @param l
     * @return 
     */
    scs_float scs_norm_infinity(
            const scs_float * RESTRICT a,
            scs_int l);


    /**
     * Performs the operation
     * \f[
     *  a \leftarrow a + \gamma b
     * \f]
     * @param a vector <code>a</code>
     * @param b vector <code>b</code>
     * @param n length of <code>a</code>
     * @param sc the scalar \f$\gamma\f$
     * 
     * \note with loop unrolling for speed
     */
    void scs_add_scaled_array(
            scs_float * RESTRICT a,
            const scs_float * RESTRICT b,
            scs_int n,
            const scs_float sc);

    /**
     * Performs the operation
     * \f[
     *  a \leftarrow a + b
     * \f]
     * @param a vector <code>a</code>
     * @param b vector <code>b</code>
     * @param n length of <code>a</code>
     * 
     * \note with loop unrolling for speed
     */
    void scs_add_array(
            scs_float * RESTRICT a,
            const scs_float * RESTRICT b,
            scs_int n);

    /**
     * Computes \f$x \leftarrow \alpha u + \beta v\f$
     * 
     * \note The pointer <code>x</code> can have the same value as 
     * <code>u</code> so as to perform
     * operations like \f$x\leftarrow \alpha x + \beta v\f$.     
     */
    void scs_axpy(
            scs_float * RESTRICT x,
            const scs_float * RESTRICT u,
            const scs_float * RESTRICT v,
            scs_float a,
            scs_float b,
            scs_int n);

    /**
     * Performs the operation
     * \f[
     *  a \leftarrow a - b
     * \f]
     * @param a vector <code>a</code>
     * @param b vector <code>b</code>
     * @param n length of <code>a</code>
     * 
     * \note with loop unrolling for speed
     */
    void scs_subtract_array(
            scs_float * RESTRICT a,
            const scs_float * RESTRICT b,
            scs_int n);

    /**
     * Returns the Euclidean norm of the difference of two vectors
     * @param a
     * @param b
     * @param l
     * @return 
     */
    scs_float scs_norm_difference(
            const scs_float * RESTRICT a,
            const scs_float * RESTRICT b,
            scs_int l);

    /**
     * Returns the infinity norm of the difference of two vectors
     * @param a
     * @param b
     * @param l
     * @return 
     */
    scs_float scs_norm_infinity_difference(
            const scs_float * RESTRICT a,
            const scs_float * RESTRICT b,
            scs_int l);
    
    

    /**
     * Perofrms the operation \f$C \leftarrow \beta C + \alpha A B,\f$
     * where \f$A\f$, \f$B\f$ and \f$C\f$ are column-packed matrices.
     * 
     * 
     * @param rows_A number of rows of matrix \f$A\f$
     * @param cols_B number of columns of matrix \f$B\f$
     * @param cols_A number of rows of matrix \f$B\f$ (columns of \f$A\f$)
     * @param alpha coefficient \f$\alpha\f$
     * @param A pointer to matrix \f$A\f$ in column-packed form
     * @param beta coefficient \f$\beta\f$
     * @param B pointer to matrix \f$B\f$ in column-packed form
     * @param C pointer to matrix \f$C\f$ in column-packed form
     * 
     */
    void scs_matrix_multiply(
            scs_int rows_A,
            scs_int cols_B,
            scs_int cols_A,
            scs_float alpha,
            const scs_float * RESTRICT A,
            scs_float beta,
            const scs_float * RESTRICT B,
            scs_float *C);

    /**
     * Perofrms the operation \f$C \leftarrow \beta C + \alpha A^{\top} B,\f$
     * where \f$A\f$, \f$B\f$ and \f$C\f$ are column-packed matrices.
     * 
     * 
     * @param rows_A number of rows of matrix \f$A\f$
     * @param cols_B number of columns of matrix \f$B\f$
     * @param cols_A number of rows of matrix \f$B\f$ (columns of \f$A\f$)
     * @param alpha coefficient \f$\alpha\f$
     * @param A pointer to matrix \f$A\f$ in column-packed form
     * @param beta coefficient \f$\beta\f$
     * @param B pointer to matrix \f$B\f$ in column-packed form
     * @param C pointer to matrix \f$C\f$ in column-packed form
     * 
     * 
     */
    void scs_matrix_transpose_multiply(
            scs_int rows_A,
            scs_int cols_B,
            scs_int cols_A,
            scs_float alpha,
            const scs_float * RESTRICT A,
            scs_float beta,
            const scs_float * RESTRICT B,
            scs_float *C);


    /** 
     * Allocates memory to be used as workspace in #scs_cgls (see 
     * documentation of #scs_cgls for details).
     * 
     * If either <code>m</code> or <code>n</code> are negative or zero, it returns #SCS_NULL.
     * 
     * \note The caller should always check whether the returned pointer is #SCS_NULL.
     * 
     * \warning The caller should free the memory allocated by this function. Example:
     * ~~~~~
     * scs_int m = 10;
     * scs_int n = 2;
     * scs_float * ws;
     * ws = scs_cgls_malloc_workspace(scs_int m, scs_int n);
     * if (ws == SCS_NULL) {
     *      // memory not allocated, take necessary action
     * }
     * // use ws in cgls
     * scs_free(ws);
     * ~~~~~
     * 
     * @param m     number of rows of matrix A
     * @param n     number of columns of matrix A
     * @return      pointer to allocated 
     * 
     * @see ::scs_cgls
     */
    scs_float * scs_cgls_malloc_workspace(scs_int m, scs_int n);

    /**
     * Solves a least squares problem using the conjugate gradient method.
     * 
     * Solves the problem: Minimize \f$\|Ax-b\|^2\f$, or, what is the same, the 
     * linear system \f$ A^{\top}Ax = A^{\top}b\f$.
     * 
     * The iterations are terminated when the Euclidean norm of the residual, 
     * \f$r = A^{\top}(b - Ax)\f$
     * becomes smaller than the specified tolerance.
     *   
     * @param m         Number of rows of matrix <code>A</code>
     * @param n         Number of columns of <code>A</code>
     * @param A         Matrix <code>A</code> (column-packed)
     * @param b         Right-hand side vector <code>b</code>
     * @param x         Solution (on entry: initial guess)
     * @param tol       Tolerance
     * @param maxiter   Maximum number of CG iterations (on exit: number of iterations)
     * @param wspace    Externally allocated memory space serving as workspace. 
     *                  This must be of size <code>(max(m,n) + m + 2 * n) * sizeof(scs_float)</code>.
     *                  On exit, the first <code>n</code> memory positions store the residual.
     *                  You may use #scs_cgls_malloc_workspace to allocate the workspace.
     * 
     * @return status code (0: success, 1: maximum number of iterations reached).
     * 
     * @see ::scs_cgls_malloc_workspace
     */
    scs_int scs_cgls(
            scs_int m,
            scs_int n,
            const scs_float * RESTRICT A,
            const scs_float * RESTRICT b,
            scs_float*RESTRICT x,
            scs_float tol,
            scs_int*RESTRICT maxiter,
            scs_float *RESTRICT wspace
            );


#ifdef __cplusplus
}
#endif
#endif
