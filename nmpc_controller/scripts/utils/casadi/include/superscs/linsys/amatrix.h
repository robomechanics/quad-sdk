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
#ifndef AMATRIX_H_GUARD
#define AMATRIX_H_GUARD

#ifdef __cplusplus
extern "C" {
#endif

/* this struct defines the data matrix A */
    
    /**
     * \brief The sparse matrix A of the conic optimization problem.
     * 
     * The data are stored in \ref page_sparse_matrices "CSC format".
     * 
     * \sa \ref page_sparse_matrices "Sparse matrices documentation"
     */
struct scs_a_data_matrix {
    /* A is supplied in column compressed format */
    /**
     * \brief Values of \c A
     */
    scs_float *x; /* A values, size: NNZ A */
    /**
     * \brief The row-index of the matrix (\f$J\f$). 
     * 
     * An array of length equal to the number of nonzero elements of \c A.
     */
    scs_int *i;
    /**
     * \brief  Array of column pointers of A in \ref page_sparse_matrices "CSC format"
     * (\f$I\f$).
     * 
     * The length of <code>A->p</code> is equal to <code>A->n + 1</code>.
     * 
     */
    scs_int *p;
    /**
     * \brief  Number of rows (\f$m\f$).
     */
    scs_int m;
    /**
     * \brief  Number of columns (\f$n\f$).
     */
    scs_int n; 
};

#ifdef __cplusplus
}
#endif
#endif
