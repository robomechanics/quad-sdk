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
#ifndef SCS_CS_H_GUARD
#define SCS_CS_H_GUARD

#include "glbopts.h"

/**
 * \brief Matrix in compressed-column or triplet form.
 * 
 * This is a subset of the routines in the CSPARSE package by
 * Tim Davis et. al. For the full package please visit
 * http://www.cise.ufl.edu/research/sparse/CSparse/.
 * 
 * \note In order to avoid conflicts in case some users want to load
 *       both SuperSCS and CSPARSE in their project, we have prepended
 *       the prefix scs_ in all function names here (although not
 *       necessary, we did it in static functions for uniformity. 
 *
 */
typedef struct scs_cs_sparse {
    scs_int nzmax; /**< \brief maximum number of entries */
    scs_int m; /**< \brief number of rows */
    scs_int n; /**< \brief number of columns */
    scs_int *p; /**< \brief column pointers (size n+1) or col indices (size nzmax) */
    scs_int *i; /**< \brief row indices, size nzmax */
    scs_float *x; /**< \brief numerical values, size nzmax */
    scs_int nz; /**< \brief Number of entries in triplet matrix, -1 for compressed-col */
} scs_cs;

/**
 * \brief Compress a triplet matrix into a column-packed representation.
 */
scs_cs *scs_cs_compress(const scs_cs *T);


/**
 * \brief Frees the memory of <code>x</code> and <code>w></code>.
 * 
 * If <code>ok</code> is nonzero, it returns <code>C</code>, otherwise
 * it frees <code>C</code> (it calls ::scs_cs_spfree) and returns ::SCS_NULL.
 * 
 * @param C
 * @param w
 * @param x
 * @param ok
 * @return 
 */
scs_cs *scs_cs_done(scs_cs *C, void *w, void *x, scs_int ok);

/**
 * \brief Allocates a sparse matrix of given dimensions.
 * 
 * @param m number of rows
 * @param n number of columns
 * @param nzmax maximum number of nonzero elements
 * @param values whether to allocate memory for the matrix values
 * @param triplet whether the triplet representation is used
 * @return 
 */
scs_cs *scs_cs_spalloc(
        scs_int m, 
        scs_int n, 
        scs_int nzmax, 
        scs_int values,
        scs_int triplet);

scs_cs *scs_cs_spfree(scs_cs *A);

scs_float scs_cs_cumsum(
        scs_int *p, 
        scs_int *c, 
        scs_int n);

scs_int *scs_cs_pinv(
        scs_int const *p, 
        scs_int n);

scs_cs *scs_cs_symperm(
        const scs_cs *A, 
        const scs_int *pinv, 
        scs_int values);

#endif
