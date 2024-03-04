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
#ifndef SCS_BLAS_H_GUARD
#define SCS_BLAS_H_GUARD

#ifdef LAPACK_LIB_FOUND

#ifdef __cplusplus
extern "C" {
#endif

/* Default to underscore for blas / lapack */
#ifndef BLASSUFFIX
#define BLASSUFFIX _
#endif

/* annoying hack because some preprocessors can't handle empty macros */
#if defined(NOBLASSUFFIX) && NOBLASSUFFIX > 0
/* single or double precision */
#ifndef FLOAT
#define BLAS(x) d##x
#else
#define BLAS(x) s##x
#endif
#else
/* this extra indirection is needed for BLASSUFFIX to work correctly as a
 * variable */
#define stitch_(pre, x, post) pre##x##post
#define stitch__(pre, x, post) stitch_(pre, x, post)
/* single or double precision */
#ifndef FLOAT
#define BLAS(x) stitch__(d, x, BLASSUFFIX)
#else
#define BLAS(x) stitch__(s, x, BLASSUFFIX)
#endif
#define LPCK BLAS
#endif

#ifdef MATLAB_MEX_FILE
typedef ptrdiff_t blasint;
#elif defined BLAS64
#include <stdint.h>
typedef int64_t blasint;
#else
typedef int blasint;
#endif

#ifdef __cplusplus
}
#endif

#endif /* LAPACK_LIB_FOUND */

#endif /* SCS_BLAS_H_GUARD */
