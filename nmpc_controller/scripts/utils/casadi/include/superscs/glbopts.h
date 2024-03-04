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
#ifndef SCS_GLB_H_GUARD
#define SCS_GLB_H_GUARD

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

#ifdef RESTRICT
#undef RESTRICT
#endif

#if(defined _WIN32 || defined _WIN64 || defined _WINDLL)
#define RESTRICT 
#else
#define RESTRICT __restrict
#endif

    /* redefine printfs and memory allocators as needed */
#ifdef MATLAB_MEX_FILE
#include "mex.h"
#define scs_printf mexPrintf
#define scs_free_ mxFree
#define scs_malloc_ mxMalloc
#define scs_calloc_ mxCalloc
#elif defined PYTHON
#include <Python.h>
#include <stdlib.h>
#define scs_printf(...)                                                        \
    {                                                                          \
        PyGILState_STATE gilstate = PyGILState_Ensure();                       \
        PySys_WriteStdout(__VA_ARGS__);                                        \
        PyGILState_Release(gilstate);                                          \
    }
#define scs_free_ free
#define scs_malloc_ malloc
#define scs_calloc_ calloc
#elif(defined(USING_R))
#include <stdlib.h>
#include <stdio.h>
#include <R_ext/Print.h> /* Rprintf etc */
#define scs_printf Rprintf
#define scs_free_ free
#define scs_malloc_ malloc
#define scs_calloc_ calloc
#elif defined CASADI
#include <stdio.h>
#include <stdlib.h>
extern int casadi_printf(const char  *fmt, ...);
#define printf casadi_printf
#define scs_printf casadi_printf
#define scs_free_ free
#define scs_malloc_ malloc
#define scs_calloc_ calloc
#else
#include <stdio.h>
#include <stdlib.h>
#define scs_printf printf
#define scs_free_ free
#define scs_malloc_ malloc
#define scs_calloc_ calloc
#endif

#define scs_free(x)  if ((x)!=NULL) {  scs_free_(x); x = SCS_NULL; }
#define scs_malloc(x) (((x) > 0) ? scs_malloc_(x) : SCS_NULL)
#define scs_calloc(x, y) scs_calloc_(x, y)

#ifdef DLONG
#ifdef _WIN64
    typedef __int64 scs_int;
    /* #define scs_int __int64 */
#else
    typedef long scs_int;
    /* #define scs_int long */
#endif
#else
    typedef long long int scs_int;
#endif

#ifndef FLOAT
    typedef double scs_float;
#ifndef NAN
#define NAN ((scs_float)0x7ff8000000000000)
#endif
#ifndef INFINITY
#define INFINITY NAN
#endif
#else
    typedef float scs_float;
#ifndef NAN
#define NAN ((float)0x7fc00000)
#endif
#ifndef INFINITY
#define INFINITY NAN
#endif
#endif

#define SCS_NULL 0

#ifndef MAX
    /**
     * Maximum value of two scalars
     */
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
    /**
     * Minimum value of two scalars
     */
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef ABS
    /**
     * Absolute value of a number
     */
#define ABS(x) (((x) < 0) ? -(x) : (x))
#endif

#ifndef SGN
    /**
     * Sign of a number
     */
#define SGN(x) (((x) >= 0) ? 1.0 : -1.0) 
#endif

#ifndef POWF
#ifdef FLOAT
#define POWF powf
#else
#define POWF pow
#endif
#endif

#ifndef SQRTF
#ifdef FLOAT
#define SQRTF sqrtf
#else
#define SQRTF sqrt
#endif
#endif


    /**
     * \brief Data of a conic optimization problem
     * 
     * Problem dimensions, matrix \f$A\f$, vectors \f$b\f$ and \f$c\f$ and
     * settings.
     */
    typedef struct scs_data ScsData;
    /**
     * \brief Solver settings
     */
    typedef struct scs_settings ScsSettings;
    /**
     * \brief Primal and dual solution.
     */
    typedef struct scs_solution ScsSolution;
    /**
     * \brief Solver statistics and information.
     */
    typedef struct scs_info ScsInfo;
    /**
     * \brief Scaling/normalization matrices.
     */
    typedef struct scs_scaling ScsScaling;
    /**
     * \brief SuperSCS Workspace structure.
     */
    typedef struct scs_work ScsWork;
    /**
     * \brief Cartesian product of cones.
     * 
     * \sa \ref page_cones "Cones documentation"
     */
    typedef struct scs_cone ScsCone;
    /**
     * A finite-memory cache where \f$(Y, U)\f$ are stored.
     */
    typedef struct scs_direction_cache ScsDirectionCache;

    /**
     * \brief Direction computation method (in SuperSCS)
     * 
     * See \ref page_directions "Documentation on directions"
     */
    typedef
    enum direction_enum {
        /**
         * Restarted Broyden method
         */
        restarted_broyden = 100,
        /**
         * Anderson's acceleration
         */
        anderson_acceleration = 150,
        /**
         * Using \f$d_k = - R_k\f$
         */
        fixed_point_residual = 200,
        /**
         * Full Broyden method
         */
        full_broyden = 300
    }
    ScsDirectionType;

#ifdef __cplusplus
}
#endif
#endif
