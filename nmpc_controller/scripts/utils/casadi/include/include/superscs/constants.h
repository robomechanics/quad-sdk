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
#ifndef SCS_CONSTANTS_H_GUARD
#define SCS_CONSTANTS_H_GUARD

#include "glbopts.h"


#ifdef __cplusplus
extern "C" {
#endif

    /* SCS VERSION NUMBER ----------------------------------------------    */
    /* string literals automatically null-terminated */
    /**
     * Version of SCS
     */
#define SCS_VERSION                                                            \
    ("1.3.3") 


    /* SCS returns one of the following integers:                           */
    
    /**
     * Potentially infeasible (inaccurate solution)
     */
#define SCS_INFEASIBLE_INACCURATE (-7)
    /**
     * Potentially unbounded (inaccurate solution)
     */
#define SCS_UNBOUNDED_INACCURATE (-6)
    /**
     * Solver interrupted by \c SIGINT.
     */
#define SCS_SIGINT (-5)
    /**
     * SCS solver failed
     */
#define SCS_FAILED (-4)
    /**
     * indeterminate solution
     */
#define SCS_INDETERMINATE (-3)
    /**
     *  primal infeasible, dual unbounded   
     */
#define SCS_INFEASIBLE (-2) 
    /**
     *  primal unbounded, dual infeasible   
     */
#define SCS_UNBOUNDED (-1)  
    /**
     *  This status code never returned, used as placeholder 
     */
#define SCS_UNFINISHED (0)  
    /**
     * Problem successfully solved within the prescribed tolerance.
     */
#define SCS_SOLVED (1)
    /**
     * Problem seems to be feasible; suboptimal solution found.
     */
#define SCS_SOLVED_INACCURATE (2)

    /* DEFAULT SOLVER PARAMETERS AND SETTINGS --------------------------    */
    
    /**
     * Default maximum allowed runtime for (Super)SCS.
     */
#define SCS_MAX_TIME_MILLISECONDS (300000.)
    /**
     * Default maximum number of iterations.
     */
#define SCS_MAX_ITERS_DEFAULT (10000)
#define SCS_EPS_DEFAULT (1E-3)
#define SCS_ALPHA_DEFAULT (1.5)
#define SCS_RHO_X_DEFAULT (0.001)
#define SCS_SCALE_DEFAULT (1.0)
#define SCS_CG_RATE_DEFAULT (2.0)
#define SCS_VERBOSE_DEFAULT (1)
#define SCS_NORMALIZE_DEFAULT (1)
#define SCS_DO_RECORD_PROGRESS_DEFAULT (0)
#define SCS_WARM_START_DEFAULT (0)

    /* Parameters for Superscs*/
#define SCS_DO_SUPERSCS_DEFAULT (1)
#define SCS_K0_DEFAULT (0)
#define SCS_K1_DEFAULT (1)
#define SCS_K2_DEFAULT (1)    
#define SCS_NOMINAL_DEFAULT (1)
#define SCS_C_BL_DEFAULT (0.999)
#define SCS_C1_DEFAULT (1.0-1E-4)
#define SCS_SSE_DEFAULT (1.0-1E-3) 

    /* Line-search defaults */
#define SCS_LS_DEFAULT  (10)
#define SCS_BETA_DEFAULT (0.5)
#define SCS_SIGMA_DEFAULT (1E-2)

    /* Direction Defaults */
#define SCS_DIRECTION_DEFAULT (anderson_acceleration)
#define SCS_TRULE_DEFAULT (3)
#define SCS_THETABAR_DEFAULT (1E-1)
#define SCS_ALPHAC_DEFAULT (1E-2) 
#define SCS_MEMORY_DEFAULT (5)
#define SCS_SC_INIT_DEFAULT (0)
#define SCS_BROYDEN_ISCS_SCALE_DEFAULT (1)
#define SCS_OVERRIDE_STREAMS_DEFAULT (0)
    
    /* Other */
#define SCS_OUT_STREAM_DEFAULT (stdout)
    /**
     * Default value for \ref ScsSettings#previous_max_iters "previous number of maximum iterations".
     */
#define SCS_PMAXITER_DEFAULT (-1)
#ifdef __cplusplus
}
#endif
#endif
