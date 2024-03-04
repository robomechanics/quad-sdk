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
#ifndef SCS_CONES_H_GUARD
#define SCS_CONES_H_GUARD

#ifdef __cplusplus
extern "C" {
#endif

#include "glbopts.h"
#include "scs_blas.h"

    /* NB: rows of data matrix A must be specified in this exact order */

    /** 
     * \brief Cone structure 
     *
     * This structure represents a Cartesian product of cones as explained in 
     * detail in \ref page_cones "this documentation page".
     * 
     * \sa \ref page_cones "Cones documentation"
     */
    struct scs_cone {
        /**
         * \brief Number of linear equality constraints \f$(n_{\mathrm{f}})\f$
         * 
         * The corresponding cone is the zero-cone 
         * \f$\mathcal{K}^{f}_{n_f} = \{0_{n_f}\}\f$
         */
        scs_int f;
        /** 
         * \brief Dimension of LP cone \f$(n_{\mathrm{l}})\f$
         * 
         * This is used to specify element-wise inequalities.
         *
         * The corresponding cone is the positive orthant
         * \f$\mathcal{K}^{l}_{n_l} = \{x\in\mathbb{R}^{n_l}: x_i \geq 0, 
         * \forall i\}\f$
         */
        scs_int l;
        /** 
         * \brief Array of SOC constraints 
         * \f$(n_{\mathrm{q},1},\ldots, n_{\mathrm{q},N_{\mathrm{q}}})\f$.
         * 
         * This is the Cartesian product of \f$N_{so}\f$ cones with 
         * dimensions \f$n_{so_1},\ldots, n_{so,N_{so}}\f$.
         * 
         * This array contains the dimensions \f$(n_{so_1},\ldots, n_{so,N_{so}})\f$.
         * 
         * The length of this array is specified in #qsize.
         * 
         * \sa \ref #qsize "number of second-order cones"
         * 
         */
        scs_int *RESTRICT q;
        /** 
         * \brief Length of SOC array, i.e., number of second-order cones 
         * \f$(N_{\mathrm{q}})\f$
         * 
         * \sa \ref #q "array of second-order cones"
         */
        scs_int qsize; 
        /** 
         * \brief array of PSD constraints \f$(k_1,\ldots, k_{N_{\mathrm{s}}})\f$
         * 
         * Array of dimensions of PSD constraints.
         * 
         * \sa \ref #ssize "number of PSD cones"
         */
        scs_int *RESTRICT s; 
        /**
         * \brief length of PSD array \f$(N_{\mathrm{s}})\f$
         * 
         * \sa \ref #s "array of positive semidefinite cones"
         */
        scs_int ssize; 
        /**
         * \brief Number of primal exponential cone triples \f$(n_{\mathrm{ep}})\f$
         * 
         * \sa \ref #ed "dual exponential cone"
         */
        scs_int ep; 
        /**
         * \brief number of dual exponential cone triples \f$(n_{\mathrm{de}})\f$
         * 
         * \sa \ref #ep "primal exponential cone"
         */
        scs_int ed; 
        /** 
         * \brief Array of power cone params \f$(\alpha_1,\ldots,\alpha_{N_{\mathrm{p}}})\f$.
         * 
         * \note Cone parameters must be in \f$[-1, 1]\f$. 
         * \note Negative values are interpreted as specifying the dual cone 
         */
        scs_float * p; 
        /** 
         * \brief Number of (primal and dual) power cone tuples \f$(N_{\mathrm{p}})\f$.
         *  
         */
        scs_int psize; 
    };

    /** private data to help cone projection step */

    /** \brief Workspace for cones */
    typedef struct scs_cone_work {
#ifdef LAPACK_LIB_FOUND
        /* workspace for eigenvector decompositions: */
        scs_float * RESTRICT Xs;
        scs_float * RESTRICT Z;
        scs_float * RESTRICT e;
        scs_float * RESTRICT work;
        blasint * RESTRICT iwork, lwork, liwork;
#endif
        scs_float total_cone_time;
    } ScsConeWork;

    /**
     * boundaries will contain array of indices of rows of A corresponding to
     * cone boundaries, boundaries[0] is starting index for cones of size larger
     * than 1
     * 
     * @return returns length of boundaries array, boundaries malloc-ed here so 
     * should be freed
     */
    scs_int scs_get_cone_boundaries(
            const ScsCone * RESTRICT k,
            scs_int ** RESTRICT boundaries);

    ScsConeWork *scs_init_conework(const ScsCone * RESTRICT k);

    char *scs_get_cone_header(const ScsCone *k);

    scs_int scs_validate_cones(
            const ScsData * RESTRICT d,
            const ScsCone * RESTRICT k);

    /** 
     * pass in iter to control how accurate the cone projection
     * with iteration, set iter < 0 for exact projection, warm_start contains guess
     * of solution, can be SCS_NULL
     */
    scs_int scs_project_dual_cone(
            scs_float * RESTRICT x,
            const ScsCone * RESTRICT k,
            ScsConeWork * RESTRICT c,
            const scs_float * RESTRICT warm_start,
            scs_int iter);

    void scs_finish_cone(
            ScsConeWork * RESTRICT coneWork);

    char *scs_get_cone_summary(
            const ScsInfo * RESTRICT info,
            ScsConeWork * RESTRICT c);

#ifdef __cplusplus
}
#endif
#endif
