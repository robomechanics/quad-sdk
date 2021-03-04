/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file interfaces/c/qpOASES_wrapper.h
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2014-2017
 *
 *	Interface that enables to call qpOASES from plain C.
 *
 */


#ifndef QPOASES_WRAPPER_H
#define QPOASES_WRAPPER_H


#ifndef QPOASES_TYPES_HPP

	/**
	 * Defined integer type for calling BLAS/LAPACK. Should usually be
	 * "(unsigned) int", currently set to "(unsigned) long" for backwards
	 * compatibility. This will change in a future release.
	 */
	typedef long la_int_t;
	typedef unsigned long la_uint_t;

	/** Defines real_t for facilitating switching between double and float. */
	#ifdef __USE_SINGLE_PRECISION__
	typedef float real_t;
	#else
	typedef double real_t;
	#endif /* __USE_SINGLE_PRECISION__ */

	/** Defines int_t for facilitating switching between int and long int. */
	#ifdef __USE_LONG_INTEGERS__
	typedef long int_t;
	typedef unsigned long uint_t;
	#else
	typedef int int_t;
	typedef unsigned int uint_t;
	#endif /* __USE_LONG_INTEGERS__ */

	/** Defines FORTRAN integer type. Might be platform dependent! */
	#ifdef __USE_LONG_FINTS__
	typedef long fint_t;
	#else
	typedef int fint_t;
	#endif /* __USE_LONG_FINTS__ */

	/**
	 * Integer type for sparse matrix row/column entries. Make this "int"
	 * for 32 bit entries, and "long" for 64-bit entries on x86_64 platform.
	 *
	 * Most sparse codes still assume 32-bit entries here (HSL, BQPD, ...)
	 */
	typedef int_t sparse_int_t;

	/* dummy definitions, not used when calling from C */
	#define QProblemBClass int_t
	#define OptionsClass int_t
	#define returnValue int_t

	/* HessianType */
	#define HST_ZERO             0
	#define HST_IDENTITY         1
	#define HST_POSDEF           2
	#define HST_POSDEF_NULLSPACE 3
	#define HST_SEMIDEF          4
	#define HST_INDEF            5
	#define HST_UNKNOWN	         6

	/* SubjectToStatus */
	#define ST_LOWER            -1
	#define ST_INACTIVE          0
	#define ST_UPPER             1
	#define ST_INFEASIBLE_LOWER  2
	#define ST_INFEASIBLE_UPPER  3
	#define ST_UNDEFINED         4

	/* PrintLevel */
	#define PL_DEBUG_ITER       -2
	#define PL_TABULAR          -1
	#define PL_NONE              0
	#define PL_LOW               1
	#define PL_MEDIUM            2
	#define PL_HIGH              3

#else 

	#define QProblemBClass QProblemB
	#define OptionsClass REFER_NAMESPACE_QPOASES Options 

	/* only declare when compiling C++ library */
	static QProblem*  globalQProblemObject  = 0;
	static QProblemB* globalQProblemBObject = 0;
	static SQProblem* globalSQProblemObject = 0;
	static Options globalOptionsObject;

#endif /* QPOASES_TYPES_HPP */



/**
 *	\brief Manages all user-specified options for solving QPs.
 *
 *	This struct manages all user-specified options used for solving
 *	quadratic programs.
 *
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2014-2017
 */
typedef struct
{
	int_t printLevel;						/**< Print level. */

	int_t enableRamping;					/**< Specifies whether ramping shall be enabled or not. */
	int_t enableFarBounds;					/**< Specifies whether far bounds shall be used or not. */
	int_t enableFlippingBounds;				/**< Specifies whether flipping bounds shall be used or not. */
	int_t enableRegularisation;				/**< Specifies whether Hessian matrix shall be regularised in case semi-definiteness is detected. */
	int_t enableFullLITests;				/**< Specifies whether condition-hardened LI test shall be used or not. */
	int_t enableNZCTests;					/**< Specifies whether nonzero curvature tests shall be used. */
	int_t enableDriftCorrection;			/**< Specifies the frequency of drift corrections (0 = off). */
	int_t enableCholeskyRefactorisation;	/**< Specifies the frequency of full refactorisation of proj. Hessian (otherwise updates). */
	int_t enableEqualities;					/**< Specifies whether equalities shall be always treated as active constraints. */

	real_t terminationTolerance;			/**< Termination tolerance. */
	real_t boundTolerance;					/**< Lower/upper (constraints') bound tolerance (an inequality constraint whose lower and 
												 upper bounds differ by less is regarded to be an equality constraint). */
	real_t boundRelaxation;					/**< Offset for relaxing (constraints') bounds at beginning of an initial homotopy. It is also as initial value for far bounds. */
	real_t epsNum;							/**< Numerator tolerance for ratio tests. */
	real_t epsDen;							/**< Denominator tolerance for ratio tests. */
	real_t maxPrimalJump;					/**< Maximum allowed jump in primal variables in nonzero curvature tests. */
	real_t maxDualJump;						/**< Maximum allowed jump in dual variables in linear independence tests. */

	real_t initialRamping;					/**< Start value for Ramping Strategy. */
	real_t finalRamping;					/**< Final value for Ramping Strategy. */
	real_t initialFarBounds;				/**< Initial size of Far Bounds. */
	real_t growFarBounds;					/**< Factor to grow Far Bounds. */
	int_t initialStatusBounds;				/**< Initial status of bounds at first iteration. */
	real_t epsFlipping;						/**< Tolerance of squared Cholesky diagonal factor which triggers flipping bound. */
	int_t numRegularisationSteps;			/**< Maximum number of successive regularisation steps. */
	real_t epsRegularisation;				/**< Scaling factor of identity matrix used for Hessian regularisation. */
	int_t numRefinementSteps;				/**< Maximum number of iterative refinement steps. */
	real_t epsIterRef;						/**< Early termination tolerance for iterative refinement. */
	real_t epsLITests;						/**< Tolerance for linear independence tests. */
	real_t epsNZCTests;						/**< Tolerance for nonzero curvature tests. */

	real_t rcondSMin;						/**< Minimum reciprocal condition number of S before refactorization is triggered */
	int_t enableInertiaCorrection;			/**< Specifies whether the working set should be repaired when negative curvature is discovered during hotstart. */

	int_t enableDropInfeasibles;			/**< ... */
	int_t dropBoundPriority;				/**< ... */
	int_t dropEqConPriority;				/**< ... */
	int_t dropIneqConPriority;				/**< ... */
	int_t printResiduals;					/**< If true, it will print the internal qpOASES residuals and other information per iteration */

} qpOASES_Options;


int_t qpOASES_Options_init(	qpOASES_Options* const options,
							int_t mode
							);

int_t qpOASES_Options_copy(	const qpOASES_Options* const from,
							OptionsClass* const to
							);


int_t qpOASES_obtainOutputs(	const QProblemBClass* const globalQpObject,
								returnValue returnvalue,
								real_t* const x,
								real_t* const y,
								real_t* const obj,
								int_t* const status
								);


int_t QProblem_setup(	int_t nV,
						int_t nC,
						int_t hessianType
						);

int_t QProblem_init(	const real_t* const H,
						const real_t* const g,
						const real_t* const A,
						const real_t* const lb,
						const real_t* const ub,
						const real_t* const lbA,
						const real_t* const ubA,
						int_t* const nWSR,
						real_t* const cputime,
						const qpOASES_Options* const options,
						real_t* const x,
						real_t* const y,
						real_t* const obj,
						int_t* const status
						);

int_t QProblem_hotstart(	const real_t* const g,
							const real_t* const lb,
							const real_t* const ub,
							const real_t* const lbA,
							const real_t* const ubA,
							int_t* const nWSR,
							real_t* const cputime,
							real_t* const x,
							real_t* const y,
							real_t* const obj,
							int_t* const status
							);

int_t QProblem_cleanup( );



int_t QProblemB_setup(	int_t nV,
						int_t hessianType
						);

int_t QProblemB_init(	const real_t* const H,
						const real_t* const g,
						const real_t* const lb,
						const real_t* const ub,
						int_t* const nWSR,
						real_t* const cputime,
						const qpOASES_Options* const options,
						real_t* const x,
						real_t* const y,
						real_t* const obj,
						int_t* const status
						);

int_t QProblemB_hotstart(	const real_t* const g,
							const real_t* const lb,
							const real_t* const ub,
							int_t* const nWSR,
							real_t* const cputime,
							real_t* const x,
							real_t* const y,
							real_t* const obj,
							int_t* const status
							);

int_t QProblemB_cleanup( );



int_t SQProblem_setup(	int_t nV,
						int_t nC,
						int_t hessianType
						);

int_t SQProblem_init(	const real_t* const H,
						const real_t* const g,
						const real_t* const A,
						const real_t* const lb,
						const real_t* const ub,
						const real_t* const lbA,
						const real_t* const ubA,
						int_t* const nWSR,
						real_t* const cputime,
						const qpOASES_Options* const options,
						real_t* const x,
						real_t* const y,
						real_t* const obj,
						int_t* const status
						);

int_t SQProblem_hotstart(	const real_t* const H,
							const real_t* const g,
							const real_t* const A,
							const real_t* const lb,
							const real_t* const ub,
							const real_t* const lbA,
							const real_t* const ubA,
							int_t* const nWSR,
							real_t* const cputime,
							real_t* const x,
							real_t* const y,
							real_t* const obj,
							int_t* const status
							);

int_t SQProblem_cleanup( );


#endif /* QPOASES_WRAPPER_H */


/*
 *	end of file
 */
