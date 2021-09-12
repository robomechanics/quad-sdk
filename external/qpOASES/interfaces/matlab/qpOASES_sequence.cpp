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
 *	\file interfaces/matlab/qpOASES_sequence.cpp
 *	\author Hans Joachim Ferreau, Christian Kirches, Andreas Potschka, Alexander Buchner
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Interface for Matlab(R) that enables to call qpOASES as a MEX function
 *  (variant for solving QP sequences).
 *
 */



#include <qpOASES.hpp>


USING_NAMESPACE_QPOASES


#include "qpOASES_matlab_utils.hpp"

/** initialise handle counter of QPInstance class */
int_t QPInstance::s_nexthandle = 1;

/** global pointer to QP objects */
static std::vector<QPInstance *> g_instances;

#include "qpOASES_matlab_utils.cpp"


/*
 *	Q P r o b l e m B _ i n i t
 */
int_t QProblemB_init(	int_t handle, 
						SymmetricMatrix* H, real_t* g,
						const real_t* const lb, const real_t* const ub,
						int_t nWSRin, real_t maxCpuTimeIn,
						const double* const x0, Options* options,
						int_t nOutputs, mxArray* plhs[],
						const double* const guessedBounds,
						const double* const _R
						)
{
	int_t nWSRout = nWSRin;
	real_t maxCpuTimeOut = (maxCpuTimeIn >= 0.0) ? maxCpuTimeIn : INFTY;

	/* 1) setup initial QP. */
	QProblemB* globalQPB = getQPInstance(handle)->qpb;

	if ( globalQPB == 0 )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): Invalid handle to QP instance!" );
		return -1;
	}

	globalQPB->setOptions( *options );
	
	/* 2) Solve initial QP. */
	returnValue returnvalue;
	int_t nV = globalQPB->getNV();
	
	/* 3) Fill the working set. */
	Bounds bounds(nV);
	if (guessedBounds != 0) {
		for (int_t i = 0; i < nV; i++) {
			if ( isEqual(guessedBounds[i],-1.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_LOWER);
			} else if ( isEqual(guessedBounds[i],1.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_UPPER);
			} else if ( isEqual(guessedBounds[i],0.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_INACTIVE);
			} else {
				char msg[MAX_STRING_LENGTH];
				snprintf(msg, MAX_STRING_LENGTH,
						"ERROR (qpOASES): Only {-1, 0, 1} allowed for status of bounds!");
				myMexErrMsgTxt(msg);
				return -1;
			}
		}
	}

	returnvalue = globalQPB->init(	H,g,lb,ub,
									nWSRout,&maxCpuTimeOut,
									x0,0,
									(guessedBounds != 0) ? &bounds : 0,
									_R
									);

	/* 3) Assign lhs arguments. */
	obtainOutputs(	0,globalQPB,returnvalue,nWSRout,maxCpuTimeOut,
					nOutputs,plhs,nV,0,handle );

	return 0;
}


/*
 *	S Q P r o b l e m _ i n i t
 */
int_t SQProblem_init(	int_t handle, 
						SymmetricMatrix* H, real_t* g, Matrix* A,
						const real_t* const lb, const real_t* const ub,
						const real_t* const lbA, const real_t* const ubA,
						int_t nWSRin, real_t maxCpuTimeIn,
						const double* const x0, Options* options,
						int_t nOutputs, mxArray* plhs[],
						const double* const guessedBounds, const double* const guessedConstraints,
						const double* const _R
						)
{
	int_t nWSRout = nWSRin;
	real_t maxCpuTimeOut = (maxCpuTimeIn >= 0.0) ? maxCpuTimeIn : INFTY;

	/* 1) setup initial QP. */
	SQProblem* globalSQP = getQPInstance(handle)->sqp;

	if ( globalSQP == 0 )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): Invalid handle to QP instance!" );
		return -1;
	}

	globalSQP->setOptions( *options );
	
	/* 2) Solve initial QP. */
	returnValue returnvalue;
	int_t nV = globalSQP->getNV();
	int_t nC = globalSQP->getNC();
	
	/* 3) Fill the working set. */
	Bounds bounds(nV);
	Constraints constraints(nC);
	if (guessedBounds != 0) {
		for (int_t i = 0; i < nV; i++) {
			if ( isEqual(guessedBounds[i],-1.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_LOWER);
			} else if ( isEqual(guessedBounds[i],1.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_UPPER);
			} else if ( isEqual(guessedBounds[i],0.0) == BT_TRUE ) {
				bounds.setupBound(i, ST_INACTIVE);
			} else {
				char msg[MAX_STRING_LENGTH];
				snprintf(msg, MAX_STRING_LENGTH,
						"ERROR (qpOASES): Only {-1, 0, 1} allowed for status of bounds!");
				myMexErrMsgTxt(msg);
				return -1;
			}
		}
	}

	if (guessedConstraints != 0) {
		for (int_t i = 0; i < nC; i++) {
			if ( isEqual(guessedConstraints[i],-1.0) == BT_TRUE ) {
				constraints.setupConstraint(i, ST_LOWER);
			} else if ( isEqual(guessedConstraints[i],1.0) == BT_TRUE ) {
				constraints.setupConstraint(i, ST_UPPER);
			} else if ( isEqual(guessedConstraints[i],0.0) == BT_TRUE ) {
				constraints.setupConstraint(i, ST_INACTIVE);
			} else {
				char msg[MAX_STRING_LENGTH];
				snprintf(msg, MAX_STRING_LENGTH,
						"ERROR (qpOASES): Only {-1, 0, 1} allowed for status of constraints!");
				myMexErrMsgTxt(msg);
				return -1;
			}
		}
	}
	
	returnvalue = globalSQP->init(	H,g,A,lb,ub,lbA,ubA,
									nWSRout,&maxCpuTimeOut,
									x0,0,
									(guessedBounds != 0) ? &bounds : 0, (guessedConstraints != 0) ? &constraints : 0,
									_R
									);

	/* 3) Assign lhs arguments. */
	obtainOutputs(	0,globalSQP,returnvalue,nWSRout,maxCpuTimeOut,
					nOutputs,plhs,nV,nC,handle );

	return 0;
}



/*
 *	Q P r o b l e m B _ h o t s t a r t
 */
int_t QProblemB_hotstart(	int_t handle,
							const real_t* const g,
							const real_t* const lb, const real_t* const ub,
							int_t nWSRin, real_t maxCpuTimeIn,
							Options* options,
							int_t nOutputs, mxArray* plhs[]
							)
{
	int_t nWSRout = nWSRin;
	real_t maxCpuTimeOut = (maxCpuTimeIn >= 0.0) ? maxCpuTimeIn : INFTY;

	QProblemB* globalQPB = getQPInstance(handle)->qpb;

	if ( globalQPB == 0 )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): QP needs to be initialised first!" );
		return -1;
	}

	int_t nV = globalQPB->getNV();

	/* 1) Solve QP with given options. */
	globalQPB->setOptions( *options );
	returnValue returnvalue = globalQPB->hotstart( g,lb,ub, nWSRout,&maxCpuTimeOut );

	/* 2) Assign lhs arguments. */
	obtainOutputs(	0,globalQPB,returnvalue,nWSRout,maxCpuTimeOut,
					nOutputs,plhs,nV,0 );

	return 0;
}


/*
 *	Q P r o b l e m _ h o t s t a r t
 */
int_t QProblem_hotstart(	int_t handle,
							const real_t* const g,
							const real_t* const lb, const real_t* const ub,
							const real_t* const lbA, const real_t* const ubA,
							int_t nWSRin, real_t maxCpuTimeIn,
							Options* options,
							int_t nOutputs, mxArray* plhs[]
							)
{
	int_t nWSRout = nWSRin;
	real_t maxCpuTimeOut = (maxCpuTimeIn >= 0.0) ? maxCpuTimeIn : INFTY;

	QProblem* globalSQP = getQPInstance(handle)->sqp;

	if ( globalSQP == 0 )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): QP needs to be initialised first!" );
		return -1;
	}

	int_t nV = globalSQP->getNV();
	int_t nC = globalSQP->getNC();

	/* 1) Solve QP with given options. */
	globalSQP->setOptions( *options );
	returnValue returnvalue = globalSQP->hotstart( g,lb,ub,lbA,ubA, nWSRout,&maxCpuTimeOut );

	/* 2) Assign lhs arguments. */
	obtainOutputs(	0,globalSQP,returnvalue,nWSRout,maxCpuTimeOut,
					nOutputs,plhs,nV,nC );

	return 0;
}


/*
 *	S Q P r o b l e m _ h o t s t a r t
 */
int_t SQProblem_hotstart(	int_t handle,
							SymmetricMatrix* H, real_t* g, Matrix* A,
							const real_t* const lb, const real_t* const ub, const real_t* const lbA, const real_t* const ubA,
							int_t nWSRin, real_t maxCpuTimeIn,
							Options* options,
							int_t nOutputs, mxArray* plhs[]
							)
{
	int_t nWSRout = nWSRin;
	real_t maxCpuTimeOut = (maxCpuTimeIn >= 0.0) ? maxCpuTimeIn : INFTY;

	SQProblem* globalSQP = getQPInstance(handle)->sqp;

	if ( globalSQP == 0 )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): QP needs to be initialised first!" );
		return -1;
	}

	int_t nV = globalSQP->getNV();
	int_t nC = globalSQP->getNC();

	/* 1) Solve QP. */
	globalSQP->setOptions( *options );
	returnValue returnvalue = globalSQP->hotstart( H,g,A,lb,ub,lbA,ubA, nWSRout,&maxCpuTimeOut );

	switch (returnvalue)
	{
		case SUCCESSFUL_RETURN:
		case RET_QP_UNBOUNDED:
		case RET_QP_INFEASIBLE:
			break;

		default:
			myMexErrMsgTxt( "ERROR (qpOASES): Hotstart failed." );
			return -1;
	}

	/* 2) Assign lhs arguments. */
	obtainOutputs(	0,globalSQP,returnvalue,nWSRout,maxCpuTimeOut,
					nOutputs,plhs,nV,nC );

	return 0;
}



/*
 *	m e x F u n c t i o n
 */
void mexFunction( int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[] )
{
	/* inputs */
	char typeString[2];

	real_t *g=0, *lb=0, *ub=0, *lbA=0, *ubA=0;
	HessianType hessianType = HST_UNKNOWN;
	double *x0=0, *R=0, *R_for=0;
	double *guessedBounds=0, *guessedConstraints=0;

	int_t H_idx=-1, g_idx=-1, A_idx=-1, lb_idx=-1, ub_idx=-1, lbA_idx=-1, ubA_idx=-1;
	int_t x0_idx=-1, auxInput_idx=-1;

	BooleanType isSimplyBoundedQp = BT_FALSE;
	#ifdef SOLVER_MA57
	BooleanType isSparse = BT_TRUE; /* This will be set to BT_FALSE later if a dense matrix is encountered. */
	#else
	BooleanType isSparse = BT_FALSE;
	#endif

	Options options;
	options.printLevel = PL_LOW;
	#ifdef __DEBUG__
	options.printLevel = PL_HIGH;
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	options.printLevel = PL_NONE;
	#endif

	/* dimensions */
	uint_t nV=0, nC=0, handle=0;
	int_t nWSRin;
	real_t maxCpuTimeIn = -1.0;
	QPInstance* globalQP = 0;

	/* I) CONSISTENCY CHECKS: */
	/* 1) Ensure that qpOASES is called with a feasible number of input arguments. */
	if ( ( nrhs < 5 ) || ( nrhs > 10 ) )
	{
		if ( nrhs != 2 )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );
			return;
		}
	}
	
	/* 2) Ensure that first input is a string ... */
	if ( mxIsChar( prhs[0] ) != 1 )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): First input argument must be a string!" );
		return;
	}

	mxGetString( prhs[0], typeString, 2 );

	/*    ... and if so, check if it is an allowed one. */
	if ( ( strcmp( typeString,"i" ) != 0 ) && ( strcmp( typeString,"I" ) != 0 ) &&
		 ( strcmp( typeString,"h" ) != 0 ) && ( strcmp( typeString,"H" ) != 0 ) &&
		 ( strcmp( typeString,"m" ) != 0 ) && ( strcmp( typeString,"M" ) != 0 ) &&
		 ( strcmp( typeString,"e" ) != 0 ) && ( strcmp( typeString,"E" ) != 0 ) &&
		 ( strcmp( typeString,"c" ) != 0 ) && ( strcmp( typeString,"C" ) != 0 ) )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): Undefined first input argument!\nType 'help qpOASES_sequence' for further information." );
		return;
	}


	/* II) SELECT RESPECTIVE QPOASES FUNCTION CALL: */
	/* 1) Init (without or with initial guess for primal solution). */
	if ( ( strcmp( typeString,"i" ) == 0 ) || ( strcmp( typeString,"I" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 7 ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		if ( ( nrhs < 5 ) || ( nrhs > 10 ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		g_idx = 2;

		if ( mxIsEmpty(prhs[1]) == 1 )
		{
			H_idx = -1;
			nV = (uint_t)mxGetM( prhs[ g_idx ] ); /* row number of Hessian matrix */
		}
		else
		{
			H_idx = 1;
			nV = (uint_t)mxGetM( prhs[ H_idx ] ); /* row number of Hessian matrix */
		}


		/* ensure that data is given in double precision */
		if ( ( ( H_idx >= 0 ) && ( mxIsDouble( prhs[ H_idx ] ) == 0 ) ) ||
		     ( mxIsDouble( prhs[ g_idx ] ) == 0 ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in double precision!" );
			return;
		}

		if ( ( H_idx >= 0 ) && ( ( mxGetN( prhs[ H_idx ] ) != nV ) || ( mxGetM( prhs[ H_idx ] ) != nV ) ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Hessian matrix dimension mismatch!" );
			return;
		}


		/* Check for 'Inf' and 'Nan' in Hessian */
		if (containsNaNorInf( prhs,H_idx, 0 ) == BT_TRUE)
			return;

		/* Check for 'Inf' and 'Nan' in gradient */
		if (containsNaNorInf(prhs,g_idx, 0 ) == BT_TRUE)
			return;

		/* determine whether is it a simply bounded QP */
		if ( nrhs <= 7 )
			isSimplyBoundedQp = BT_TRUE;
		else
			isSimplyBoundedQp = BT_FALSE;

		if ( isSimplyBoundedQp == BT_TRUE )
		{
			lb_idx = 3;
			ub_idx = 4;

			if (containsNaNorInf( prhs,lb_idx, 1 ) == BT_TRUE)
				return;

			if (containsNaNorInf( prhs,ub_idx, 1 ) == BT_TRUE)
				return;

			/* Check inputs dimensions and assign pointers to inputs. */
			nC = 0; /* row number of constraint matrix */


			if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,2 ) != SUCCESSFUL_RETURN )
				return;

			if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,3 ) != SUCCESSFUL_RETURN )
				return;

			if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,4 ) != SUCCESSFUL_RETURN )
				return;

			/* default value for nWSR */
			nWSRin = 5*nV;

			/* Check whether x0 and options are specified .*/
			if ( nrhs >= 6 )
			{
				if ((!mxIsEmpty(prhs[5])) && (mxIsStruct(prhs[5])))
					setupOptions( &options,prhs[5],nWSRin,maxCpuTimeIn );

				if ( ( nrhs >= 7 ) && ( !mxIsEmpty(prhs[6]) ) )
				{ 
					/* auxInput specified */
					if ( mxIsStruct(prhs[6]) )
					{
						auxInput_idx = 6;
						x0_idx = -1;
					}
					else
					{
						auxInput_idx = -1;
						x0_idx = 6;
					}
				}
				else
				{
					auxInput_idx = -1;
					x0_idx = -1;
				}
			}
		}
		else
		{
			A_idx = 3;

			/* ensure that data is given in double precision */
			if ( mxIsDouble( prhs[ A_idx ] ) == 0 )
			{
				myMexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in double precision!" );
				return;
			}
		
			/* Check inputs dimensions and assign pointers to inputs. */
			nC = (uint_t)mxGetM( prhs[ A_idx ] ); /* row number of constraint matrix */

			lb_idx = 4;
			ub_idx = 5;
			lbA_idx = 6;
			ubA_idx = 7;

			if (containsNaNorInf( prhs,A_idx, 0 ) == BT_TRUE)
				return;

			if (containsNaNorInf( prhs,lb_idx, 1 ) == BT_TRUE)
				return;

			if (containsNaNorInf( prhs,ub_idx, 1 ) == BT_TRUE)
				return;

			if (containsNaNorInf( prhs,lbA_idx, 1 ) == BT_TRUE)
				return;

			if (containsNaNorInf( prhs,ubA_idx, 1 ) == BT_TRUE)
				return;

			if ( ( mxGetN( prhs[ A_idx ] ) != 0 ) && ( mxGetN( prhs[ A_idx ] ) != nV ) )
			{
				myMexErrMsgTxt( "ERROR (qpOASES): Constraint matrix dimension mismatch!" );
				return;
			}
		
			if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,g_idx ) != SUCCESSFUL_RETURN )
				return;

			if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,lb_idx ) != SUCCESSFUL_RETURN )
				return;

			if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,ub_idx ) != SUCCESSFUL_RETURN )
				return;

			if ( smartDimensionCheck( &lbA,nC,1, BT_TRUE,prhs,lbA_idx ) != SUCCESSFUL_RETURN )
				return;
			
			if ( smartDimensionCheck( &ubA,nC,1, BT_TRUE,prhs,ubA_idx ) != SUCCESSFUL_RETURN )
				return;

			/* default value for nWSR */
			nWSRin = 5*(nV+nC);

			/* Check whether x0 and options are specified .*/
			if ( nrhs >= 9 )
			{
				if ((!mxIsEmpty(prhs[8])) && (mxIsStruct(prhs[8])))
					setupOptions( &options,prhs[8],nWSRin,maxCpuTimeIn );

				if ( ( nrhs >= 10 ) && ( !mxIsEmpty(prhs[9]) ) )
				{ 
					/* auxInput specified */
					if ( mxIsStruct(prhs[9]) )
					{
						auxInput_idx = 9;
						x0_idx = -1;
					}
					else
					{
						auxInput_idx = -1;
						x0_idx = 9;
					}
				}
				else
				{
					auxInput_idx = -1;
					x0_idx = -1;
				}
			}
		}


		/* check dimensions and copy auxInputs */
		if ( smartDimensionCheck( &x0,nV,1, BT_TRUE,prhs,x0_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( auxInput_idx >= 0 )
			setupAuxiliaryInputs( prhs[auxInput_idx],nV,nC, &hessianType,&x0,&guessedBounds,&guessedConstraints,&R_for );

		/* convert Cholesky factor to C storage format */
		if ( R_for != 0 )
		{
			R = new real_t[nV*nV];
			convertFortranToC( R_for, nV,nV, R );
		}

		/* check if QP is sparse */
		if ( H_idx >= 0 && !mxIsSparse( prhs[H_idx] ) )
			isSparse = BT_FALSE;
		if ( nC > 0 && A_idx >= 0 && !mxIsSparse( prhs[A_idx] ) )
			isSparse = BT_FALSE;
		
		/* allocate instance */
		handle = allocateQPInstance( nV,nC,hessianType, isSimplyBoundedQp, isSparse, &options );	
		globalQP = getQPInstance( handle );

		/* make a deep-copy of the user-specified Hessian matrix (possibly sparse) */
		if ( H_idx >= 0 )
			setupHessianMatrix(	prhs[H_idx],nV, &(globalQP->H),&(globalQP->Hir),&(globalQP->Hjc),&(globalQP->Hv) );
		
		/* make a deep-copy of the user-specified constraint matrix (possibly sparse) */
		if ( ( nC > 0 ) && ( A_idx >= 0 ) )
			setupConstraintMatrix( prhs[A_idx],nV,nC, &(globalQP->A),&(globalQP->Air),&(globalQP->Ajc),&(globalQP->Av) );

		/* Create output vectors and assign pointers to them. */
		allocateOutputs( nlhs,plhs, nV,nC,1,handle );

		/* Call qpOASES. */
		if ( isSimplyBoundedQp == BT_TRUE )
		{
			QProblemB_init(	handle,
							globalQP->H,g,
							lb,ub,
							nWSRin,maxCpuTimeIn,
							x0,&options,
							nlhs,plhs,
							guessedBounds,R
							);
		}
		else
		{
			SQProblem_init(	handle,
							globalQP->H,g,globalQP->A,
							lb,ub,lbA,ubA,
							nWSRin,maxCpuTimeIn,
							x0,&options,
							nlhs,plhs,
							guessedBounds,guessedConstraints,R
							);
		}

		if (R != 0) delete R;
		return;
	}

	/* 2) Hotstart. */
	if ( ( strcmp( typeString,"h" ) == 0 ) || ( strcmp( typeString,"H" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 6 ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		if ( ( nrhs < 5 ) || ( nrhs > 8 ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		/* determine whether is it a simply bounded QP */
		if ( nrhs < 7 )
			isSimplyBoundedQp = BT_TRUE;
		else
			isSimplyBoundedQp = BT_FALSE;


		if ( ( mxIsDouble( prhs[1] ) == false ) || ( mxIsScalar( prhs[1] ) == false ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Expecting a handle to QP object as second argument!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		/* get QP instance */
		handle = (uint_t)mxGetScalar( prhs[1] );
		globalQP = getQPInstance( handle );
		if ( globalQP == 0 )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid handle to QP instance!" );
			return;
		}

		nV = globalQP->getNV();

		g_idx = 2;
		lb_idx = 3;
		ub_idx = 4;

		if (containsNaNorInf( prhs,g_idx, 0 ) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,lb_idx, 1 ) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,ub_idx, 1 ) == BT_TRUE)
			return;


		/* Check inputs dimensions and assign pointers to inputs. */
		if ( isSimplyBoundedQp == BT_TRUE )
		{
			nC = 0;

			if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,g_idx ) != SUCCESSFUL_RETURN )
				return;

			if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,lb_idx ) != SUCCESSFUL_RETURN )
				return;

			if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,ub_idx ) != SUCCESSFUL_RETURN )
				return;

			/* default value for nWSR */
			nWSRin = 5*nV;

			/* Check whether options are specified .*/
			if ( nrhs == 6 )
				if ( ( !mxIsEmpty( prhs[5] ) ) && ( mxIsStruct( prhs[5] ) ) )
					setupOptions( &options,prhs[5],nWSRin,maxCpuTimeIn );
		}
		else
		{
			nC = globalQP->getNC( );

			lbA_idx = 5;
			ubA_idx = 6;

			if (containsNaNorInf( prhs,lbA_idx, 1 ) == BT_TRUE)
				return;

			if (containsNaNorInf( prhs,ubA_idx, 1 ) == BT_TRUE)
				return;

			if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,g_idx ) != SUCCESSFUL_RETURN )
				return;

			if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,lb_idx ) != SUCCESSFUL_RETURN )
				return;

			if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,ub_idx ) != SUCCESSFUL_RETURN )
				return;

			if ( smartDimensionCheck( &lbA,nC,1, BT_TRUE,prhs,lbA_idx ) != SUCCESSFUL_RETURN )
				return;

			if ( smartDimensionCheck( &ubA,nC,1, BT_TRUE,prhs,ubA_idx ) != SUCCESSFUL_RETURN )
				return;

			/* default value for nWSR */
			nWSRin = 5*(nV+nC);

			/* Check whether options are specified .*/
			if ( nrhs == 8 )
				if ( ( !mxIsEmpty( prhs[7] ) ) && ( mxIsStruct( prhs[7] ) ) )
					setupOptions( &options,prhs[7],nWSRin,maxCpuTimeIn );
		}

		/* Create output vectors and assign pointers to them. */
		allocateOutputs( nlhs,plhs, nV,nC );

		/* call qpOASES */
		if ( isSimplyBoundedQp == BT_TRUE )
		{
			QProblemB_hotstart(	handle, g,
								lb,ub,
								nWSRin,maxCpuTimeIn,
								&options,
								nlhs,plhs
								);
		}
		else
		{
			QProblem_hotstart(	handle, g,
								lb,ub,lbA,ubA,
								nWSRin,maxCpuTimeIn,
								&options,
								nlhs,plhs
								);
		}

		return;
	}

	/* 3) Modify matrices. */
	if ( ( strcmp( typeString,"m" ) == 0 ) || ( strcmp( typeString,"M" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 6 ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		if ( ( nrhs < 9 ) || ( nrhs > 10 ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		if ( ( mxIsDouble( prhs[1] ) == false ) || ( mxIsScalar( prhs[1] ) == false ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Expecting a handle to QP object as second argument!\nType 'help qpOASES_sequence' for further information." );
			return;
		}


		/* get QP instance */
		handle = (uint_t)mxGetScalar( prhs[1] );
		globalQP = getQPInstance( handle );
		if ( globalQP == 0 )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid handle to QP instance!" );
			return;
		}

		/* Check inputs dimensions and assign pointers to inputs. */
		g_idx = 3;
		
		if ( mxIsEmpty(prhs[2]) == 1 )
		{
			H_idx = -1;
			nV = (uint_t)mxGetM( prhs[ g_idx ] ); /* if Hessian is empty, row number of gradient vector */
		}
		else
		{
			H_idx = 2;
			nV = (uint_t)mxGetM( prhs[ H_idx ] ); /* row number of Hessian matrix */
		}
		
		A_idx = 4;
		nC = (uint_t)mxGetM( prhs[ A_idx ] ); /* row number of constraint matrix */
				
		lb_idx = 5;
		ub_idx = 6;
		lbA_idx = 7;
		ubA_idx = 8;


		/* ensure that data is given in double precision */
		if ( ( ( H_idx >= 0 ) && ( mxIsDouble( prhs[H_idx] ) == 0 ) ) ||
			 ( mxIsDouble( prhs[g_idx] ) == 0 ) ||
			 ( mxIsDouble( prhs[A_idx] ) == 0 ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in real_t precision!" );
			return;
		}

		/* check if supplied data contains 'NaN' or 'Inf' */
		if (containsNaNorInf(prhs,H_idx, 0) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,g_idx, 0 ) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,A_idx, 0 ) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,lb_idx, 1 ) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,ub_idx, 1 ) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,lbA_idx, 1 ) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,ubA_idx, 1 ) == BT_TRUE)
			return;

		/* Check that dimensions are consistent with existing QP instance */
		if (nV != (uint_t) globalQP->getNV () || nC != (uint_t) globalQP->getNC ())
		{
			myMexErrMsgTxt( "ERROR (qpOASES): QP dimensions must be constant during a sequence! Try creating a new QP instance instead." );
			return;
		}

		if ( ( H_idx >= 0 ) && ( ( mxGetN( prhs[ H_idx ] ) != nV ) || ( mxGetM( prhs[ H_idx ] ) != nV ) ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Hessian matrix dimension mismatch!" );
			return;
		}

		if ( ( mxGetN( prhs[ A_idx ] ) != 0 ) && ( mxGetN( prhs[ A_idx ] ) != nV ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Constraint matrix dimension mismatch!" );
			return;
		}

		if ( smartDimensionCheck( &g,nV,1, BT_FALSE,prhs,g_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,1, BT_TRUE,prhs,lb_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,1, BT_TRUE,prhs,ub_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lbA,nC,1, BT_TRUE,prhs,lbA_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,1, BT_TRUE,prhs,ubA_idx ) != SUCCESSFUL_RETURN )
			return;

		/* default value for nWSR */
		nWSRin = 5*(nV+nC);

		/* Check whether options are specified .*/
		if ( nrhs > 9 )
			if ( ( !mxIsEmpty( prhs[9] ) ) && ( mxIsStruct( prhs[9] ) ) )
				setupOptions( &options,prhs[9],nWSRin,maxCpuTimeIn );

		globalQP->deleteQPMatrices( );

		/* make a deep-copy of the user-specified Hessian matrix (possibly sparse) */
		if ( H_idx >= 0 )
			setupHessianMatrix(	prhs[H_idx],nV, &(globalQP->H),&(globalQP->Hir),&(globalQP->Hjc),&(globalQP->Hv) );

		/* make a deep-copy of the user-specified constraint matrix (possibly sparse) */
		if ( ( nC > 0 ) && ( A_idx >= 0 ) )
			setupConstraintMatrix( prhs[A_idx],nV,nC, &(globalQP->A),&(globalQP->Air),&(globalQP->Ajc),&(globalQP->Av) );

		/* Create output vectors and assign pointers to them. */
		allocateOutputs( nlhs,plhs, nV,nC );

		/* Call qpOASES */
		SQProblem_hotstart(	handle, globalQP->H,g,globalQP->A,
							lb,ub,lbA,ubA,
							nWSRin,maxCpuTimeIn,
							&options,
							nlhs,plhs
							);

		return;
	}

	/* 4) Solve current equality constrained QP. */
	if ( ( strcmp( typeString,"e" ) == 0 ) || ( strcmp( typeString,"E" ) == 0 ) )
	{
		/* consistency checks */
		if ( ( nlhs < 1 ) || ( nlhs > 4 ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		if ( ( nrhs < 7 ) || ( nrhs > 8 ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		if ( ( mxIsDouble( prhs[1] ) == false ) || ( mxIsScalar( prhs[1] ) == false ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Expecting a handle to QP object as second argument!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		/* get QP instance */
		handle = (uint_t)mxGetScalar( prhs[1] );
		globalQP = getQPInstance( handle );
		if ( globalQP == 0 )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid handle to QP instance!" );
			return;
		}

		/* Check inputs dimensions and assign pointers to inputs. */
		int_t nRHS = (int_t)mxGetN(prhs[2]);
		nV = globalQP->getNV( );
		nC = globalQP->getNC( );
		real_t *x_out, *y_out;

		g_idx = 2;
		lb_idx = 3;
		ub_idx = 4;
		lbA_idx = 5;
		ubA_idx = 6;

		/* check if supplied data contains 'NaN' or 'Inf' */
		if (containsNaNorInf(prhs,g_idx, 0) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,lb_idx, 1 ) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,ub_idx, 1 ) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,lbA_idx, 1 ) == BT_TRUE)
			return;

		if (containsNaNorInf( prhs,ubA_idx, 1 ) == BT_TRUE)
			return;

		if ( smartDimensionCheck( &g,nV,nRHS, BT_FALSE,prhs,g_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lb,nV,nRHS, BT_TRUE,prhs,lb_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ub,nV,nRHS, BT_TRUE,prhs,ub_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &lbA,nC,nRHS, BT_TRUE,prhs,lbA_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,nRHS, BT_TRUE,prhs,ubA_idx ) != SUCCESSFUL_RETURN )
			return;

		/* Check whether options are specified .*/
		if ( ( nrhs == 8 ) && ( !mxIsEmpty( prhs[7] ) ) && ( mxIsStruct( prhs[7] ) ) )
		{
			nWSRin = 5*(nV+nC);
			setupOptions( &options,prhs[7],nWSRin,maxCpuTimeIn );
			globalQP->sqp->setOptions( options );
		}

		/* Create output vectors and assign pointers to them. */
		plhs[0] = mxCreateDoubleMatrix( nV, nRHS, mxREAL );
		x_out = mxGetPr(plhs[0]);
		if (nlhs >= 2)
		{
			plhs[1] = mxCreateDoubleMatrix( nV+nC, nRHS, mxREAL );
			y_out = mxGetPr(plhs[1]);

			if (nlhs >= 3)
			{
				plhs[2] = mxCreateDoubleMatrix( nV, nRHS, mxREAL );
				real_t* workingSetB = mxGetPr(plhs[2]);
				globalQP->sqp->getWorkingSetBounds(workingSetB);

				if ( nlhs >= 4 )
				{
					plhs[3] = mxCreateDoubleMatrix( nC, nRHS, mxREAL );
					real_t* workingSetC = mxGetPr(plhs[3]);
					globalQP->sqp->getWorkingSetConstraints(workingSetC);
				}
			}
		}
		else
			y_out = new real_t[nV+nC];

		/* Solve equality constrained QP */
		returnValue returnvalue = globalQP->sqp->solveCurrentEQP( nRHS,g,lb,ub,lbA,ubA, x_out,y_out );

		if (nlhs < 2)
			delete[] y_out;

		if (returnvalue != SUCCESSFUL_RETURN)
		{
			char msg[MAX_STRING_LENGTH];
			snprintf(msg, MAX_STRING_LENGTH, "ERROR (qpOASES): Couldn't solve current EQP (code %d)!", returnvalue);
			myMexErrMsgTxt(msg);
			return;
		}

		return;
	}

	/* 5) Cleanup. */
	if ( ( strcmp( typeString,"c" ) == 0 ) || ( strcmp( typeString,"C" ) == 0 ) )
	{		
		/* consistency checks */
		if ( nlhs != 0 )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of output arguments!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		if ( nrhs != 2 )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		if ( ( mxIsDouble( prhs[1] ) == false ) || ( mxIsScalar( prhs[1] ) == false ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Expecting a handle to QP object as second argument!\nType 'help qpOASES_sequence' for further information." );
			return;
		}

		/* Cleanup SQProblem instance. */
		handle = (uint_t)mxGetScalar( prhs[1] );
		deleteQPInstance( handle );
		
		return;
	}

}

/*
 *	end of file
 */
