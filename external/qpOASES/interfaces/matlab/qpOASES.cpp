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
 *	\file interfaces/matlab/qpOASES.cpp
 *	\author Hans Joachim Ferreau, Alexander Buchner (thanks to Aude Perrin)
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Interface for Matlab(R) that enables to call qpOASES as a MEX function.
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
 *	Q P r o b l e m _ q p O A S E S
 */
int_t QProblem_qpOASES(	int_t nV, int_t nC, HessianType hessianType, int_t nP,
						SymmetricMatrix* H, double* g, Matrix* A,
						double* lb, double* ub,
						double* lbA, double* ubA,
						int_t nWSRin, real_t maxCpuTimeIn,
						const double* const x0, Options* options,
						int_t nOutputs, mxArray* plhs[],
						const double* const guessedBounds, const double* const guessedConstraints,
						const double* const _R,
						BooleanType isSparse
						)
{
	int_t nWSRout;
	real_t maxCpuTimeOut;
	
	/* 1) Setup initial QP. */
	QProblem *QP;
	if ( isSparse == BT_TRUE )
    {
        #ifdef SOLVER_MA57
		QP = new SQProblemSchur ( nV,nC,hessianType );
        #else
        QP = new QProblem ( nV,nC,hessianType );
        #endif
    }
	else
		QP = new QProblem ( nV,nC,hessianType );
	QP->setOptions( *options );

	/* 2) Solve initial QP. */
	returnValue returnvalue;

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

	nWSRout = nWSRin;
	maxCpuTimeOut = (maxCpuTimeIn >= 0.0) ? maxCpuTimeIn : INFTY;

	returnvalue = QP->init(	H,g,A,lb,ub,lbA,ubA,
							nWSRout,&maxCpuTimeOut,
							x0,0,
							(guessedBounds != 0) ? &bounds : 0, (guessedConstraints != 0) ? &constraints : 0,
							_R
							);

	/* 3) Solve remaining QPs and assign lhs arguments. */
	/*    Set up pointers to the current QP vectors */
	real_t* g_current   = g;
	real_t* lb_current  = lb;
	real_t* ub_current  = ub;
	real_t* lbA_current = lbA;
	real_t* ubA_current = ubA;

	/* Loop through QP sequence. */
	for ( int_t k=0; k<nP; ++k )
	{
		if ( k > 0 )
		{
			/* update pointers to the current QP vectors */
			g_current = &(g[k*nV]);
			if ( lb != 0 )
				lb_current = &(lb[k*nV]);
			if ( ub != 0 )
				ub_current = &(ub[k*nV]);
			if ( lbA != 0 )
				lbA_current = &(lbA[k*nC]);
			if ( ubA != 0 )
				ubA_current = &(ubA[k*nC]);

			nWSRout = nWSRin;
			maxCpuTimeOut = (maxCpuTimeIn >= 0.0) ? maxCpuTimeIn : INFTY;
			returnvalue = QP->hotstart( g_current,lb_current,ub_current,lbA_current,ubA_current, nWSRout,&maxCpuTimeOut );
		}

		/* write results into output vectors */
		obtainOutputs(	k,QP,returnvalue,nWSRout,maxCpuTimeOut,
						nOutputs,plhs,nV,nC );
	}

	//QP->writeQpDataIntoMatFile( "qpDataMat0.mat" );

	delete QP;
	return 0;
}



/*
 *	Q P r o b l e m B _ q p O A S E S
 */
int_t QProblemB_qpOASES(	int_t nV, HessianType hessianType, int_t nP,
							SymmetricMatrix *H, double* g,
							double* lb, double* ub,
							int_t nWSRin, real_t maxCpuTimeIn,
							const double* const x0, Options* options,
							int_t nOutputs, mxArray* plhs[],
							const double* const guessedBounds,
							const double* const _R
							)
{
	int_t nWSRout;
	real_t maxCpuTimeOut;

	/* 1) Setup initial QP. */
	QProblemB QP( nV,hessianType );
	QP.setOptions( *options );

	/* 2) Solve initial QP. */
	returnValue returnvalue;

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

	nWSRout = nWSRin;
	maxCpuTimeOut = (maxCpuTimeIn >= 0.0) ? maxCpuTimeIn : INFTY;
	
	returnvalue = QP.init(	H,g,lb,ub,
							nWSRout,&maxCpuTimeOut,
							x0,0,
							(guessedBounds != 0) ? &bounds : 0,
							_R
							);

	/* 3) Solve remaining QPs and assign lhs arguments. */
	/*    Set up pointers to the current QP vectors */
	real_t* g_current  = g;
	real_t* lb_current = lb;
	real_t* ub_current = ub;

	/* Loop through QP sequence. */
	for ( int_t k=0; k<nP; ++k )
	{
		if ( k > 0 )
		{
			/* update pointers to the current QP vectors */
			g_current = &(g[k*nV]);
			if ( lb != 0 )
				lb_current = &(lb[k*nV]);
			if ( ub != 0 )
				ub_current = &(ub[k*nV]);

            nWSRout = nWSRin;
			maxCpuTimeOut = (maxCpuTimeIn >= 0.0) ? maxCpuTimeIn : INFTY;
			returnvalue = QP.hotstart( g_current,lb_current,ub_current, nWSRout,&maxCpuTimeOut );
		}

		/* write results into output vectors */
		obtainOutputs(	k,&QP,returnvalue,nWSRout,maxCpuTimeOut,
						nOutputs,plhs,nV );
	}

	return 0;
}



/*
 *	m e x F u n c t i o n
 */
void mexFunction( int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[] )
{
	/* inputs */
	SymmetricMatrix *H=0;
	Matrix *A=0;

	real_t *g=0, *lb=0, *ub=0, *lbA=0, *ubA=0;
	HessianType hessianType = HST_UNKNOWN;
	double *x0=0, *R=0, *R_for=0;
	double *guessedBounds=0, *guessedConstraints=0;

	int H_idx=-1, g_idx=-1, A_idx=-1, lb_idx=-1, ub_idx=-1, lbA_idx=-1, ubA_idx=-1;
	int options_idx=-1, x0_idx=-1, auxInput_idx=-1;

    /* Setup default options */
	Options options;
	options.printLevel = PL_LOW;
	#ifdef __DEBUG__
	options.printLevel = PL_HIGH;
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	options.printLevel = PL_NONE;
	#endif

	/* dimensions */
	uint_t nV=0, nC=0, nP=0;
	BooleanType isSimplyBoundedQp = BT_FALSE;
	#ifdef SOLVER_MA57
	BooleanType isSparse = BT_TRUE; // This will be set to BT_FALSE later if a dense matrix is encountered.
	#else
	BooleanType isSparse = BT_FALSE;
	#endif

	/* sparse matrix indices and values */
	sparse_int_t *Hir=0, *Hjc=0, *Air=0, *Ajc=0;
	real_t *Hv=0, *Av=0;

	/* I) CONSISTENCY CHECKS: */
	/* 1a) Ensure that qpOASES is called with a feasible number of input arguments. */
	if ( ( nrhs < 4 ) || ( nrhs > 9 ) )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): Invalid number of input arguments!\nType 'help qpOASES' for further information." );
		return;
	}
    
	/* 2) Check for proper number of output arguments. */
	if ( nlhs > 6 )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): At most six output arguments are allowed: \n    [x,fval,exitflag,iter,lambda,auxOutput]!" );
		return;
	}
	if ( nlhs < 1 )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): At least one output argument is required: [x,...]!" );
		return;
	}


	/* II) PREPARE RESPECTIVE QPOASES FUNCTION CALL: */
	/*     Choose between QProblem and QProblemB object and assign the corresponding
	 *     indices of the input pointer array in to order to access QP data correctly. */
	g_idx = 1;

	if ( mxIsEmpty(prhs[0]) == 1 )
	{
		H_idx = -1;
		nV = (int_t)mxGetM( prhs[ g_idx ] ); /* if Hessian is empty, row number of gradient vector */
	}
	else
	{
		H_idx = 0;
		nV = (int_t)mxGetM( prhs[ H_idx ] ); /* row number of Hessian matrix */
	}
	
	nP = (int_t)mxGetN( prhs[ g_idx ] ); /* number of columns of the gradient matrix (vectors series have to be stored columnwise!) */

	if ( nrhs <= 6 )
        isSimplyBoundedQp = BT_TRUE;
	else
		isSimplyBoundedQp = BT_FALSE;


	/* 0) Check whether options are specified .*/
	if ( isSimplyBoundedQp == BT_TRUE )
	{
		if ( ( nrhs >= 5 ) && ( !mxIsEmpty(prhs[4]) ) && ( mxIsStruct(prhs[4]) ) )
			options_idx = 4;
	}
	else
	{
		/* Consistency check */
		if ( ( !mxIsEmpty(prhs[4]) ) && ( mxIsStruct(prhs[4]) ) )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Fifth input argument must not be a struct when solving QP with general constraints!\nType 'help qpOASES' for further information." );
			return;
		}

		if ( ( nrhs >= 8 ) && ( !mxIsEmpty(prhs[7]) ) && ( mxIsStruct(prhs[7]) ) )
			options_idx = 7;
	}

	// Is the third argument constraint Matrix A?
	int_t numberOfColumns = (int_t)mxGetN(prhs[2]);

	/* 1) Simply bounded QP. */
	if ( ( isSimplyBoundedQp == BT_TRUE ) ||
		 ( ( numberOfColumns == 1 ) && ( nV != 1 ) ) )
	{
		lb_idx   = 2;
		ub_idx   = 3;

		if ( ( nrhs >= 6 ) && ( !mxIsEmpty(prhs[5]) ) )
		{ 
			/* auxInput specified */
			if ( mxIsStruct(prhs[5]) )
			{
				auxInput_idx = 5;
				x0_idx = -1;
			}
			else
			{
				auxInput_idx = -1;
				x0_idx = 5;
			}
		}
		else
		{
			auxInput_idx = -1;
			x0_idx = -1;
		}
	}
	else
	{
		A_idx = 2;

		/* If constraint matrix is empty, use a QProblemB object! */
		if ( mxIsEmpty( prhs[ A_idx ] ) )
		{
			lb_idx   = 3;
			ub_idx   = 4;

			nC = 0;
		}
		else
		{
			lb_idx   = 3;
			ub_idx   = 4;
			lbA_idx  = 5;
			ubA_idx  = 6;

			nC = (int_t)mxGetM( prhs[ A_idx ] ); /* row number of constraint matrix */
		}

		if ( ( nrhs >= 9 ) && ( !mxIsEmpty(prhs[8]) ) )
		{ 
			/* auxInput specified */
			if ( mxIsStruct(prhs[8]) )
			{
				auxInput_idx = 8;
				x0_idx = -1;
			}
			else
			{
				auxInput_idx = -1;
				x0_idx = 8;
			}
		}
		else
		{
			auxInput_idx = -1;
			x0_idx = -1;
		}
	}


	/* ensure that data is given in real_t precision */
	if ( ( ( H_idx >= 0 ) && ( mxIsDouble( prhs[ H_idx ] ) == 0 ) ) ||
		 ( mxIsDouble( prhs[ g_idx ] ) == 0 ) )
	{
		myMexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in double precision!" );
		return;
	}

	/* check if supplied data contains 'NaN' or 'Inf' */
	if (containsNaNorInf( prhs,H_idx, 0 ) == BT_TRUE)
		return;

	if (containsNaNorInf( prhs,g_idx, 0 ) == BT_TRUE)
		return;

	if (containsNaNorInf( prhs,lb_idx, 1 ) == BT_TRUE)
		return;

	if (containsNaNorInf( prhs,ub_idx, 1 ) == BT_TRUE)
		return;

	/* Check inputs dimensions and assign pointers to inputs. */
	if ( ( H_idx >= 0 ) && ( ( mxGetN( prhs[ H_idx ] ) != nV ) || ( mxGetM( prhs[ H_idx ] ) != nV ) ) )
	{
		char msg[MAX_STRING_LENGTH]; 
		snprintf(msg, MAX_STRING_LENGTH, "ERROR (qpOASES): Hessian matrix dimension mismatch (%ld != %d)!", 
				(long int)mxGetN(prhs[H_idx]), (int)nV);
		myMexErrMsgTxt(msg);
		return;
	}

	if ( nC > 0 )
	{
		/* ensure that data is given in real_t precision */
		if ( mxIsDouble( prhs[ A_idx ] ) == 0 )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): All data has to be provided in real_t precision!" );
			return;
		}

		/* Check inputs dimensions and assign pointers to inputs. */
		if ( mxGetN( prhs[ A_idx ] ) != nV )
		{
			char msg[MAX_STRING_LENGTH]; 
			snprintf(msg, MAX_STRING_LENGTH, "ERROR (qpOASES): Constraint matrix input dimension mismatch (%ld != %d)!", 
					(long int)mxGetN(prhs[A_idx]), (int)nV);
			myMexErrMsgTxt(msg);
			return;
		}

		if (containsNaNorInf(prhs,A_idx, 0 ) == BT_TRUE)
			return;

		if (containsNaNorInf(prhs,lbA_idx, 1 ) == BT_TRUE)
			return;

		if (containsNaNorInf(prhs,ubA_idx, 1 ) == BT_TRUE)
			return;
	}

	/* check dimensions and copy auxInputs */
	if ( smartDimensionCheck( &g,nV,nP, BT_FALSE,prhs,g_idx ) != SUCCESSFUL_RETURN )
		return;

	if ( smartDimensionCheck( &lb,nV,nP, BT_TRUE,prhs,lb_idx ) != SUCCESSFUL_RETURN )
		return;

	if ( smartDimensionCheck( &ub,nV,nP, BT_TRUE,prhs,ub_idx ) != SUCCESSFUL_RETURN )
		return;

	if ( smartDimensionCheck( &x0,nV,1, BT_TRUE,prhs,x0_idx ) != SUCCESSFUL_RETURN )
		return;

	if ( nC > 0 )
	{
		if ( smartDimensionCheck( &lbA,nC,nP, BT_TRUE,prhs,lbA_idx ) != SUCCESSFUL_RETURN )
			return;

		if ( smartDimensionCheck( &ubA,nC,nP, BT_TRUE,prhs,ubA_idx ) != SUCCESSFUL_RETURN )
			return;
	}

	if ( auxInput_idx >= 0 )
		setupAuxiliaryInputs( prhs[auxInput_idx],nV,nC, &hessianType,&x0,&guessedBounds,&guessedConstraints,&R_for );

	/* convert Cholesky factor to C storage format */
	if ( R_for != 0 )
	{
		R = new real_t[nV*nV];
		convertFortranToC( R_for, nV,nV, R );
	}
	
	/* III) ACTUALLY PERFORM QPOASES FUNCTION CALL: */
	int_t nWSRin = 5*(nV+nC);
	real_t maxCpuTimeIn = -1.0;

	if ( options_idx > 0 )
		setupOptions( &options,prhs[options_idx],nWSRin,maxCpuTimeIn );

	/* make a deep-copy of the user-specified Hessian matrix (possibly sparse) */
	if ( H_idx >= 0 )
		setupHessianMatrix(	prhs[H_idx],nV, &H,&Hir,&Hjc,&Hv );
	
	/* make a deep-copy of the user-specified constraint matrix (possibly sparse) */
	if ( ( nC > 0 ) && ( A_idx >= 0 ) )
		setupConstraintMatrix( prhs[A_idx],nV,nC, &A,&Air,&Ajc,&Av );

	allocateOutputs( nlhs,plhs,nV,nC,nP );

	/* check if QP is sparse */
	if ( H_idx >= 0 && !mxIsSparse( prhs[H_idx] ) )
		isSparse = BT_FALSE;
	if ( nC > 0 && A_idx >= 0 && !mxIsSparse( prhs[A_idx] ) )
		isSparse = BT_FALSE;

	if ( nC == 0 )
	{
		/* Call qpOASES (using QProblemB class). */
		QProblemB_qpOASES(	nV,hessianType, nP,
							H,g,
							lb,ub,
							nWSRin,maxCpuTimeIn,
							x0,&options,
							nlhs,plhs,
							guessedBounds,R
							);
		
        if (R != 0) delete R;
		if (H != 0) delete H;
		if (Hv != 0) delete[] Hv;
		if (Hjc != 0) delete[] Hjc;
		if (Hir != 0) delete[] Hir;
		return;
	}
	else
	{
		if ( A == 0 )
		{
			myMexErrMsgTxt( "ERROR (qpOASES): Internal interface error related to constraint matrix!" );
			return;
		}

		/* Call qpOASES (using QProblem class). */
		QProblem_qpOASES(	nV,nC,hessianType, nP,
							H,g,A,
							lb,ub,lbA,ubA,
							nWSRin,maxCpuTimeIn,
							x0,&options,
							nlhs,plhs,
							guessedBounds,guessedConstraints,R,
							isSparse
							);
		
		if (R != 0) delete R;
		if (A != 0) delete A;
		if (H != 0) delete H;
		if (Av != 0) delete[] Av;
		if (Ajc != 0) delete[] Ajc;
		if (Air != 0) delete[] Air;
		if (Hv != 0) delete[] Hv;
		if (Hjc != 0) delete[] Hjc;
		if (Hir != 0) delete[] Hir;
		return;
	}
}

/*
 *	end of file
 */
