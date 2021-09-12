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
 *	\file interfaces/simulink/qpOASES_SQProblem.cpp
 *	\author Hans Joachim Ferreau (thanks to Aude Perrin)
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Interface for Simulink(R) that enables to call qpOASES as a S function
 *  (variant for QPs with varying matrices).
 *
 */


#include <stdlib.h>

#include <qpOASES.hpp>
#include "qpOASES_simulink_utils.cpp"


#ifdef __cplusplus
extern "C" {
#endif


#define S_FUNCTION_NAME   qpOASES_SQProblem		/**< Name of the S function. */
#define S_FUNCTION_LEVEL  2						/**< S function level. */

#define MDL_START								/**< Activate call to mdlStart. */

#include "simstruc.h"


/* SETTINGS */
#define SAMPLINGTIME   -1						/**< Sampling time. */
#define NCONTROLINPUTS  2						/**< Number of control inputs. */
#define MAXITER         100	    				/**< Maximum number of iterations. */
#define HESSIANTYPE     HST_UNKNOWN				/**< Hessian type, see documentation of SQProblem class constructor. */


static void mdlInitializeSizes (SimStruct *S)   /* Init sizes array */
{
	int ii;
	int nU = NCONTROLINPUTS;

	/* Specify the number of continuous and discrete states */
	ssSetNumContStates(S, 0);
	ssSetNumDiscStates(S, 0);

	/* Specify the number of parameters */
	ssSetNumSFcnParams(S, 0);
	if ( ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S) )
		return;

	/* Specify the number of intput ports */
	if ( !ssSetNumInputPorts(S, 7) )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Invalid number of input ports!" );
		#endif
		return;
	}

	/* Specify the number of output ports */
	if ( !ssSetNumOutputPorts(S, 4) )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Invalid number of output ports!" );
		#endif
		return;
	}

	/* Specify dimension information for the input ports */
	ssSetInputPortVectorDimension(S, 0, DYNAMICALLY_SIZED);	/* H */
	ssSetInputPortVectorDimension(S, 1, DYNAMICALLY_SIZED); /* g */
	ssSetInputPortVectorDimension(S, 2, DYNAMICALLY_SIZED); /* A */
	ssSetInputPortVectorDimension(S, 3, DYNAMICALLY_SIZED); /* lb */
	ssSetInputPortVectorDimension(S, 4, DYNAMICALLY_SIZED); /* ub */
	ssSetInputPortVectorDimension(S, 5, DYNAMICALLY_SIZED); /* lbA */
	ssSetInputPortVectorDimension(S, 6, DYNAMICALLY_SIZED); /* ubA */

	/* Specify dimension information for the output ports */
	ssSetOutputPortVectorDimension(S, 0, nU );  /* uOpt */
	ssSetOutputPortVectorDimension(S, 1, 1 );   /* fval */
	ssSetOutputPortVectorDimension(S, 2, 1 );   /* exitflag */
	ssSetOutputPortVectorDimension(S, 3, 1 );   /* iter */

	/* Specify the direct feedthrough status */
	for( ii=0; ii<7; ++ii ) 
	{
		ssSetInputPortDirectFeedThrough(S, ii, 1);
		//ssSetInputPortRequiredContiguous(S, ii, 1);
	}

	/* One sample time */
	ssSetNumSampleTimes(S, 1);

	/* global variables:
     * 0: problem
     * 1: H
     * 2: g
     * 3: A
     * 4: lb
     * 5: ub
     * 6: lbA
     * 7: ubA
     */

	/* Specify the size of the block's pointer work vector */
    ssSetNumPWork(S, 8);
}


#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
	if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )
		return;
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
	if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )
		return;
}

#endif


static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, SAMPLINGTIME);
	ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
	USING_NAMESPACE_QPOASES

	int nU = NCONTROLINPUTS;
	int size_H, size_g, size_A, size_lb, size_ub, size_lbA, size_ubA;
	int nV, nC;

	SQProblem* problem;


	/* get block inputs dimensions */
	size_H   = ssGetInputPortWidth(S, 0);
	size_g   = ssGetInputPortWidth(S, 1);
	size_A   = ssGetInputPortWidth(S, 2);
	size_lb  = ssGetInputPortWidth(S, 3);
	size_ub  = ssGetInputPortWidth(S, 4);
	size_lbA = ssGetInputPortWidth(S, 5);
	size_ubA = ssGetInputPortWidth(S, 6);


	/* dimension checks */
	nV = size_g;
	nC = (int) ( ((real_t) size_A) / ((real_t) nV) );

	if ( MAXITER < 0 )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Maximum number of iterations must not be negative!" );
		#endif
		return;
	}

	if ( nV <= 0 )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch!" );
		#endif
		return;
	}

	if ( ( size_H != nV*nV ) && ( size_H != 0 ) )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch in H!" );
		#endif
		return;
	}

	if ( ( nU < 1 ) || ( nU > nV ) )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Invalid number of control inputs!" );
		#endif
		return;
	}

	if ( ( size_lb != nV ) && ( size_lb != 0 ) )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch in lb!" );
		#endif
		return;
	}

	if ( ( size_ub != nV ) && ( size_ub != 0 ) )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch in ub!" );
		#endif
		return;
	}

	if ( ( size_lbA != nC ) && ( size_lbA != 0 ) )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch in lbA!" );
		#endif
		return;
	}

	if ( ( size_ubA != nC ) && ( size_ubA != 0 ) )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Dimension mismatch in ubA!" );
		#endif
		return;
	}


	/* allocate QProblem object */
	problem = new SQProblem( nV,nC,HESSIANTYPE );
	if ( problem == 0 )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Unable to create SQProblem object!" );
		#endif
		return;
	}

	Options problemOptions;
	problemOptions.setToMPC();
	problem->setOptions( problemOptions );

	#ifndef __DEBUG__
	problem->setPrintLevel( PL_LOW );
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	problem->setPrintLevel( PL_NONE );
	#endif

	ssGetPWork(S)[0] = (void *) problem;

	/* allocate memory for QP data ... */
	if ( size_H > 0 )
		ssGetPWork(S)[1] = (void *) calloc( size_H, sizeof(real_t) );	/* H */
	else
		ssGetPWork(S)[1] = 0;

	ssGetPWork(S)[2] = (void *) calloc( size_g, sizeof(real_t) );		/* g */
	ssGetPWork(S)[3] = (void *) calloc( size_A, sizeof(real_t) );		/* A */

	if ( size_lb > 0 )
		ssGetPWork(S)[4] = (void *) calloc( size_lb, sizeof(real_t) );	/* lb */
	else
		ssGetPWork(S)[4] = 0;

	if ( size_ub > 0 )
		ssGetPWork(S)[5] = (void *) calloc( size_ub, sizeof(real_t) );	/* ub */
	else
		ssGetPWork(S)[5] = 0;
	
	if ( size_lbA > 0 )
		ssGetPWork(S)[6] = (void *) calloc( size_lbA, sizeof(real_t) );	/* lbA */
	else
		ssGetPWork(S)[6] = 0;

	if ( size_ubA > 0 )
		ssGetPWork(S)[7] = (void *) calloc( size_ubA, sizeof(real_t) );	/* ubA */
	else
		ssGetPWork(S)[7] = 0;
}



static void mdlOutputs(SimStruct *S, int_T tid)
{
	USING_NAMESPACE_QPOASES

	int ii;
	int nV, nC;
	returnValue status;

	int_t nWSR = MAXITER;
	int nU     = NCONTROLINPUTS;

	InputRealPtrsType in_H, in_g, in_A, in_lb, in_ub, in_lbA, in_ubA;

	SQProblem* problem;
	real_t *H, *g, *A, *lb, *ub, *lbA, *ubA;

	real_t *xOpt;

	real_T *out_uOpt, *out_objVal, *out_status, *out_nWSR;


	/* get pointers to block inputs ... */
	in_H   = ssGetInputPortRealSignalPtrs(S, 0);
	in_g   = ssGetInputPortRealSignalPtrs(S, 1);
	in_A   = ssGetInputPortRealSignalPtrs(S, 2);
	in_lb  = ssGetInputPortRealSignalPtrs(S, 3);
	in_ub  = ssGetInputPortRealSignalPtrs(S, 4);
	in_lbA = ssGetInputPortRealSignalPtrs(S, 5);
	in_ubA = ssGetInputPortRealSignalPtrs(S, 6);


	/* ... and to the QP data */
	problem = (SQProblem*) ssGetPWork(S)[0];

	H   = (real_t *) ssGetPWork(S)[1];
	g   = (real_t *) ssGetPWork(S)[2];
	A   = (real_t *) ssGetPWork(S)[3];
	lb  = (real_t *) ssGetPWork(S)[4];
	ub  = (real_t *) ssGetPWork(S)[5];
	lbA = (real_t *) ssGetPWork(S)[6];
	ubA = (real_t *) ssGetPWork(S)[7];


	/* setup QP data */
	nV = ssGetInputPortWidth(S, 1); /* nV = size_g */
	nC = (int) ( ((real_t) ssGetInputPortWidth(S, 2)) / ((real_t) nV) ); /* nC = size_A / size_g */

	if ( H != 0 )
	{
		/* no conversion from FORTRAN to C as Hessian is symmetric! */
		for ( ii=0; ii<nV*nV; ++ii )
			H[ii] = (*in_H)[ii];
	}

	convertFortranToC( *in_A,nV,nC, A );

	for ( ii=0; ii<nV; ++ii )
		g[ii] = (*in_g)[ii];

	if ( lb != 0 )
	{
		for ( ii=0; ii<nV; ++ii )
			lb[ii] = (*in_lb)[ii];
	}

	if ( ub != 0 )
	{
		for ( ii=0; ii<nV; ++ii )
			ub[ii] = (*in_ub)[ii];
	}

	if ( lbA != 0 )
	{
		for ( ii=0; ii<nC; ++ii )
			lbA[ii] = (*in_lbA)[ii];
	}

	if ( ubA != 0 )
	{
		for ( ii=0; ii<nC; ++ii )
			ubA[ii] = (*in_ubA)[ii];
	}

	xOpt = new real_t[nV];

	if ( problem->getCount() == 0 )
	{
		/* initialise and solve first QP */
		status = problem->init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
		problem->getPrimalSolution( xOpt );
	}
	else
	{
		/* solve neighbouring QP using hotstart technique */
		status = problem->hotstart( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
		if ( ( status != SUCCESSFUL_RETURN ) && ( status != RET_MAX_NWSR_REACHED ) )
		{
			/* if an error occurs, reset problem data structures ... */
			problem->reset( );
            
            /* ... and initialise/solve again with remaining number of iterations. */
            int_t nWSR_retry = MAXITER - nWSR;
			status = problem->init( H,g,A,lb,ub,lbA,ubA, nWSR_retry,0 );
            nWSR += nWSR_retry;
		}
	        
        /* obtain optimal solution */
        problem->getPrimalSolution( xOpt );
	}

	/* generate block output: status information ... */
	out_uOpt   = ssGetOutputPortRealSignal(S, 0);
	out_objVal = ssGetOutputPortRealSignal(S, 1);
	out_status = ssGetOutputPortRealSignal(S, 2);
	out_nWSR   = ssGetOutputPortRealSignal(S, 3);

	for ( ii=0; ii<nU; ++ii )
		out_uOpt[ii] = (real_T)(xOpt[ii]);

	out_objVal[0] = (real_T)(problem->getObjVal( ));
	out_status[0] = (real_t)(getSimpleStatus( status ));
	out_nWSR[0]   = (real_T)(nWSR);

	removeNaNs( out_uOpt,nU );
	removeInfs( out_uOpt,nU );
	removeNaNs( out_objVal,1 );
	removeInfs( out_objVal,1 );

	delete[] xOpt;
}


static void mdlTerminate(SimStruct *S)
{
	USING_NAMESPACE_QPOASES

	int ii;

	/* reset global message handler */
	getGlobalMessageHandler( )->reset( );

	if ( ssGetPWork(S)[0] != 0 )
		delete ((SQProblem*)(ssGetPWork(S)[0]));

	for ( ii=1; ii<8; ++ii )
	{
		if ( ssGetPWork(S)[ii] != 0 )
			free( ssGetPWork(S)[ii] );
	}
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif


#ifdef __cplusplus
}
#endif


/*
 *	end of file
 */
