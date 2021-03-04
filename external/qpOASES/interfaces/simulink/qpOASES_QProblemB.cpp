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
 *	\file interfaces/simulink/qpOASES_QProblemB.cpp
 *	\author Hans Joachim Ferreau (thanks to Aude Perrin)
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Interface for Simulink(R) that enables to call qpOASES as a S function
 *  (variant for simply bounded QPs with fixed matrices).
 *
 */


#include <stdlib.h>

#include <qpOASES.hpp>
#include "qpOASES_simulink_utils.cpp"


#ifdef __cplusplus
extern "C" {
#endif


#define S_FUNCTION_NAME   qpOASES_QProblemB		/**< Name of the S function. */
#define S_FUNCTION_LEVEL  2						/**< S function level. */

#define MDL_START								/**< Activate call to mdlStart. */

#include "simstruc.h"


/* SETTINGS */
#define SAMPLINGTIME   -1						/**< Sampling time. */
#define NCONTROLINPUTS  2						/**< Number of control inputs. */
#define MAXITER         100						/**< Maximum number of iteration. */
#define HESSIANTYPE     HST_UNKNOWN				/**< Hessian type, see documentation of QProblemB class constructor. */


static void mdlInitializeSizes (SimStruct *S)   /* Init sizes array */
{
	int ii;
	int nU = NCONTROLINPUTS;

	/* Specify the number of continuous and discrete states */
	ssSetNumContStates(S, 0);
	ssSetNumDiscStates(S, 0);

	/* Specify the number of parameters */
	ssSetNumSFcnParams(S, 1); /* H */
	if ( ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S) )
		return;

	/* Specify the number of intput ports */
	if ( !ssSetNumInputPorts(S, 3) )
		return;

	/* Specify the number of output ports */
	if ( !ssSetNumOutputPorts(S, 4) )
		return;

	/* Specify dimension information for the input ports */
	ssSetInputPortVectorDimension(S, 0, DYNAMICALLY_SIZED);	/* g */
	ssSetInputPortVectorDimension(S, 1, DYNAMICALLY_SIZED);	/* lb */
	ssSetInputPortVectorDimension(S, 2, DYNAMICALLY_SIZED);	/* ub */

	/* Specify dimension information for the output ports */
	ssSetOutputPortVectorDimension(S, 0, nU );  /* uOpt */
    ssSetOutputPortVectorDimension(S, 1, 1 );   /* fval */
	ssSetOutputPortVectorDimension(S, 2, 1 );   /* exitflag */
	ssSetOutputPortVectorDimension(S, 3, 1 );   /* iter */

	/* Specify the direct feedthrough status */
	for( ii=0; ii<3; ++ii ) 
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
     * 3: lb
     * 4: ub
     */

	/* Specify the size of the block's pointer work vector */
    ssSetNumPWork(S, 5);
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
	int size_g, size_lb, size_ub;
	int size_H, nRows_H, nCols_H;
	int nV;

	QProblemB* problem;


	/* get block inputs dimensions */
	const mxArray* in_H = ssGetSFcnParam(S, 0);

	if ( mxIsEmpty(in_H) == 1 )
	{
		if ( ( HESSIANTYPE != HST_ZERO ) && ( HESSIANTYPE != HST_IDENTITY ) )
		{
			#ifndef __SUPPRESSANYOUTPUT__
			mexErrMsgTxt( "ERROR (qpOASES): Hessian can only be empty if type is set to HST_ZERO or HST_IDENTITY!" );
			#endif
			return;
		}
		
	    nRows_H = 0;
		nCols_H = 0;
		size_H  = 0;
	}
	else
	{
	    nRows_H = (int)mxGetM(in_H);
		nCols_H = (int)mxGetN(in_H);
		size_H  = nRows_H * nCols_H;
	}

	size_g   = ssGetInputPortWidth(S, 0);
	size_lb  = ssGetInputPortWidth(S, 1);
	size_ub  = ssGetInputPortWidth(S, 2);


	/* dimension checks */
	nV = size_g;

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

	if ( nRows_H != nCols_H )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Hessian matrix must be square matrix!" );
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


	/* allocate QProblemB object */
	problem = new QProblemB( nV,HESSIANTYPE );
	if ( problem == 0 )
	{
		#ifndef __SUPPRESSANYOUTPUT__
		mexErrMsgTxt( "ERROR (qpOASES): Unable to create QProblemB object!" );
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

	if ( size_lb > 0 )
		ssGetPWork(S)[3] = (void *) calloc( size_lb, sizeof(real_t) );	/* lb */
	else
		ssGetPWork(S)[3] = 0;

	if ( size_ub > 0 )
		ssGetPWork(S)[4] = (void *) calloc( size_ub, sizeof(real_t) );	/* ub */
	else
		ssGetPWork(S)[4] = 0;
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
	USING_NAMESPACE_QPOASES

	int ii;
	int nV;
	returnValue status;

	int_t nWSR = MAXITER;
	int nU     = NCONTROLINPUTS;

	InputRealPtrsType in_g, in_lb, in_ub;

	QProblemB* problem;
	real_t *H, *g, *lb, *ub;

	real_t *xOpt;

	real_T *out_uOpt, *out_objVal, *out_status, *out_nWSR;


	/* get pointers to block inputs ... */
	const mxArray* in_H = ssGetSFcnParam(S, 0);
	in_g  = ssGetInputPortRealSignalPtrs(S, 0);
	in_lb = ssGetInputPortRealSignalPtrs(S, 1);
	in_ub = ssGetInputPortRealSignalPtrs(S, 2);

	/* ... and to the QP data */
	problem = (QProblemB*) ssGetPWork(S)[0];

	H  = (real_t *) ssGetPWork(S)[1];
	g  = (real_t *) ssGetPWork(S)[2];
	lb = (real_t *) ssGetPWork(S)[3];
	ub = (real_t *) ssGetPWork(S)[4];


	/* setup QP data */
	nV = ssGetInputPortWidth(S, 1); /* nV = size_g */

	if ( H != 0 )
	{
		/* no conversion from FORTRAN to C as Hessian is symmetric! */
		for ( ii=0; ii<nV*nV; ++ii )
			H[ii] = (mxGetPr(in_H))[ii];
	}

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

	xOpt = new real_t[nV];

	if ( problem->getCount() == 0 )
	{
		/* initialise and solve first QP */
		status = problem->init( H,g,lb,ub, nWSR,0 );
		problem->getPrimalSolution( xOpt );
	}
	else
	{
		/* solve neighbouring QP using hotstart technique */
		status = problem->hotstart( g,lb,ub, nWSR,0 );
		if ( ( status != SUCCESSFUL_RETURN ) && ( status != RET_MAX_NWSR_REACHED ) )
		{
			/* if an error occurs, reset problem data structures ... */
			problem->reset( );
            
            /* ... and initialise/solve again with remaining number of iterations. */
            int_t nWSR_retry = MAXITER - nWSR;
			status = problem->init( H,g,lb,ub, nWSR_retry,0 );
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
    
    out_objVal[0] = (real_T)(problem->getObjVal());
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
		delete ((QProblemB*)(ssGetPWork(S)[0]));

	for ( ii=1; ii<5; ++ii )
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
