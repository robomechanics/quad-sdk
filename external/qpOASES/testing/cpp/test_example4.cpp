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
 *	\file testing/cpp/test_example4.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2009-2017
 *
 *	Very simple example for testing qpOASES (using the possibility to specify 
 *	user-defined constraint product function).
 */



#include <stdlib.h>

#include <qpOASES.hpp>
#include "../../examples/example4CP.cpp"
#include <qpOASES/UnitTesting.hpp>


/**	Example for qpOASES main function using the possibility to specify 
 *	user-defined constraint product function. */
int main( )
{
	USING_NAMESPACE_QPOASES

	int_t i,j;

	/* Setup data of first QP... */
	real_t H[7*7];
	real_t A[50*7];
	real_t g[7];
	real_t lbA[50];

	/*	    ( 1.0 0.5 |                    )
	 *	    ( 0.5 2.0 |                    )
	 *	    ( --------+------------------- )
	 *	H = (         | 1e-6               )
	 *	    (         |      1e-6          )
	 *	    (         |           ...      )
	 *	    (         |               1e-6 ) */
	for( i=0; i<7*7; ++i )
		H[i] = 0.0;
	for( i=2; i<7; ++i )
		H[i*7+i] = 1.0e-6;
	H[0] = 1.0;
	H[1] = 0.5;
	H[7] = 0.5;
	H[8] = 2.0;

	/*	    ( x.x x.x | 1.0             )
	 *	    ( x.x x.x | ...             )
	 *	    ( x.x x.x | 1.0             )
	 *	    ( x.x x.x |     1.0         )
	 *	A = ( x.x x.x |     ...         )
	 *	    ( x.x x.x |     1.0         )
	 *	    ( x.x x.x |         ...     )
	 *	    ( x.x x.x |             1.0 )
	 *	    ( x.x x.x |             ... )
	 *	    ( x.x x.x |             1.0 ) */
	for( i=0; i<50*7; ++i )
		A[i] = 0.0;
	for( i=0; i<50; ++i )
	{
		for( j=0; j<2; ++j )
			A[i*7+j] = (real_t)rand() / (real_t)RAND_MAX;

		A[i*7 + (i/10)+2] = 1.0;
	}

	/*	    ( -1.0 )
	 *	    ( -0.5 )
	 *	    ( ---- )
	 *	g = (      )
	 *	    (      )
	 *	    (      )
	 *	    (      ) */
	for( i=0; i<7; ++i )
		g[i] = 0.0;
	g[0] = -1.0;
	g[1] = -0.5;

	for( i=0; i<50; ++i )
		lbA[i] = 1.0;

	/* ... and setting up user-defined constraint product function. */
	MyConstraintProduct myCP( 7,50,A );


	/* Setting up QProblem object and set construct product function. */
	QProblem exampleCP( 7,50 );
	exampleCP.setPrintLevel( PL_NONE );
	
	exampleCP.setConstraintProduct( &myCP );


	/* Solve first QP. */
	real_t cputime = 1.0;
	int_t nWSR = 100;
	exampleCP.init( H,g,A,0,0,lbA,0, nWSR,&cputime );


	/* Solve second QP using a modified gradient. */
	g[0] = -2.0;
	g[1] =  0.5;

	cputime = 1.0;
	nWSR = 100;
	exampleCP.hotstart( g,0,0,lbA,0, nWSR,&cputime );

	/* Get and print solution of second QP. */
	real_t xOptCP[7];
	real_t yOptCP[7+50];
	exampleCP.getPrimalSolution( xOptCP );
	exampleCP.getDualSolution( yOptCP );
	printf( "\nxOpt = [ %e, %e, %e ... ];  objVal = %e\n", xOptCP[0],xOptCP[1],xOptCP[2],exampleCP.getObjVal() );
	printf( "CPU time:  %.3f microseconds\n\n", cputime*1.0e6 ); 

	/* Compute KKT tolerances */
	real_t stat, feas, cmpl;
	SolutionAnalysis analyzerCP;

	analyzerCP.getKktViolation( &exampleCP, &stat,&feas,&cmpl );
	printf( "stat = %e\nfeas = %e\ncmpl = %e\n", stat,feas,cmpl );

	QPOASES_TEST_FOR_TOL( stat,1e-15 );
	QPOASES_TEST_FOR_TOL( feas,1e-15 );
	QPOASES_TEST_FOR_TOL( cmpl,1e-15 );
	
	
	/* Do the same without specifying constraint product. */
	QProblem example( 7,50 );
	example.setPrintLevel( PL_NONE );
	
	/* Solve first QP. */
	g[0] = -1.0;
	g[1] = -0.5;
	
	cputime = 1.0;
	nWSR = 100;
	example.init( H,g,A,0,0,lbA,0, nWSR,&cputime );
	
	/* Solve second QP using a modified gradient. */
	g[0] = -2.0;
	g[1] =  0.5;

	cputime = 1.0;
	nWSR = 100;
	example.hotstart( g,0,0,lbA,0, nWSR,&cputime );

	/* Get and print solution of second QP. */
	real_t xOpt[7];
	real_t yOpt[7+50];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e, %e ... ];  objVal = %e\n", xOpt[0],xOpt[1],xOpt[2],example.getObjVal() );
	printf( "CPU time:  %.3f microseconds\n\n", cputime*1.0e6 ); 

	/* Compute KKT tolerances */
	SolutionAnalysis analyzer;

	analyzer.getKktViolation( &example, &stat,&feas,&cmpl );
	printf( "stat = %e\nfeas = %e\ncmpl = %e\n", stat,feas,cmpl );

	QPOASES_TEST_FOR_TOL( stat,1e-15 );
	QPOASES_TEST_FOR_TOL( feas,1e-15 );
	QPOASES_TEST_FOR_TOL( cmpl,1e-15 );

	for( int_t ii=0; ii<7; ++ii )
		QPOASES_TEST_FOR_NEAR( xOptCP[ii],xOpt[ii] );

	for( int_t ii=0; ii<7+50; ++ii )
		QPOASES_TEST_FOR_NEAR( yOptCP[ii],yOpt[ii] );

	QPOASES_TEST_FOR_NEAR( exampleCP.getObjVal(),example.getObjVal() );

	return TEST_PASSED;
}


/*
 *	end of file
 */
