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
 *	\file testing/cpp/test_guessedWS1.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2014-2017
 *
 *	Very simple example for testing qpOASES (using QProblem class).
 */



#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>


/** Example for qpOASES main function using the QProblem class. */
int main( )
{
	USING_NAMESPACE_QPOASES

	/* Setup data of first QP. */
	real_t H[4*4] = { 1.0, 0.0, 0.0, 0.5, 
	                  0.0, 1.0, 0.0, 0.0,
	                  0.0, 0.0, 1.0, 0.0,
	                  0.5, 0.0, 0.0, 1.0 };
	real_t A[3*4] = { 1.0, 1.0, 0.0, 0.0,
	                  1.0, 1.0, 1.0, 0.0,
	                  1.0, 1.0, 1.0, 1.0 };
	real_t g[4] = { 1.5, 1.0, -1.0, -1.0 };
	real_t lb[4] = { 0.5, -2.0, 0.0, 0.0 };
	real_t ub[4] = { 1.0, 2.0, 1.0, 0.5 };
	real_t lbA[3] = { -1.0, -1.0, -1.0 };
	real_t ubA[3] = { 0.0, 0.25, 1.0 };

	

	/* Setting up QProblem object. */
	QProblem example( 4,3 );

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 10;
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

	/* Get and print solution of second QP. */
	real_t xOpt[4];
	real_t yOpt[4+3];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	print( xOpt,4,"xOpt" );
	print( yOpt,4+3,"yOpt" );
	printf( "objVal = %e\n\n", example.getObjVal() );
	
	/* Compute KKT tolerances */
	real_t stat, feas, cmpl;
	SolutionAnalysis analyzer;

	analyzer.getKktViolation( &example, &stat,&feas,&cmpl );
	printf( "stat = %e\nfeas = %e\ncmpl = %e\n", stat,feas,cmpl );

	QPOASES_TEST_FOR_TOL( stat,1e-15 );
	QPOASES_TEST_FOR_TOL( feas,1e-15 );
	QPOASES_TEST_FOR_TOL( cmpl,1e-15 );


	/* Solve first QP again (with optimal guess for working set). */
	Bounds prevBounds;
	Constraints prevConstraints;
	
	example.getBounds( prevBounds );
	example.getConstraints( prevConstraints );

	nWSR = 10;
	example.hotstart( g,lb,ub,lbA,ubA, nWSR,0,&prevBounds,&prevConstraints );

	/* Get and print solution of second QP. */
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	print( xOpt,4,"xOpt" );
	print( yOpt,4+3,"yOpt" );
	printf( "objVal = %e\n\n", example.getObjVal() );
	
	/* Compute KKT tolerances */
	analyzer.getKktViolation( &example, &stat,&feas,&cmpl );
	printf( "stat = %e\nfeas = %e\ncmpl = %e\n", stat,feas,cmpl );

	QPOASES_TEST_FOR_TOL( stat,1e-15 );
	QPOASES_TEST_FOR_TOL( feas,1e-15 );
	QPOASES_TEST_FOR_TOL( cmpl,1e-15 );


	/* Solve first QP again (with inaccurate guess for working set). */
	prevBounds.print();
	prevBounds.rotate(1);
	//prevBounds.moveFixedToFree(0);
	prevBounds.print();

	prevConstraints.print();
	//prevConstraints.moveInactiveToActive(0,ST_LOWER);
	prevConstraints.moveActiveToInactive(1);
	prevConstraints.print();

	nWSR = 10;
	example.hotstart( g,lb,ub,lbA,ubA, nWSR,0,&prevBounds,&prevConstraints );

	/* Get and print solution of second QP. */
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	print( xOpt,4,"xOpt" );
	print( yOpt,4+3,"yOpt" );
	printf( "objVal = %e\n\n", example.getObjVal() );
	
	/* Compute KKT tolerances */
	analyzer.getKktViolation( &example, &stat,&feas,&cmpl );
	printf( "stat = %e\nfeas = %e\ncmpl = %e\n", stat,feas,cmpl );

	QPOASES_TEST_FOR_TOL( stat,1e-15 );
	QPOASES_TEST_FOR_TOL( feas,1e-15 );
	QPOASES_TEST_FOR_TOL( cmpl,1e-15 );

	return TEST_PASSED;
}


/*
 *	end of file
 */
