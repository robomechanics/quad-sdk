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
 *	\file testing/cpp/test_exampleLP.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2008-2017
 *
 *	Very simple example for solving a LP sequence using qpOASES.
 */



#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>


/** Example for qpOASES main function solving LPs. */
int main( )
{
	USING_NAMESPACE_QPOASES

	real_t tol = 1e-14;

	/* Setup data of first LP. */
	real_t A[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

	/* Setup data of second LP. */
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };


	/* Setting up QProblem object with zero Hessian matrix. */
	QProblem example( 2,1,HST_ZERO );

	Options options;
 	/*options.setToMPC();*/
	example.setOptions( options );

	/* Solve first LP. */
	int_t nWSR = 10;
	example.init( 0,g,A,lb,ub,lbA,ubA, nWSR,0 );

	real_t xOpt[2];
	real_t yOpt[2+1];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );

	/* Compute KKT tolerances */
	real_t stat, feas, cmpl;
	SolutionAnalysis analyzer;
	printf( "%d\n",example.getHessianType() );

	analyzer.getKktViolation( &example, &stat,&feas,&cmpl );
	printf( "stat = %e\nfeas = %e\ncmpl = %e\n", stat,feas,cmpl );

	QPOASES_TEST_FOR_TOL( stat,tol );
	QPOASES_TEST_FOR_TOL( feas,tol );
	QPOASES_TEST_FOR_TOL( cmpl,tol );


	/* Solve second LP. */
	nWSR = 10;
	example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,0 );


	/* Get and print solution of second LP. */
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );

	analyzer.getKktViolation( &example, &stat,&feas,&cmpl );
	printf( "stat = %e\nfeas = %e\ncmpl = %e\n", stat,feas,cmpl );

	QPOASES_TEST_FOR_TOL( stat,tol );
	QPOASES_TEST_FOR_TOL( feas,tol );
	QPOASES_TEST_FOR_TOL( cmpl,tol );

	return TEST_PASSED;
}


/*
 *	end of file
 */
