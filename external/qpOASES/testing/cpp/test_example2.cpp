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
 *	\file testing/cpp/test_example2.cpp
 *	\author Hans Joachim Ferreau (thanks to Boris Houska)
 *	\version 3.2
 *	\date 2008-2017
 *
 *	Very simple example for testing qpOASES in combination
 *  with the SolutionAnalysis class.
 */



#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>


/** Example for qpOASES main function using the SolutionAnalysis class. */
int main( )
{
	USING_NAMESPACE_QPOASES

	/* Setup data of first QP. */
	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

	/* Setup data of second QP. */
	real_t H_new[2*2] = { 1.0, 0.5, 0.5, 0.5 };
	real_t A_new[1*2] = { 1.0, 5.0 };
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };


	/* Setting up SQProblem object and solution analyser. */
	SQProblem example( 2,1 );
	SolutionAnalysis analyser;

	/* Solve first QP ... */
	int_t nWSR = 10;
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );

	/* ... and analyse it. */
	real_t maxKktViolation = analyser.getKktViolation( &example );
    printf( "maxKktViolation: %e\n", maxKktViolation );

	QPOASES_TEST_FOR_TOL( maxKktViolation,1e-15 );


	/* Solve second QP ... */
	nWSR = 10;
	example.hotstart( H_new,g_new,A_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,0 );

	/* ... and analyse it. */
	maxKktViolation = analyser.getKktViolation( &example );
    printf( "maxKktViolation: %e\n", maxKktViolation );

	QPOASES_TEST_FOR_TOL( maxKktViolation,2e-15 );


//  ------------ VARIANCE-COVARIANCE EVALUATION --------------------

        real_t *Var              = new real_t[5*5];
        real_t *Primal_Dual_Var  = new real_t[5*5];

        int_t run1, run2;
        for( run1 = 0; run1 < 5*5; run1++ )
            Var[run1] = 0.0;

        Var[0] = 1.0;
        Var[6] = 1.0;

//                  (  1   0   0   0   0   )
//                  (  0   1   0   0   0   )
//     Var     =    (  0   0   0   0   0   )
//                  (  0   0   0   0   0   )
//                  (  0   0   0   0   0   )


        analyser.getVarianceCovariance( &example, Var,Primal_Dual_Var );

        printf("\nPrimal_Dual_VAR = \n");
        for( run1 = 0; run1 < 5; run1++ ){
          for( run2 = 0; run2 < 5; run2++ ){
            printf(" %10f", Primal_Dual_Var[run1*5+run2]);
          }
          printf("\n");
        }

        delete[] Primal_Dual_Var;
        delete[] Var;

		QPOASES_TEST_FOR_NEAR( Primal_Dual_Var[3*5+3], 26.0 );

	return TEST_PASSED;
}


/*
 *	end of file
 */
