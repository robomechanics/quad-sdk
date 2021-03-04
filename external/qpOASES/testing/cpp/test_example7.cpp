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
 *	\file testing/cpp/test_example7.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Example that caused troubles in an earlier release.
 */



#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>


int main( )
{
	USING_NAMESPACE_QPOASES

	real_t H[5*5] = {0.8514828085899353, -0.15739890933036804, -0.081726007163524628, -0.530426025390625, 0.16773293912410736, -0.15739890933036804, 1.1552412509918213, 0.57780224084854126, -0.0072606131434440613, 0.010559185408055782, -0.081726007163524628, 0.57780224084854126, 0.28925251960754395, 5.324830453901086e-006, -3.0256599075073609e-006, -0.530426025390625, -0.0072606131434440613, 5.324830453901086e-006, 0.35609596967697144, -0.15124998986721039, 0.16773293912410736, 0.010559185408055782, -3.0256599075073609e-006, -0.15124998986721039, 0.15129712224006653};
	real_t g[5] = {0.30908384919166565, 0.99325823783874512, 0.49822014570236206, -0.26309865713119507, 0.024296050891280174};
	real_t A[5*5] =   {1,0,0,0,0,
					   0,1,0,0,0,
					   0,0,1,0,0,              
					   0,0,0,1,0,
					   0,0,0,0,1};
	real_t lb[5] = {-0.052359879016876221, -0.052359879016876221, -0.052359879016876221, -0.052359879016876221, -0.052359938621520996};
	real_t ub[5] = { 0.052359879016876221, 0.052359879016876221, 0.052359879016876221, 0, 0};
	real_t lbA[5] = {-0.052359879016876221, -0.052359879016876221, -0.052359879016876221, -0.052359879016876221, -0.052359938621520996};
	real_t ubA[5] = {0.052359879016876221, 0.052359879016876221, 0.052359879016876221, 0, 0};

	/* Setting up QProblem object. */
	QProblem example( 5,5 );

	/* Solve first QP. */
	int_t nWSR = 100;
	returnValue retVal = example.init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
	printf( "nWSR = %d,  retVal = %d (%s)\n", (int)nWSR,retVal,getGlobalMessageHandler( )->getErrorCodeMessage(retVal) );
	
	
	real_t sol[5]    = {0};
	real_t yOpt[5+5] = {0};
	example.getPrimalSolution(sol);
	example.getDualSolution(yOpt);
	
	printf("l1 = %f, l2 = %f, l3 = %f, l4 = %f, l5 = %f\n",(float) lb[0], (float) lb[1], (float) lb[2], (float) lb[3], (float) lb[4]);
	printf("x1 = %f, x2 = %f, x3 = %f, x4 = %f, x5 = %f\n",(float) sol[0], (float) sol[1], (float) sol[2], (float) sol[3], (float) sol[4]);
	printf("u1 = %f, u2 = %f, u3 = %f, u4 = %f, u5 = %f\n",(float) ub[0], (float) ub[1], (float) ub[2], (float) ub[3], (float) ub[4]);

	/* Compute KKT tolerances */
	real_t stat, feas, cmpl;
	SolutionAnalysis analyzer;

	analyzer.getKktViolation( &example, &stat,&feas,&cmpl );
	printf( "stat = %e\nfeas = %e\ncmpl = %e\n", stat,feas,cmpl );

	QPOASES_TEST_FOR_TOL( stat,1e-15 );
	QPOASES_TEST_FOR_TOL( feas,1e-15 );
	QPOASES_TEST_FOR_TOL( cmpl,1e-15 );

	return TEST_PASSED;
}

