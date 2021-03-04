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
 *	\file testing/cpp/test_sebastien1.cpp
 *	\author Sebastien B.
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Example that caused troubles in an earlier release.
 */



#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>


/** qpOASES main function defining a unit test. */
int main( )
{
	REFER_NAMESPACE_QPOASES real_t solution[2]       = {0.0f, 0.0f};
	REFER_NAMESPACE_QPOASES real_t expectedFirst[2]  = {0.5f, -1.5f};
  
	REFER_NAMESPACE_QPOASES real_t H[2*2] = {1.0f, 0.0f, 0.0f, 0.5f};
	REFER_NAMESPACE_QPOASES real_t g[2]   = {1.5f, 1.0f};

	REFER_NAMESPACE_QPOASES real_t A[1*2] = {1.0f, 1.0f};
	REFER_NAMESPACE_QPOASES real_t lbA[1] = {-1.0f};
	REFER_NAMESPACE_QPOASES real_t ubA[1] = {2.0f};

	REFER_NAMESPACE_QPOASES real_t lb[2]  = {0.5f, -2.0f};
	REFER_NAMESPACE_QPOASES real_t ub[2]  = {5.0f, 2.0f};

	REFER_NAMESPACE_QPOASES QProblem example(2, 1);
	REFER_NAMESPACE_QPOASES Options options = example.getOptions();
	//options.enableFarBounds = REFER_NAMESPACE_QPOASES BT_FALSE;
	example.setOptions(options);
	example.setPrintLevel(REFER_NAMESPACE_QPOASES PL_NONE);

	// Solve first QP.
	REFER_NAMESPACE_QPOASES int_t nWSR = 10;
	QPOASES_TEST_FOR_TRUE( example.init(H, g, A, lb, ub, lbA, ubA, nWSR, NULL) == REFER_NAMESPACE_QPOASES SUCCESSFUL_RETURN );
	QPOASES_TEST_FOR_TRUE( example.isSolved() == REFER_NAMESPACE_QPOASES BT_TRUE );
	example.getPrimalSolution(solution);

	printf( "\nxOpt = [ %e, %e ];\n\n", solution[0],solution[1] );

	for( REFER_NAMESPACE_QPOASES uint_t i=0; i<2; i++ )
		QPOASES_TEST_FOR_NEAR( solution[i],expectedFirst[i] );

	return TEST_PASSED;
}
