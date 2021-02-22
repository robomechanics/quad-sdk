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
 *	\file testing/cpp/test_vanBarelsUnboundedQP.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Example that causes troubles when hotstarting.
 */



#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>

#include <stdio.h>



int main( )
{
	USING_NAMESPACE_QPOASES

	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.0 };
	real_t g[2] = { 1.5, 1.0 };
		
	Options options;
	//options.enableFarBounds = BT_FALSE;

	QProblemB qp(2);
	qp.setOptions( options );

	int_t iter = 10;
	qp.init( H,g,0,0,iter );

	real_t xOpt[2];
	qp.getPrimalSolution( xOpt );
	print( xOpt,2 );

	return TEST_PASSED;
}


/*
 *	end of file
 */
