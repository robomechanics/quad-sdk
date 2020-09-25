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
 *	\file interfaces/c/example1b.c
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2014-2017
 *
 *	Very simple example for testing qpOASES (using QProblemB class through C interface).
 */

#include <stdio.h>

#include <qpOASES_wrapper.h>


/** Example for qpOASES main function using the QProblem class. */
int main( )
{
	/* Setup data of first QP. */
	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };

	/* Setup data of second QP. */
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };

	int nWSR;
	qpOASES_Options options;

	real_t xOpt[2];
	real_t yOpt[2];
	real_t obj;
	int status;

	qpOASES_Options_init( &options,0 );
	/*options.enableFlippingBounds = 0; */
	options.initialStatusBounds = ST_INACTIVE;
	options.numRefinementSteps = 1;
	options.enableCholeskyRefactorisation = 1;


	QProblemB_setup( 2,HST_UNKNOWN );

	/* Solve first QP. */
	nWSR = 10;
	QProblemB_init(	H,g,lb,ub,
					(int_t* const)&nWSR,0,&options,
					xOpt,yOpt,&obj,(int_t* const)&status
					);

	/* Print solution of first QP. */	
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1], obj );


	/* Solve second QP. */
	nWSR = 10;
	QProblemB_hotstart(	g_new,lb_new,ub_new,
						(int_t* const)&nWSR,0,
						xOpt,yOpt,&obj,(int_t* const)&status
						);

	/* Print solution of first QP. */	
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1], obj );

	
	QProblemB_cleanup();
	
	return 0;
}


/*
 *	end of file
 */
