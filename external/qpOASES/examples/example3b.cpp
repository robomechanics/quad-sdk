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
 *	\file examples/example3b.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2008-2017
 *
 *	Example demonstrating usage of qpOASES for solving a QP sequence of the
 *	Online QP Benchmark Collection. In order to run it, you have to download
 *	"Example 02" from http://www.qpOASES.org/onlineQP/ and store it into 
 *	the directory bin/chain80/.
 */



#include <qpOASES.hpp>


/** Example for qpOASES main function using the OQP interface. */
int main( )
{
	USING_NAMESPACE_QPOASES

	/* 1) Define benchmark arguments. */
	BooleanType isSparse = BT_FALSE;
	Options options;
	options.setToMPC();
	options.printLevel = PL_NONE;
	
	int_t nWSR = 300;
	real_t maxCPUtime = 10.0; /* seconds */
	real_t maxStationarity, maxFeasibility, maxComplementarity;

	/* 2) Run benchmark. */
	if ( runOqpBenchmark(	"./chain80/",
							isSparse,
							options,
							nWSR,
							maxCPUtime,
							maxStationarity,
							maxFeasibility,
							maxComplementarity
							) != SUCCESSFUL_RETURN )
	{
		myPrintf( "In order to run this example, you need to download example no. 02\nfrom the Online QP Benchmark Collection website first!\n" );
		return -1;
	}

	/* 3) Print results. */
	printf( "\n\n" );
	printf( "OQP Benchmark Results:\n" );
	printf( "======================\n\n" );
	printf( "maximum violation stationarity:     %.3e\n",maxStationarity );
	printf( "maximum violation feasibility:      %.3e\n",maxFeasibility );
	printf( "maximum violation complementarity:  %.3e\n",maxComplementarity );
	printf( "\n" );
	printf( "maximum CPU time:             %.3f milliseconds\n\n",1000.0*maxCPUtime );

	return 0;
}


/*
 *	end of file
 */
