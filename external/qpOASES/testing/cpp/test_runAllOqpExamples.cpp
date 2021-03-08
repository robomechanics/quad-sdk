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
 *	\file testing/cpp/test_runAllOqpExamples.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2013-2017
 *
 *	Use qpOASES for solving all QP sequences of the Online QP Benchmark 
 *	Collection. In order to run it, you have to download all examples
 *	from http://www.qpOASES.org/onlineQP/.
 */



#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>


/** Example for qpOASES main function using the OQP interface. */
int main( )
{
	USING_NAMESPACE_QPOASES

	/* 1) Define benchmark arguments. */
	BooleanType isSparse = BT_FALSE;
	BooleanType useHotstarts;

	Options options;
	options.setToMPC();
	options.printLevel = PL_LOW;

	int_t maxAllowedNWSR;
	real_t maxNWSR, avgNWSR, maxCPUtime, avgCPUtime;
	real_t maxStationarity, maxFeasibility, maxComplementarity;

	const int_t numBenchmarks = 4; //5
	const char *benchmarkPath[numBenchmarks];
	benchmarkPath[0] = "../testing/cpp/data/oqp/chain80/";
	benchmarkPath[1] = "../testing/cpp/data/oqp/chain80w/";
	benchmarkPath[2] = "../testing/cpp/data/oqp/diesel/";
	benchmarkPath[3] = "../testing/cpp/data/oqp/crane/";
	//benchmarkPath[4] = "../testing/cpp/data/oqp/CDU/";


	/* 2) Run all benchmarks in a loop */
	for ( int_t ii=0; ii<2*numBenchmarks; ++ii )
	{
		if ( ii%2 == 0 )
			useHotstarts = BT_FALSE;
		else
			useHotstarts = BT_TRUE;
		
		maxAllowedNWSR = 1000;
		maxNWSR = 0.0;
		avgNWSR = 0.0;
		maxCPUtime = 1000.0; /* seconds */
		avgCPUtime =    0.0; /* seconds */
		maxStationarity    = 0.0;
		maxFeasibility     = 0.0;
		maxComplementarity = 0.0;

		if ( runOqpBenchmark(	benchmarkPath[ii/2],
								isSparse,useHotstarts,
								options,maxAllowedNWSR,
								maxNWSR,avgNWSR,maxCPUtime,avgCPUtime,
								maxStationarity,maxFeasibility,maxComplementarity
								) != SUCCESSFUL_RETURN )
		{
			myPrintf( "Something went wrong when running benchmark!\n" );
			return TEST_DATA_NOT_FOUND;
		}

		/* 3) Print results. */
		printf( "\n\n" );
		if ( useHotstarts == BT_FALSE )
			printf( "OQP Benchmark Results for %s (cold-starts):\n", benchmarkPath[ii/2] );
		else
			printf( "OQP Benchmark Results for %s (hot-starts):\n", benchmarkPath[ii/2] );

		printf( "===========================================================================\n\n" );
		printf( "maximum CPU time:             %.2f milliseconds\n",1000.0*maxCPUtime );
		printf( "average CPU time:             %.2f milliseconds\n",1000.0*avgCPUtime );
		printf( "\n" );
		printf( "maximum iterations:    %.1f\n",maxNWSR );
		printf( "average iterations:    %.1f\n",avgNWSR );
		printf( "\n" );
		printf( "maximum violation stationarity:     %.3e\n",maxStationarity );
		printf( "maximum violation feasibility:      %.3e\n",maxFeasibility );
		printf( "maximum violation complementarity:  %.3e\n",maxComplementarity );
		printf( "\n" );

		QPOASES_TEST_FOR_TOL( maxStationarity,    1e-9  );
		QPOASES_TEST_FOR_TOL( maxFeasibility,     2e-11 );
		QPOASES_TEST_FOR_TOL( maxComplementarity, 2e-10 );
		
		switch( ii )
		{
			case 0:
				/* chain80 (cold) */
				QPOASES_TEST_FOR_TRUE( maxNWSR <= 62.5 );
				QPOASES_TEST_FOR_TRUE( avgNWSR <=  7.5 );
				break;

			case 1:
				/* chain80 (hot) */
				QPOASES_TEST_FOR_TRUE( maxNWSR <= 19.5 );
				QPOASES_TEST_FOR_TRUE( avgNWSR <=  2.4 );
				break;

			case 2:
				/* chain80w (cold) */
				QPOASES_TEST_FOR_TRUE( maxNWSR <= 84.5 );
				QPOASES_TEST_FOR_TRUE( avgNWSR <= 10.1 );
				break;

			case 3:
				/* chain80w (hot) */
				QPOASES_TEST_FOR_TRUE( maxNWSR <= 16.5 );
				QPOASES_TEST_FOR_TRUE( avgNWSR <=  2.7 );
				break;

			case 4:
				/* diesel (cold) */
				QPOASES_TEST_FOR_TRUE( maxNWSR <= 26.5 );
				QPOASES_TEST_FOR_TRUE( avgNWSR <=  0.5 );
				break;

			case 5:
				/* diesel (hot) */
				QPOASES_TEST_FOR_TRUE( maxNWSR <= 22.5 );
				QPOASES_TEST_FOR_TRUE( avgNWSR <=  0.3 );
				break;

			case 6:
				/* crane (cold) */
				QPOASES_TEST_FOR_TRUE( maxNWSR <= 64.5 );
				QPOASES_TEST_FOR_TRUE( avgNWSR <= 44.0 );
				break;

			case 7:
				/* crane (hot) */
				QPOASES_TEST_FOR_TRUE( maxNWSR <= 42.5 );
				QPOASES_TEST_FOR_TRUE( avgNWSR <=  0.4 );
				break;
		}
	}
	
	return TEST_PASSED;
}


/*
 *	end of file
 */
