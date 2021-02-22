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
 *	\file testing/cpp/test_bench.cpp
 *	\author Andreas Potschka, Christian Kirches, Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2010-2017
 *
 *	Unit test running all benchmark examples stored in problems directory.
 */



#include <dirent.h>
#include <cstring>
#include <cstdlib>
#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>


/** Run benchmark examples. */
int main( int argc, char *argv[] )
{
	USING_NAMESPACE_QPOASES
	#ifdef __USE_SINGLE_PRECISION__
	const real_t TOL = 5e-2;
	#else
	const real_t TOL = 1e-5;
	#endif

	/* 1) Define benchmark arguments. */
	BooleanType isSparse = BT_FALSE;
	//BooleanType isSparse = BT_TRUE;
	Options options;
 	options.setToDefault();
	//options.setToMPC();
	//options.setToReliable();
	options.printLevel = PL_LOW;
	//options.printLevel = PL_MEDIUM;
	//options.printLevel = PL_TABULAR;
	//options.enableFarBounds = BT_FALSE;

// 	options.initialStatusBounds = ST_LOWER;
	//options.numRegularisationSteps = 1;
	//options.epsRegularisation = 1.0e3 * EPS;
 	
	//options.enableFlippingBounds = BT_FALSE;
	//options.enableFlippingBounds = BT_FALSE;
	//options.enableRamping = BT_TRUE;
	//options.enableFarBounds = BT_FALSE;
	//options.enableNZCTests = BT_FALSE;
	//options.epsNZCTests = 1.0e4 * EPS;
	//options.epsFlipping = 1.0e5 * EPS;
	//options.enableFullLITests = BT_TRUE;
 	//options.enableDriftCorrection = 1;
 	//options.enableEqualities = BT_TRUE;
	//options.enableEqualities = BT_FALSE;
 	//options.epsNum = -1.0e3 * EPS;
 	//options.epsDen =  1.0e3 * EPS;


	int_t nWSR;
	real_t maxCPUtime; /* seconds */
	real_t maxStationarity = 0.0, maxFeasibility = 0.0, maxComplementarity = 0.0;
	real_t avgStationarity = 0.0, avgFeasibility = 0.0, avgComplementarity = 0.0;

	int_t scannedDir = 0;
	int_t nfail = 0, npass = 0;
	int_t nproblems, i;
	struct dirent **namelist;
	char resstr[MAX_STRING_LENGTH], oqpProblem[MAX_STRING_LENGTH];
	char *problem;
	returnValue returnvalue;

	int_t expectedNumSolvedProblems = 44;
	real_t expectedAvgStationarity    = TOL;
	real_t expectedAvgFeasibility     = TOL;
	real_t expectedAvgComplementarity = TOL;

	
	if ( argv[argc-1][0] == 'O' )
	{
		if ( strlen(argv[argc-1]) != 3 )
		{
			fprintf( stdout,"ERROR (testbench): Invalid options passed!\n" );
			return TEST_DATA_NOT_FOUND;
		}
		
		fprintf( stdout,"Analysing passed options:  " );
		switch ( argv[argc-1][1] )
		{
			case 'd':
				fprintf( stdout,"default options, " );
				options.setToDefault();
				if ( argv[argc-1][2] == 's' )
				{
					expectedNumSolvedProblems  = 44;
					expectedAvgStationarity    = 1e-9;
					expectedAvgFeasibility     = 1e-9;
					expectedAvgComplementarity = 5e-7;
				}
				else
				{
					expectedNumSolvedProblems  = 44;
					expectedAvgStationarity    = 5e-10;
					expectedAvgFeasibility     = 5e-10;
					expectedAvgComplementarity = 5e-8;
				}
				break;
				
			case 'r':
				fprintf( stdout,"reliable options, " );
				options.setToReliable();
				if ( argv[argc-1][2] == 's' )
				{
					expectedNumSolvedProblems  = 44;
					expectedAvgStationarity    = 2e-9;
					expectedAvgFeasibility     = 2e-11;
					expectedAvgComplementarity = 3e-9;
				}
				else
				{
					expectedNumSolvedProblems  = 44;
					expectedAvgStationarity    = 2e-9;
					expectedAvgFeasibility     = 2e-9;
					expectedAvgComplementarity = 3e-7;
				}
				break;
				
			case 'm':
				fprintf( stdout,"MPC options, " );
				options.setToMPC();
				if ( argv[argc-1][2] == 's' )
				{
					expectedNumSolvedProblems  = 42;
					expectedAvgStationarity    = 2e-8;
					expectedAvgFeasibility     = 1e-8;
					expectedAvgComplementarity = 2e-7;
				}
				else
				{
					expectedNumSolvedProblems  = 42;
					expectedAvgStationarity    = 3e-8;
					expectedAvgFeasibility     = 1e-8;
					expectedAvgComplementarity = 5e-8;
				}
				break;
				
			default:
				fprintf( stdout,"ERROR (testbench): Invalid options passed!\n" );
				return TEST_DATA_NOT_FOUND;
		}
		
		switch ( argv[argc-1][2] )
		{
			case 's':
				fprintf( stdout,"sparse QP data\n" );
				isSparse = BT_TRUE;
				break;
				
			case 'd':
				fprintf( stdout,"dense QP data\n" );
				isSparse = BT_FALSE;
				break;
				
			default:
				fprintf( stdout,"ERROR (testbench): Invalid options passed!\n" );
				return TEST_DATA_NOT_FOUND;
		}
		options.printLevel = PL_NONE;
		//options.enableFlippingBounds = BT_FALSE;
		
		nproblems = argc-2;
	}
	else
	{
		nproblems = argc-1;
	}

	
	if (nproblems == 0)
	{
		/* 2a) Scan problem directory */
		nproblems = scandir("../testing/cpp/data/problems", &namelist, NULL, alphasort);
		if (nproblems <= 0)
		{
			myPrintf( "No test problems found!\n" );
			return TEST_DATA_NOT_FOUND;
		}
		scannedDir = 1;
	}
	else
	{
		/* 2b) Use problem list given by arguments */
		scannedDir = 0;
	}

	/* 3) Run benchmark. */
	printf("%10s %9s %9s %9s %6s  %-12s\n", "problem", "stat",
			"feas", "compl", "nWSR", "result");
	for (i = 0; i < nproblems; i++)
	{
		if (scannedDir)
		{
			/* skip special directories and zip file cuter.*bz2 */
			if (namelist[i]->d_name[0] == '.' || namelist[i]->d_name[0] == 'c')
			{
				free(namelist[i]);
				continue;
			}
			problem = namelist[i]->d_name;
		}
		else
		{
			problem = argv[i+1];
		}

		fprintf(stdFile, "%-10s ", problem);
		fflush(stdFile);

		snprintf(oqpProblem, MAX_STRING_LENGTH, "../testing/cpp/data/problems/%s/", problem);
		maxCPUtime = 300.0;
		nWSR = 2500;

		returnvalue = runOqpBenchmark(	oqpProblem, isSparse, options,
										nWSR, maxCPUtime, maxStationarity, maxFeasibility, maxComplementarity 
										);
		if (returnvalue	== SUCCESSFUL_RETURN
				&& maxStationarity < TOL
				&& maxFeasibility < TOL
				&& maxComplementarity < TOL)
		{
			npass++;
			
			avgStationarity    += maxStationarity;
			avgFeasibility     += maxFeasibility;
			avgComplementarity += maxComplementarity;

			strncpy(resstr, "pass", MAX_STRING_LENGTH);
		}
		else
		{
			if ( returnvalue == RET_BENCHMARK_ABORTED )
				return TEST_DATA_NOT_FOUND;

			nfail++;
			snprintf (resstr, MAX_STRING_LENGTH, "fail (%d)",(int)returnvalue);
		}
		fprintf(stdFile, "%9.2e %9.2e %9.2e %6d  %-12s\n", maxStationarity,
				maxFeasibility, maxComplementarity, (int)nWSR, resstr);

		if (scannedDir) free(namelist[i]);
	}
	if (scannedDir) free(namelist);

	avgStationarity    /= (real_t)npass;
	avgFeasibility     /= (real_t)npass;
	avgComplementarity /= (real_t)npass;


	/* 4) Print results. */
	printf( "\n\n" );
	printf( "Testbench results:\n" );
	printf( "======================\n\n" );
	printf( "Pass:  %3d\n",(int)npass );
	printf( "Fail:  %3d\n",(int)nfail );
	printf( "Ratio: %5.1f%%\n", 100.0 * (real_t)npass / (real_t)(npass+nfail) );
	printf( "\n" );

	QPOASES_TEST_FOR_TRUE( npass >= expectedNumSolvedProblems );


	printf( "avg. stat:  %e\n", avgStationarity    );
	printf( "avg. feas:  %e\n", avgFeasibility     );
	printf( "avg. cmpl:  %e\n", avgComplementarity );

	QPOASES_TEST_FOR_TOL( avgStationarity,    expectedAvgStationarity    );
	QPOASES_TEST_FOR_TOL( avgFeasibility,     expectedAvgFeasibility     );
	QPOASES_TEST_FOR_TOL( avgComplementarity, expectedAvgComplementarity );


	return 0;
}


/*
 *	end of file
 */
