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
 *	\file testing/cpp/test_hs268.cpp
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

	/* 1) Define benchmark arguments. */
	BooleanType isSparse = BT_FALSE;
	//BooleanType isSparse = BT_TRUE;
	Options options;
 	options.setToDefault();
	//options.setToMPC();
	//options.setToReliable();
	//options.printLevel = PL_LOW;
	//options.printLevel = PL_MEDIUM;
	options.printLevel = PL_TABULAR;


	int_t nWSR;
	int_t npass = 0;
	real_t maxCPUtime; /* seconds */
	real_t maxStationarity = 0.0, maxFeasibility = 0.0, maxComplementarity = 0.0;

	char oqpProblem[MAX_STRING_LENGTH];
	char problem[] = "HS268";
	returnValue returnvalue;

	
	/* 3) Run benchmark. */
	fprintf(stdFile, "%-10s ", problem);
	fflush(stdFile);

	snprintf(oqpProblem, MAX_STRING_LENGTH, "../testing/cpp/data/problems/%s/", problem);
	maxCPUtime = 100.0;
	nWSR = 100;

	returnvalue = runOqpBenchmark(	oqpProblem, isSparse, options,
									nWSR, maxCPUtime, maxStationarity, maxFeasibility, maxComplementarity 
									);

	if(returnvalue == RET_UNABLE_TO_READ_BENCHMARK)
		return TEST_DATA_NOT_FOUND;

	if(returnvalue == SUCCESSFUL_RETURN)
		npass += 1;

	QPOASES_TEST_FOR_TRUE( npass >= 1 );

	printf( "\n" );
	printf( "stat:  %e\n", maxStationarity    );
	printf( "feas:  %e\n", maxFeasibility     );
	printf( "cmpl:  %e\n", maxComplementarity );

	QPOASES_TEST_FOR_TOL( maxStationarity,    1e-11 );
	QPOASES_TEST_FOR_TOL( maxFeasibility,     1e-14 );
	QPOASES_TEST_FOR_TOL( maxComplementarity, 1e-14 );


	return TEST_PASSED;
}


/*
 *	end of file
 */
