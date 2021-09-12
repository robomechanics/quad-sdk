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
 *	\file testing/cpp/test_matrices2.cpp
 *	\author Hans Joachim Ferreau,Andreas Potschka, Christian Kirches
 *	\version 3.2
 *	\date 2014-2017
 *
 *	Unit test for Matrix classes.
 */


#include <stdlib.h>

#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>


#include "test_qrecipe_data.hpp"


/** Compare deviations when performing matrix operations. */
int main( )
{
	USING_NAMESPACE_QPOASES
	
	int_t i;

	real_t errH=0.0, errA=0.0;
	real_t v[180];
	real_t resHs[180];
	real_t resHd[180];
	real_t resAs[91];
	real_t resAd[91];
	
	/* create sparse matrices */
	SymSparseMat *H = new SymSparseMat(180, 180, H_ir, H_jc, H_val);
	SparseMatrix *A = new SparseMatrix(91, 180, A_ir, A_jc, A_val);

	H->createDiagInfo();

	real_t* H_full = H->full();
	real_t* A_full = A->full();

	//print( A_full,91,180 );


	SymDenseMat *Hd = new SymDenseMat(180,180,180,H_full);
	DenseMatrix *Ad = new DenseMatrix(91,180,180,A_full);

	for( i=0; i<180; ++i )
		v[i] = 2.0 * ((real_t)rand()) / ((real_t)RAND_MAX) - 1.0;
	
	H ->times(1, 1.0, v, 180, 0.0, resHs, 180);
	Hd->times(1, 1.0, v, 180, 0.0, resHd, 180);
	
	A ->times(1, 1.0, v, 180, 0.0, resAs, 91);
	Ad->times(1, 1.0, v, 180, 0.0, resAd, 91);

	
	for ( i=0; i<180; ++i )
		if ( getAbs(resHs[i] - resHd[i]) > errH)
			errH = getAbs(resHs[i] - resHd[i]);
	
	fprintf(stdFile, "maximum difference in H*v: %9.2e\n", errH);
	
	
	for ( i=0; i<91; ++i )
		if ( getAbs(resAs[i] - resAd[i]) > errA)
			errA = getAbs(resAs[i] - resAd[i]);
	
	fprintf(stdFile, "maximum difference in A*v: %9.2e\n", errA);

	delete H;
	delete A;
	delete[] H_full;
	delete[] A_full;
	delete Hd;
	delete Ad;

	
	QPOASES_TEST_FOR_TOL( errH,1e-13 )
	QPOASES_TEST_FOR_TOL( errA,1e-13 )

	return TEST_PASSED;
}


/*
 *	end of file
 */
