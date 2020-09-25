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
 *	\file testing/cpp/test_matrices3.cpp
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

	real_t errH=0.0;
	real_t v[180];
	real_t resHn[180];
	real_t resHt[180];
	
	/* create sparse matrices */
	SymSparseMat *H = new SymSparseMat(180, 180, H_ir, H_jc, H_val);
	
	H->createDiagInfo();

	real_t* H_full = H->full();
	
	SymDenseMat *Hd = new SymDenseMat(180,180,180,H_full);

	for( i=0; i<180; ++i )
		v[i] = 2.0 * ((real_t)rand()) / ((real_t)RAND_MAX) - 1.0;
	
	Hd->times(     1, 1.0, v, 180, 0.0, resHn, 180);
	Hd->transTimes(1, 1.0, v, 180, 0.0, resHt, 180);
	
	for ( i=0; i<180; ++i )
		if ( getAbs(resHn[i] - resHt[i]) > errH)
			errH = getAbs(resHn[i] - resHt[i]);
	
	fprintf(stdFile, "maximum difference in H*v vs. H'*v: %9.2e\n", errH);
	
	delete H;
	delete[] H_full;
	delete Hd;
	
	
	QPOASES_TEST_FOR_TOL( errH,1e-15 )
	
	return TEST_PASSED;
}


/*
 *	end of file
 */
