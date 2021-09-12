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
 *	\file testing/cpp/test_constraintProduct2.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2014-2017
 *
 *	Another example for testing qpOASES using the possibility to specify 
 *	user-defined constraint product function.
 */



#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>



USING_NAMESPACE_QPOASES


/** 
 *	\brief Example illustrating the use of the \a ConstraintProduct class.
 *
 *	Example illustrating the use of the \a ConstraintProduct class.
 *
 *	\author Hans Joachim Ferreau
 *	\version 3.1
 *	\date 2007-2017
 */
class MpcConstraintProduct : public ConstraintProduct
{
	public:
		/** Default constructor. */
		MpcConstraintProduct( ) {};

		/** Constructor. */
		MpcConstraintProduct(	int_t _nV,
								int_t _nC,
								int_t _diagOffset,
								real_t* _A
								)
		{
			nV = _nV;
			nC = _nC;
			diagOffset = _diagOffset;
			A  = _A;
		};

		/** Copy constructor (flat copy). */
		MpcConstraintProduct(	const MpcConstraintProduct& rhs
								)
		{
			nV = rhs.nV;
			nC = rhs.nC;
			diagOffset = rhs.diagOffset;
			A  = rhs.A;
		};

		/** Destructor. */
		virtual ~MpcConstraintProduct( ) {};
		
		/** Assignment operator (flat copy). */
		MpcConstraintProduct& operator=(	const MpcConstraintProduct& rhs
										)
		{
			if ( this != &rhs )
			{
				nV = rhs.nV;
				nC = rhs.nC;
				diagOffset = rhs.diagOffset;
				A  = rhs.A;
			}
			else
				return *this;
		};

		virtual int_t operator() (	int_t constrIndex,
									const real_t* const x,
									real_t* const constrValue
									) const
		{
			int_t i;
			int_t maxI = (int_t)(((real_t)constrIndex) * ((real_t)nV) / ((real_t)nC)) + diagOffset;
			maxI = getMin( maxI,nV );

			constrValue[0] = 0.0;

			for( i=0; i<maxI; ++i )
				constrValue[0] += A[constrIndex*nV + i] * x[i];

			return 0;
		};

	protected:
		int_t nV;			/**< Number of variables. */
		int_t nC;			/**< Number of constraints. */
		int_t diagOffset;	/**< ... */
		real_t* A;			/**< Pointer to full constraint matrix (typically not needed!). */
		
};


/**	Example for qpOASES main function using the possibility to specify 
 *	user-defined constraint product function. */
int main( )
{
	int_t nQP, nV, nC, nEC;
	real_t *H, *g, *A, *lb, *ub, *lbA, *ubA;
	real_t cputime;
	
	real_t xOpt[1000];
	real_t yOpt[1000];
	real_t xOptCP[1000+1000];
	real_t yOptCP[1000+1000];
		
	const char* path = "./cpp/data/oqp/diesel/";
	int_t k = 200; //th problem
	
	
	if ( readOqpDimensions(	path, nQP,nV,nC,nEC ) != SUCCESSFUL_RETURN )
		return TEST_DATA_NOT_FOUND;

	readOqpData(	path, nQP,nV,nC,nEC,
					&H,&g,&A,&lb,&ub,&lbA,&ubA,
					0,0,0
					);
	
	Options myOptions;
	myOptions.setToMPC();
	myOptions.printLevel = PL_LOW;
	
	int_t nWSR = 500;
	cputime = 10.0;
	QProblem qp( nV,nC );
	qp.setOptions( myOptions );
	qp.init( H,&(g[k*nV]),A,&(lb[k*nV]),&(ub[k*nV]),&(lbA[k*nC]),&(ubA[k*nC]),nWSR,&cputime );
	qp.getPrimalSolution( xOpt );
	qp.getDualSolution( yOpt );
	printf( "cputime without constraintProduct: %.3ems\n", cputime*1000.0 );
	
	
	nWSR = 500;
	cputime = 10.0;
	MpcConstraintProduct myCP( nV,nC,1,A );
	QProblem qpCP( nV,nC );
	qpCP.setOptions( myOptions );
	qpCP.setConstraintProduct( &myCP );
	qpCP.init( H,&(g[k*nV]),A,&(lb[k*nV]),&(ub[k*nV]),&(lbA[k*nC]),&(ubA[k*nC]),nWSR,&cputime );
	qpCP.getPrimalSolution( xOptCP );
	qpCP.getDualSolution( yOptCP );
	printf( "cputime without constraintProduct: %.3ems\n", cputime*1000.0 );
	
	delete[] ubA;
	delete[] lbA;
	delete[] ub;
	delete[] lb;
	delete[] A;
	delete[] g;
	delete[] H;
	
	for( int_t ii=0; ii<nV; ++ii )
		QPOASES_TEST_FOR_NEAR( xOptCP[ii],xOpt[ii] );

	for( int_t ii=0; ii<nV+nC; ++ii )
		QPOASES_TEST_FOR_NEAR( yOptCP[ii],yOpt[ii] );

	return TEST_PASSED;
}


/*
 *	end of file
 */
