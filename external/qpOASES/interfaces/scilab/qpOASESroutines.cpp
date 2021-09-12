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
 *	\file interfaces/scilab/qpOASESroutines.cpp
 *	\author Holger Diedam, Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Interface that enables to call qpOASES from scilab
 *  (C++ file to provide an interface between the files that
 *  have to be compiled with gcc and the qpOASES library).
 *
 */


#include <scilab/Scierror.h>

#include <qpOASES.hpp>


USING_NAMESPACE_QPOASES

/*extern "C" {
#include "../c/qpOASES_wrapper.h"
}*/


/* global pointers to qpOASES objects */
static QProblem*  qp  = 0;
static QProblemB* qpb = 0;
static SQProblem* sqp = 0;


extern "C"
{
	void sci_qpOASES(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
						int_t *nV, int_t* nC, int_t* nWSR,
						real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
						);

	void sci_QProblem_init( 	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
								int_t* nV, int_t* nC, int_t* nWSR,
								real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
								);
	void sci_QProblemB_init(	real_t* H, real_t* g, real_t* lb, real_t* ub,
								int_t* nV, int_t* nWSR,
								real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
								);
	void sci_SQProblem_init(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
								int_t* nV, int_t* nC, int_t* nWSR,
								real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
								);

	void sci_QProblem_hotstart( 	real_t* g, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
									int_t* nWSR,
									real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
									);
	void sci_QProblemB_hotstart(	real_t* g, real_t* lb, real_t* ub,
									int_t* nWSR,
									real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
									);
	void sci_SQProblem_hotstart(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
									int_t* nWSR,
									real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
									);

	void sci_QProblem_cleanup( );
	void sci_QProblemB_cleanup( );
	void sci_SQProblem_cleanup( );
} /* extern "C" */



/*
 *	t r a n s f o r m A
 */
void transformA( real_t* A, int_t nV, int_t nC )
{
	int_t i, j;

	real_t* A_tmp = new real_t[nC*nV];

	for( i=0; i<nV*nC; ++i )
		A_tmp[i] = A[i];

	for( i=0; i<nC; ++i )
		for( j=0; j<nV; ++j )
			A[i*nV + j] = A_tmp[j*nC + i];

	delete[] A_tmp;

	return;
}


/*
 *	q p O A S E S
 */
void sci_qpOASES(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
					int_t *nV, int_t* nC, int_t* nWSR,
					real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
					)
{
	/* transform A into C style matrix */
	transformA( A, *nV,*nC );
	
	/* setup and solve initial QP */
	QProblem single_qp( *nV,*nC );
	single_qp.setPrintLevel( PL_LOW );
	returnValue returnvalue = single_qp.init( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	single_qp.getPrimalSolution( x );
	*obj = single_qp.getObjVal( );
	*status = getSimpleStatus( returnvalue );
	*nWSRout = *nWSR;
	single_qp.getDualSolution( y );

	return;
}


/*
 *	Q P r o b l e m _ i n i t
 */
void sci_QProblem_init( 	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
							int_t* nV, int_t* nC, int_t* nWSR,
							real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
							)
{
	sci_QProblem_cleanup( );

	/* transform A into C style matrix */
	transformA( A, *nV,*nC );

	/* setup and solve initial QP */
	qp = new QProblem( *nV,*nC );
	qp->setPrintLevel( PL_LOW );
	returnValue returnvalue = qp->init( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	qp->getPrimalSolution( x );
	*obj = qp->getObjVal( );
	*status = getSimpleStatus( returnvalue );
	*nWSRout = *nWSR;
	qp->getDualSolution( y );

	return;
}


/*
 *	Q P r o b l e m B _ i n i t
 */
void sci_QProblemB_init(	real_t* H, real_t* g, real_t* lb, real_t* ub,
							int_t* nV, int_t* nWSR,
							real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
							)
{
	sci_QProblemB_cleanup( );

	/* setup and solve initial QP */
	qpb = new QProblemB( *nV );
	qpb->setPrintLevel( PL_LOW );
	returnValue returnvalue = qpb->init( H,g,lb,ub, *nWSR,0 );

	/* assign lhs arguments */
	qpb->getPrimalSolution( x );
	*obj = qpb->getObjVal( );
	*status = getSimpleStatus( returnvalue );
	*nWSRout = *nWSR;
	qpb->getDualSolution( y );

	return;
}


/*
 *	S Q P r o b l e m _ i n i t
 */
void sci_SQProblem_init(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
							int_t* nV, int_t* nC, int_t* nWSR,
							real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
							)
{
	sci_SQProblem_cleanup( );

	/* transform A into C style matrix */
	transformA( A, *nV,*nC );

	/* setup and solve initial QP */
	sqp = new SQProblem( *nV,*nC );
	sqp->setPrintLevel( PL_LOW );
	returnValue returnvalue = sqp->init( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	sqp->getPrimalSolution( x );
	*obj = sqp->getObjVal( );
	*status = getSimpleStatus( returnvalue );
	*nWSRout = *nWSR;
	sqp->getDualSolution( y );

	return;
}


/*
 *	Q P r o b l e m _ h o t s t a r t
 */
void sci_QProblem_hotstart( 	real_t* g, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
								int_t* nWSR,
								real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
								)
{
	/* has QP been initialised? */
	if ( qp == 0 )
	{
		*status = -1;
		Scierror( 999,"ERROR (qpOASES): Need to call qpOASES_init first!\n" );
		return;
	}

	/* solve QP */
	returnValue returnvalue = qp->hotstart( g,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	qp->getPrimalSolution( x );
	*obj = qp->getObjVal( );
	*status = getSimpleStatus( returnvalue );
	*nWSRout = *nWSR;
	qp->getDualSolution( y );

	return;
}


/*
 *	Q P r o b l e m B _ h o t s t a r t
 */
void sci_QProblemB_hotstart(	real_t* g, real_t* lb, real_t* ub,
								int_t* nWSR,
								real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
								)
{
	/* has QP been initialised? */
	if ( qpb == 0 )
	{
		*status = -1;
		Scierror( 999,"ERROR (qpOASES): Need to call qpOASES_initSB first!\n" );
		return;
	}

	/* solve QP */
	returnValue returnvalue = qpb->hotstart( g,lb,ub, *nWSR,0 );

	/* assign lhs arguments */
	qpb->getPrimalSolution( x );
	*obj = qpb->getObjVal( );
	*status = getSimpleStatus( returnvalue );
	*nWSRout = *nWSR;
	qpb->getDualSolution( y );

	return;
}


/*
 *	S Q P r o b l e m _ h o t s t a r t
 */
void sci_SQProblem_hotstart(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
								int_t* nWSR,
								real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
								)
{
	/* has QP been initialised? */
	if ( sqp == 0 )
	{
		*status = -1;
		Scierror( 999,"ERROR (qpOASES): Need to call qpOASES_initVM first!\n" );
		return;
	}

	/* transform A into C style matrix */
	transformA( A, sqp->getNV( ),sqp->getNC( ) );

	/* solve QP */
	returnValue returnvalue = sqp->hotstart( H,g,A,lb,ub,lbA,ubA, *nWSR,0 );

	/* assign lhs arguments */
	sqp->getPrimalSolution( x );
	*obj = sqp->getObjVal( );
	*status = getSimpleStatus( returnvalue );
	*nWSRout = *nWSR;
	sqp->getDualSolution( y );

	return;
}


/*
 *	Q P r o b l e m _ c l e a n u p
 */
void sci_QProblem_cleanup( )
{
	if ( qp != 0 )
	{
		delete qp;
		qp = 0;
	}

	return;
}


/*
 *	Q P r o b l e m B _ c l e a n u p
 */
void sci_QProblemB_cleanup( )
{
	if ( qpb != 0 )
	{
		delete qpb;
		qpb = 0;
	}

	return;
}


/*
 *	S Q P r o b l e m _ c l e a n u p
 */
void sci_SQProblem_cleanup( )
{
	if ( sqp != 0 )
	{
		delete sqp;
		sqp = 0;
	}

	return;
}


/*
 *	end of file
 */
