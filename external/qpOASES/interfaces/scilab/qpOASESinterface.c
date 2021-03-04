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
 *	\file interfaces/scilab/qpOASESinterface.c
 *	\author Holger Diedam, Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Interface that enables to call qpOASES from scilab.
 *  (Please excuse a lot of copy and paste...)
 *
 */


#include <stdio.h>
#include <string.h>

#include <scilab/stack-c.h>
#include <scilab/Scierror.h>

#include "../c/qpOASES_wrapper.h"



extern int interface_qpOASES( char* fname );

extern int interface_QProblem_init(  char* fname );
extern int interface_QProblemB_init( char* fname );
extern int interface_SQProblem_init( char* fname );

extern int interface_QProblem_hotstart(  char* fname );
extern int interface_QProblemB_hotstart( char* fname );
extern int interface_SQProblem_hotstart( char* fname );

extern int interface_QProblem_cleanup(  char* fname );
extern int interface_QProblemB_cleanup( char* fname );
extern int interface_SQProblem_cleanup( char* fname );


typedef int (*gate_function) ( char* );
extern int sci_gateway( char* name, gate_function f );
extern int C2F(qpOASESgateway)();


/* forward declaration of C++ routines */
void sci_qpOASES(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
					int_t *nV, int_t* nC, int_t* nWSR,
					real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
					);

void sci_QProblem_init( real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
						int_t *nV, int_t* nC, int_t* nWSR,
						real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
						);
void sci_QProblemB_init(	real_t* H, real_t* g, real_t* lb, real_t* ub,
							int_t *nV, int_t* nWSR,
							real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
							);
void sci_SQProblem_init(	real_t* H, real_t* g, real_t* A, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
							int_t *nV, int_t* nC, int_t* nWSR,
							real_t* x, real_t* obj, int_t* status, int_t* nWSRout, real_t* y
							);

void sci_QProblem_hotstart(		real_t* g, real_t* lb, real_t* ub, real_t* lbA, real_t* ubA,
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


/* global variables containing dimensions of matrices
 * (also used to check whether qpOASES object were initialised) */
static int_t qp_rowsH = -1;
static int_t qp_rowsA = -1;
static int_t qpb_rowsH = -1;
static int_t sqp_rowsH = -1;
static int_t sqp_rowsA = -1;


/*
 *	i n t e r f a c e _ q p O A S E S
 */
int interface_qpOASES( char* fname )
{
	int_t H, H_rows, H_cols;
	int_t g, g_rows, g_cols;
	int_t A, A_rows, A_cols;
	int_t lb, lb_rows, lb_cols;
	int_t ub, ub_rows, ub_cols;
	int_t lbA, lbA_rows, lbA_cols;
	int_t ubA, ubA_rows, ubA_cols;
	int_t nWSR, nWSR_rows, nWSR_cols;

	int_t x, obj, status, nWSRout, y;


	int minlhs = 1, maxlhs = 5, minrhs = 8, maxrhs = 8, one = 1, y_size;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	/* check dimensions */
	GetRhsVar( 1,"d", &H_rows,&H_cols,&H );
	if ( ( H_rows != H_cols ) || ( H_rows < 1 ) )
	{
		Scierror( 111,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 2,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == H_rows ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == H_rows ) ) ) )
	{
		Scierror( 112,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 3,"d", &A_rows,&A_cols,&A );
	if ( ( A_cols != H_rows ) || ( A_rows < 1 ) )
	{
		Scierror( 113,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 4,"d", &lb_rows,&lb_cols,&lb);
	if ( !( ( ( lb_rows == H_rows ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		Scierror( 114,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 5,"d", &ub_rows,&ub_cols,&ub);
	if ( !( ( ( ub_rows == H_rows ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		Scierror( 115,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 6,"d", &lbA_rows,&lbA_cols,&lbA);
	if ( !( ( ( lbA_rows == A_rows ) && ( lbA_cols == 1 ) ) || ( ( lbA_rows == 0 ) && ( lbA_cols == 0 ) ) ) )
	{
		Scierror( 116,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 7,"d", &ubA_rows,&ubA_cols,&ubA);
	if ( !( ( ( ubA_rows == A_rows ) && ( ubA_cols == 1 ) ) || ( ( ubA_rows == 0 ) && ( ubA_cols == 0 ) ) ) )
	{
		Scierror( 117,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 8,"i", &nWSR_rows,&nWSR_cols,&nWSR);
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		Scierror( 118,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}


	y_size = H_rows + A_rows;

	CreateVar(  9,"d", &H_rows,&one,&x );
	CreateVar( 10,"d", &one,&one,&obj );
	CreateVar( 11,"i", &one,&one,&status );
	CreateVar( 12,"i", &one,&one,&nWSRout );
	CreateVar( 13,"d", &y_size,&one,&y );


	/* call interfaced qpOASES routines with appropriate arguments */
	sci_qpOASES(	stk(H),stk(g),stk(A), (lb_rows!=0) ? stk(lb) : 0, (ub_rows!=0) ? stk(ub) : 0, (lbA_rows!=0) ? stk(lbA) : 0, (ubA_rows!=0) ? stk(ubA) : 0,
					&H_rows,&A_rows,istk(nWSR),
					stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
					);

	LhsVar(1) = 9;
	LhsVar(2) = 10;
	LhsVar(3) = 11;
	LhsVar(4) = 12;
	LhsVar(5) = 13;

	return 0;
}


/*
 *	i n t e r f a c e _ Q P r o b l e m _ i n i t
 */
int interface_QProblem_init( char* fname )
{
	int_t H, H_rows, H_cols;
	int_t g, g_rows, g_cols;
	int_t A, A_rows, A_cols;
	int_t lb, lb_rows, lb_cols;
	int_t ub, ub_rows, ub_cols;
	int_t lbA, lbA_rows, lbA_cols;
	int_t ubA, ubA_rows, ubA_cols;
	int_t nWSR, nWSR_rows, nWSR_cols;

	int_t x, obj, status, nWSRout, y;


	int minlhs = 1, maxlhs = 5, minrhs = 8, maxrhs = 8, one = 1, y_size;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	/* check dimensions */
	GetRhsVar( 1,"d", &H_rows,&H_cols,&H );
	if ( ( H_rows != H_cols ) || ( H_rows < 1 ) )
	{
		Scierror( 211,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 2,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == H_rows ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == H_rows ) ) ) )
	{
		Scierror( 212,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 3,"d", &A_rows,&A_cols,&A );
	if ( ( A_cols != H_rows ) || ( A_rows < 1 ) )
	{
		Scierror( 213,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 4,"d", &lb_rows,&lb_cols,&lb);
	if ( !( ( ( lb_rows == H_rows ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		Scierror( 214,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 5,"d", &ub_rows,&ub_cols,&ub);
	if ( !( ( ( ub_rows == H_rows ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		Scierror( 215,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 6,"d", &lbA_rows,&lbA_cols,&lbA);
	if ( !( ( ( lbA_rows == A_rows ) && ( lbA_cols == 1 ) ) || ( ( lbA_rows == 0 ) && ( lbA_cols == 0 ) ) ) )
	{
		Scierror( 216,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 7,"d", &ubA_rows,&ubA_cols,&ubA);
	if ( !( ( ( ubA_rows == A_rows ) && ( ubA_cols == 1 ) ) || ( ( ubA_rows == 0 ) && ( ubA_cols == 0 ) ) ) )
	{
		Scierror( 217,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 8,"i", &nWSR_rows,&nWSR_cols,&nWSR);
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		Scierror( 218,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}


	y_size = H_rows + A_rows;

	CreateVar(  9,"d", &H_rows,&one,&x );
	CreateVar( 10,"d", &one,&one,&obj );
	CreateVar( 11,"i", &one,&one,&status );
	CreateVar( 12,"i", &one,&one,&nWSRout );
	CreateVar( 13,"d", &y_size,&one,&y );


	qp_rowsH = H_rows;
	qp_rowsA = A_rows;


	/* call interfaced qpOASES routines with appropriate arguments */
	sci_QProblem_init(	stk(H),stk(g),stk(A), (lb_rows!=0) ? stk(lb) : 0, (ub_rows!=0) ? stk(ub) : 0, (lbA_rows!=0) ? stk(lbA) : 0, (ubA_rows!=0) ? stk(ubA) : 0,
						&H_rows,&A_rows,istk(nWSR),
						stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
						);

	LhsVar(1) = 9;
	LhsVar(2) = 10;
	LhsVar(3) = 11;
	LhsVar(4) = 12;
	LhsVar(5) = 13;

	return 0;
}


/*
 *	i n t e r f a c e _ Q P r o b l e m B _ i n i t
 */
int interface_QProblemB_init( char* fname )
{
	int_t H, H_rows, H_cols;
	int_t g, g_rows, g_cols;
	int_t lb, lb_rows, lb_cols;
	int_t ub, ub_rows, ub_cols;
	int_t nWSR, nWSR_rows, nWSR_cols;

	int_t x, obj, status, nWSRout, y;


	int minlhs = 1, maxlhs = 5, minrhs = 5, maxrhs = 5, one = 1;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	/* check dimensions */
	GetRhsVar( 1,"d", &H_rows,&H_cols,&H );
	if ( ( H_rows != H_cols ) || ( H_rows < 1 ) )
	{
		Scierror( 221,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 2,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == H_rows ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == H_rows ) ) ) )
	{
		Scierror( 222,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 3,"d", &lb_rows,&lb_cols,&lb);
	if ( !( ( ( lb_rows == H_rows ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		Scierror( 223,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 4,"d", &ub_rows,&ub_cols,&ub);
	if ( !( ( ( ub_rows == H_rows ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		Scierror( 224,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 5,"i", &nWSR_rows,&nWSR_cols,&nWSR);
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		Scierror( 225,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}


	CreateVar(  9,"d", &H_rows,&one,&x );
	CreateVar( 10,"d", &one,&one,&obj );
	CreateVar( 11,"i", &one,&one,&status );
	CreateVar( 12,"i", &one,&one,&nWSRout );
	CreateVar( 13,"d", &H_rows,&one,&y );


	qpb_rowsH = H_rows;


	/* call interfaced qpOASES routines with appropriate arguments */
	sci_QProblemB_init( 	stk(H),stk(g), (lb_rows!=0) ? stk(lb) : 0, (ub_rows!=0) ? stk(ub) : 0,
							&H_rows,istk(nWSR),
							stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
							);

	LhsVar(1) = 9;
	LhsVar(2) = 10;
	LhsVar(3) = 11;
	LhsVar(4) = 12;
	LhsVar(5) = 13;

	return 0;
}


/*
 *	i n t e r f a c e _ S Q P r o b l e m _ i n i t
 */
int interface_SQProblem_init( char* fname )
{
	int_t H, H_rows, H_cols;
	int_t g, g_rows, g_cols;
	int_t A, A_rows, A_cols;
	int_t lb, lb_rows, lb_cols;
	int_t ub, ub_rows, ub_cols;
	int_t lbA, lbA_rows, lbA_cols;
	int_t ubA, ubA_rows, ubA_cols;
	int_t nWSR, nWSR_rows, nWSR_cols;

	int_t x, obj, status, nWSRout, y;


	int minlhs = 1, maxlhs = 5, minrhs = 8, maxrhs = 8, one = 1, y_size;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	/* check dimensions */
	GetRhsVar( 1,"d", &H_rows,&H_cols,&H );
	if ( ( H_rows != H_cols ) || ( H_rows < 1 ) )
	{
		Scierror( 231,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 2,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == H_rows ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == H_rows ) ) ) )
	{
		Scierror( 232,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 3,"d", &A_rows,&A_cols,&A );
	if ( ( A_cols != H_rows ) || ( A_rows < 1 ) )
	{
		Scierror( 233,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 4,"d", &lb_rows,&lb_cols,&lb );
	if ( !( ( ( lb_rows == H_rows ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		Scierror( 234,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 5,"d", &ub_rows,&ub_cols,&ub );
	if ( !( ( ( ub_rows == H_rows ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		Scierror( 235,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 6,"d", &lbA_rows,&lbA_cols,&lbA );
	if ( !( ( ( lbA_rows == A_rows ) && ( lbA_cols == 1 ) ) || ( ( lbA_rows == 0 ) && ( lbA_cols == 0 ) ) ) )
	{
		Scierror( 236,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 7,"d", &ubA_rows,&ubA_cols,&ubA );
	if ( !( ( ( ubA_rows == A_rows ) && ( ubA_cols == 1 ) ) || ( ( ubA_rows == 0 ) && ( ubA_cols == 0 ) ) ) )
	{
		Scierror( 237,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 8,"i", &nWSR_rows,&nWSR_cols,&nWSR) ;
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		Scierror( 238,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}


	y_size = H_rows + A_rows;

	CreateVar(  9,"d", &H_rows,&one,&x );
	CreateVar( 10,"d", &one,&one,&obj );
	CreateVar( 11,"i", &one,&one,&status );
	CreateVar( 12,"i", &one,&one,&nWSRout );
	CreateVar( 13,"d", &y_size,&one,&y );


	sqp_rowsH = H_rows;
	sqp_rowsA = A_rows;


	/* call interfaced qpOASES routines with appropriate arguments */
	sci_SQProblem_init( 	stk(H),stk(g),stk(A), (lb_rows!=0) ? stk(lb) : 0, (ub_rows!=0) ? stk(ub) : 0, (lbA_rows!=0) ? stk(lbA) : 0, (ubA_rows!=0) ? stk(ubA) : 0,
							&H_rows,&A_rows,istk(nWSR),
							stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
							);
	
	LhsVar(1) = 9;
	LhsVar(2) = 10;
	LhsVar(3) = 11;
	LhsVar(4) = 12;
	LhsVar(5) = 13;

	return 0;
}


/*
 *	i n t e r f a c e _ Q P r o b l e m _ h o t s t a r t
 */
int interface_QProblem_hotstart( char* fname )
{
	int_t g, g_rows, g_cols;
	int_t lb, lb_rows, lb_cols;
	int_t ub, ub_rows, ub_cols;
	int_t lbA, lbA_rows, lbA_cols;
	int_t ubA, ubA_rows, ubA_cols;
	int_t nWSR, nWSR_rows, nWSR_cols;

	int_t x, obj, status, nWSRout, y;


	int minlhs = 1, maxlhs = 5, minrhs = 6, maxrhs = 6, one = 1, y_size;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	if ( ( qp_rowsH == -1 ) || ( qp_rowsA == -1 ) )
	{
		Scierror( 311,"ERROR (qpOASES): Need to call qpOASES_init first!\n" );
		return 0;
	}

	/* check dimensions */
	GetRhsVar( 1,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == qp_rowsH ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == qp_rowsH ) ) ) )
	{
		Scierror( 312,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 2,"d", &lb_rows,&lb_cols,&lb );
	if ( !( ( ( lb_rows == qp_rowsH ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		Scierror( 313,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 3,"d", &ub_rows,&ub_cols,&ub );
	if ( !( ( ( ub_rows == qp_rowsH ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		Scierror( 314,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 4,"d", &lbA_rows,&lbA_cols,&lbA );
	if ( !( ( ( lbA_rows == qp_rowsA ) && ( lbA_cols == 1 ) ) || ( ( lbA_rows == 0 ) && ( lbA_cols == 0 ) ) ) )
	{
		Scierror( 315,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 5,"d", &ubA_rows,&ubA_cols,&ubA );
	if ( !( ( ( ubA_rows == qp_rowsA ) && ( ubA_cols == 1 ) ) || ( ( ubA_rows == 0 ) && ( ubA_cols == 0 ) ) ) )
	{
		Scierror( 316,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 6,"i", &nWSR_rows,&nWSR_cols,&nWSR );
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		Scierror( 317,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}


	y_size = qp_rowsH + qp_rowsA;

	CreateVar(  7,"d", &qp_rowsH,&one,&x );
	CreateVar(  8,"d", &one,&one,&obj );
	CreateVar(  9,"i", &one,&one,&status );
	CreateVar( 10,"i", &one,&one,&nWSRout );
	CreateVar( 11,"d", &y_size,&one,&y );


	/* call interfaced qpOASES routines with appropriate arguments */
	sci_QProblem_hotstart( 	stk(g), (lb_rows!=0) ? stk(lb) : 0, (ub_rows!=0) ? stk(ub) : 0, (lbA_rows!=0) ? stk(lbA) : 0, (ubA_rows!=0) ? stk(ubA) : 0,
							istk(nWSR),
							stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
							);
	
	LhsVar(1) = 7;
	LhsVar(2) = 8;
	LhsVar(3) = 9;
	LhsVar(4) = 10;
	LhsVar(5) = 11;

	return 0;
}


/*
 *	i n t e r f a c e _ Q P r o b l e m B _ h o t s t a r t
 */
int interface_QProblemB_hotstart( char* fname )
{
	int_t g, g_rows, g_cols;
	int_t lb, lb_rows, lb_cols;
	int_t ub, ub_rows, ub_cols;
	int_t nWSR, nWSR_rows, nWSR_cols;

	int_t x, obj, status, nWSRout, y;


	int minlhs = 1, maxlhs = 5, minrhs = 4, maxrhs = 4, one = 1;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	if ( qpb_rowsH == -1 )
	{
		Scierror( 321,"ERROR (qpOASES): Need to call qpOASES_initSB first!\n" );
		return 0;
	}

	/* check dimensions */
	GetRhsVar( 1,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == qpb_rowsH ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == qpb_rowsH ) ) ) )
	{
		Scierror( 322,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 2,"d", &lb_rows,&lb_cols,&lb );
	if ( !( ( ( lb_rows == qpb_rowsH ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		Scierror( 323,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 3,"d", &ub_rows,&ub_cols,&ub );
	if ( !( ( ( ub_rows == qpb_rowsH ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		Scierror( 324,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 4,"i", &nWSR_rows,&nWSR_cols,&nWSR );
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		Scierror( 325,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}


	CreateVar( 5,"d", &qpb_rowsH,&one,&x );
	CreateVar( 6,"d", &one,&one,&obj );
	CreateVar( 7,"i", &one,&one,&status );
	CreateVar( 8,"i", &one,&one,&nWSRout );
	CreateVar( 9,"d", &qpb_rowsH,&one,&y );


	/* call interfaced qpOASES routines with appropriate arguments */
	sci_QProblemB_hotstart( 	stk(g), (lb_rows!=0) ? stk(lb) : 0, (ub_rows!=0) ? stk(ub) : 0,
								istk(nWSR),
								stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
								);

	LhsVar(1) = 5;
	LhsVar(2) = 6;
	LhsVar(3) = 7;
	LhsVar(4) = 8;
	LhsVar(5) = 9;

	return 0;
}


/*
 *	i n t e r f a c e _ S Q P r o b l e m _ h o t s t a r t
 */
int interface_SQProblem_hotstart( char* fname )
{
	int_t H, H_rows, H_cols;
	int_t g, g_rows, g_cols;
	int_t A, A_rows, A_cols;
	int_t lb, lb_rows, lb_cols;
	int_t ub, ub_rows, ub_cols;
	int_t lbA, lbA_rows, lbA_cols;
	int_t ubA, ubA_rows, ubA_cols;
	int_t nWSR, nWSR_rows, nWSR_cols;

	int_t obj, x, y, status, nWSRout;


	int minlhs = 1, maxlhs = 5, minrhs = 8, maxrhs = 8, one = 1, y_size;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );


	if ( ( sqp_rowsH == -1 ) || ( sqp_rowsA == -1 ) )
	{
		Scierror( 331,"ERROR (qpOASES): Need to call qpOASES_initVM first!\n" );
		return 0;
	}

	/* check dimensions */
	GetRhsVar( 1,"d", &H_rows,&H_cols,&H );
	if ( ( H_rows != H_cols ) || ( H_rows < 1 ) )
	{
		Scierror( 332,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 2,"d", &g_rows,&g_cols,&g );
	if ( !( ( ( g_rows == H_rows ) && ( g_cols == 1 ) ) || ( ( g_rows == 1 ) && ( g_cols == H_rows ) ) ) )
	{
		Scierror( 333,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 3,"d", &A_rows,&A_cols,&A );
	if ( ( A_cols != H_rows ) || ( A_rows < 1 ) )
	{
		Scierror( 334,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 4,"d", &lb_rows,&lb_cols,&lb);
	if ( !( ( ( lb_rows == H_rows ) && ( lb_cols == 1 ) ) || ( ( lb_rows == 0 ) && ( lb_cols == 0 ) ) ) )
	{
		Scierror( 335,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 5,"d", &ub_rows,&ub_cols,&ub);
	if ( !( ( ( ub_rows == H_rows ) && ( ub_cols == 1 ) ) || ( ( ub_rows == 0 ) && ( ub_cols == 0 ) ) ) )
	{
		Scierror( 399,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 6,"d", &lbA_rows,&lbA_cols,&lbA);
	if ( !( ( ( lbA_rows == A_rows ) && ( lbA_cols == 1 ) ) || ( ( lbA_rows == 0 ) && ( lbA_cols == 0 ) ) ) )
	{
		Scierror( 336,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 7,"d", &ubA_rows,&ubA_cols,&ubA);
	if ( !( ( ( ubA_rows == A_rows ) && ( ubA_cols == 1 ) ) || ( ( ubA_rows == 0 ) && ( ubA_cols == 0 ) ) ) )
	{
		Scierror( 337,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	GetRhsVar( 8,"i", &nWSR_rows,&nWSR_cols,&nWSR);
	if ( ( nWSR_rows != nWSR_cols ) || ( nWSR_cols != 1 ) )
	{
		Scierror( 338,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}

	/* have matrices same dimension as last QP? */
	if ( ( sqp_rowsH != H_rows ) || ( sqp_rowsA != A_rows ) )
	{
		Scierror( 339,"ERROR (qpOASES): Dimension mismatch!\n" );
		return 0;
	}


	y_size = H_rows + A_rows;

	CreateVar(  9,"d", &H_rows,&one,&x );
	CreateVar( 10,"d", &one,&one,&obj );
	CreateVar( 11,"i", &one,&one,&status );
	CreateVar( 12,"i", &one,&one,&nWSRout );
	CreateVar( 13,"d", &y_size,&one,&y );


	/* call interfaced qpOASES routines with appropriate arguments */
	sci_SQProblem_hotstart( 	stk(H),stk(g),stk(A), (lb_rows!=0) ? stk(lb) : 0, (ub_rows!=0) ? stk(ub) : 0, (lbA_rows!=0) ? stk(lbA) : 0, (ubA_rows!=0) ? stk(ubA) : 0,
								istk(nWSR),
								stk(x),stk(obj),istk(status),istk(nWSRout),stk(y)
								);

	LhsVar(1) = 9;
	LhsVar(2) = 10;
	LhsVar(3) = 11;
	LhsVar(4) = 12;
	LhsVar(5) = 13;

	return 0;
}


/*
 *	i n t e r f a c e _ Q P r o b l e m _ c l e a n u p
 */
int interface_QProblem_cleanup( char* fname )
{
	const int minlhs = 0, maxlhs = 1, minrhs = 0, maxrhs = 0;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );

	sci_QProblem_cleanup( );
	qp_rowsH = -1;
	qp_rowsA = -1;

	return 0;
}


/*
 *	i n t e r f a c e _ Q P r o b l e m B _ c l e a n u p
 */
int interface_QProblemB_cleanup( char* fname )
{
	const int minlhs = 0, maxlhs = 1, minrhs = 0, maxrhs = 0;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );

	sci_QProblemB_cleanup( );
	qpb_rowsH = -1;

	return 0;
}


/*
 *	i n t e r f a c e _ S Q P r o b l e m _ c l e a n u p
 */
int interface_SQProblem_cleanup( char* fname )
{
	const int minlhs = 0, maxlhs = 1, minrhs = 0, maxrhs = 0;

	CheckRhs( minrhs,maxrhs );
	CheckLhs( minlhs,maxlhs );

	sci_SQProblem_cleanup( );
	sqp_rowsH = -1;
	sqp_rowsA = -1;

	return 0;
}


/*
 *	q p O A S E S g a t e w a y
 */
int C2F(qpOASESgateway)( )
{
	gate_function function[] = {	interface_qpOASES,
									interface_QProblem_init,     interface_QProblemB_init,     interface_SQProblem_init,
									interface_QProblem_hotstart, interface_QProblemB_hotstart, interface_SQProblem_hotstart,
									interface_QProblem_cleanup,  interface_QProblemB_cleanup,  interface_SQProblem_cleanup
									};
	char* name[] = {	"qpOASES",
						"qpOASES_init",     "qpOASES_initSB",     "qpOASES_initVM",
						"qpOASES_hotstart", "qpOASES_hotstartSB", "qpOASES_hotstartVM",
						"qpOASES_cleanup",  "qpOASES_cleanupSB",  "qpOASES_cleanupVM"
						};

	Rhs = Max( 0,Rhs );
	sci_gateway( name[Fin-1],function[Fin-1] );

	return 0;
}


/*
 *	end of file
 */
