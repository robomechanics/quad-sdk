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
 *	\file testing/cpp/test_smallSchur.cpp
 *	\author Dennis Janka
 *	\version 3.2
 *	\date 2007-2017
 *
 */



#include <qpOASES.hpp>
#include <qpOASES/UnitTesting.hpp>

int main( )
{
	USING_NAMESPACE_QPOASES

	int_t i;
	const int_t m = 200;
	const int_t n = 2*m + 1;

	/* problem data */
	sparse_int_t H_jc[n+1], H_ir[n], A_jc[n+1], A_ir[m];
	real_t H_val[n], A_val[m], g[n], lb[n], ub[n], lbA[m], ubA[m];
	for (i = 0; i < n+1; i++) H_jc[i] = i;
	for (i = 0; i < n; i++) H_ir[i] = i;
	for (i = 0; i < n; i++) H_val[i] = 2.0;
	for (i = 0; i < m; i++) A_jc[i] = i;
	for (i = m; i < n+1; i++) A_jc[i] = m;
	for (i = 0; i < m; i++) A_ir[i] = i;
	for (i = 0; i < m; i++) A_val[i] = 1.0;
	for (i = 0; i < n; i++) g[i] = 0.0;
	for (i = 0; i < n; i++) lb[i] = -1.0;
	for (i = 0; i < n; i++) ub[i] = 1.0;
	for (i = 0; i < m; i++) lbA[i] = 0.5;
	for (i = 0; i < m; i++) ubA[i] = 0.5;

	int_t nWSR;
	real_t errP, errD, tic, toc;
	real_t *xref = new real_t[n];
	real_t *yref = new real_t[n+m];
	real_t *x = new real_t[n];
	real_t *y = new real_t[n+m];

	Options options;
	options.setToDefault();
	options.printLevel = PL_TABULAR;
	options.initialStatusBounds = ST_UPPER;

	/* create sparse matrices */
	SymSparseMat *H = new SymSparseMat(n, n, H_ir, H_jc, H_val);
	SparseMatrix *A = new SparseMatrix(m, n, A_ir, A_jc, A_val);

	H->createDiagInfo();

	/* solve with dense matrices */
	nWSR = 1000;
	QProblem qpD(n, m);
	qpD.setOptions(options);
	tic = getCPUtime();
	qpD.init(H, g, A, lb, ub, lbA, ubA, nWSR, 0);
	toc = getCPUtime();
	qpD.getPrimalSolution(xref);
	qpD.getDualSolution(yref);

	fprintf(stdFile, "Solved problem with dense LA in %d iterations, %.3f seconds.\n", (int)nWSR, toc-tic);

	/* solve with sparse matrices (Schur complement) */
	#ifndef SOLVER_NONE
	nWSR = 1000;
	SQProblemSchur qp(n, m);
	qp.setOptions(options);
	tic = getCPUtime();
	qp.init(H, g, A, lb, ub, lbA, ubA, nWSR, 0);
	toc = getCPUtime();
	qp.getPrimalSolution(x);
	qp.getDualSolution(y);

	fprintf(stdFile, "Solved problem with Schur complement approach in %d iterations, %.3f seconds.\n", (int)nWSR, toc-tic);
	#endif /* SOLVER_NONE */

	/* check distance of solutions */
	errP = 0.0;
	#ifndef SOLVER_NONE
	for (i = 0; i < n; i++)
		if (getAbs(x[i] - xref[i]) > errP)
			errP = getAbs(x[i] - xref[i]);
	#endif /* SOLVER_NONE */
	fprintf(stdFile, "Primal error: %9.2e\n", errP);

	errD = 0.0;
	#ifndef SOLVER_NONE
	for (i = 0; i < n+m; i++)
		if (getAbs(y[i] - yref[i]) > errD)
			errD = getAbs(y[i] - yref[i]);
	#endif /* SOLVER_NONE */
	fprintf(stdFile, "Dual error: %9.2e\n", errD);

	delete H;
	delete A;

	delete[] y;
	delete[] x;
	delete[] yref;
	delete[] xref;

	QPOASES_TEST_FOR_TOL( errP,1e-12 );
	QPOASES_TEST_FOR_TOL( errD,1e-12 );

	return TEST_PASSED;
}


/*
 *	end of file
 */
