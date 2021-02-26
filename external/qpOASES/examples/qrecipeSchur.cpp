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
 *	\file examples/qrecipeSchur.cpp
 *	\author Dennis Janka
 *	\version 3.2
 *	\date 2007-2017
 *
 *	QRECIPE example from the CUTEr test set with sparse matrices.
 *	Comparison between nullspace factorization (dense and sparse) and
 *	Schur complement approach.
 */



#include <qpOASES.hpp>

#include "qrecipe_data.hpp"



int main( )
{
	USING_NAMESPACE_QPOASES

	long i;
	int_t nWSR;
	real_t errP1, errP2, errP3, errD1, errD2, errD3, tic, toc;
	real_t *x1 = new real_t[180];
	real_t *y1 = new real_t[271];
	real_t *x2 = new real_t[180];
	real_t *y2 = new real_t[271];
	real_t *x3 = new real_t[180];
	real_t *y3 = new real_t[271];

	/* create sparse matrices */
	SymSparseMat *H = new SymSparseMat(180, 180, H_ir, H_jc, H_val);
	SparseMatrix *A = new SparseMatrix(91, 180, A_ir, A_jc, A_val);

	H->createDiagInfo();

	real_t* H_full = H->full();
	real_t* A_full = A->full();

	SymDenseMat *Hd = new SymDenseMat(180,180,180,H_full);
	DenseMatrix *Ad = new DenseMatrix(91,180,180,A_full);

	/* solve with dense matrices */
	nWSR = 1000;
	QProblem qrecipeD(180, 91);
	tic = getCPUtime();
	qrecipeD.init(Hd, g, Ad, lb, ub, lbA, ubA, nWSR, 0);
	toc = getCPUtime();
	qrecipeD.getPrimalSolution(x1);
	qrecipeD.getDualSolution(y1);

	fprintf(stdFile, "Solved dense problem in %d iterations, %.3f seconds.\n", (int)nWSR, toc-tic);

	/* solve with sparse matrices (nullspace factorization) */
	nWSR = 1000;
	QProblem qrecipeS(180, 91);
	tic = getCPUtime();
	qrecipeS.init(H, g, A, lb, ub, lbA, ubA, nWSR, 0);
	toc = getCPUtime();
	qrecipeS.getPrimalSolution(x2);
	qrecipeS.getDualSolution(y2);

	fprintf(stdFile, "Solved sparse problem in %d iterations, %.3f seconds.\n", (int)nWSR, toc-tic);

	/* solve with sparse matrices (Schur complement) */
	nWSR = 1000;
	SQProblemSchur qrecipeSchur(180, 91);
	tic = getCPUtime();
	qrecipeSchur.init(H, g, A, lb, ub, lbA, ubA, nWSR, 0);
	toc = getCPUtime();
	qrecipeSchur.getPrimalSolution(x3);
	qrecipeSchur.getDualSolution(y3);

	fprintf(stdFile, "Solved sparse problem (Schur complement approach) in %d iterations, %.3f seconds.\n", (int)nWSR, toc-tic);

	/* check distance of solutions */
	errP1 = 0.0;
	errP2 = 0.0;
	errP3 = 0.0;
	#ifndef SOLVER_NONE
	for (i = 0; i < 180; i++)
		if (getAbs(x1[i] - x2[i]) > errP1)
			errP1 = getAbs(x1[i] - x2[i]);
	for (i = 0; i < 180; i++)
		if (getAbs(x1[i] - x3[i]) > errP2)
			errP2 = getAbs(x1[i] - x3[i]);
	for (i = 0; i < 180; i++)
		if (getAbs(x2[i] - x3[i]) > errP3)
			errP3 = getAbs(x2[i] - x3[i]);
	#endif /* SOLVER_NONE */
	fprintf(stdFile, "Primal error (dense and sparse): %9.2e\n", errP1);
	fprintf(stdFile, "Primal error (dense and Schur):  %9.2e\n", errP2);
	fprintf(stdFile, "Primal error (sparse and Schur): %9.2e\n", errP3);

	errD1 = 0.0;
	errD2 = 0.0;
	errD3 = 0.0;
	for (i = 0; i < 271; i++)
		if (getAbs(y1[i] - y2[i]) > errD1)
			errD1 = getAbs(y1[i] - y2[i]);
	#ifndef SOLVER_NONE
	for (i = 0; i < 271; i++)
		if (getAbs(y1[i] - y3[i]) > errD2)
			errD2 = getAbs(y1[i] - y3[i]);
	for (i = 0; i < 271; i++)
		if (getAbs(y2[i] - y3[i]) > errD3)
			errD3 = getAbs(y2[i] - y3[i]);
	#endif /* SOLVER_NONE */
	fprintf(stdFile, "Dual error (dense and sparse): %9.2e  (might not be unique)\n", errD1);
	fprintf(stdFile, "Dual error (dense and Schur):  %9.2e  (might not be unique)\n", errD2);
	fprintf(stdFile, "Dual error (sparse and Schur): %9.2e  (might not be unique)\n", errD3);

	delete H;
	delete A;
	delete[] H_full;
	delete[] A_full;
	delete Hd;
	delete Ad;

	delete[] y3;
	delete[] x3;
	delete[] y2;
	delete[] x2;
	delete[] y1;
	delete[] x1;

	return 0;
}


/*
 *	end of file
 */
