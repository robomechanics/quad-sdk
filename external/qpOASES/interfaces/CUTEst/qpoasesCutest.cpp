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
 *	\file interfaces/CUTEst/qpoasesCutest.cpp
 *	\author Dennis Janka
 *	\version 3.2
 *	\date 2015-2017
 *
 *	Solve the CUTEst problem in prob/ subdirectory
 */

/* Choose one or more methods to solve the QP */
#define SOLVE_DENSE  0          /* Standard qpOASES, matrices passed dense */
#define SOLVE_SPARSE 0          /* Standard qpOASES, matrices passed sparse */
#define SOLVE_SCHUR  1          /* Schur complement qpOASES */


#include <qpOASES.hpp>
#include <sys/time.h>

extern "C" {   /* To prevent C++ compilers from mangling symbols */
#include <cutest.h>
}


int convertTripletToHbf( int m, int n, int nnz,
                         double *vals, int *iRow, int *jCol,
                         double *vals_hbf, int *iRow_hbf, int *jCol_hbf )
{
    int i, ii, j, k;

    /* Initialize output arrays */
    for( k=0; k<n+1; k++ )
        jCol_hbf[k] = 0;
    for( k=0; k<nnz; k++ )
    {
        iRow_hbf[k] = 0;
        vals_hbf[k] = 0.0;
    }

    /* Count elements for each column */
    for( k=0; k<nnz; k++ )
    {
        j = jCol[k]-1; // Convert Fortran to C indices!
        jCol_hbf[j]++;
    }

    /* Set the column pointers */
    jCol_hbf[n] = nnz;
    for( k=n-1; k>-1; k-- )
        jCol_hbf[k] = jCol_hbf[k+1] - jCol_hbf[k];

    /* Put row indices and values at the right places, use jCol_hbf[j] to track elements of the jth column */
    for( k=0; k<nnz; k++ )
    {
        i = iRow[k]-1; // Convert Fortran to C indices!
        j = jCol[k]-1;
        ii = jCol_hbf[j];

        vals_hbf[ii] = vals[k];
        iRow_hbf[ii] = i;
        jCol_hbf[j] = ii + 1;
    }

    /* Shift all column pointers back again */
    for( k=n-1; k>0; k-- )
        jCol_hbf[k] = jCol_hbf[k-1];
    jCol_hbf[0] = 0;

    return 0;
}



int main( )
{
    /*
     * PART I: Extract CUTEst problemdata:
     *
     * - problem dimension
     * - variable and constraint bounds
     * - objective gradient
     * - sparse and dense Jacobian
     * - sparse and dense Hessian
     * - initial values for x and y
     */

    /* From CUTEst generic c package */
    char *fname = "prob/OUTSDIF.d"; /* CUTEst data file */
    int funit = 42;                 /* FORTRAN unit number for OUTSDIF.d */
    int iout = 6;                   /* FORTRAN unit number for error output */
    int io_buffer = 11;             /* FORTRAN unit internal input/output */
    int ierr;                       /* Exit flag from OPEN and CLOSE */
    int status;                     /* Exit flag from CUTEst tools */
    int e_order = 0, l_order = 0, v_order = 0;


    /* Problem description: the following variables will be set for qpOASES call */
    int nVar;                       /* number of variables */
    int nCon;                       /* number of constraints */
    double *lb, *ub;                /* lower and upper bounds for variables */
    double *lbA, *ubA;              /* lower and upper bounds for constraints */
    double *g;                      /* objective gradient */
#if SOLVE_DENSE
    double *hesDense, *jacDense;    /* dense Hessian and Jacobian */
#endif
#if SOLVE_SCHUR || SOLVE_SPARSE
                                    /* Sparse Hessian and Jacobian in Harwell-Boeing format: */
    int nnzj, nnzh;                 /* number of nonzero elements in Jacobian and Hessian */
    double *hesVal, *jacVal;        /* nonzero elements of Hessian and Jacobian */
    int *hesIndCol, *hesIndRow;     /* column and row indices of Hessian */
    int *jacIndCol, *jacIndRow;     /* column and row indices of Jacobian */
#endif

    /* Additional problem description: these variables are set and may also be used by qpOASES */
    double obj;                     /* constant objective offset */
    double *xInit;                  /* initial values for primal variables */
    double *yInit;                  /* initial values for dual variables */
    logical *isEq;                  /* logical array to mark equality constraints */


    /* Open problem description file OUTSDIF.d */
    ierr = 0;
    FORTRAN_open( &funit, fname, &ierr );
    if( ierr ) {
        printf("Error opening file OUTSDIF.d.\nAborting.\n");
        exit(1);
    }

    /* Determine problem size */
    CUTEST_cdimen( &status, &funit, &nVar, &nCon );
    if( status ) {
        printf("** CUTEst error, status = %d, aborting\n", status);
        exit(status);
    }

    /* Reserve memory for variables, bounds, and multipliers */
    /* and call appropriate initialization routine for CUTEst */
    logical *isLinear;
    xInit = new double[nVar];
    yInit = new double[nCon];
    lb = new double[nVar];
    ub = new double[nVar];
    lbA = new double[nCon];
    ubA = new double[nCon];
    isEq = new logical[nCon];
    isLinear = new logical[nCon];

    CUTEST_csetup( &status, &funit, &iout, &io_buffer,
                    &nVar, &nCon, xInit, lb, ub,
                    yInit, lbA, ubA, isEq, isLinear,
                    &e_order, &l_order, &v_order );
    if( status ) {
        printf("** CUTEst_csetup: error, status = %d, aborting\n", status);
        exit(status);
    }

    /* Evaluate gradient and objective at x=0 to get g */
    logical grad = 1;
    g = new double[nVar];
    double *xZero = new double[nVar];
    for( int k=0; k<nVar; k++ ) xZero[k] = 0.0;
    CUTEST_cofg( &status, &nVar, xZero, &obj, g, &grad );
    if( status ) {
        printf("** CUTEst_cofg: error, status = %d, aborting\n", status);
        exit(status);
    }

    double *cVal = new double[nCon];
#if SOLVE_SPARSE || SOLVE_SCHUR
    /* Evaluate sparse Jacobian in triplet format */
    int lj = nCon*nVar; // overestimate
    double *J_val = new double[lj];
    int *J_var = new int[lj];
    int *J_fun = new int[lj];

    if( nCon > 0 )
    {
        /* Evaluate sparse constraints at x=0 to get the rhs of equality constraints (they are not set by csetup!) */
        CUTEST_ccfsg( &status, &nVar, &nCon, xZero, cVal, &nnzj, &lj, J_val, J_var, J_fun, &grad );
        if( status ) {
            printf("** CUTEst_ccfsg: error, status = %d, aborting\n", status);
            exit(status);
        }

        for( int k=0; k<nCon; k++ )
            if( isEq[k] )
                lbA[k] = ubA[k] = -cVal[k];

        jacIndCol = new int[nVar+1];
        jacIndRow = new int[nnzj];
        jacVal = new double[nnzj];
        convertTripletToHbf( nCon, nVar, nnzj, J_val, J_fun, J_var, jacVal, jacIndRow, jacIndCol );

    }
    delete[] J_val;
    delete[] J_fun;
    delete[] J_var;

    /* Evaluate sparse Hessian in triplet format */
    int lh = nVar*nVar; // overestimate
    double *H_val = new double[lh];
    int *H_col = new int[lh];
    int *H_row = new int[lh];
    CUTEST_csh( &status, &nVar, &nCon, xInit, yInit, &nnzh, &lh, H_val, H_row, H_col );
    if( status ) {
        printf("** CUTEst_csh: error, status = %d, aborting\n", status);
        exit(status);
    }
    /* Only upper triangular matrix is returned by CUTEst, to be safe, set H to be the full matrix */
    int count = 0;
    for( int k=0; k<nnzh; k++ )
        if( H_row[k] != H_col[k] )
        {
            H_val[nnzh+count] = H_val[k];
            H_col[nnzh+count] = H_row[k];
            H_row[nnzh+count] = H_col[k];
            count++;
        }
    nnzh = nnzh + count;

    hesIndCol = new int[nVar+1];
    hesIndRow = new int[nnzh];
    hesVal = new double[nnzh];
    convertTripletToHbf( nVar, nVar, nnzh, H_val, H_row, H_col, hesVal, hesIndRow, hesIndCol );

    delete[] H_val;
    delete[] H_row;
    delete[] H_col;
#endif

#if SOLVE_DENSE
    if( nCon > 0 )
    {
        /* Evaluate dense constraints at x=0 to get the rhs of equality constraints (they are not set by csetup!) */
        logical trans = 1;
        jacDense = new double[nCon*nVar];
        CUTEST_ccfg( &status, &nVar, &nCon, xZero, cVal, &trans, &nVar, &nCon, jacDense, &grad );
    }

    /* Evaluate dense Hessian */
    hesDense = new double[nVar*nVar];
    CUTEST_cdh( &status, &nVar, &nCon, xInit, yInit, &nVar, hesDense );
    if( status ) {
        printf("** CUTEst_cdh: error, status = %d, aborting\n", status);
        exit(status);
    }
#endif
    delete[] xZero;

    /*
     * End of CUTEst stuff
     */


    /*
     * PART II: qpOASES
     */

    USING_NAMESPACE_QPOASES

    Options opts;
#if SOLVE_DENSE
    SymDenseMat *H_d;
    DenseMatrix *A_d;
#endif
#if SOLVE_SCHUR || SOLVE_SPARSE
    SymSparseMat *H_s;
    SparseMatrix *A_s;
#endif
    Bounds bInit( nVar );
    Constraints cInit( nCon );
    double *xOpt = new double[nVar];
    double *yOpt = new double[nVar+nCon];
    Bounds bOpt( nVar );
    Constraints cOpt( nCon );

#if SOLVE_DENSE
    A_d = new DenseMatrix( nCon, nVar, nVar, jacDense );
    H_d = new SymDenseMat( nVar, nVar, nVar, hesDense );
#endif
#if SOLVE_SCHUR || SOLVE_SPARSE
    A_s = new SparseMatrix( nCon, nVar, jacIndRow, jacIndCol, jacVal );
    H_s = new SymSparseMat( nVar, nVar, hesIndRow, hesIndCol, hesVal );
    H_s->createDiagInfo();
#endif

    /* Set initial working set depending on initial values for x and y */
    /// \todo I have not tested this
    double eps = 2.0e-16;
    for( int k=0; k<nVar; k++ )
    {
        if( getAbs( xInit[k] - lb[k] ) < eps )
            bInit.setStatus( k, ST_LOWER );
        else if( getAbs( xInit[k] - ub[k] ) < eps )
            bInit.setStatus( k, ST_UPPER );
        else
            bInit.setStatus( k, ST_INACTIVE );

        bInit.setType( k, ST_UNKNOWN );
    }
    /// \todo should do this based on yInit (take care of signs of lambda)
    for( int k=0; k<nCon; k++ )
    {
        if( getAbs( cVal[k] - lbA[k] ) < eps )
            cInit.setStatus( k, ST_LOWER );
        else if( getAbs( cVal[k] - ubA[k] ) < eps )
            cInit.setStatus( k, ST_UPPER );
        else
            cInit.setStatus( k, ST_INACTIVE );

        if( isEq[k] )
            cInit.setType( k, ST_EQUALITY );
        else
            cInit.setType( k, ST_UNKNOWN );
    }
    delete[] cVal;

    /* Set up QProblem object(s). */
#if SOLVE_SCHUR
    SQProblemSchur qpSchur( nVar, nCon, HST_UNKNOWN, 75 );
#endif
#if SOLVE_DENSE
    SQProblem qpDense( nVar, nCon, HST_UNKNOWN );
#endif
#if SOLVE_SPARSE
    SQProblem qpSparse( nVar, nCon, HST_UNKNOWN );
#endif

    /* Set options */
    returnValue ret;
    int maxIt = 10000;
    double maxTime = 100000;

    //opts.setToReliable();
    opts.setToDefault();

    //opts.printLevel = PL_HIGH;
    //opts.enableRamping = BT_TRUE;
    //opts.enableFarBounds = BT_TRUE;
    //opts.enableFlippingBounds = BT_TRUE;
    //opts.enableRegularisation = BT_FALSE;
    //opts.enableFullLITests = BT_TRUE;
    //opts.enableNZCTests = BT_TRUE;
    //opts.enableDriftCorrection = 1;
    //opts.enableCholeskyRefactorisation = 1;
    opts.enableEqualities = BT_TRUE;
    //opts.terminationTolerance =  2.2204e-09;
    //opts.boundTolerance =  2.2204e-10;
    //opts.boundRelaxation =  10000;
    //opts.epsNum = -2.2204e-13;
    //opts.epsDen =  2.2204e-13;
    //opts.maxPrimalJump =  100000000;
    //opts.maxDualJump =  100000000;
    //opts.initialRamping =  0.50000;
    //opts.finalRamping =  1;
    //opts.initialFarBounds =  1000000;
    //opts.growFarBounds =  1000;
    opts.initialStatusBounds =  ST_INACTIVE;
    //opts.epsFlipping =  2.2204e-13;
    //opts.numRegularisationSteps = 4;
    //opts.epsRegularisation =  2.2204e-13;
    opts.numRefinementSteps = 3;
    //opts.epsIterRef =  2.2204e-14;
    //opts.epsLITests =  2.2204e-09;
    //opts.epsNZCTests =  6.6613e-13;
    opts.enableInertiaCorrection = BT_TRUE;
    opts.rcondSMin = 1.0e-14;


    /* Solve QPs. */
    int nWSR;
    double time;
    struct timeval startTime;
    struct timeval endTime;
    int elapsedTime;
    gettimeofday(&startTime, NULL);
#if SOLVE_DENSE
    printf( "\n----------Begin Standard version with dense matrices----------\n" );
    qpDense.setOptions( opts );
    nWSR = maxIt;
    time = maxTime;

    for( int k=0; k<nVar; k++ )
    {
        xOpt[k] = xInit[k];
        yOpt[k] = 0.0;
    }
    for( int k=0; k<nCon; k++ ) yOpt[k+nVar] = yInit[k]; // what about lambda for bound constraints?
    bOpt = Bounds( bInit );
    cOpt = Constraints( cInit );
    ret = qpDense.init( H_d, g, A_d, lb, ub, lbA, ubA, nWSR, &time, xOpt, yOpt, &bOpt, &cOpt );

    if( ret == SUCCESSFUL_RETURN ) printf("Obj. = %23.16e\n", qpDense.getObjVal() + obj );
    printf( "\n-----------End Standard version with dense matrices-----------\n" );
#endif

#if SOLVE_SCHUR
    printf( "\n----------Begin Schur complement version----------\n" );
    qpSchur.setOptions( opts );
    nWSR = maxIt;
    time = maxTime;

    for( int k=0; k<nVar; k++ )
    {
        xOpt[k] = xInit[k];
        yOpt[k] = 0.0;
    }
    for( int k=0; k<nCon; k++ ) yOpt[k+nVar] = yInit[k]; // what about lambda for bound constraints?
    bOpt = Bounds( bInit );
    cOpt = Constraints( cInit );
    //ret = qpSchur.init( H_s, g, A_s, lb, ub, lbA, ubA, nWSR, &time, xOpt, yOpt, &bOpt, &cOpt );
    ret = qpSchur.init( H_s, g, A_s, lb, ub, lbA, ubA, nWSR, &time );

    if( ret == SUCCESSFUL_RETURN ) printf("Obj. = %23.16e\n", qpSchur.getObjVal() + obj );
    printf( "\n\n-----------End Schur complement version-----------\n" );
#endif

#if SOLVE_SPARSE
    printf( "\n----------Begin Standard version with sparse matrices----------\n" );
    qpSparse.setOptions( opts );
    nWSR = maxIt;
    time = maxTime;

    for( int k=0; k<nVar; k++ )
    {
        xOpt[k] = xInit[k];
        yOpt[k] = 0.0;
    }
    for( int k=0; k<nCon; k++ ) yOpt[k+nVar] = yInit[k]; // what about lambda for bound constraints?
    bOpt = Bounds( bInit );
    cOpt = Constraints( cInit );
    ret = qpSparse.init( H_s, g, A_s, lb, ub, lbA, ubA, nWSR, &time, xOpt, yOpt, &bOpt, &cOpt );

    if( ret == SUCCESSFUL_RETURN ) printf("Obj. = %23.16e\n", qpSparse.getObjVal() + obj );
    printf( "\n-----------End Standard version with sparse matrices-----------\n" );
#endif
    gettimeofday(&endTime, NULL);

    elapsedTime = (endTime.tv_sec*1000000  + (endTime.tv_usec)) -
                  (startTime.tv_sec*1000000  + (startTime.tv_usec));
    printf( "Took %g seconds.\n", elapsedTime/1000000.0 );

    /* Clean up */
    delete[] g;
    delete[] lb;
    delete[] ub;
    delete[] lbA;
    delete[] ubA;
    delete[] isEq;
    delete[] isLinear;
    delete[] xOpt;
    delete[] yOpt;
    delete[] xInit;
    delete[] yInit;

#if SOLVE_SCHUR || SOLVE_SPARSE
    delete[] hesVal;
    delete[] hesIndRow;
    delete[] hesIndCol;
    if( nCon > 0 )
    {
        delete[] jacVal;
        delete[] jacIndRow;
        delete[] jacIndCol;
    }
#endif
#if SOLVE_DENSE
    delete[] hesDense;
    if( nCon > 0 )
        delete[] jacDense;
#endif

    return ret;
}

