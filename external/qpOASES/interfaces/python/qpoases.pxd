##
##  This file is part of qpOASES.
##
##  qpOASES -- An Implementation of the Online Active Set Strategy.
##  Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
##  Christian Kirches et al. All rights reserved.
##
##  qpOASES is free software; you can redistribute it and/or
##  modify it under the terms of the GNU Lesser General Public
##  License as published by the Free Software Foundation; either
##  version 2.1 of the License, or (at your option) any later version.
##
##  qpOASES is distributed in the hope that it will be useful,
##  but WITHOUT ANY WARRANTY; without even the implied warranty of
##  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##  See the GNU Lesser General Public License for more details.
##
##  You should have received a copy of the GNU Lesser General Public
##  License along with qpOASES; if not, write to the Free Software
##  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##

##
##    Filename:  qpoases.pxd
##    Author:    Sebastian F. Walter, Manuel Kudruss (thanks to Felix Lenders)
##    Version:   3.2
##    Date:      2013-2017
##

cdef extern from "qpOASES.hpp" namespace "qpOASES":

    ctypedef double real_t
    # NOTE we use long type for integers to be compatible with the C/C++
    #      interface in all cases, i.e, either __USE_LONG_INTEGERS__ is defined
    #      or not.
    ctypedef long int_t

    cdef enum BooleanType:

        BT_FALSE
        BT_TRUE

    cdef enum PrintLevel:

        PL_DEBUG_ITER = -2
        PL_TABULAR
        PL_NONE
        PL_LOW
        PL_MEDIUM
        PL_HIGH

    cdef enum VisibilityStatus:

        VS_HIDDEN
        VS_VISIBLE

    cdef enum QProblemStatus:

        QPS_NOTINITIALISED
        QPS_PREPARINGAUXILIARYQP

        QPS_AUXILIARYQPSOLVED

        QPS_PERFORMINGHOMOTOPY

        QPS_HOMOTOPYQPSOLVED
        QPS_SOLVED

    cdef enum HessianType:
        HST_ZERO
        HST_IDENTITY
        HST_POSDEF
        HST_POSDEF_NULLSPACE
        HST_SEMIDEF
        HST_INDEF
        HST_UNKNOWN

    cdef enum SubjectToType:

        ST_UNBOUNDED
        ST_BOUNDED
        ST_EQUALITY
        ST_DISABLED
        ST_UNKNOWN

    cdef enum SubjectToStatus:

        ST_LOWER = -1
        ST_INACTIVE
        ST_UPPER
        ST_INFEASIBLE_LOWER
        ST_INFEASIBLE_UPPER
        ST_UNDEFINED

    cdef enum  returnValue:
        TERMINAL_LIST_ELEMENT = -1
        SUCCESSFUL_RETURN = 0
        RET_DIV_BY_ZERO
        RET_INDEX_OUT_OF_BOUNDS
        RET_INVALID_ARGUMENTS
        RET_ERROR_UNDEFINED
        RET_WARNING_UNDEFINED
        RET_INFO_UNDEFINED
        RET_EWI_UNDEFINED
        RET_AVAILABLE_WITH_LINUX_ONLY
        RET_UNKNOWN_BUG
        RET_PRINTLEVEL_CHANGED
        RET_NOT_YET_IMPLEMENTED
        RET_INDEXLIST_MUST_BE_REORDERD
        RET_INDEXLIST_EXCEEDS_MAX_LENGTH
        RET_INDEXLIST_CORRUPTED
        RET_INDEXLIST_OUTOFBOUNDS
        RET_INDEXLIST_ADD_FAILED
        RET_INDEXLIST_INTERSECT_FAILED
        RET_INDEX_ALREADY_OF_DESIRED_STATUS
        RET_ADDINDEX_FAILED
        RET_REMOVEINDEX_FAILED
        RET_SWAPINDEX_FAILED
        RET_NOTHING_TO_DO
        RET_SETUP_BOUND_FAILED
        RET_SETUP_CONSTRAINT_FAILED
        RET_MOVING_BOUND_FAILED
        RET_MOVING_CONSTRAINT_FAILED
        RET_SHIFTING_FAILED
        RET_ROTATING_FAILED
        RET_QPOBJECT_NOT_SETUP
        RET_QP_ALREADY_INITIALISED
        RET_NO_INIT_WITH_STANDARD_SOLVER
        RET_RESET_FAILED
        RET_INIT_FAILED
        RET_INIT_FAILED_TQ
        RET_INIT_FAILED_CHOLESKY
        RET_INIT_FAILED_HOTSTART
        RET_INIT_FAILED_INFEASIBILITY
        RET_INIT_FAILED_UNBOUNDEDNESS
        RET_INIT_FAILED_REGULARISATION
        RET_INIT_SUCCESSFUL
        RET_OBTAINING_WORKINGSET_FAILED
        RET_SETUP_WORKINGSET_FAILED
        RET_SETUP_AUXILIARYQP_FAILED
        RET_NO_CHOLESKY_WITH_INITIAL_GUESS
        RET_NO_EXTERN_SOLVER
        RET_QP_UNBOUNDED
        RET_QP_INFEASIBLE
        RET_QP_NOT_SOLVED
        RET_QP_SOLVED
        RET_UNABLE_TO_SOLVE_QP
        RET_INITIALISATION_STARTED
        RET_HOTSTART_FAILED
        RET_HOTSTART_FAILED_TO_INIT
        RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED
        RET_ITERATION_STARTED
        RET_SHIFT_DETERMINATION_FAILED
        RET_STEPDIRECTION_DETERMINATION_FAILED
        RET_STEPLENGTH_DETERMINATION_FAILED
        RET_OPTIMAL_SOLUTION_FOUND
        RET_HOMOTOPY_STEP_FAILED
        RET_HOTSTART_STOPPED_INFEASIBILITY
        RET_HOTSTART_STOPPED_UNBOUNDEDNESS
        RET_WORKINGSET_UPDATE_FAILED
        RET_MAX_NWSR_REACHED
        RET_CONSTRAINTS_NOT_SPECIFIED
        RET_INVALID_FACTORISATION_FLAG
        RET_UNABLE_TO_SAVE_QPDATA
        RET_STEPDIRECTION_FAILED_TQ
        RET_STEPDIRECTION_FAILED_CHOLESKY
        RET_CYCLING_DETECTED
        RET_CYCLING_NOT_RESOLVED
        RET_CYCLING_RESOLVED
        RET_STEPSIZE
        RET_STEPSIZE_NONPOSITIVE
        RET_SETUPSUBJECTTOTYPE_FAILED
        RET_ADDCONSTRAINT_FAILED
        RET_ADDCONSTRAINT_FAILED_INFEASIBILITY
        RET_ADDBOUND_FAILED
        RET_ADDBOUND_FAILED_INFEASIBILITY
        RET_REMOVECONSTRAINT_FAILED
        RET_REMOVEBOUND_FAILED
        RET_REMOVE_FROM_ACTIVESET
        RET_ADD_TO_ACTIVESET
        RET_REMOVE_FROM_ACTIVESET_FAILED
        RET_ADD_TO_ACTIVESET_FAILED
        RET_CONSTRAINT_ALREADY_ACTIVE
        RET_ALL_CONSTRAINTS_ACTIVE
        RET_LINEARLY_DEPENDENT
        RET_LINEARLY_INDEPENDENT
        RET_LI_RESOLVED
        RET_ENSURELI_FAILED
        RET_ENSURELI_FAILED_TQ
        RET_ENSURELI_FAILED_NOINDEX
        RET_ENSURELI_FAILED_CYCLING
        RET_BOUND_ALREADY_ACTIVE
        RET_ALL_BOUNDS_ACTIVE
        RET_CONSTRAINT_NOT_ACTIVE
        RET_BOUND_NOT_ACTIVE
        RET_HESSIAN_NOT_SPD
        RET_HESSIAN_INDEFINITE
        RET_MATRIX_SHIFT_FAILED
        RET_MATRIX_FACTORISATION_FAILED
        RET_PRINT_ITERATION_FAILED
        RET_NO_GLOBAL_MESSAGE_OUTPUTFILE
        RET_DISABLECONSTRAINTS_FAILED
        RET_ENABLECONSTRAINTS_FAILED
        RET_ALREADY_ENABLED
        RET_ALREADY_DISABLED
        RET_NO_HESSIAN_SPECIFIED
        RET_USING_REGULARISATION
        RET_EPS_MUST_BE_POSITVE
        RET_REGSTEPS_MUST_BE_POSITVE
        RET_HESSIAN_ALREADY_REGULARISED
        RET_CANNOT_REGULARISE_IDENTITY
        RET_CANNOT_REGULARISE_SPARSE
        RET_NO_REGSTEP_NWSR
        RET_FEWER_REGSTEPS_NWSR
        RET_CHOLESKY_OF_ZERO_HESSIAN
        RET_ZERO_HESSIAN_ASSUMED
        RET_CONSTRAINTS_ARE_NOT_SCALED
        RET_INITIAL_BOUNDS_STATUS_NYI
        RET_ERROR_IN_CONSTRAINTPRODUCT
        RET_FIX_BOUNDS_FOR_LP
        RET_USE_REGULARISATION_FOR_LP
        RET_UPDATEMATRICES_FAILED
        RET_UPDATEMATRICES_FAILED_AS_QP_NOT_SOLVED
        RET_UNABLE_TO_OPEN_FILE
        RET_UNABLE_TO_WRITE_FILE
        RET_UNABLE_TO_READ_FILE
        RET_FILEDATA_INCONSISTENT
        RET_UNABLE_TO_ANALYSE_QPROBLEM
        RET_OPTIONS_ADJUSTED
        RET_NWSR_SET_TO_ONE
        RET_UNABLE_TO_READ_BENCHMARK
        RET_BENCHMARK_ABORTED
        RET_INITIAL_QP_SOLVED
        RET_QP_SOLUTION_STARTED
        RET_BENCHMARK_SUCCESSFUL
        RET_NO_DIAGONAL_AVAILABLE
        RET_DIAGONAL_NOT_INITIALISED
        RET_ENSURELI_DROPPED
        RET_KKT_MATRIX_SINGULAR
        RET_QR_FACTORISATION_FAILED
        RET_INERTIA_CORRECTION_FAILED
        RET_NO_SPARSE_SOLVER
        RET_SIMPLE_STATUS_P1
        RET_SIMPLE_STATUS_P0
        RET_SIMPLE_STATUS_M1
        RET_SIMPLE_STATUS_M2
        RET_SIMPLE_STATUS_M3


    cdef cppclass Options:

        Options()
        Options(const Options&)
        # Options& operator=( const Options&)  # equality operator cannot be overloaded in Python
        returnValue setToDefault()
        returnValue setToReliable()
        returnValue setToMPC()
        returnValue setToFast()
        returnValue ensureConsistency()
        # returnValue print() # print is a reserved keyword in Python
        returnValue copy(const Options& )

        PrintLevel printLevel

        BooleanType enableRamping
        BooleanType enableFarBounds
        BooleanType enableFlippingBounds
        BooleanType enableRegularisation
        BooleanType enableFullLITests
        BooleanType enableNZCTests
        int_t       enableDriftCorrection
        int_t       enableCholeskyRefactorisation
        BooleanType enableEqualities

        real_t terminationTolerance
        real_t boundTolerance
        real_t boundRelaxation
        real_t epsNum
        real_t epsDen
        real_t maxPrimalJump
        real_t maxDualJump

        real_t initialRamping
        real_t finalRamping
        real_t initialFarBounds
        real_t growFarBounds
        SubjectToStatus initialStatusBounds
        real_t epsFlipping
        int_t numRegularisationSteps
        real_t epsRegularisation
        int_t numRefinementSteps
        real_t epsIterRef
        real_t epsLITests
        real_t epsNZCTests

        real_t rcondSMin
        BooleanType enableInertiaCorrection

        BooleanType enableDropInfeasibles
        int_t dropBoundPriority
        int_t dropEqConPriority
        int_t dropIneqConPriority


    cdef cppclass Bounds:
        Bounds()


    cdef cppclass Constraints:
        Constraints()


    cdef cppclass QProblemB:
        QProblemB()
        QProblemB(int_t, HessianType, BooleanType)

        QProblemB(const QProblemB&)

        returnValue init(real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         int_t&)

        returnValue init(real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         int_t&,
                         real_t*)

        returnValue hotstart(real_t*,
                             real_t*,
                             real_t*,
                             int_t&)

        returnValue hotstart(real_t*,
                             real_t*,
                             real_t*,
                             int_t&,
                             real_t*)


        returnValue getPrimalSolution(real_t*)
        returnValue getDualSolution(real_t*)
        returnValue printOptions()
        real_t getObjVal()

        Options getOptions()
        returnValue setOptions(Options&)

    cdef cppclass QProblem:
        QProblem()
        QProblem(int_t, int_t, HessianType, BooleanType)

        QProblem(const QProblem&)

        returnValue init(real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         int_t&)

        returnValue init(real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         int_t&,
                         real_t*)

        returnValue hotstart(real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             int_t&)

        returnValue hotstart(real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             int_t&,
                             real_t*)

        returnValue getPrimalSolution(real_t*)
        returnValue getDualSolution(real_t*)
        returnValue printOptions()
        real_t getObjVal()

        Options getOptions()
        returnValue setOptions(Options&)


    cdef cppclass SQProblem:
        SQProblem()
        SQProblem(int_t, int_t, HessianType, BooleanType)

        SQProblem(const QProblem&)

        returnValue init(real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         int_t&)

        returnValue init(real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         real_t*,
                         int_t&,
                         real_t*)

        returnValue init(real_t* _H,
                         real_t* _g,
                         real_t* _A,
                         real_t* _lb,
                         real_t* _ub,
                         real_t* _lbA,
                         real_t* _ubA,
                         int_t& nWSR,
                         real_t* cputime,
                         real_t* xOpt,
                         real_t* yOpt,
                         Bounds* guessedBounds,
                         Constraints* guessedConstraints,
                         real_t* _R)


        returnValue hotstart(real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             int_t&)

        returnValue hotstart(real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             real_t*,
                             int_t&,
                             real_t*)

        returnValue getPrimalSolution(real_t*)
        returnValue getDualSolution(real_t*)
        returnValue printOptions()
        real_t getObjVal()

        Options getOptions()
        returnValue setOptions(Options&)

cdef extern from "qpOASES/extras/SolutionAnalysis.hpp" namespace "qpOASES":
    cdef cppclass SolutionAnalysis:
        SolutionAnalysis()
        SolutionAnalysis(const SolutionAnalysis&)
        # ~SolutionAnalysis()
        # SolutionAnalysis& operator=(const SolutionAnalysis&)
        real_t getKktViolation(const QProblem*,  const real_t*, const real_t*, const real_t*)
        real_t getKktViolation(const QProblemB*, const real_t*, const real_t*, const real_t*)
        real_t getKktViolation(const SQProblem*, const real_t*, const real_t*, const real_t*)
        returnValue getVarianceCovariance(QProblem*, real_t*, real_t*)
        returnValue getVarianceCovariance(QProblemB*, real_t*, real_t*)
        returnValue getVarianceCovariance(SQProblem*, real_t*, real_t*)


cdef extern from "qpOASES/Utils.hpp" namespace "qpOASES":
    pass
    #void getKktViolation(int_t nV,                # Number of variables.
    #                     int_t nC,                # Number of constraints.
    #                     const real_t* const H,   # Hessian matrix.
    #                     const real_t* const g,   # Sequence of gradient vectors.
    #                     const real_t* const A,   # Constraint matrix.
    #                     const real_t* const lb,  # Sequence of lower bound vectors (on variables).
    #                     const real_t* const ub,  # Sequence of upper bound vectors (on variables).
    #                     const real_t* const lbA, # Sequence of lower constraints' bound vectors.
    #                     const real_t* const ubA, # Sequence of upper constraints' bound vectors.
    #                     const real_t* const x,   # Sequence of primal trial vectors.
    #                     const real_t* const y,   # Sequence of dual trial vectors.
    #                     real_t& stat,            # Maximum value of stationarity condition residual.
    #                     real_t& feas,            # Maximum value of primal feasibility violation.
    #                     real_t& cmpl             # Maximum value of complementarity residual.
    #                     )


cdef extern from "qpOASES/extras/OQPinterface.hpp" namespace "qpOASES":
    returnValue runOqpBenchmark(const char* path,           # Full path of the benchmark files (without trailing slash!).
                                BooleanType isSparse,       # Shall convert matrices to sparse format before solution?
                                BooleanType useHotstarts,   # Shall QP solution be hotstarted?
                                const Options& options,     # QP solver options to be used while solving benchmark problems.
                                int_t maxAllowedNWSR,       # Maximum number of working set recalculations to be performed.
                                real_t& maxNWSR,            # Output: Maximum number of performed working set recalculations.
                                real_t& avgNWSR,            # Output: Average number of performed working set recalculations.
                                real_t& maxCPUtime,         # Output: Maximum CPU time required for solving each QP.
                                real_t& avgCPUtime,         # Output: Average CPU time required for solving each QP.
                                real_t& maxStationarity,    # Output: Maximum residual of stationarity condition.
                                real_t& maxFeasibility,     # Output: Maximum residual of primal feasibility condition.
                                real_t& maxComplementarity  # Output: Maximum residual of complementarity condition.
                                )
