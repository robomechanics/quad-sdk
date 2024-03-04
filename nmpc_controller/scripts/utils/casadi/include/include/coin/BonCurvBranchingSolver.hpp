// Copyright (C) 2006, 2007 International Business Machines
// Corporation and others.  All Rights Reserved.
//
//
#error "BonCurvBranchingSolver not supported anymore"
#ifndef BonCurvBranchingSolver_H
#define BonCurvBranchingSolver_H

#include "BonStrongBranchingSolver.hpp"
#include "BonCurvatureEstimator.hpp"

namespace Bonmin
{

  /** Implementation of BonChooseVariable for curvature-based braching.
  */

  class CurvBranchingSolver : public StrongBranchingSolver
  {

  public:

    /// Constructor from solver (so we can set up arrays etc)
    CurvBranchingSolver (OsiTMINLPInterface * solver);

    /// Copy constructor
    CurvBranchingSolver (const CurvBranchingSolver &);

    /// Assignment operator
    CurvBranchingSolver & operator= (const CurvBranchingSolver& rhs);

    /// Destructor
    virtual ~CurvBranchingSolver ();

    /// Called to initialize solver before a bunch of strong branching
    /// solves
    virtual void markHotStart(OsiTMINLPInterface* tminlp_interface);

    /// Called to solve the current TMINLP (with changed bound information)
    virtual TNLPSolver::ReturnStatus solveFromHotStart(OsiTMINLPInterface* tminlp_interface);

    /// Called after all strong branching solves in a node
    virtual void unmarkHotStart(OsiTMINLPInterface* tminlp_interface);

  private:
    /// Default Constructor
    CurvBranchingSolver ();

    SmartPtr<CurvatureEstimator> cur_estimator_;

    /** @name Stuff for the curvature estimator */
    //@{
    bool new_bounds_;
    bool new_x_;
    bool new_mults_;
    double* orig_d_;
    double* projected_d_;
    Number* x_l_orig_;
    Number* x_u_orig_;
    Number* g_l_orig_;
    Number* g_u_orig_;
    //@}

    /** @name Information about the problem */
    //@{
    int numCols_;
    int numRows_;
    const double* solution_;
    const double* duals_;
    double obj_value_;
    //@}

  };

}

#endif
