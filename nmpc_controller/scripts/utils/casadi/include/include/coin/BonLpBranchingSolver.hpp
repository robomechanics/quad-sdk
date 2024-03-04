// Copyright (C) 2006, 2007 International Business Machines
// Corporation and others.  All Rights Reserved.
#ifndef BonLpBranchingSolver_H
#define BonLpBranchingSolver_H

#include "BonStrongBranchingSolver.hpp"
#include "BonEcpCuts.hpp"

namespace Bonmin
{

  /** Implementation of BonChooseVariable for curvature-based braching.
  */

  class LpBranchingSolver : public StrongBranchingSolver
  {

  public:

    /// Constructor from setup 
    LpBranchingSolver (BabSetupBase *b);
    /// Copy constructor
    LpBranchingSolver (const LpBranchingSolver &);

    /// Assignment operator
    LpBranchingSolver & operator= (const LpBranchingSolver& rhs);

    /// Destructor
    virtual ~LpBranchingSolver ();

    /// Called to initialize solver before a bunch of strong branching
    /// solves
    virtual void markHotStart(OsiTMINLPInterface* tminlp_interface);

    /// Called to solve the current TMINLP (with changed bound information)
    virtual TNLPSolver::ReturnStatus solveFromHotStart(OsiTMINLPInterface* tminlp_interface);

    /// Called after all strong branching solves in a node
    virtual void unmarkHotStart(OsiTMINLPInterface* tminlp_interface);

    void setMaxCuttingPlaneIter(int num)
    {
      maxCuttingPlaneIterations_ = num;
    }

    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

  private:
    /// Default Constructor
    LpBranchingSolver ();

    /// Linear solver
    OsiSolverInterface* lin_;

    /// Warm start object for linear solver
    CoinWarmStart* warm_;

    /// Ecp cut generate
    EcpCuts* ecp_;

    /// Number of maximal ECP cuts
    int maxCuttingPlaneIterations_;

    /// absolute tolerance for ECP cuts
    double abs_ecp_tol_;

    /// relative tolerance for ECP cuts
    double rel_ecp_tol_;


   enum WarmStartMethod {
     Basis=0 /** Use basis*/,
     Clone /** clone problem*/
   };
   /// Way problems are warm started
   WarmStartMethod warm_start_mode_;
  };

}
#endif
