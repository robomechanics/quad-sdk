// (C) Copyright International Business Machines (IBM) 2006
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// P. Bonami, International Business Machines
//
// Date :  12/07/2006


// Code separated from BonOaDecBase to try to clarify OAs
#ifndef BonSubMipSolver_HPP
#define BonSubMipSolver_HPP
#include "IpSmartPtr.hpp"
#include <string>
/* forward declarations.*/
class OsiSolverInterface;
class OsiClpSolverInterface;
class OsiCpxSolverInterface;
class CbcStrategy;
class CbcStrategyDefault;

#include "OsiCuts.hpp"

namespace Bonmin {
    class RegisteredOptions;
    class BabSetupBase; 
    /** A very simple class to provide a common interface for solving MIPs with Cplex and Cbc.*/
    class SubMipSolver
    {
    public:
      enum MILP_solve_strategy{
         FindGoodSolution,
         GetOptimum};
      /** Constructor */
      SubMipSolver(BabSetupBase &b, const std::string &prefix);

      /** Copy Constructor */
      SubMipSolver(const SubMipSolver &copy);

      ~SubMipSolver();

      /** Assign lp solver. */
      void setLpSolver(OsiSolverInterface * lp);

      /** Assign a strategy. */
      void setStrategy(CbcStrategyDefault * strategy);

      /** get the solution found in last local search (return NULL if no solution). */
      const double * getLastSolution()
      {
        return integerSolution_;
      }

      double getLowerBound()
      {
        return lowBound_;
      }

      void solve(double cutoff,
          int loglevel,
          double maxTime){
         if(milp_strat_ == FindGoodSolution){
            find_good_sol(cutoff, loglevel, maxTime);
         }
         else
            optimize(cutoff, loglevel, maxTime);
      }
 

      /** update cutoff and perform a local search to a good solution. */
      void find_good_sol(double cutoff,
          int loglevel,
          double maxTime);

      /** update cutoff and optimize MIP. */
      void optimize(double cutoff,
          int loglevel,
          double maxTime);

      /** update cutoff, put OA constraints in cs as lazy constraints and optimize MIP. */
      void optimize_with_lazy_constraints(double cutoff,
          int loglevel,
          double maxTime, const OsiCuts & cs);

      /** Returns lower bound. */
      inline double lowBound()
      {
        return lowBound_;
      }

      /** returns optimality status. */
      inline bool optimal()
      {
        return optimal_;
      }

      /** Returns number of nodes in last solve.*/
      inline int nodeCount()
      {
        return nodeCount_;
      }

      /** Returns number of simplex iterations in last solve.*/
      inline int iterationCount()
      {
        return iterationCount_;
      }


      OsiSolverInterface * solver();

     /** Register options for that Oa based cut generation method. */
     static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);
    private:
      /** If lp solver is clp (then have to use Cbc) (not owned).*/
      OsiClpSolverInterface *clp_;
      /** If mip solver is cpx this is it (owned). */
      OsiCpxSolverInterface * cpx_;
      /** lower bound obtained */
      double lowBound_;
      /** Is optimality proven? */
      bool optimal_;
      /** Has an integer solution? then it is here*/
      double * integerSolution_;
      /** Strategy for solving sub mips with cbc. */
      CbcStrategyDefault * strategy_;
      /** number of nodes in last mip solved.*/
      int nodeCount_;
      /** number of simplex iteration in last mip solved.*/
      int iterationCount_;
      /** MILP search strategy.*/
      MILP_solve_strategy milp_strat_;
      /** setting for gap tolerance.*/
      double gap_tol_;
      /** say if owns copy of clp_.*/
      bool ownClp_;
    };

}

#endif

