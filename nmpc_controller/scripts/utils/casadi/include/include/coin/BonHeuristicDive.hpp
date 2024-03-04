// Copyright (C) 2007, International Business Machines Corporation and others. 
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Joao P. Goncalves, International Business Machines Corporation
//
// Date : November 12, 2007

#ifndef BonHeuristicDive_HPP
#define BonHeuristicDive_HPP
#include "BonOsiTMINLPInterface.hpp"
#include "BonBonminSetup.hpp"
#include "CbcHeuristic.hpp"

namespace Bonmin
{
  class HeuristicDive : public CbcHeuristic
  {
  public:
    /// Default constructor
    HeuristicDive();

    /// Constructor with setup
    HeuristicDive(BonminSetup * setup);

    /// Copy constructor
    HeuristicDive(const HeuristicDive &copy);

    /// Destructor
    ~HeuristicDive() {}

    /// Assignment operator
    HeuristicDive & operator=(const HeuristicDive & rhs);

    /// Clone
    virtual CbcHeuristic * clone() const = 0;

    /// Resets stuff if model changes
    virtual void resetModel(CbcModel * model){
      setModel(model);
    }

    /** Change setup used for heuristic.*/
    virtual void setSetup(BonminSetup * setup){
      setup_ = setup;
      //      Initialize(setup_->options());
    }

    /// Set percentage of integer variables to fix at bounds
    void setPercentageToFix(double value)
    { percentageToFix_ = value; }

    /// Performs heuristic
    virtual int solution(double &solutionValue, double *betterSolution);

    /// sets internal variables
    virtual void setInternalVariables(TMINLP2TNLP* minlp) = 0;

    /// Selects the next variable to branch on
    /** If bestColumn = -1, it means that no variable was found
    */
    virtual void selectVariableToBranch(TMINLP2TNLP* minlp,
					const vector<int> & integerColumns,
					const double* newSolution,
					int& bestColumn,
					int& bestRound) = 0;

  protected:
    /** Setup to use for local searches (will make copies).*/
    BonminSetup * setup_; 

    /// Percentage of integer variables to fix at bounds
    double percentageToFix_;

  private:
    /// How often to do (code can change)
    int howOften_;

  };

  /// checks if the NLP relaxation of the problem is feasible
  bool isNlpFeasible(TMINLP2TNLP* minlp, const double primalTolerance);
  
  /// Adjusts the primalTolerance in case some of the constraints are violated
  void adjustPrimalTolerance(TMINLP2TNLP* minlp, double & primalTolerance);
}
#endif
