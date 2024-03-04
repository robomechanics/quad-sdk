// Copyright (C) 2007, International Business Machines Corporation and others. 
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Joao P. Goncalves, International Business Machines Corporation
//
// Date : November 12, 2007

#ifndef BonHeuristicDiveMIP_HPP
#define BonHeuristicDiveMIP_HPP
#include "BonOsiTMINLPInterface.hpp"
#include "BonBonminSetup.hpp"
#include "CbcHeuristic.hpp"
#include "CbcStrategy.hpp"
namespace Bonmin
{
  class SubMipSolver;
  class HeuristicDiveMIP : public CbcHeuristic
  {
  public:
#if 0
    /// Default constructor
    HeuristicDiveMIP();
#endif

    /// Constructor with setup
    HeuristicDiveMIP(BonminSetup * setup);

    /// Copy constructor
    HeuristicDiveMIP(const HeuristicDiveMIP &copy);

    /// Destructor
    ~HeuristicDiveMIP();

    /// Assignment operator
    HeuristicDiveMIP & operator=(const HeuristicDiveMIP & rhs);

    /// Clone
    virtual CbcHeuristic * clone() const = 0;

    /// Initialize method 
    void Initialize(BonminSetup * setup);

    /// Resets stuff if model changes
    virtual void resetModel(CbcModel * model){
      setModel(model);
    }

    /** Change setup used for heuristic.*/
    virtual void setSetup(BonminSetup * setup){
      setup_ = setup;
      //      Initialize(setup_->options());
    }

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

  private:
    /// How often to do (code can change)
    int howOften_;
    /// A subsolver for MIP
    SubMipSolver * mip_;

  };
}
#endif
