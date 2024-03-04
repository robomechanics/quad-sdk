// Copyright (C) 2007, International Business Machines Corporation and others. 
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Joao P. Goncalves, International Business Machines Corporation
//
// Date : November 12, 2007

#ifndef BonHeuristicFPump_HPP
#define BonHeuristicFPump_HPP
#include "BonOsiTMINLPInterface.hpp"
#include "BonBonminSetup.hpp"
#include "CbcHeuristic.hpp"

namespace Bonmin
{
  class  HeuristicFPump : public CbcHeuristic
  {
  public:
    /// Default constructor
    HeuristicFPump();

    /// Constructor with setup
    HeuristicFPump(BonminSetup * setup);

    /// Copy constructor
    HeuristicFPump(const HeuristicFPump &copy);

    /// Destructor
    ~HeuristicFPump() {}

    /// Assignment operator
    HeuristicFPump & operator=(const HeuristicFPump & rhs);

    /** Virtual constructor.*/
    virtual CbcHeuristic * clone() const{
      return new HeuristicFPump(*this);
    }

    /// Resets stuff if model changes
    virtual void resetModel(CbcModel * model){
      setModel(model);
    }

    /** Change setup used for heuristic.*/
    void setSetup(BonminSetup * setup){
      setup_ = setup;
      Initialize(setup_->options());
    }

    /// Performs heuristic
    virtual int solution(double &solutionValue, double *betterSolution);

    /// Performs heuristic with add cust
    virtual int solution(double &solutionValue, double *betterSolution, OsiCuts & cs)
    {
      return solution(solutionValue, betterSolution);
    }

    /** Register the options for this heuristic */
    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

    /** Initiaize using passed options.*/
    void Initialize(Ipopt::SmartPtr<Ipopt::OptionsList> options);

  private:
    /** Setup to use for local searches (will make copies).*/
    BonminSetup * setup_; 

    /** Norm of the objective function - either 1 or 2 */
    int objective_norm_;

    /// To enable advanced unstable stuff
    int enableAdvanced_;
  };

  class RoundingFPump
  {
  public:
    /// Default constructor
    RoundingFPump(TMINLP2TNLP* minlp);

    /// Destructor
    ~RoundingFPump();

    /// Rounds the solution
    void round(const double integerTolerance, 
	       const double primalTolerance,
	       double* solution);

  private:
    /// gutsOfConstructor
    void gutsOfConstructor();

    /// Pointer to problem
    TMINLP2TNLP* minlp_;

    /// Number of rows
    int numberRows_;

    /// Number of columns
    int numberColumns_;

    /// Jacobian of g
    std::vector<std::pair<int, int> >* col_and_jac_g_;

  };

}
#endif
