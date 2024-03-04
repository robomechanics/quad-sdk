// Copyright (C) 2010, International Business Machines Corporation and others. 
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami CNRS
//
// Date : May, 26 2010

#ifndef BonMilpRounding_HPP
#define BonMilpRounding_HPP
#include "BonOsiTMINLPInterface.hpp"
#include "BonBonminSetup.hpp"
#include "CbcHeuristic.hpp"
#include "CbcStrategy.hpp"
#include "OsiCuts.hpp"

namespace Bonmin
{
  class SubMipSolver;
  class MilpRounding : public CbcHeuristic
  {
  public:

    /// Constructor with setup
    MilpRounding(BonminSetup * setup);

    /// Copy constructor
    MilpRounding(const MilpRounding &copy);

    /// Destructor
    ~MilpRounding();

    /// Assignment operator
    MilpRounding & operator=(const MilpRounding & rhs);

    /// Clone
    virtual CbcHeuristic * clone() const{
      return new MilpRounding(*this);
    }

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


    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);
  protected:
    /** Setup to use for local searches (will make copies).*/
    BonminSetup * setup_; 

  private:
    /// How often to do (code can change)
    int howOften_;
    /// A subsolver for MIP
    SubMipSolver * mip_;

    OsiCuts noGoods;
  };
}
#endif
