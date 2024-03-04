// (C) Copyright Carnegie Mellon University 2005
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// P. Bonami, Carnegie Mellon University
//
// Date :  05/26/2005

#ifndef BonDummyHeuristic_HPP
#define BonDummyHeuristic_HPP
#include "BonOsiTMINLPInterface.hpp"

#include "CbcHeuristic.hpp"
namespace Bonmin
{
  class  DummyHeuristic : public CbcHeuristic
  {
  public:
    /// Default constructor
    DummyHeuristic(OsiTMINLPInterface * si = NULL);
    /// Usefull constructor
    DummyHeuristic(CbcModel &model, OsiTMINLPInterface * si = NULL);
    ///Copy constructor
    DummyHeuristic( const DummyHeuristic &copy):
        CbcHeuristic(copy),
        nlp_(copy.nlp_),
        knowsSolution(copy.knowsSolution)
    {}
    /// Set nlp_
    void setNlp(OsiTMINLPInterface * si);
    /// heuristic method
    virtual int solution(double &solutionValue, double *betterSolution);
    virtual int solution(double &solutionValue, double *betterSolution, OsiCuts & cs)
    {
      return solution(solutionValue, betterSolution);
    }
    virtual CbcHeuristic * clone()const
    {
      return new DummyHeuristic(*this);
    }
    virtual void resetModel(CbcModel*)
    {}
  virtual bool shouldHeurRun(int whereFrom){
     return true;}
  private:
    /// Pointer to the Ipopt interface
    OsiTMINLPInterface * nlp_;
    /// Do I have a solution?
    bool knowsSolution;
  };
}
#endif
