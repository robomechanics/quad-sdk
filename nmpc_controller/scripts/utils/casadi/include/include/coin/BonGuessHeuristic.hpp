// (C) Copyright International Business Machines  2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Andreas Waechter          IBM       2007-09-01

#ifndef BonGuessHeuristic_HPP
#define BonGuessHeuristic_HPP
#include "BonOsiTMINLPInterface.hpp"

#include "CbcHeuristic.hpp"

namespace Bonmin
{
  class  GuessHeuristic : public CbcHeuristic
  {
  public:
    /// Usefull constructor
    GuessHeuristic(CbcModel &model);
    ///Copy constructor
    GuessHeuristic( const GuessHeuristic &copy):
        CbcHeuristic(copy)
    {}

    /// heuristic method providing guess, based on pseudo costs
    virtual int solution(double &solutionValue, double *betterSolution);
    virtual int solution(double &solutionValue, double *betterSolution, OsiCuts & cs)
    {
      return solution(solutionValue, betterSolution);
    }
    virtual CbcHeuristic * clone()const
    {
      return new GuessHeuristic(*this);
    }
    virtual void resetModel(CbcModel*)
    {}
  private:
    /// Default constructor
    GuessHeuristic();

    /// Assignment operator
    GuessHeuristic & operator=(const GuessHeuristic& rhs);
  };
}
#endif
