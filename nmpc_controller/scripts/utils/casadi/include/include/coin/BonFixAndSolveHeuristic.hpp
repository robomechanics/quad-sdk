// (C) Copyright CNRS
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, LIF Université de la Méditérannée-CNRS
//
// Date : 06/18/2008

#ifndef BonFixAndSolveHeuristic_H
#define BonFixAndSolveHeuristic_H
#include "BonLocalSolverBasedHeuristic.hpp"

namespace Bonmin {
  class FixAndSolveHeuristic:public LocalSolverBasedHeuristic {
    public:
     /** Default constructor*/
     FixAndSolveHeuristic();
    /** Constructor with setup.*/
    FixAndSolveHeuristic(BonminSetup * setup);

     /** Copy constructor.*/
     FixAndSolveHeuristic(const FixAndSolveHeuristic &other);
     /** Virtual constructor.*/
     virtual CbcHeuristic * clone() const{
      return new FixAndSolveHeuristic(*this);
     }

     /** Destructor*/
     virtual ~FixAndSolveHeuristic();

     /** Runs heuristic*/
     int solution(double & objectiveValue,
                  double * newSolution);
   /** Register the options common to all local search based heuristics.*/
   static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

   /** Initiaize using passed options.*/
   void Initialize(Ipopt::SmartPtr<Ipopt::OptionsList> options);
  };

}/* Ends Bonmin namepace.*/
#endif

