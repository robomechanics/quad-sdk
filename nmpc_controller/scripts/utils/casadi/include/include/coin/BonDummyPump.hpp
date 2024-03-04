// (C) Copyright CNRS
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, LIF Université de la Méditérannée-CNRS
//
// Date : 06/18/2008

#ifndef BonDummyPump_H
#define BonDummyPump_H
#include "BonLocalSolverBasedHeuristic.hpp"

namespace Bonmin {
  class DummyPump:public LocalSolverBasedHeuristic {
    public:
     /** Default constructor*/
     DummyPump();
    /** Constructor with setup.*/
    DummyPump(BonminSetup * setup);

     /** Copy constructor.*/
     DummyPump(const DummyPump &other);
     /** Virtual constructor.*/
     virtual CbcHeuristic * clone() const{
      return new DummyPump(*this);
     }

     /** Destructor*/
     virtual ~DummyPump();

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

