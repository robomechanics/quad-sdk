// (C) Copyright CNRS and International Business Machines Corporation
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, LIF Université de la Méditérannée-CNRS
// Joao Goncalves, International Business Machines Corporation
//
// Date : 06/18/2008

#ifndef BonHeuristicRINS_H
#define BonHeuristicRINS_H
#include "BonLocalSolverBasedHeuristic.hpp"

namespace Bonmin {
  class HeuristicRINS:public LocalSolverBasedHeuristic {
    public:
     /** Default constructor*/
     HeuristicRINS();
    /** Constructor with setup.*/
    HeuristicRINS(BonminSetup * setup);

     /** Copy constructor.*/
     HeuristicRINS(const HeuristicRINS &other);
     /** Virtual constructor.*/
     virtual CbcHeuristic * clone() const{
      return new HeuristicRINS(*this);
     }

     /** Destructor*/
     virtual ~HeuristicRINS();

     /** Runs heuristic*/
     int solution(double & objectiveValue,
                  double * newSolution);
   /** Register the options common to all local search based heuristics.*/
   static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

   /** Initiaize using passed options.*/
   void Initialize(Ipopt::SmartPtr<Ipopt::OptionsList> options);

    /// Sets how often to do it
    inline void setHowOften(int value)
    { howOften_=value;}

  private:
    /// How often to do (code can change)
    int howOften_;
    /// Number of solutions so we can do something at solution
    int numberSolutions_;

  };

}/* Ends Bonmin namepace.*/
#endif
