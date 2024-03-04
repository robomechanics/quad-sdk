// (C) Copyright CNRS and International Business Machines Corporation
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, LIF Université de la Méditérannée-CNRS
// Joao Goncalves, International Business Machines Corporation
//
// Date : 06/18/2008

#ifndef BonHeuristicLocalBranching_H
#define BonHeuristicLocalBranching_H
#include "BonLocalSolverBasedHeuristic.hpp"

namespace Bonmin {
  class HeuristicLocalBranching:public LocalSolverBasedHeuristic {
    public:
     /** Default constructor*/
     HeuristicLocalBranching();
    /** Constructor with setup.*/
    HeuristicLocalBranching(BonminSetup * setup);

     /** Copy constructor.*/
     HeuristicLocalBranching(const HeuristicLocalBranching &other);
     /** Virtual constructor.*/
     virtual CbcHeuristic * clone() const{
      return new HeuristicLocalBranching(*this);
     }

     /** Destructor*/
     virtual ~HeuristicLocalBranching();
    
    /// Update model
    virtual void setModel(CbcModel * model);

    /// Validate model i.e. sets when_ to 0 if necessary
    virtual void validate();

     /** Runs heuristic*/
     int solution(double & objectiveValue,
                  double * newSolution);

    /** Register the options common to all local search based heuristics.*/
    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

    /** Initiaize using passed options.*/
    void Initialize(Ipopt::SmartPtr<Ipopt::OptionsList> options);

  private:
    /// How often to do (code can change)
    int howOften_;
    /// Number of solutions so we can do something at solution
    int numberSolutions_;

  };

}/* Ends Bonmin namepace.*/
#endif

