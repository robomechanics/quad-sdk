// Copyright (C) 2007, International Business Machines Corporation and others. 
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Joao P. Goncalves, International Business Machines Corporation
//
// Date : November 12, 2007

#ifndef BonHeuristicDiveMIPFractional_H
#define BonHeuristicDiveMIPFractional_H

#include "BonBonminSetup.hpp"
#include "BonHeuristicDiveMIP.hpp"

/** DiveMIPFractional class
 */

namespace Bonmin
{
  class HeuristicDiveMIPFractional : public HeuristicDiveMIP {
  public:
    /// Default Constructor 
    HeuristicDiveMIPFractional ();

    /// Constructor with setup
    HeuristicDiveMIPFractional(BonminSetup * setup);

    /// Copy constructor
    HeuristicDiveMIPFractional(const HeuristicDiveMIPFractional &copy);

    /// Destructor
    ~HeuristicDiveMIPFractional() {}

    /// Assignment operator
    HeuristicDiveMIPFractional & operator=(const HeuristicDiveMIPFractional & rhs);

    /// Clone
    virtual CbcHeuristic * clone() const;

    /** Change setup used for heuristic.*/
    virtual void setSetup(BonminSetup * setup){
      HeuristicDiveMIP::setSetup(setup);
      Initialize(setup->options());
    }

    /// sets internal variables
    virtual void setInternalVariables(TMINLP2TNLP* minlp);

    /// Selects the next variable to branch on
    /** If bestColumn = -1, it means that no variable was found
    */
    virtual void selectVariableToBranch(TMINLP2TNLP* minlp,
					const vector<int> & integerColumns,
					const double* newSolution,
					int& bestColumn,
					int& bestRound);

    /** Register the options common to all local search based heuristics.*/
    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);
    
    /** Initiaize using passed options.*/
    void Initialize(Ipopt::SmartPtr<Ipopt::OptionsList> options);

  };
}
#endif
