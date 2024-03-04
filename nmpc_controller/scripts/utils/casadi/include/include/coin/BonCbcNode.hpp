// (C) Copyright International Business Machines Corporation and Carnegie Mellon University 2006
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// John J. Forrest, International Business Machines Corporation
// Pierre Bonami, Carnegie Mellon University,
//
// Date : 03/15/2006

#ifndef BonminCbcNode_H
#define BonminCbcNode_H

#include "CbcNode.hpp"
#include "BonRegisteredOptions.hpp"


namespace Bonmin
{
  /** \brief Holds information for recreating a subproblem by incremental change
  	   from the parent for Bonmin

    A BonminBonminCbcPartialNodeInfo object contains changes to the bounds and basis, and
    additional cuts, required to recreate a subproblem by modifying and
    augmenting the parent subproblem.
  */

  class BonCbcFullNodeInfo : public CbcFullNodeInfo
  {

  public:
    friend class BonCbcPartialNodeInfo;
    // Default Constructor
    BonCbcFullNodeInfo ();

    // Constructor from current state
    BonCbcFullNodeInfo (CbcModel * model, int numberRowsAtContinuous);

    // Copy constructor
    BonCbcFullNodeInfo ( const BonCbcFullNodeInfo &);

    // Destructor
    ~BonCbcFullNodeInfo ();

    /// Clone
    virtual CbcNodeInfo * clone() const;

    /**Method called when all direct sons have been explored to flush
       useless warm start information.*/
    virtual void allBranchesGone();

    /** Number of consecutive infeasible parents only recorded if node is infeasible*/
    inline int getSequenceOfInfeasiblesSize()
    {
      return sequenceOfInfeasiblesSize_;
    }
    /** Number of consecutive unsolved parents only recorded if node is infeasible*/
    inline int getSequenceOfUnsolvedSize()
    {
      return sequenceOfUnsolvedSize_;
    }
    /** Register all the options for class instance.*/
    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

  private:
    /* Data values */
    /** Number of consecutive infeasible parents only recorded if node is infeasible*/
    int sequenceOfInfeasiblesSize_;
    /** Number of consecutive unsolved parents only recorded if node is infeasible*/
    int sequenceOfUnsolvedSize_;
  private:

    /// Illegal Assignment operator
    BonCbcFullNodeInfo & operator=(const BonCbcFullNodeInfo& rhs);
  };

  /** \brief Holds information for recreating a subproblem by incremental change
  	   from the parent for

    A BonminCbcPartialNodeInfo object contains changes to the bounds and basis, and
    additional cuts, required to recreate a subproblem by modifying and
    augmenting the parent subproblem.
  */

  class BonCbcPartialNodeInfo : public CbcPartialNodeInfo
  {

  public:
    // Default Constructor
    BonCbcPartialNodeInfo ();

    // Constructor from current state
    BonCbcPartialNodeInfo (CbcModel * model, CbcNodeInfo * parent, CbcNode * owner,
        int numberChangedBounds,const int * variables,
        const double * boundChanges,
        const CoinWarmStartDiff *basisDiff) ;

    // Copy constructor
    BonCbcPartialNodeInfo ( const BonCbcPartialNodeInfo &);

    // Destructor
    ~BonCbcPartialNodeInfo ();

    /// Clone
    virtual CbcNodeInfo * clone() const;

    /**Method called when all direct sons have been explored to flush
       useless warm start information.*/
    virtual void allBranchesGone();

    /** Number of consecutive infeasible parents only recorded if node is infeasible*/
    inline int getSequenceOfInfeasiblesSize()
    {
      return sequenceOfInfeasiblesSize_;
    }
    /** Number of consecutive unsolved parents only recorded if node is infeasible*/
    inline int getSequenceOfUnsolvedSize()
    {
      return sequenceOfUnsolvedSize_;
    }
  private:
    /* Data values */
    /** Number of consecutive infeasible parents only recorded if node is infeasible*/
    int sequenceOfInfeasiblesSize_;
    /** Number of consecutive unsolved parents only recorded if node is infeasible*/
    int sequenceOfUnsolvedSize_;
  private:

    /// Illegal Assignment operator
    BonCbcPartialNodeInfo & operator=(const Bonmin::BonCbcPartialNodeInfo& rhs);
  };
}
#endif
