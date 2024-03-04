// (C) Copyright International Business Machines Corporation and Carnegie Mellon University 2006
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// John J. Forrest, International Business Machines Corporation
// Pierre Bonami, Carnegie Mellon University,
//
// Date : 03/15/2006

#ifndef BonCbcNlpStrategy_H
#define BonCbcNlpStrategy_H

#include "CbcStrategy.hpp"
class CglPreProcess;
class CbcNodeInfo;
class CbcNode;
class CoinWarmStartDiff;


namespace Bonmin
{
  class CbcNlpStrategy : public CbcStrategy
  {
  public:

    // Default Constructor
    CbcNlpStrategy (int maxFailures,
        int maxInfeasibles,
        int pretendFailIsInfeasible);

    // Copy constructor
    CbcNlpStrategy ( const CbcNlpStrategy &);

    // Destructor
    virtual ~CbcNlpStrategy ();

    /// Clone
    virtual CbcStrategy * clone() const;

    /// Return a new Full node information pointer (descendant of CbcFullNodeInfo)
    virtual CbcNodeInfo * fullNodeInfo(CbcModel * model,int numberRowsAtContinuous) const;
    /// Return a new Partial node information pointer (descendant of CbcPartialNodeInfo)
    virtual CbcNodeInfo * partialNodeInfo(CbcModel * model, CbcNodeInfo * parent, CbcNode * owner,
        int numberChangedBounds,const int * variables,
        const double * boundChanges,
        const CoinWarmStartDiff *basisDiff) const;
    /** After a CbcModel::resolve this can return a status
        -1 no effect
        0 treat as optimal
        1 as 0 but do not do any more resolves (i.e. no more cuts)
        2 treat as infeasible
    */
    virtual int status(CbcModel * model, CbcNodeInfo * parent, int whereFrom);
    /// set maximum number of consecutive failures in a branch before giving up
    inline void setMaxFailure(int value)
    {
      maxFailure_ = value;
    }
    /// maximum number of consecutive infeasible nodes before giving up
    inline void setMaxInfeasible(int value)
    {
      maxInfeasible_ = value;
    }

    /// Setup cut generators
    virtual void setupCutGenerators(CbcModel & model);
    /// Setup heuristics
    virtual void setupHeuristics(CbcModel & model);
    /// Do printing stuff
    virtual void setupPrinting(CbcModel & model,int modelLogLevel);
    /// Other stuff e.g. strong branching and preprocessing
    virtual void setupOther(CbcModel & model);

    bool hasFailed()
    {
      return hasFailed_;
    }
  protected:
    // Data
    /// did we fail?
    bool hasFailed_;
    /// maximum number of consecutive failures in a branch before giving up
    int maxFailure_;
    /// maximum number of consecutive infeasible nodes before giving up
    int maxInfeasible_;
    /** If yes when a problem is not solved (failed to be solved)
        will pretend that it is infeasible. */
    int pretendFailIsInfeasible_;

  private:
    /// Illegal Assignment operator
    CbcNlpStrategy & operator=(const CbcNlpStrategy& rhs);

  };
}

#endif
