// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 04/19/2007

#ifndef BonCbc_H
#define BonCbc_H

//#include "BonBabSetupBase.hpp"
#include "CbcModel.hpp"

namespace Bonmin
{
  class BabSetupBase;
  class Bab
  {
  public:
    /** Integer optimization return codes.*/
    enum MipStatuses {FeasibleOptimal /** Optimum solution has been found and its optimality proved.*/,
        ProvenInfeasible /** Problem has been proven to be infeasible.*/,
        Feasible /** An integer solution to the problem has been found.*/,
        UnboundedOrInfeasible /*Coninuous relaxation is unbounded.*/,
        NoSolutionKnown/** No feasible solution to the problem is known*/,
        NumMipStats};


    /** Constructor.*/
    Bab();
    /** destructor.*/
    virtual ~Bab();
    /** Perform a branch-and-bound using given setup.*/
    virtual void branchAndBound(BabSetupBase & s);

    /**operator() performs the branchAndBound*/
    virtual void operator()(BabSetupBase & s);

    /**operator() performs the branchAndBound*/
    virtual void operator()(BabSetupBase * s){
       operator()(*s);}

    /** get the best solution known to the problem (is optimal if MipStatus is FeasibleOptimal).
      if no solution is known returns NULL.*/
    const double * bestSolution() const
    {
      return bestSolution_;
    }
    /** return objective value of the bestSolution */
    double bestObj() const
    {
      return bestObj_;
    }

    /** return Mip Status */
    MipStatuses mipStatus() const
    {
      return mipStatus_;
    }

    /** return the best known lower bound on the objective value*/
    double bestBound();

    /** return the total number of nodes explored.*/
    int numNodes() const
    {
      return numNodes_;
    }
    /** return the total number of iterations in the last mip solved.*/
    int iterationCount()
    {
      return mipIterationCount_;
    }
    /** returns the value of the continuous relaxation. */
    double continuousRelaxation()
    {
      return continuousRelaxation_;
    }

    /** virtual callback function to eventually modify objects for integer variable
      (replace with user set). This is called after CbcModel::findIntegers */
    virtual void replaceIntegers(OsiObject ** objects, int numberObjects)
    {}
    /** Get cbc model used to solve. */
    const CbcModel&  model() const
    {
      return model_;
    }

    /** Get cbc model used to solve as non-const, in case we want to
        change options before things happen */
    CbcModel&  model()
    {
      return model_;
    }

  protected:
    /** Stores the solution of MIP. */
    double * bestSolution_;

    /** Status of the mip solved*/
    MipStatuses mipStatus_;
    /** objValue of MIP */
    double bestObj_;
    /** best known (lower) bound.*/
    double bestBound_;
    /** Continuous relaxation of the problem */
    double continuousRelaxation_;
    /** Number of nodes enumerated.*/
    int numNodes_;
    /** get total number of iterations in last mip solved.*/
    int mipIterationCount_;
    /** CbcModel used to solve problem.*/
    CbcModel model_;
    /** Message handler for CbcModel. */
    CoinMessageHandler * modelHandler_;
    /** \brief OsiObjects of the model.
      * this is not null if and only if there are some non-simple-integer branching objects such as SOS constraints.
      * It is up to Bab to pass them over to appropriate components of the algorithm. */
    OsiObject** objects_;
    /** number of objects.*/
    int nObjects_;
  };
}
#endif
