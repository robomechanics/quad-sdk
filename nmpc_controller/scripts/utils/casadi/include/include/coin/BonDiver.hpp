// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 09/01/2007

#ifndef BonDiver_H
#define BonDiver_H

#include "BonminConfig.h"
#include "CbcCompareBase.hpp"
#include "CbcTree.hpp"
#include "IpRegOptions.hpp"
#include "IpOptionsList.hpp"
#include "CbcCompareActual.hpp"
#include "BonRegisteredOptions.hpp"
#include <list>
namespace Bonmin
{
  class BabSetupBase;
  /** Class to do diving in the tree. Principle is that branch-and-bound follows current branch of the tree untill it
      hits the bottom at which point it goes to the best candidate (according to CbcCompare) on the heap.*/
  class CbcDiver : public CbcTree
  {
  public:
    /// Default constructor.
    CbcDiver();

    ///Copy constructor.
    CbcDiver(const CbcDiver &rhs);

    /// Assignment operator.
    CbcDiver & operator=(const CbcDiver &rhs);

    /// Destructor.
    virtual ~CbcDiver();

    ///Virtual copy constructor.
    virtual CbcTree * clone() const;

    /** \name Heap access and maintenance methods.*/
    /**@{*/
    ///Return top node (next node to process.*/
    virtual CbcNode * top() const;

    /// Add node to the heap.
    virtual void push(CbcNode * x);
    /// Remove the top node of the heap.
    virtual void pop();
    /// Remove the best node from the heap and return it
    virtual CbcNode * bestNode(double cutoff);
    /** @} */

    /// \name vector methods
    /** @{ */
    /** Test if empty. */
    virtual bool empty();
    /** Give size of the tree.*/
    virtual int size()
    {
      return (static_cast<int>(nodes_.size()) + (nextOnBranch_ != NULL) );
    }
    /** @} */

    /*! \brief Prune the tree using an objective function cutoff
      
    This routine removes all nodes with objective worst than the
    specified cutoff value.
    It also sets bestPossibleObjective to best
    of all on tree before deleting.
    */
    virtual void cleanTree(CbcModel * model, double cutoff, double & bestPossibleObjective);

    /// Get best possible objective function in the tree
    virtual double getBestPossibleObjective();


    ///Don't know what this is yet?
    virtual void endSearch()
    {
      nextOnBranch_ = NULL;
    }

    ///Register the options of the method.
    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

    /// Initialize the method (get options)
    void initialize(BabSetupBase &b);

  private:
    /** Say if we are cleaning the tree (then only call CbcTree functions).*/
    bool treeCleaning_;
    /** Noext node on the branch.*/
    CbcNode * nextOnBranch_;
    /** Flag indicating if we want to stop diving based on the guessed
    objective value and the cutoff value */
    bool stop_diving_on_cutoff_;
  };


  /** Class to do probed diving in the tree.
    * Principle is that branch-and-bound follows current branch of the tree by exploring the two children at each level
    * and continuing the dive on the best one of the two. Untill it 
    *  hits the bottom at which point it goes to the best candidate (according to CbcCompare) on the heap.*/
  class CbcProbedDiver : public CbcTree
  {
  public:
    /// Default constructor.
    CbcProbedDiver();

    ///Copy constructor.
    CbcProbedDiver(const CbcProbedDiver &rhs);

    /// Assignment operator.
    CbcProbedDiver & operator=(const CbcProbedDiver &rhs);

    /// Destructor.
    virtual ~CbcProbedDiver();

    ///Virtual copy constructor.
    virtual CbcTree * clone() const;

    /** \name Heap access and maintenance methods.*/
    /**@{*/
    ///Return top node (next node to process.*/
    virtual CbcNode * top() const;

    /// Add node to the heap.
    virtual void push(CbcNode * x);
    /// Remove the top node of the heap.
    virtual void pop();
    /// Remove the best node from the heap and return it
    virtual CbcNode * bestNode(double cutoff);
    /** @} */

    /// \name vector methods
    /** @{ */
    /** Test if empty. */
    virtual bool empty();
    /** Give size of the tree.*/
    virtual int size()
    {
      return (static_cast<int>(nodes_.size()) + (nextOnBranch_ != NULL) + (candidateChild_ != NULL) );
    }
    /** @} */

    /*! \brief Prune the tree using an objective function cutoff
      
    This routine removes all nodes with objective worst than the
    specified cutoff value.
    It also sets bestPossibleObjective to best
    of all on tree before deleting.
    */
    virtual void cleanTree(CbcModel * model, double cutoff, double & bestPossibleObjective);

    /// Get best possible objective function in the tree
    virtual double getBestPossibleObjective();


    ///Don't know what this is yet?
    virtual void endSearch()
    {
      nextOnBranch_ = NULL;
    }

    /// Initialize the method (get options)
    void initialize(BabSetupBase &b);

  private:
    /** Say if we are cleaning the tree (then only call CbcTree functions).*/
    bool treeCleaning_;
    /** Next node on the branch.*/
    CbcNode * nextOnBranch_;
    /** Candidate child explored.*/
    CbcNode * candidateChild_;
    /** Flag indicating if we want to stop diving based on the guessed
    objective value and the cutoff value */
    bool stop_diving_on_cutoff_;
  };


  /** A more elaborate diving class. First there are several modes which can be commanded by the Comparison class below.
     In particular can command to dive to find solutions, to try to close the bound as possible or to limit the size of
     the tree.

     The diving goes into the tree doing depth-first search until one of the following happens:
     \li A prescibed \c maxDiveBacktrack_ number of backtracking are performed.
     \li The guessed objective value of the current node is worst than the best incumbent.
     \li The depth of the dive is bigger than \c maxDiveDepth_

     In the first case all the nodes are put on the tree and the next node on top will be the top of the heap, in the
     two latter case we just put the node on the tree and backtrack in the list of depth-first search nodes.

     \bug This won't work in a non-convex problem where objective does not decrease down branches.
   */
  class CbcDfsDiver :public CbcTree
  {
  public:
    enum ComparisonModes{
      Enlarge/** At the very beginning we might want to enlarge the tree just a bit*/,
      FindSolutions,
      CloseBound,
      LimitTreeSize};
    /// Default constructor.
    CbcDfsDiver();

    ///Copy constructor.
    CbcDfsDiver(const CbcDfsDiver &rhs);

    /// Assignment operator.
    CbcDfsDiver & operator=(const CbcDfsDiver &rhs);

    /// Destructor.
    virtual ~CbcDfsDiver();

    ///Virtual copy constructor.
    virtual CbcTree * clone() const;

    /** \name Heap access and maintenance methods.*/
    /**@{*/
    ///Return top node (next node to process.*/
    virtual CbcNode * top() const;

    /// Add node to the heap.
    virtual void push(CbcNode * x);
    /// Remove the top node of the heap.
    virtual void pop();
    /// Remove the best node from the heap and return it
    virtual CbcNode * bestNode(double cutoff);
    /** @} */

    /// \name vector methods
    /** @{ */
    /** Test if empty. */
    virtual bool empty();
    /** Give size of the tree.*/
    virtual int size()
    {
      return static_cast<int>(nodes_.size()) + diveListSize_;
    }
    /** @} */

    /*! \brief Prune the tree using an objective function cutoff
      
    This routine removes all nodes with objective worst than the
    specified cutoff value.
    It also sets bestPossibleObjective to best
    of all on tree before deleting.
     \bug This won't work in a non-convex problem where objective does not decrease down branches.
    */
    virtual void cleanTree(CbcModel * model, double cutoff, double & bestPossibleObjective);

    /// Get best possible objective function in the tree
    virtual double getBestPossibleObjective();

//#ifdef COIN_HAS_BONMIN
    ///Register the options of the method.
    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

    /// Initialize the method (get options)
    void initialize(BabSetupBase &b);
//#endif
    ///Don't know what this is yet?
    virtual void endSearch()
    {}

    /** Changes the mode of comparison of the tree for "safety reasons" if the mode really changes we always
        finish the current dive and put all the node back onto the heap.*/
    void setComparisonMode(ComparisonModes newMode);
    /** get the mode of comparison of the tree.*/
    ComparisonModes getComparisonMode()
    {
      return mode_;
    }
  protected:
    /**Flag to say that we are currently cleaning the tree and should work only
       on the heap.*/
    int treeCleaning_;
    /** List of the nodes in the current dive.*/
    std::list<CbcNode *> dive_;
    /** Record dive list size for constant time access.*/
    int diveListSize_;
    /** Depth of the node from which diving was started (we call this node the diving board).*/
    int divingBoardDepth_;
    /** Last reported cutoff.*/
    double cutoff_;
    /** number of backtracks done in current dive.*/
    int nBacktracks_;
    /** \name Parameters of the method.*/
    /** @{ */
    /** Maximum depth until which we'll do a bredth-first-search.*/
    int maxDepthBFS_;
    /** Maximum number of backtrack in one dive.*/
    int maxDiveBacktracks_;
    /** Maximum depth to go from divingBoard.*/
    int maxDiveDepth_;
    /** Current mode of the diving strategy.*/
    ComparisonModes mode_;
    /** @} */
  private:
    /** Pushes onto heap all the nodes with objective value > cutoff. */
    void pushDiveOntoHeap(double cutoff);

  };

  class DiverCompare : public CbcCompareBase
  {
  public:
    // Default Constructor
    DiverCompare ():
        CbcCompareBase(),
        diver_(NULL),
        numberSolToStopDive_(5),
        numberNodesToLimitTreeSize_(1000000),
        comparisonDive_(NULL),
        comparisonBound_(NULL)
    {}


    virtual ~DiverCompare()
    {
      delete comparisonDive_;
      delete comparisonBound_;
    }

    // Copy constructor
    DiverCompare ( const DiverCompare & rhs):
        CbcCompareBase(rhs),
        diver_(rhs.diver_),
        numberSolToStopDive_(rhs.numberSolToStopDive_),
        numberNodesToLimitTreeSize_(rhs.numberNodesToLimitTreeSize_),
        comparisonDive_(rhs.comparisonDive_->clone()),
        comparisonBound_(rhs.comparisonBound_->clone())
    {}

    // Assignment operator
    DiverCompare & operator=( const DiverCompare& rhs)
    {
      if (this != &rhs) {
        CbcCompareBase::operator=(rhs);
        diver_ = rhs.diver_;
        numberSolToStopDive_ = rhs.numberSolToStopDive_;
        numberNodesToLimitTreeSize_ = rhs.numberNodesToLimitTreeSize_;
        delete comparisonDive_;
        delete comparisonBound_;
        comparisonDive_ = NULL;
        comparisonBound_ = NULL;
        if (rhs.comparisonDive_) comparisonDive_ = rhs.comparisonDive_->clone();
        if (rhs.comparisonBound_) comparisonBound_ = rhs.comparisonBound_->clone();
      }
      return *this;
    }

    /// Clone
    virtual CbcCompareBase * clone() const
    {
      return new DiverCompare(*this);
    }

    /// This is test function
    virtual bool test (CbcNode * x, CbcNode * y);

    ///  Called after each new solution
    virtual bool newSolution(CbcModel * model);

    ///  Called after each new solution
    virtual bool newSolution(CbcModel * model,
        double objectiveAtContinuous,
        int numberInfeasibilitiesAtContinuous);

    /** Called 1000 nodes.
      * Return true if want tree re-sorted.*/
    virtual bool every1000Nodes(CbcModel * model,int numberNodes);

    /** Set the dfs diver to use.*/
    void setDiver(CbcDfsDiver * diver)
    {
      diver_ = diver;
    }

    /** Set numberSolToStopDive_ */
    void setNumberSolToStopDive(int val)
    {
      numberSolToStopDive_ = val;
    }

    /** Set numberNodesToLimitTreeSize_.*/
    void setNumberNodesToLimitTreeSize(int val)
    {
      numberNodesToLimitTreeSize_ = val;
    }

    /** Set comparison method when diving.*/
    void setComparisonDive(const CbcCompareBase & val)
    {
      comparisonDive_ = val.clone();
    }
    /** Set comparison method when closing bound.*/
    void setComparisonBound(const CbcCompareBase & val)
    {
      comparisonBound_ = val.clone();
    }
  private:
    /** Pointer to the CbcDfsDiver handling the tree.*/
    CbcDfsDiver * diver_;
    /** Number of solution before we command diver_ to stop diving.*/
    int numberSolToStopDive_;
    /** Number of nodes before we command diver_ to limit the tree size.*/
    int numberNodesToLimitTreeSize_;
    /** Comparison method used in diving mode*/
    CbcCompareBase * comparisonDive_;
    /** Comparison method used bound mode*/
    CbcCompareBase * comparisonBound_;
    /** Comparison method used when limit tree size.*/
    CbcCompareDepth comparisonDepth_;
  };

}/* Ends bonmin namespace.*/

#endif

