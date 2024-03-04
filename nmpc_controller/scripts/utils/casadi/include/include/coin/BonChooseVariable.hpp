// Copyright (C) 2006, 2008 International Business Machines
// Corporation and others.  All Rights Reserved.
#ifndef BonChooseVariable_H
#define BonChooseVariable_H

#include "OsiChooseVariable.hpp"
#ifdef BONMIN_CURVATURE_BRANCHING
#include "BonCurvatureEstimator.hpp"
#endif
#include "BonOsiTMINLPInterface.hpp"
#include "CoinMessageHandler.hpp"
#include "BonBabSetupBase.hpp"
// Forward declaration
class CbcModel;

#define OLD_USEFULLNESS

namespace Bonmin
{

  class HotInfo : public OsiHotInfo {
    public:
    /// Default constructor
    HotInfo();

    /// Constructor from usefull information
    HotInfo( OsiSolverInterface * solver,
             const OsiBranchingInformation *info,
             const OsiObject * const * objects, int whichObject);

    /// Copy constructor
    HotInfo(const HotInfo & other);

    /// Assignment operator
    HotInfo & operator=(const HotInfo & rhs);

    /// Clone
    virtual OsiHotInfo * clone() const;

    /// Destructor
    virtual ~HotInfo();

    /// Fill in some usefull information after a strong branching is done:
    int updateInformation( const OsiSolverInterface * solver, const OsiBranchingInformation * info,
	                   OsiChooseVariable * choose);

    /// up infeasibility
    double upInfeasibility() const{
     return infeasibilities_[1];
    }

    /// down infeasibility
    double downInfeasibility() const{
     return infeasibilities_[0];
    }


    /// Set the down infeasibility
    void setUpInfeasibility(double x){
      assert(branchingObject_->numberBranches()==2);
      infeasibilities_[1] = x;
    }

    /// Set the down infeasibility
    void setDownInfeasibility(double x){
      assert(branchingObject_->numberBranches()==2);
      infeasibilities_[0] = x;
    }
    private:
    /// infeasibilities of children
    vector<double> infeasibilities_; 
  };

  /** This class chooses a variable to branch on

      This is the base class for the branching rules in Bonmin (inherits
      from OsiChooseVariable). This class implements a simple strong branching algorithm where the changes in the objective
      value induced by branching on a specific object are estimated with the pure virtual function fill_changes.
  */

  class BonChooseVariable : public OsiChooseVariable
  {
  protected:
  /**  This is a utility function which does strong branching on
       a list of objects and stores the results in OsiHotInfo.objects.
       On entry the object sequence is stored in the OsiHotInfo object
       and maybe more.
       It returns -
       -1 - one branch was infeasible both ways
       0 - all inspected - nothing can be fixed
       1 - all inspected - some can be fixed (returnCriterion==0)
       2 - may be returning early - one can be fixed (last one done) (returnCriterion==1) 
       3 - returning because max time
       
  */
  virtual int doStrongBranching( OsiSolverInterface * solver, 
				 OsiBranchingInformation *info,
				 int numberToDo, int returnCriterion);
#ifndef OLD_USEFULLNESS
    /** Criterion applied to sort candidates.*/
    enum CandidateSortCriterion {
      DecrPs = 0,
      IncrPs,
      DecrInfeas,
      IncrInfeas};
#endif

    /** Statuses for strong branching candidates.*/
    enum StrongStatus{
      NotDone=-1,
      Feasible/** Child is proven feasible.*/,
      Infeasible /** Child is proven infeasible.*/,
      NotFinished /** Child is not finished.*/};
  public:
    /** \name Message handling.*/
    /** @{ */
    enum Messages_Types {
      PS_COST_HISTORY = 0,
      PS_COST_MULT,
      PS_COST_ESTIMATES,
      CANDIDATE_LIST,
      CANDIDATE_LIST2,
      CANDIDATE_LIST3,
      SB_START,
      SB_HEADER,
      SB_RES,
      BRANCH_VAR,
      CHOSEN_VAR,
      UPDATE_PS_COST,
      BON_CHOOSE_MESSAGES_DUMMY_END
    };

  class Messages : public CoinMessages
    {
    public:
      Messages();
    };

    void passInMessageHandler(CoinMessageHandler * handler) {
      int logLevel = handler_->logLevel();
      delete handler_;
      handler_ = handler->clone();
      handler_->setLogLevel(logLevel);
    }

    CoinMessageHandler& message(Messages_Types type) const {
      return handler_->message(type, messages_);
    }
    /** @} */



    enum DoStrongReturnStatuses{
      provenInfeasible = -1 /** One branch has two infeasible children.*/,
      doneNoFixing /** All done no variable can be fixed.*/,
      doneCanFix /** Several variable can be fixed.*/,
      interuptedCanFix /** Interrupted and found a variable to fix.*/,
      maxTime /** Interrupted because of time limit.*/};

    /** Return statuses for chooseVariable.*/
    enum chooseVariableReturnStatuses{
      infeasibleNode = -1/** Node has been proven infeasible.*/,
      hasCandidate /** Normal termination, found a variable to branch on.*/,
      feasibleNode /** All variable are feasible, the node is feasible.*/,
      canFixAndStrongBranch /** Found variable to fix and also has remaining candidate for strong branching.*/,
      canFixAndBranch/** Found variable to fix and also has a (non-strong) branching candidate.*/,
      canFixNoCandidate /** Can fix variables but does not have strong branching candidates.*/
    };
    /// Constructor from solver (so we can set up arrays etc)
    BonChooseVariable (BabSetupBase& b, const OsiSolverInterface* solver);

    /// Copy constructor
    BonChooseVariable (const BonChooseVariable &);

    /// Assignment operator
    BonChooseVariable & operator= (const BonChooseVariable& rhs);

    /// Clone
    virtual OsiChooseVariable * clone() const;

    /// Destructor
    virtual ~BonChooseVariable ();

    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

    /** Helper functions for setupList and chooseVariable */
    double maxminCrit(const OsiBranchingInformation* info) const;
    void computeMultipliers(double& upMult, double& downMult) const;
    double computeUsefulness(const double MAXMIN_CRITERION,
        const double upMult, const double dowMult,
        const double value,
        const OsiObject* object, int i,
        double& value2) const;

    /** Sets up strong list and clears all if initialize is true.
        Returns number of infeasibilities. */
    virtual int setupList ( OsiBranchingInformation *info, bool initialize);

    /** Choose a variable
        Returns - 
       -1 Node is infeasible
       0  Normal termination - we have a candidate
       1  All looks satisfied - no candidate
       2  We can change the bound on a variable - but we also have a strong branching candidate
       3  We can change the bound on a variable - but we have a non-strong branching candidate
       4  We can change the bound on a variable - no other candidates
       We can pick up branch from bestObjectIndex() and bestWhichWay()
       We can pick up a forced branch (can change bound) from firstForcedObjectIndex() and firstForcedWhichWay()
       If we have a solution then we can pick up from goodObjectiveValue() and goodSolution()
       If fixVariables is true then 2,3,4 are all really same as problem changed
    */
    virtual int chooseVariable( OsiSolverInterface * solver, OsiBranchingInformation *info, bool fixVariables);
    /**  This is a utility function which does strong branching on
         a list of objects and stores the results in OsiHotInfo.objects.
         On entry the object sequence is stored in the OsiHotInfo object
         and maybe more.
         It returns -
         -1 - one branch was infeasible both ways
         0 - all inspected - nothing can be fixed
         1 - all inspected - some can be fixed (returnCriterion==0)
         2 - may be returning early - one can be fixed (last one done) (returnCriterion==1) 
         3 - returning because max time
      
    */

    /// Given a candidate fill in useful information e.g. estimates
    virtual void updateInformation(const OsiBranchingInformation *info,
        int branch, OsiHotInfo * hotInfo);
#if 1
    /// Given a branch fill in useful information e.g. estimates
    virtual void updateInformation( int whichObject, int branch,
        double changeInObjective, double changeInValue,
        int status);
#endif

    /** Method for setting CbcModel, which is used to get statusOfSearch */
    void setCbcModel(CbcModel* cbc_model)
    {
      cbc_model_ = cbc_model;
    }

    void setOnlyPseudoWhenTrusted(bool only_pseudo_when_trusted)
    {
      only_pseudo_when_trusted_ = only_pseudo_when_trusted;
    }


    /** Access to pseudo costs storage.*/
    const OsiPseudoCosts & pseudoCosts() const{
      return pseudoCosts_;}

    /** Access to pseudo costs storage.*/
    OsiPseudoCosts & pseudoCosts() {
      return pseudoCosts_;}
  protected:

    /// Holding on the a pointer to the journalist
    Ipopt::SmartPtr<Ipopt::Journalist> jnlst_;

    /// verbosity level
    int bb_log_level_;

    /** Stores strong branching results.*/
    vector<HotInfo> results_;

    /** Determine status of strong branching solution.*/
    int determineStatus(OsiSolverInterface * solver) const {
      if (solver->isProvenOptimal())
        return 0; // optimal
      else if (solver->isIterationLimitReached()
               &&!solver->isDualObjectiveLimitReached())
        return 2; // unknown 
      else
        return 1; // infeasible
    }

  private:
    /** Default Constructor, forbiden for some reason.*/
    BonChooseVariable ();

    /** Global time limit for algorithm. */
    double time_limit_;

    /** Starting time of algorithm.*/
    double start_time_;
  protected:
    /// CbcModel, used to get status of search
    CbcModel* cbc_model_;

    /** Flag indicating whether we don't want to mix strong branching
     *  and pseudo costs during the decision which variable to branch
     *  on */
    bool only_pseudo_when_trusted_;

    /** Number of variables put into the list because there were not
     *  trusted */
    int number_not_trusted_;

    /** Message handler.*/
    CoinMessageHandler * handler_;

    /** Messages.*/
    Messages messages_;
    // ToDo: Make this options
    /** @name Algoirithmic options */
    //@{
    /** maxmin weight in branching decision when no solution has been
     *  found yet */
    double maxmin_crit_no_sol_;
    /** maxmin weight in branching decision when no solution has been
     *  found yet */
    double maxmin_crit_have_sol_;
    /** fraction of branching candidates that are not trusted yet */
    double setup_pseudo_frac_;
    /** number of times a branch has to happen so that it is trusted in
     *  setupList */
    int numberBeforeTrustedList_;
    /** number of strong branching points at root node */
    int numberStrongRoot_;
    /** backup of numberStrong_ before Root node solve */
    int numberStrongBackup_;
    /** number of look-ahead strong-branching steps */
    int numberLookAhead_;
#ifndef OLD_USEFULLNESS
    /** Criterion to use in setup list.*/
    CandidateSortCriterion sortCrit_;
#endif
    /** Always strong branch that many first candidate in the list regardless of numberTrusted.*/
    int minNumberStrongBranch_;
    /** Stores the pseudo costs. */
    OsiPseudoCosts pseudoCosts_;
    /** Wether or not to trust strong branching results for updating pseudo costs.*/
    int trustStrongForPseudoCosts_;
   
    //@}

    /** detecting if this is root node */
    bool isRootNode(const OsiBranchingInformation *info) const;

    /** Stores the class name for throwing errors.*/
    static const std::string CNAME;
  };

}
#endif
