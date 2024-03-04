// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 04/13/2007
#ifndef BonminSetup_H
#define BonminSetup_H
#include "BonBabSetupBase.hpp"
namespace Bonmin
{
  /** Type of algorithms which can be used.*/
  enum Algorithm{
    Dummy=-1/** Dummy value before initialization.*/,
    B_BB=0/** Bonmin's Branch-and-bound.*/,
    B_OA=1/** Bonmin's Outer Approximation Decomposition.*/,
    B_QG=2/** Bonmin's Quesada & Grossmann branch-and-cut.*/,
    B_Hyb=3/** Bonmin's hybrid outer approximation.*/,
    B_Ecp=4/** Bonmin's implemantation of ecp cuts based branch-and-cut a la FilMINT.*/,
    B_IFP=5/** Bonmin's implemantation of iterated feasibility pump for MINLP.*/
  };
  /* Bonmin algorithm setup. */
  class BonminSetup : public BabSetupBase
  {
  public:
    /** Default constructor. */
    BonminSetup(const CoinMessageHandler * handler = NULL);
    /** Copy constructor. */
    BonminSetup(const BonminSetup & other);

    /** Copy but uses an other nlp.*/
    BonminSetup(const BonminSetup &setup,
                OsiTMINLPInterface &nlp);

    /** Copy but uses another nlp and algorithm.*/
    BonminSetup(const BonminSetup &setup,
                OsiTMINLPInterface &nlp,
                const std::string & prefix);
    /** virtual copy constructor. */
    virtual BabSetupBase * clone() const
    {
      return new BonminSetup(*this);
    }
    /** Make a copy with solver replace by one passed .*/
    //    virtual BabSetupBase *clone(OsiTMINLPInterface&nlp)const{
    //      return new BonminSetup(*this, nlp);
    //    }
    /** Make a copy with solver replace by one passed .*/
    BonminSetup *clone(OsiTMINLPInterface&nlp)const{
      return new BonminSetup(*this, nlp);
    }
    /** Make a copy but take options with different prefix.*/
    BonminSetup *clone(OsiTMINLPInterface &nlp, const std::string & prefix)const{
      return new BonminSetup(*this, nlp, prefix);
    }
    virtual ~BonminSetup()
    {}
    /** @name Methods to instantiate: Registering and retrieving options and initializing everything. */
    /** @{ */
    /** Register all the options for this algorithm instance.*/
    virtual void registerOptions();
    /** Setup the defaults options for this algorithm. */
    virtual void setBabDefaultOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions)
    {}
    /** @} */
    /** Register all bonmin type executable options.*/
    static void registerAllOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);
    /** Initialize, read options and create appropriate bonmin setup.*/
    void initialize(Ipopt::SmartPtr<TMINLP> tminlp, bool createContinuousSolver = true);
    /** Initialize, read options and create appropriate bonmin setup.*/
    void initialize(const OsiTMINLPInterface& nlpSi, bool createContinuousSolver = true);
    /** Get the algorithm used.*/
    Bonmin::Algorithm getAlgorithm();

    void addCutGenerator(CuttingMethod & cg){
      BabSetupBase::addCutGenerator(cg);
    }
  protected:
    /** Register standard MILP cut generators. */
    static void registerMilpCutGenerators(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);
    /** Add milp cut generators according to options.*/
    void addMilpCutGenerators();
    /** Initialize a plain branch-and-bound.*/
    void initializeBBB();
    /** Initialize a branch-and-cut with some OA.*/
    void initializeBHyb(bool createContinuousSolver = false);
  private:
    Algorithm algo_;
  };
}/** end namespace Bonmin*/

#endif

