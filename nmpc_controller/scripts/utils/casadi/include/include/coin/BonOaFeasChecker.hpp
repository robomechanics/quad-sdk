// (C) Copyright International Business Machines 2006
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// P. Bonami, Carnegie Mellon University
//
// Date :  12/26/2006


#ifndef BonOaFeasibilityChecker_HPP
#define BonOaFeasibilityChecker_HPP
#include "BonOaDecBase.hpp"

namespace Bonmin
{
  /** Class to perform OA in its classical form.*/
  class OaFeasibilityChecker : public OaDecompositionBase
  {
  public:
    /// New usefull constructor
    OaFeasibilityChecker(BabSetupBase &b);
    /// Copy constructor
    OaFeasibilityChecker(const OaFeasibilityChecker &copy)
        :
        OaDecompositionBase(copy),
        pol_(copy.pol_),
        type_(copy.type_),
        cut_count_(copy.cut_count_),
        maximum_oa_cuts_(copy.maximum_oa_cuts_)
    {}
    /// Destructor
    ~OaFeasibilityChecker();

    /** Register OA options.*/
    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

    virtual CglCutGenerator * clone() const
    {
      return new OaFeasibilityChecker(*this);
    }
  protected:
    /// virtual method which performs the OA algorithm by modifying lp and nlp.
    virtual double performOa(OsiCuts & cs, solverManip &lpManip,
        BabInfo * babInfo, double &cutoff, const CglTreeInfo & info) const;
    /// virutal method to decide if local search is performed
    virtual bool doLocalSearch(BabInfo * babInfo) const
    {
      return 0;
    }

    /** See documentation for feas_check_discard_policy option.*/
    enum CutsPolicies {
      DetectCycles = 0,
      KeepAll,
      TreatAsNormal};
    /** Policy for keeping cuts.*/
    CutsPolicies pol_;
 
    /** See documentation for feas_check_cut_types option.*/
    enum CutsTypes {
      OA = 0,
      Benders};
    /** Type of cuts.*/
    CutsTypes type_;

    /** Count the total number of cuts generated.*/
    mutable unsigned int cut_count_;
    /** maximum number of OA cuts.*/
    unsigned int maximum_oa_cuts_;
  };
}
#endif
