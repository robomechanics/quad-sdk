// (C) Copyright Carnegie Mellon University 2005
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// P. Bonami, Carnegie Mellon University
//
// Date :  05/26/2005


#ifndef BonOACutGenerator2_HPP
#define BonOACutGenerator2_HPP
#include "BonOaDecBase.hpp"

namespace Bonmin
{
  /** Class to perform OA in its classical form.*/
  class OACutGenerator2 : public OaDecompositionBase
  {
  public:
    /// Constructor with basic setup
    OACutGenerator2(BabSetupBase & b);

    /// Copy constructor
    OACutGenerator2(const OACutGenerator2 &copy)
        :
        OaDecompositionBase(copy),
        subMip_(new SubMipSolver (*copy.subMip_))
    {}
    /// Destructor
    ~OACutGenerator2();

    void setStrategy(const CbcStrategy & strategy)
    {
      parameters_.setStrategy(strategy);
    }

    virtual CglCutGenerator * clone() const
    {
      return new OACutGenerator2(*this);
    }
    /** Register OA options.*/
    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

  protected:
    /// virtual method which performs the OA algorithm by modifying lp and nlp.
    virtual double performOa(OsiCuts & cs, solverManip &lpManip,
               BabInfo * babInfo, double &cutoff, const CglTreeInfo & info) const;
    /// virutal method to decide if local search is performed
    virtual bool doLocalSearch(BabInfo * babInfo) const;

  private:
    SubMipSolver * subMip_;
  };
}
#endif
