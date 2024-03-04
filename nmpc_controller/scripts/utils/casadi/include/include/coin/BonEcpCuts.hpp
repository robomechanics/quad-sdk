// (C) Copyright International Business Machines (IBM) 2006, 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// P. Bonami, International Business Machines
//
// Date :  12/20/2006

#ifndef BonECPCuts_HPP
#define BonECPCuts_HPP

#include "BonOaDecBase.hpp"
#include "CglCutGenerator.hpp"
namespace Bonmin
{
  class EcpCuts: public OaDecompositionBase
  {
  public:
    EcpCuts(BabSetupBase & b);

    /// Copy constructor
    EcpCuts(const EcpCuts & copy):
        OaDecompositionBase(copy),
        objValue_(copy.objValue_),
        numRounds_(copy.numRounds_),
        abs_violation_tol_(copy.abs_violation_tol_),
        rel_violation_tol_(copy.rel_violation_tol_),
        beta_(copy.beta_)
    {}

    /// clone
    CglCutGenerator * clone() const
    {
      return new EcpCuts(*this);
    }

    /// Destructor
    virtual ~EcpCuts()
    {}
    /** Standard cut generation methods. */
    virtual void generateCuts(const OsiSolverInterface &si,  OsiCuts & cs,
        const CglTreeInfo info = CglTreeInfo());
    double doEcpRounds(OsiSolverInterface &si,
        bool leaveSiUnchanged,
        double* violation = NULL);

    void setNumRounds(int value)
    {
      numRounds_ = value;
    }

    void setPropabilityFactor(double value)
    {
      beta_ = value;
    }

    void setAbsViolationTolerance(double value)
    {
      abs_violation_tol_ = value;
    }
    void setRelViolationTolerance(double value)
    {
      rel_violation_tol_ = value;
    }

    /** Register ecp cuts options.*/
    static void registerOptions(Ipopt::SmartPtr<Bonmin::RegisteredOptions> roptions);

  protected:
    /// virtual method which performs the OA algorithm by modifying lp and nlp.
    virtual double performOa(OsiCuts & cs, solverManip &lpManip,
                   BabInfo * babInfo, double &cutoff, const CglTreeInfo &info) const
    {
      throw -1;
    }
    /// virutal method to decide if local search is performed
    virtual bool doLocalSearch(BabInfo * babInfo) const
    {
      return 0;
    }
  private:
    /** Record obj value at final point of Ecp. */
    mutable double objValue_;
    /** Record NLP infeasibility at final point of Ecp */
    mutable double violation_;
    /** maximum number of iterations of generation. */
    int numRounds_;
    /** absolute tolerance for NLP constraint violation to stop ECP rounds */
    double abs_violation_tol_;
    /** relative tolerance for NLP constraint violation to stop ECP rounds */
    double rel_violation_tol_;
    /** Factor for probability for skipping cuts */
    double beta_;
  };
} /* end namespace Bonmin.*/
#endif
