// (C) Copyright International Business Machines Corporation, Carnegie Mellon University 2006
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, Carnegie Mellon University,
// Andreas Waechter, International Business Machines Corporation
//
// Date : 02/15/2006


#ifndef IpoptWarmStart_HPP
#define IpoptWarmStart_HPP
#include "CoinWarmStartBasis.hpp"
#include "CoinWarmStartPrimalDual.hpp"
#include "BonIpoptInteriorWarmStarter.hpp"


namespace Bonmin
{
  class TMINLP2TNLP;

  /** \brief Class for storing warm start informations for Ipopt.<br>
   * This class inherits from CoinWarmStartPrimalDual, because that's what
   * this warmstart really is. <br>
   * For practical reason (integration in Cbc) this class also inherits from
   * CoinWarmStartBasis. <br>
   * This class stores a starting point (primal and dual values) for Ipopt.
   * <br>
   * <p>
   * The primal part of the base class contains the value of each primal
   * variable.
   * <p>
   * The dual part of the base class consists of three sections (the number of
   * values is 2*numcols+numrows):
     <UL>
     <li> First, values for dual variables associated with the lower bound
          constraints on structurals, i.e., primal variables (constraints
	  \f$ l \leq x \f$); 
     <li> Then values for dual variables associated with upper bound
           constraints on structurals (constraints \f$ x \leq u\f$).
     <li> the values for dual variables associated with regular constraints
          (constraints \f$ g(x) \leq 0 \f$) 
     </UL>
   */
  class IpoptWarmStart :
    public virtual CoinWarmStartPrimalDual, public virtual CoinWarmStartBasis
  {
  public:

    /// Default constructor
    IpoptWarmStart(bool empty = 1, int numvars = 0, int numcont = 0);
    /// Usefull constructor, stores the current optimum of ipopt
    IpoptWarmStart(const Ipopt::SmartPtr<TMINLP2TNLP> tnlp,
        Ipopt::SmartPtr<IpoptInteriorWarmStarter> warm_starter);
    /// Another usefull constructor, stores the passed point
    IpoptWarmStart(int primal_size, int dual_size,
                   const double * primal, const double * dual);
    /// Copy constructor
    IpoptWarmStart( const IpoptWarmStart &other, bool ownValues = 1);
    /// A constructor from a CoinWarmStartPrimalDual
    IpoptWarmStart(const CoinWarmStartPrimalDual& pdws);
    /// Abstract destructor
    virtual ~IpoptWarmStart();

    /// `Virtual constructor'
    virtual CoinWarmStart *clone() const
    {
      return new IpoptWarmStart(*this,1);
    }

    /** Generate the "differences" between two IpoptWarmStart.*/
    virtual CoinWarmStartDiff*
    generateDiff(const CoinWarmStart *const oldCWS) const;
    /** \brief Apply 'differences' to an Ipopt warm start.
     * What this actually does is get a copy to the vector of values stored
     in IpoptWarmStartDiff.*/
    virtual void
    applyDiff (const CoinWarmStartDiff *const cwsdDiff);

    /** Accessor to warm start information obecjt */
    Ipopt::SmartPtr<IpoptInteriorWarmStarter> warm_starter() const
    {
      return warm_starter_;
    }

    /// flush the starting point
    void flushPoint();

    ///Is this an empty warm start?
    bool empty() const
    {
      return empty_;
    }
  private:
    /** warm start information object */
    mutable Ipopt::SmartPtr<IpoptInteriorWarmStarter> warm_starter_;
    ///Say if warm start is empty
    bool empty_;
  };

  //###########################################################################

  /** \brief Diff class for IpoptWarmStart.
   * Actually get the differences from CoinWarmStartBasis and stores the
   whole vector of values.
   \todo Find a way to free unused values.
  */
  class IpoptWarmStartDiff : public CoinWarmStartPrimalDualDiff
  {
  public:
    friend class IpoptWarmStart;
    /** Useful constructor; takes over the data in \c diff */
    IpoptWarmStartDiff(CoinWarmStartPrimalDualDiff * diff,
		       Ipopt::SmartPtr<IpoptInteriorWarmStarter> warm_starter):
      CoinWarmStartPrimalDualDiff(),
      warm_starter_(NULL)//(warm_starter)
    {
      CoinWarmStartPrimalDualDiff::swap(*diff);
    }
    /** Copy constructor. */
    IpoptWarmStartDiff(const IpoptWarmStartDiff &other):
        CoinWarmStartPrimalDualDiff(other),
        warm_starter_(NULL /*other.warm_starter_*/) {}

    /// Abstract destructor
    virtual ~IpoptWarmStartDiff() {}

    /// `Virtual constructor'
    virtual CoinWarmStartDiff *clone() const
    {
      return new IpoptWarmStartDiff(*this);
    }

    /** Accessor to warm start information obecjt */
    Ipopt::SmartPtr<IpoptInteriorWarmStarter> warm_starter() const
    {
      return warm_starter_;
    }
    void flushPoint();
  private:

    /** warm start information object */
    Ipopt::SmartPtr<IpoptInteriorWarmStarter> warm_starter_;
  };

}
#endif
