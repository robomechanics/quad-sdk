// (C) Copyright International Business Machines Corporation 2006
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id$
//
// Authors:  Andreas Waechter               IBM    2006-03-02

#ifndef __IPOPTINTERIORWARMSTARTER_HPP__
#define __IPOPTINTERIORWARMSTARTER_HPP__

#include "IpSmartPtr.hpp"
#include "IpNLP.hpp"
#include <vector>

namespace Bonmin
{
  class IpoptInteriorWarmStarter : public Ipopt::ReferencedObject
  {
  public:
    /**@name Constructors/Destructors */
    //@{
    /** Constructor. We give it the values of the current bounds so that
     *  it can figure out which variables are fixed for this NLP. */
    IpoptInteriorWarmStarter(Ipopt::Index n, const Ipopt::Number* x_L, const Ipopt::Number* x_u,
        Ipopt::Number nlp_lower_bound_inf,
        Ipopt::Number nlp_upper_bound_inf,
        bool store_several_iterates);

    /** Default destructor */
    ~IpoptInteriorWarmStarter();
    //@}

    /** Method for possibly storing another iterate during the current
     *  optimizatin for possible use for a warm start for a new
     *  problem */
    bool UpdateStoredIterates(Ipopt::AlgorithmMode mode,
        const Ipopt::IpoptData& ip_data,
        Ipopt::IpoptCalculatedQuantities& ip_cq);

    /** Method for doing whatever needs to be done after the parent NLP
     *  has been solved */
    bool Finalize();

    /** Method for computing the initial point based on the stored
     *  information */
    bool WarmStartIterate(Ipopt::Index n, const Ipopt::Number* x_l_new, const Ipopt::Number* x_u_new,
        Ipopt::IteratesVector& warm_start_iterate);

  private:
    /**@name Default Compiler Generated Methods
     * (Hidden to avoid implicit creation/calling).
     * These methods are not implemented and
     * we do not want the compiler to implement
     * them for us, so we declare them private
     * and do not define them. This ensures that
     * they will not be implicitly created/called. */
    //@{
    /** Default constructor. */
    IpoptInteriorWarmStarter();

    /** Copy Constructor */
    IpoptInteriorWarmStarter(const IpoptInteriorWarmStarter&);

    /** Overloaded Equals Operator */
    void operator=(const IpoptInteriorWarmStarter&);
    //@}

    //@{
    /** Value for a lower bound that denotes -infinity */
    Ipopt::Number nlp_lower_bound_inf_;
    /** Value for a upper bound that denotes infinity */
    Ipopt::Number nlp_upper_bound_inf_;
    /** Flag indicating whether more than one iterate is to be
     *  stored. */
    bool store_several_iterates_;
    //@}

    /** @name Copy of the bounds for the previously solved NLP.  This is
     *  required to find out the remapping for fixed variables, and it
     *  might also help to see how large the perturbation of the new
     *  problem is. */
    //@{
    Ipopt::Index n_;
    Ipopt::Number* x_l_prev_;
    Ipopt::Number* x_u_prev_;
    //@}

    /** @name Selected Iterates and quantities from the previous
     *  optimization */
    //@{
    Ipopt::Index n_stored_iterates_;
    std::vector<Ipopt::Index> stored_iter_;
    std::vector<Ipopt::SmartPtr<const Ipopt::IteratesVector> > stored_iterates_;
    std::vector<Ipopt::Number> stored_mu_;
    std::vector<Ipopt::Number> stored_nlp_error_;
    std::vector<Ipopt::Number> stored_primal_inf_;
    std::vector<Ipopt::Number> stored_dual_inf_;
    std::vector<Ipopt::Number> stored_compl_;
    //@}
  };
}
#endif
