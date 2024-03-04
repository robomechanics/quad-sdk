// (C) Copyright International Business Machines Corporation and
// Carnegie Mellon University 2006, 2008
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Andreas Waechter, International Business Machines Corporation
//                   (derived from BonTMINLP2TNLP.hpp)            12/22/2006

#ifndef __BONBRANCHINGTQP_HPP__
#define __BONBRANCHINGTQP_HPP__

#include "BonTMINLP2TNLP.hpp"

namespace Bonmin
{
  /** This is an adapter class that converts a TMINLP2TNLP object into
   *  a TNLP, which is now just a QP.  The QP is the linear quadratic
   *  of the TNLP at the optimal point.  The purpose of the
   *  BranchingTQP is that it is used in a strong-branching framework,
   *  strong branching is only done for the QP approximation of the
   *  TNLP, not on the TNLP itself.  The variables of the QP are the
   *  displacement from the reference point.
   */
  class BranchingTQP : public Ipopt::TNLP
  {
  public:
    /**@name Constructors/Destructors */
    //@{
    BranchingTQP(Ipopt::SmartPtr<TMINLP2TNLP> tminlp2tnlp);

    /** Default destructor */
    virtual ~BranchingTQP();
    //@}

    /**@name methods to gather information about the NLP, only those
     *  that need to be overloaded from TNLP */
    //@{
    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, Ipopt::TNLP::IndexStyleEnum& index_style);
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);
    /** Returns the constraint linearity.  array should be alocated
     * with length at least n. Since this is a QP, all constraints are
     * linear.*/
    virtual bool get_constraints_linearity(Ipopt::Index m, LinearityType* const_types);
    /** Method called by Ipopt to get the starting point. The bools
     *  init_x and init_lambda are both inputs and outputs. As inputs,
     *  they indicate whether or not the algorithm wants you to
     *  initialize x and lambda respectively. If, for some reason, the
     *  algorithm wants you to initialize these and you cannot, set
     *  the respective bool to false.
     */
    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
        bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
        Ipopt::Index m, bool init_lambda,
        Ipopt::Number* lambda);

    /** Returns the value of the objective function in x*/
    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number& obj_value);

    /** Returns the vector of the gradient of
     *  the objective w.r.t. x */
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number* grad_f);

    /** Returns the vector of constraint values in x*/
    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Index m, Ipopt::Number* g);

    /** Returns the jacobian of the
     *  constraints. The vectors iRow and jCol only need to be set
     *  once. The first call is used to set the structure only (iRow
     *  and jCol will be non-NULL, and values will be NULL) For
     *  subsequent calls, iRow and jCol will be NULL. */
    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
        Ipopt::Index *jCol, Ipopt::Number* values);

    /** Return the hessian of the
     *  lagrangian. The vectors iRow and jCol only need to be set once
     *  (during the first call). The first call is used to set the
     *  structure only (iRow and jCol will be non-NULL, and values
     *  will be NULL) For subsequent calls, iRow and jCol will be
     *  NULL. This matrix is symmetric - specify the lower diagonal
     *  only */
    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
        bool new_lambda, Ipopt::Index nele_hess,
        Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values);
    virtual void finalize_solution(Ipopt::SolverReturn status,
                                   Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
                                   Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
                                   Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data,
                                   Ipopt::IpoptCalculatedQuantities* ip_cq);
    //@}

    /** Accessor Methods for QP data */
    //@{
    const Ipopt::Number ObjVal()
    {
      return obj_val_;
    }
    const Ipopt::Number* ObjGrad()
    {
      return obj_grad_;
    }
    const Ipopt::Number* ObjHessVals()
    {
      return obj_hess_;
    }
    const Ipopt::Index* ObjHessIRow()
    {
      return obj_hess_irow_;
    }
    const Ipopt::Index* ObjHessJCol()
    {
      return obj_hess_jcol_;
    }
    const Ipopt::Number* ConstrRhs()
    {
      return g_vals_;
    }
    const Ipopt::Number* ConstrJacVals()
    {
      return g_jac_;
    }
    const Ipopt::Index* ConstrJacIRow()
    {
      return g_jac_irow_;
    }
    const Ipopt::Index* ConstrJacJCol()
    {
      return g_jac_jcol_;
    }
    //@}

  private:
    /**@name Default Compiler Generated Methods
     * (Hidden to avoid implicit creation/calling).
     * These methods are not implemented and
     * we do not want the compiler to implement
     * them for us, so we declare them private
     * and do not define them. This ensures that
     * they will not be implicitly created/called. */
    //@{
    /** Default Constructor */
    BranchingTQP();

    /** Copy Constructor */
    BranchingTQP(const BranchingTQP&);

    /** Overloaded Equals Operator */
    void operator=(const BranchingTQP&);
    //@}

    /** @name static information about the QP's constraints and
     *  objective function */
    //@{
    Ipopt::Number obj_val_;
    Ipopt::Number* obj_grad_;
    Ipopt::Number* obj_hess_;
    Ipopt::Index* obj_hess_irow_;
    Ipopt::Index* obj_hess_jcol_;
    Ipopt::Number* g_vals_;
    Ipopt::Number* g_jac_;
    Ipopt::Index* g_jac_irow_;
    Ipopt::Index* g_jac_jcol_;
    //@}

    /** @name Data from the MINLP */
    //@{
    Ipopt::Index n_;
    Ipopt::Index m_;
    Ipopt::Index nnz_jac_g_;
    Ipopt::Index nnz_h_lag_;
    Ipopt::TNLP::IndexStyleEnum index_style_;
    //@}

    /** Copy of original x_sol_.  x_sol_ is changed after the first QP
     *  has been solved once. */
    Ipopt::Number* x_sol_copy_;

    /** Copy of original duals_sol_.  duals_sol_ is changed after the
     *  first QP has been solved once. */
    Ipopt::Number* duals_sol_copy_;

    /** Pointer to the TMINLP2TNLP model which stores the bounds
     *  information */
    Ipopt::SmartPtr<TMINLP2TNLP> tminlp2tnlp_;
  };

} // namespace Ipopt

#endif
