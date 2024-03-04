// (C) Copyright International Business Machines Corporation and Carnegie Mellon University 2004, 2006
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, Carnegie Mellon University,
// Carl D. Laird, Carnegie Mellon University,
// Andreas Waechter, International Business Machines Corporation
//
// Date : 12/01/2004

#ifndef __TMINLP2TNLP_HPP__
#define __TMINLP2TNLP_HPP__

#include "IpTNLP.hpp"
#include "BonTMINLP.hpp"
#include "IpSmartPtr.hpp"
#include "IpIpoptApplication.hpp"
#include "IpOptionsList.hpp"
#include "BonTypes.hpp"

namespace Bonmin
{
  class IpoptInteriorWarmStarter;

  /** This is an adapter class that converts a TMINLP to
   *  a TNLP to be solved by Ipopt. It allows an external
   *  caller to modify the bounds of variables, allowing
   *  the treatment of binary and integer variables as
   *  relaxed, or fixed
   */
  class TMINLP2TNLP : public Ipopt::TNLP
  {
  public:
    /**@name Constructors/Destructors */
    //@{
    TMINLP2TNLP(const Ipopt::SmartPtr<TMINLP> tminlp
#ifdef WARM_STARTER
        ,
        const OptionsList& options
#endif
        );

    /** Copy Constructor 
      * \warning source and copy point to the same tminlp_.
      */
    TMINLP2TNLP(const TMINLP2TNLP&);

    /** virtual copy .*/
    virtual TMINLP2TNLP * clone() const{
       return new TMINLP2TNLP(*this);}

    /** Default destructor */
    virtual ~TMINLP2TNLP();
    //@}

    /**@name Methods to modify the MINLP and form the NLP */
    //@{

    /** Get the number of variables */
    inline Ipopt::Index num_variables() const
    {
      assert(x_l_.size() == x_u_.size());
      return static_cast<int>(x_l_.size());
    }

    /** Get the number of constraints */
    inline Ipopt::Index num_constraints() const
    {
      assert(g_l_.size() == g_u_.size());
      return static_cast<int>(g_l_.size());
    }
    /** Get the nomber of nz in hessian */
    Ipopt::Index nnz_h_lag()
    {
      return nnz_h_lag_;
    }
    /** Get the variable types */
    const TMINLP::VariableType* var_types()
    {
      return &var_types_[0];
    }

    /** Get the current values for the lower bounds */
    const Ipopt::Number* x_l()
    {
      return &x_l_[0];
    }
    /** Get the current values for the upper bounds */
    const Ipopt::Number* x_u()
    {
      return &x_u_[0];
    }

    /** Get the original values for the lower bounds */
    const Ipopt::Number* orig_x_l() const
    {
      return &orig_x_l_[0];
    }
    /** Get the original values for the upper bounds */
    const Ipopt::Number* orig_x_u() const
    {
      return orig_x_u_();
    }

    /** Get the current values for constraints lower bounds */
    const Ipopt::Number* g_l()
    {
      return g_l_();
    }
    /** Get the current values for constraints upper bounds */
    const Ipopt::Number* g_u()
    {
      return g_u_();
    }

    /** get the starting primal point */
    const Ipopt::Number * x_init() const
    {
      return x_init_();
    }

    /** get the user provided starting primal point */
    const Ipopt::Number * x_init_user() const
    {
      return x_init_user_();
    }

    /** get the starting dual point */
    const Ipopt::Number * duals_init() const
    {
      return duals_init_;
    }

    /** get the solution values */
    const Ipopt::Number* x_sol() const
    {
      return x_sol_();
    }

    /** get the g solution (activities) */
    const Ipopt::Number* g_sol() const
    {
      return g_sol_();
    }

    /** get the dual values */
    const Ipopt::Number* duals_sol() const
    {
      return duals_sol_();
    }

    /** Get Optimization status */
    Ipopt::SolverReturn optimization_status() const
    {
      return return_status_;
    }

    /** Get the objective value */
    Ipopt::Number obj_value() const
    {
      return obj_value_;
    }

    /** Manually set objective value. */
    void set_obj_value(Ipopt::Number value)
    {
      obj_value_ = value;
    }

    /** force solution to be fractionnal.*/
    void force_fractionnal_sol();

    /** Change the bounds on the variables */
    void SetVariablesBounds(Ipopt::Index n,
                            const Ipopt::Number * x_l,
                            const Ipopt::Number * x_u);

    /** Change the lower bound on the variables */
    void SetVariablesLowerBounds(Ipopt::Index n,
                               const Ipopt::Number * x_l);

    /** Change the upper bound on the variable */
    void SetVariablesUpperBounds(Ipopt::Index n,
                                const Ipopt::Number * x_u);

    /** Change the bounds on the variable */
    void SetVariableBounds(Ipopt::Index var_no, Ipopt::Number x_l, Ipopt::Number x_u);

    /** Change the lower bound on the variable */
    void SetVariableLowerBound(Ipopt::Index var_no, Ipopt::Number x_l);

    /** Change the upper bound on the variable */
    void SetVariableUpperBound(Ipopt::Index var_no, Ipopt::Number x_u);

    /** reset the starting point to original one. */
    void resetStartingPoint();

    /** set the starting point to x_init */
    void setxInit(Ipopt::Index n,const Ipopt::Number* x_init);

    /** set the dual starting point to duals_init */
    void setDualsInit(Ipopt::Index n, const Ipopt::Number* duals_init);

    /** xInit has been set?
      * \return 0 if not, 1 if only primal 2 if primal dual.*/
    int has_x_init(){
      if(x_init_.empty()) return 0;
      if(duals_init_) return 2;
      return 1;
    }
    /** Set the contiuous solution */
    void Set_x_sol(Ipopt::Index n, const Ipopt::Number* x_sol);

    /** Set the contiuous dual solution */
    void Set_dual_sol(Ipopt::Index n, const Ipopt::Number* dual_sol);

    /** Change the type of the variable */
    void SetVariableType(Ipopt::Index n, TMINLP::VariableType type);
    //@}
    /** Procedure to ouptut relevant informations to reproduce a sub-problem.
      Compare the current problem to the problem to solve
      and writes files with bounds which have changed and current starting point.
      */
    void outputDiffs(const std::string& probName, const std::string* varNames);

    /**@name methods to gather information about the NLP */
    //@{
    /** This call is just passed onto the TMINLP object */
    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
        Ipopt::Index& nnz_h_lag,
        TNLP::IndexStyleEnum& index_style);

    /** The caller is allowed to modify the bounds, so this
     *  method returns the internal bounds information
     */
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
        Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    /** Returns the constraint linearity.
     * array should be alocated with length at least m..*/
    virtual bool get_constraints_linearity(Ipopt::Index m, LinearityType* const_types)
    {
      return tminlp_->get_constraints_linearity(m, const_types);
    }

    /** Returns the variables linearity.
     * array should be alocated with length at least n..*/
    virtual bool get_variables_linearity(Ipopt::Index n, LinearityType* var_types)
    {
      return tminlp_->get_variables_linearity(n, var_types);
    }

    /** returns true if objective is linear.*/
    virtual bool hasLinearObjective(){return tminlp_->hasLinearObjective();}
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

    /** Method that returns scaling parameters. 
     */
    virtual bool get_scaling_parameters(Ipopt::Number& obj_scaling,
                                        bool& use_x_scaling, Ipopt::Index n,
                                        Ipopt::Number* x_scaling,
                                        bool& use_g_scaling, Ipopt::Index m,
                                        Ipopt::Number* g_scaling);


    /** Methat that returns an Ipopt IteratesVector that has the
     *  starting point for all internal varibles. */
    virtual bool get_warm_start_iterate(Ipopt::IteratesVector& warm_start_iterate);

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

    /** compute the value of a single constraint */
    virtual bool eval_gi(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			 Ipopt::Index i, Ipopt::Number& gi);
    /** compute the structure or values of the gradient for one
	constraint */
    virtual bool eval_grad_gi(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			      Ipopt::Index i, Ipopt::Index& nele_grad_gi, Ipopt::Index* jCol,
			      Ipopt::Number* values);

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
    //@}

    /** @name Solution Methods */
    //@{
    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution(Ipopt::SolverReturn status,
        Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
        Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
        Ipopt::Number obj_value,
        const Ipopt::IpoptData* ip_data,
        Ipopt::IpoptCalculatedQuantities* ip_cq);
    /** Intermediate Callback method for the user.  Providing dummy
     *  default implementation.  For details see IntermediateCallBack
     *  in IpNLP.hpp. */
    virtual bool intermediate_callback(Ipopt::AlgorithmMode mode,
        Ipopt::Index iter, Ipopt::Number obj_value,
        Ipopt::Number inf_pr, Ipopt::Number inf_du,
        Ipopt::Number mu, Ipopt::Number d_norm,
        Ipopt::Number regularization_size,
        Ipopt::Number alpha_du, Ipopt::Number alpha_pr,
        Ipopt::Index ls_trials,
        const Ipopt::IpoptData* ip_data,
        Ipopt::IpoptCalculatedQuantities* ip_cq);
    //@}

    /** Method called to check wether a problem has still some variable not fixed. If there are no more
        unfixed vars, checks wether the solution given by the bounds is feasible.*/

    /** @name Methods for setting and getting the warm starter */
    //@{
    void SetWarmStarter(Ipopt::SmartPtr<IpoptInteriorWarmStarter> warm_starter);

      Ipopt::SmartPtr<IpoptInteriorWarmStarter> GetWarmStarter();

    //@}
      
      /** Say if has a specific function to compute upper bounds*/
      virtual bool hasUpperBoundingObjective(){
        return tminlp_->hasUpperBoundingObjective();}

      /** Evaluate the upper bounding function at given point and store the result.*/
    double evaluateUpperBoundingFunction(const double * x);
    
    /** \name Cuts management. */
    /** Methods are not implemented at this point. But I need the interface.*/
    //@{


    /** Add some linear cuts to the problem formulation (not implemented yet in base class).*/
   virtual void addCuts(unsigned int numberCuts, const OsiRowCut ** cuts){
    if(numberCuts > 0)
    throw CoinError("BonTMINLP2TNLP", "addCuts", "Not implemented");}


  /** Add some cuts to the problem formulaiton (handles Quadratics).*/
  virtual void addCuts(const OsiCuts &cuts){
    if(cuts.sizeRowCuts() > 0 || cuts.sizeColCuts() > 0)
    throw CoinError("BonTMINLP2TNLP", "addCuts", "Not implemented");}

  /** Remove some cuts to the formulation */
  virtual void removeCuts(unsigned int number ,const int * toRemove){
    if(number > 0)
    throw CoinError("BonTMINLP2TNLP", "removeCuts", "Not implemented");}

    //@}


  /** Access array describing constraint to which perspectives should be applied.*/
  virtual const int * get_const_xtra_id() const{
     return tminlp_->get_const_xtra_id();
  }

    /** Round and check the current solution, return norm inf of constraint violation.*/
    double check_solution(OsiObject ** objects = 0, int nObjects = -1);
   protected:
   /** \name These should be modified in derived class to always maintain there correctness.
             They are directly queried by OsiTMINLPInterface without virtual function for 
             speed.*/
    /** @{ */
    /// Types of the variable (TMINLP::CONTINUOUS, TMINLP::INTEGER, TMINLP::BINARY).
    vector<TMINLP::VariableType> var_types_;
    /// Current lower bounds on variables
    vector<Ipopt::Number> x_l_;
    /// Current upper bounds on variables
    vector<Ipopt::Number> x_u_;
    /// Original lower bounds on variables
    vector<Ipopt::Number> orig_x_l_;
    /// Original upper bounds on variables
    vector<Ipopt::Number> orig_x_u_;
    /// Lower bounds on constraints values
    vector<Ipopt::Number> g_l_; 
    /// Upper bounds on constraints values
    vector<Ipopt::Number> g_u_;
    /// Initial primal point
    vector<Ipopt::Number> x_init_;
    /** Initial values for all dual multipliers (constraints then lower bounds then upper bounds) */
    Ipopt::Number * duals_init_;
    /// User-provideed initial prmal point
    vector<Ipopt::Number> x_init_user_;
    /// Optimal solution
    vector<Ipopt::Number> x_sol_;
    /// Activities of constraint g( x_sol_)
    vector<Ipopt::Number> g_sol_;
    /** Dual multipliers of constraints and bounds*/
    vector<Ipopt::Number> duals_sol_;
    /** @} */

    /** Access number of entries in tminlp_ hessian*/
    Ipopt::Index nnz_h_lag() const{
     return nnz_h_lag_;}
    /** Access number of entries in tminlp_ hessian*/
    Ipopt::Index nnz_jac_g() const{
     return nnz_jac_g_;}

    /** Acces index_style.*/
     TNLP::IndexStyleEnum index_style() const{
       return index_style_;}
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
    TMINLP2TNLP();

    /** Overloaded Equals Operator */
    TMINLP2TNLP& operator=(const TMINLP2TNLP&);
    //@}

    /** pointer to the tminlp that is being adapted */
    Ipopt::SmartPtr<TMINLP> tminlp_;

    /** @name Internal copies of data allowing caller to modify the MINLP */
    //@{
    /// Number of non-zeroes in the constraints jacobian.
    Ipopt::Index nnz_jac_g_;
    /// Number of non-zeroes in the lagrangian hessian
    Ipopt::Index nnz_h_lag_;
    /**index style (fortran or C)*/
    TNLP::IndexStyleEnum index_style_;

    /** Return status of the optimization process*/
    Ipopt::SolverReturn return_status_;
    /** Value of the optimal solution found by Ipopt */
    Ipopt::Number obj_value_;
    //@}

    /** @name Warmstart object and related data */
    //@{
    /** Pointer to object that holds warmstart information */
    Ipopt::SmartPtr<IpoptInteriorWarmStarter> curr_warm_starter_;
    /** Value for a lower bound that denotes -infinity */
    Ipopt::Number nlp_lower_bound_inf_;
    /** Value for a upper bound that denotes infinity */
    Ipopt::Number nlp_upper_bound_inf_;
    /** Option from Ipopt - we currently use it to see if we want to
     *  use some clever warm start or just the last iterate from the
     *  previous run */
    bool warm_start_entire_iterate_;
    /** Do we need a new warm starter object */
    bool need_new_warm_starter_;
    //@}


    /** Private method that throws an exception if the variable bounds
     * are not consistent with the variable type */
    void throw_exception_on_bad_variable_bound(Ipopt::Index i);
    
    private:
    // Delete all arrays
    void gutsOfDelete();
    
  /** Copies all the arrays. 
      \warning this and other should be two instances of the same problem
      \warning AW: I am trying to mimic a copy construction for Cbc
      use with great care not safe.
  */
    void gutsOfCopy(const TMINLP2TNLP &source);
  };

} // namespace Ipopt

#endif
