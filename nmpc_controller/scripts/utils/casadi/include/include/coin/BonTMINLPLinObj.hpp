// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 08/16/2007


#ifndef TMINLPLinObj_H
#define TMINLPLinObj_H

#include "BonTMINLP.hpp"

namespace Bonmin {
/** From a TMINLP, this class adapts to another TMINLP where the original objective is transformed into a constraint
    by adding an extra variable which is minimized.

    More precisely 
    \f[
    \begin{array}{l}
    \min f(x)\\
    s.t\\
    g_l \leq g(x) \leq g_u\\
    x_l \leq x \leq u
    \end{array}
    \f]
    is transformed ino
    \begin{array}{l}
    \min \eta\\
    s.t\\
    -\infty \leq f(x) - \eta \leq 0\\
    g_l \leq g(x) \leq g_u\\
    x_l \leq x \leq u
    \end{array}
    \f]
    The objective is put as first constraint of the problem and the extra variable is the last one.
 .*/
class TMINLPLinObj: public Bonmin::TMINLP {
  public:
   /** Default constructor*/
   TMINLPLinObj();

  /** destructor.*/
  virtual ~TMINLPLinObj();

  /** set reference TMINLP */
  void setTminlp(Ipopt::SmartPtr<TMINLP> tminlp);
  
    /**@name methods to gather information about the MINLP */
    //@{
    /** Return the number of variables
     *  and constraints, and the number of non-zeros in the jacobian and
     *  the hessian. Call tminlp_ one  but number of constraints and non-zeroes in the jacobian is stored internally.*/
        virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                                  Ipopt::Index& nnz_h_lag, Ipopt::TNLP::IndexStyleEnum& index_style);
    /** Return scaling parameters. If tminlp_ method returns true, translate
      * constraint scaling (if asked).
     */
    virtual bool get_scaling_parameters(Ipopt::Number& obj_scaling,
                                        bool& use_x_scaling, Ipopt::Index n,
                                        Ipopt::Number* x_scaling,
                                        bool& use_g_scaling, Ipopt::Index m,
                                        Ipopt::Number* g_scaling);


    /** Get the variable type. Just call tminlp_'s method;. */
    virtual bool get_variables_types(Ipopt::Index n, VariableType* var_types){
      assert(IsValid(tminlp_));
      assert(n == n_);
      var_types[n-1] = TMINLP::CONTINUOUS;
      return tminlp_->get_variables_types(n - 1, var_types);
    }

    /** Return the constraints linearity. Call tminlp_'s method and translate.
      */
    virtual bool get_constraints_linearity(Ipopt::Index m, 
					   Ipopt::TNLP::LinearityType* const_types);

    /** Return the information about the bound
     *  on the variables and constraints. Call tminlp_'s method and translate
     *  constraints bounds.*/
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
        Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    /** Return the starting point. 
        Have to translate z_L and z_U.
     */
    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
        Ipopt::Index m, bool init_lambda,
        Ipopt::Number* lambda);

    /** Return the value of the objective function.
      * Just call tminlp_ method. */
    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number& obj_value){
        assert(n == n_);
        obj_value = x[n-1];
       return true;}

    /** Return the vector of the gradient of
     *  the objective w.r.t. x. Just call tminlp_ method. */
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number* grad_f){
       assert(IsValid(tminlp_));
       assert(n == n_);
       n--;
       for(int  i = 0 ; i < n ; i++){
        grad_f[i] = 0;}
       grad_f[n] = 1;
       return true;}

    /** Return the vector of constraint values.
      * Use tminlp_ functions and use mapping to get the needed values. */
    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Index m, Ipopt::Number* g);

    /** Return the jacobian of the constraints. 
      * In first call nothing to change. In later just fix the values for the simple concaves
      * and remove entries corresponding to nonConvex constraints. */
    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
        Ipopt::Index *jCol, Ipopt::Number* values);

    /** \brief Return the hessian of the lagrangian. 
      * Here we just put lambda in the correct format and call
      * tminlp_'s function.*/
    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
        bool new_lambda, Ipopt::Index nele_hess,
        Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values);
    /** Compute the value of a single constraint. The constraint
     *  number is i (starting counting from 0. */
    virtual bool eval_gi(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			 Ipopt::Index i, Ipopt::Number& gi);
    /** Compute the structure or values of the gradient for one
     *  constraint. The constraint * number is i (starting counting
     *  from 0.  Other things are like with eval_jac_g. */
    virtual bool eval_grad_gi(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			      Ipopt::Index i, Ipopt::Index& nele_grad_gi, Ipopt::Index* jCol,
			      Ipopt::Number* values);
    //@}
   
    virtual bool get_variables_linearity(Ipopt::Index n, Ipopt::TNLP::LinearityType* c){
      assert(IsValid(tminlp_));
      assert(n == n_);
      bool r_val = tminlp_->get_variables_linearity(n-1, c);
      c[n - 1] = Ipopt::TNLP::LINEAR;
      return r_val;
    }


    /** @name Solution Methods */
    //@{
     /**  Use tminlp_ function.*/
    virtual void finalize_solution(TMINLP::SolverReturn status,
                                   Ipopt::Index n, const Ipopt::Number* x, Ipopt::Number obj_value){
       return tminlp_->finalize_solution(status, n - 1, x,
                                  obj_value);
    }
    //@}
    
     /**  Use tminlp_ function.*/
    virtual const BranchingInfo * branchingInfo() const{
      return tminlp_->branchingInfo();
    }

     /**  Use tminlp_ function.
          \bug Has to translate sos information.*/
    virtual const SosInfo * sosConstraints() const{
      return tminlp_->sosConstraints();
    }
     /**  Use tminlp_ function.*/
    virtual const PerturbInfo* perturbInfo() const
    {
      return tminlp_->perturbInfo();
    }

    /**  Use tminlp_ function.*/
    virtual bool hasUpperBoundingObjective(){
      assert(IsValid(tminlp_));
      return tminlp_->hasUpperBoundingObjective();}
    
    /** Use tminlp_ function.*/
    virtual bool eval_upper_bound_f(Ipopt::Index n, const Ipopt::Number* x,
                                    Ipopt::Number& obj_value){
       assert(IsValid(tminlp_));
       return tminlp_->eval_upper_bound_f(n - 1, x, obj_value); }

  /** Say if problem has a linear objective (for OA) */
  virtual bool hasLinearObjective(){return true;}
  /** return pointer to tminlp_.*/
  Ipopt::SmartPtr<TMINLP> tminlp(){return tminlp_;}
  private:
  /** Reset all data.*/
   void gutsOfDestructor();

  /** Reference TMINLP which is to be relaxed.*/
  Ipopt::SmartPtr<TMINLP> tminlp_;
  /** Ipopt::Number of constraints in the transformed MINLP.*/
  int m_;
  /** Ipopt::Number of variables in the transformed MINLP.*/
  int n_;
  /** number of non-zeroes in the jacobian of the transformed MINLP.*/
  int nnz_jac_;
  /** offset for jacobian.*/
  int offset_;
   
};


}/* Ends Bonmin namepsace.*/

#endif

