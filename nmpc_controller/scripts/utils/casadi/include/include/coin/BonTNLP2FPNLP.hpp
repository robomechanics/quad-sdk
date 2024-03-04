// Copyright (C) 2004, International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
//
// Authors:  Pierre Bonami 06/10/2005

#ifndef _TNLP2FPNLP_HPP_
#define _TNLP2FPNLP_HPP_

#include "IpTNLP.hpp"
#include "BonTMINLP.hpp"
#include "IpSmartPtr.hpp"
#include "BonTypes.hpp"

namespace Bonmin
{
  /** This is an adapter class to convert an NLP to a Feasibility Pump NLP
   *  by changing the objective function to the (2-norm) distance to a point.
   * The extra function is set_dist_to_point_obj(size_t n, const double *, const int *)
   */
  class TNLP2FPNLP : public Ipopt::TNLP
  {
  public:
    /**@name Constructors/Destructors */
    //@{
    /** Build using tnlp as source problem.*/
    TNLP2FPNLP(const Ipopt::SmartPtr<Ipopt::TNLP> tnlp, double objectiveScalingFactor = 100);

    /** Build using tnlp as source problem and using other for all other parameters..*/
    TNLP2FPNLP(const Ipopt::SmartPtr<TNLP> tnlp, const Ipopt::SmartPtr<TNLP2FPNLP> other);

    /** Default destructor */
    virtual ~TNLP2FPNLP();
    //@}
    void use(Ipopt::SmartPtr<TNLP> tnlp){
      tnlp_ = GetRawPtr(tnlp);}
    /**@name Methods to select the objective function and extra constraints*/
    //@{
    /// Flag to indicate that we want to use the feasibility pump objective
    void set_use_feasibility_pump_objective(bool use_feasibility_pump_objective) 
    { use_feasibility_pump_objective_ = use_feasibility_pump_objective; }

    /** Flag to indicate that we want to use a cutoff constraint
     *	This constraint has the form f(x) <= (1-epsilon) f(x') */ 
    void set_use_cutoff_constraint(bool use_cutoff_constraint)
    { use_cutoff_constraint_ = use_cutoff_constraint; }

    /// Flag to indicate that we want to use a local branching constraint
    void set_use_local_branching_constraint(bool use_local_branching_constraint)
    { use_local_branching_constraint_ = use_local_branching_constraint; }
    //@}

    /**@name Methods to provide the rhs of the extra constraints*/
    //@{
    /// Set the cutoff value to use in the cutoff constraint
    void set_cutoff(Ipopt::Number cutoff);

    /// Set the rhs of the local branching constraint
    void set_rhs_local_branching_constraint(double rhs_local_branching_constraint)
    { assert(rhs_local_branching_constraint >= 0);
      rhs_local_branching_constraint_ = rhs_local_branching_constraint; }
    //@}

    /**@name Methods to change the objective function*/
    //@{
    /** \brief Set the point to which distance is minimized.
    * The distance is minimize in a subspace define by a subset of coordinates
     * \param n number of coordinates on which distance is minimized
     * \param inds indices of the coordinates on which distance is minimized
     * \param vals values of the point for coordinates in ind
     */
    void set_dist_to_point_obj(size_t n, const Ipopt::Number * vals, const Ipopt::Index * inds);

     /** Set the value for sigma */
     void setSigma(double sigma){
       assert(sigma >= 0.);
       sigma_ = sigma;}
     /** Set the value for lambda*/
     void setLambda(double lambda){
       assert(lambda >= 0. && lambda <= 1.);
       lambda_ = lambda;}
     /** Set the value for simgma */
     void setNorm(int norm){
       assert(norm >0 && norm < 3);
       norm_ = norm;}
    //@}

    /**@name methods to gather information about the NLP */
    //@{
    /** get info from nlp_ and add hessian information */
    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
        Ipopt::Index& nnz_h_lag, Ipopt::TNLP::IndexStyleEnum& index_style);

    /** This call is just passed onto tnlp_
     */
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
				 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    /** Passed onto tnlp_
     */
    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
        bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
        Ipopt::Index m, bool init_lambda,
        Ipopt::Number* lambda)
    {
      int m2 = m;
      if(use_cutoff_constraint_) {
        m2--;
        if(lambda!=NULL)lambda[m2] = 0;
      }
      if(use_local_branching_constraint_) {
        m2--;
        if(lambda!= NULL)lambda[m2] = 0;
      }
      int ret_code = tnlp_->get_starting_point(n, init_x, x,
          init_z, z_L, z_U, m2, init_lambda, lambda);
      return ret_code;
    }

    /** overloaded to return the value of the objective function */
    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number& obj_value);

    /** overload this method to return the vector of the gradient of
     *  the objective w.r.t. x */
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number* grad_f);

    /** overload to return the values of the left-hand side of the
        constraints */
    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			Ipopt::Index m, Ipopt::Number* g);

    /** overload to return the jacobian of g */
    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
			    Ipopt::Index *jCol, Ipopt::Number* values);

    /** Evaluate the modified Hessian of the Lagrangian*/
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
    //@}

    virtual bool get_variables_linearity(Ipopt::Index n, LinearityType* var_types)
    {
      return tnlp_->get_variables_linearity(n, var_types);;
    }

    /** overload this method to return the constraint linearity.
     * array should be alocated with length at least n. (default implementation
     *  just return false and does not fill the array).*/
    virtual bool get_constraints_linearity(Ipopt::Index m, LinearityType* const_types)
    {
      int m2 = m;
      if(use_cutoff_constraint_) {
        m2--;
        const_types[m2] = Ipopt::TNLP::NON_LINEAR;
      } 
      if(use_local_branching_constraint_) {
        m2--;
        const_types[m2] = Ipopt::TNLP::LINEAR;
      }
      return tnlp_->get_constraints_linearity(m2, const_types);
    }
    /** @name Scaling of the objective function */
    //@{
    void setObjectiveScaling(double value)
    {
      objectiveScalingFactor_ = value;
    }
    double getObjectiveScaling() const
    {
      return objectiveScalingFactor_;
    }

  private:
    /** @name Internal methods to help compute the distance, its gradient and hessian */
    //@{
    /** Compute the norm-2 distance to the current point to which distance is minimized. */
    double dist_to_point(const Ipopt::Number *x);
    //@}
    /**@name Default Compiler Generated Methods
     * (Hidden to avoid implicit creation/calling).
     * These methods are not implemented and
     * we do not want the compiler to implement
     * them for us, so we declare them private
     * and do not define them. This ensures that
     * they will not be implicitly created/called. */
    //@{
    /** Default Constructor */
    TNLP2FPNLP();

    /** Copy Constructor */
    TNLP2FPNLP(const TNLP2FPNLP&);

    /** Overloaded Equals Operator */
    void operator=(const TNLP2FPNLP&);
    //@}

    /** pointer to the tminlp that is being adapted */
    Ipopt::SmartPtr<TNLP> tnlp_;

    /** @name Data for storing the point the distance to which is minimized */
    //@{
    /// Indices of the variables for which distance is minimized (i.e. indices of integer variables in a feasibility pump setting)
    vector<Ipopt::Index> inds_;
    /// Values of the point to which we separate (if x is the point vals_[i] should be x[inds_[i]] )
    vector<Ipopt::Number> vals_;
    /** value for the convex combination to take between original objective and distance function.
      * ( take lambda_ * distance + (1-lambda) sigma f(x).*/
    double lambda_;
    /** Scaling for the original objective.*/
    double sigma_;
   /** Norm to use (L_1 or L_2).*/
   int norm_;
    //@}

    /// Scaling factor for the objective
    double objectiveScalingFactor_;

    /**@name Flags to  select the objective function and extra constraints*/
    //@{
    /// Flag to indicate that we want to use the feasibility pump objective
    bool use_feasibility_pump_objective_;

    /** Flag to indicate that we want to use a cutoff constraint
     *	This constraint has the form f(x) <= (1-epsilon) f(x') */ 
    bool use_cutoff_constraint_;    

    /// Flag to indicate that we want to use a local branching constraint
    bool use_local_branching_constraint_;
    //@}

    /**@name Data for storing the rhs of the extra constraints*/
    //@{
    /// Value of best solution known
    double cutoff_;

    /// RHS of local branching constraint
    double rhs_local_branching_constraint_;
    //@}

    /// Ipopt::Index style (C++ or Fortran)
    Ipopt::TNLP::IndexStyleEnum index_style_;

  };

} // namespace Ipopt

#endif /*_TNLP2FPNLP_HPP_*/
