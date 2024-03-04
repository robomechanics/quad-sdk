// (C) Copyright International Business Machines Corporation and
// Carnegie Mellon University 2004, 2007
//
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, Carnegie Mellon University,
// Carl D. Laird, Carnegie Mellon University,
// Andreas Waechter, International Business Machines Corporation
//
// Date : 12/01/2004

#ifndef __TMINLP_HPP__
#define __TMINLP_HPP__

#include "IpUtils.hpp"
#include "IpReferenced.hpp"
#include "IpException.hpp"
#include "IpAlgTypes.hpp"
#include "CoinPackedMatrix.hpp"
#include "OsiCuts.hpp"
#include "IpTNLP.hpp"
#include "CoinError.hpp"
#include "CoinHelperFunctions.hpp"

namespace Bonmin
{
  DECLARE_STD_EXCEPTION(TMINLP_INVALID);
  DECLARE_STD_EXCEPTION(TMINLP_INVALID_VARIABLE_BOUNDS);

  /** Base class for all MINLPs that use a standard triplet matrix form
   *  and dense vectors.
   *  The class TMINLP2TNLP allows the caller to produce a viable TNLP
   *  from the MINLP (by relaxing binary and/or integers, or by
   *  fixing them), which can then be solved by Ipopt.
   *
   *  This interface presents the problem form:
   *  \f[
   *  \begin{array}{rl}
   *     &min f(x)\\
   *
   *     \mbox{s.t.}&\\
   *      &   g^L <= g(x) <= g^U\\
   *
   *       &   x^L <=  x   <= x^U\\
   *   \end{array}
   *  \f]
   *  Where each x_i is either a continuous, binary, or integer variable.
   *  If x_i is binary, the bounds [xL,xU] are assumed to be [0,1].
   *  In order to specify an equality constraint, set gL_i = gU_i =
   *  rhs.  The value that indicates "infinity" for the bounds
   *  (i.e. the variable or constraint has no lower bound (-infinity)
   *  or upper bound (+infinity)) is set through the option
   *  nlp_lower_bound_inf and nlp_upper_bound_inf.  To indicate that a
   *  variable has no upper or lower bound, set the bound to
   *  -ipopt_inf or +ipopt_inf respectively
   */
  class TMINLP : public Ipopt::ReferencedObject
  {
  public:
    friend class TMINLP2TNLP;
    /** Return statuses of algorithm.*/
    enum SolverReturn{
      SUCCESS,
      INFEASIBLE,
      CONTINUOUS_UNBOUNDED,
      LIMIT_EXCEEDED,
      USER_INTERRUPT,
      MINLP_ERROR};
    /** Class to store sos constraints for model */
    struct SosInfo
    {
      /** Number of SOS constraints.*/
      int num;
      /** Type of sos. At present Only type '1' SOS are supported by Cbc*/
      char * types;
      /** priorities of sos constraints.*/
      int * priorities;
      
      /** \name Sparse storage of the elements of the SOS constraints.*/
      /** @{ */
      /** Total number of non zeroes in SOS constraints.*/
      int numNz;
      /** For 0 <= i < nums, start[i] gives the indice of indices and weights arrays at which the description of constraints i begins..*/ 
      int * starts;
      /** indices of elements belonging to the SOS.*/
      int * indices;
      /** weights of the elements of the SOS.*/
      double * weights;
      /** @} */
      /** default constructor. */
      SosInfo();
      /** Copy constructor.*/
      SosInfo(const SosInfo & source);
      

      /** destructor*/
      ~SosInfo()
      {
        gutsOfDestructor();
      }


      /** Reset information */
      void gutsOfDestructor();

    };

    /** Stores branching priorities information. */
    struct BranchingInfo
    {
      /**number of variables*/
      int size;
      /** User set priorities on variables. */
      int * priorities;
      /** User set preferered branching direction. */
      int * branchingDirections;
      /** User set up pseudo costs.*/
      double * upPsCosts;
      /** User set down pseudo costs.*/
      double * downPsCosts;
      BranchingInfo():
      size(0),
      priorities(NULL),
      branchingDirections(NULL),
      upPsCosts(NULL),
      downPsCosts(NULL)
      {}
      BranchingInfo(const BranchingInfo &other)
      {
        gutsOfDestructor();
        size = other.size;
        priorities = CoinCopyOfArray(other.priorities, size);
        branchingDirections = CoinCopyOfArray(other.branchingDirections, size);
        upPsCosts = CoinCopyOfArray(other.upPsCosts, size);
        downPsCosts = CoinCopyOfArray(other.downPsCosts, size);
      }
      void gutsOfDestructor()
      {
      if (priorities != NULL) delete [] priorities;
      priorities = NULL;
      if (branchingDirections != NULL) delete [] branchingDirections;  
      branchingDirections = NULL;
      if (upPsCosts != NULL) delete [] upPsCosts;
      upPsCosts = NULL;
      if (downPsCosts != NULL) delete [] downPsCosts;
      downPsCosts = NULL;
      }
      ~BranchingInfo()
      {
	gutsOfDestructor();
      }
    };

    /** Class to store perturbation radii for variables in the model */
    class PerturbInfo
    {
    public:
      /** default constructor. */
      PerturbInfo() :
	perturb_radius_(NULL)
      {}

      /** destructor*/
      ~PerturbInfo()
      {
        delete [] perturb_radius_;
      }

      /** Method for setting the perturbation radii. */
      void SetPerturbationArray(Ipopt::Index numvars, const double* perturb_radius);

      /** Method for getting the array for the perturbation radii in
       *  order to use the values. */
      const double* GetPerturbationArray() const {
	return perturb_radius_;
      }

    private:
      /** Copy constructor.*/
      PerturbInfo(const PerturbInfo & source);

      /** Perturbation radii for all variables.  A negative value
       *  means that the radius has not been given. If the pointer is
       *  NULL, then no variables have been assigned a perturbation
       *  radius. */
      double* perturb_radius_;
    };

    /** Type of the variables.*/
    enum VariableType
    {
      CONTINUOUS,
      BINARY,
      INTEGER
    };

    /**@name Constructors/Destructors */
    //@{
    TMINLP();

    /** Default destructor */
    virtual ~TMINLP();
    //@}

    /**@name methods to gather information about the MINLP */
    //@{
    /** overload this method to return the number of variables
     *  and constraints, and the number of non-zeros in the jacobian and
     *  the hessian. */
    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
        Ipopt::Index& nnz_h_lag, Ipopt::TNLP::IndexStyleEnum& index_style)=0;

    /** overload this method to return scaling parameters. This is
     *  only called if the options are set to retrieve user scaling.
     *  There, use_x_scaling (or use_g_scaling) should get set to true
     *  only if the variables (or constraints) are to be scaled.  This
     *  method should return true only if the scaling parameters could
     *  be provided.
     */
    virtual bool get_scaling_parameters(Ipopt::Number& obj_scaling,
                                        bool& use_x_scaling, Ipopt::Index n,
                                        Ipopt::Number* x_scaling,
                                        bool& use_g_scaling, Ipopt::Index m,
                                        Ipopt::Number* g_scaling)
    {
      return false;
    }


    /** overload this method to provide the variables types. The var_types
     *  array will be allocated with length n. */
    virtual bool get_variables_types(Ipopt::Index n, VariableType* var_types)=0;

    /** overload this method to provide the variables linearity.
     * array should be allocated with length at least n.*/
    virtual bool get_variables_linearity(Ipopt::Index n, 
					   Ipopt::TNLP::LinearityType* var_types) = 0;

    /** overload this method to provide the constraint linearity.
     * array should be allocated with length at least m.*/
    virtual bool get_constraints_linearity(Ipopt::Index m, 
					   Ipopt::TNLP::LinearityType* const_types) = 0;

    /** overload this method to return the information about the bound
     *  on the variables and constraints. The value that indicates
     *  that a bound does not exist is specified in the parameters
     *  nlp_lower_bound_inf and nlp_upper_bound_inf.  By default,
     *  nlp_lower_bound_inf is -1e19 and nlp_upper_bound_inf is
     *  1e19.
     *  An exception will be thrown if x_l and x_u are not 0,1 for binary variables
     */
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
        Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u)=0;

    /** overload this method to return the starting point. The bools
     *  init_x and init_lambda are both inputs and outputs. As inputs,
     *  they indicate whether or not the algorithm wants you to
     *  initialize x and lambda respectively. If, for some reason, the
     *  algorithm wants you to initialize these and you cannot, set
     *  the respective bool to false.
     */
    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
        Ipopt::Index m, bool init_lambda,
        Ipopt::Number* lambda)=0;

    /** overload this method to return the value of the objective function */
    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number& obj_value)=0;

    /** overload this method to return the vector of the gradient of
     *  the objective w.r.t. x */
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number* grad_f)=0;

    /** overload this method to return the vector of constraint values */
    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Index m, Ipopt::Number* g)=0;

    /** overload this method to return the jacobian of the
     *  constraints. The vectors iRow and jCol only need to be set
     *  once. The first call is used to set the structure only (iRow
     *  and jCol will be non-NULL, and values will be NULL) For
     *  subsequent calls, iRow and jCol will be NULL. */
    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
        Ipopt::Index *jCol, Ipopt::Number* values)=0;

    /** overload this method to return the hessian of the
     *  lagrangian. The vectors iRow and jCol only need to be set once
     *  (during the first call). The first call is used to set the
     *  structure only (iRow and jCol will be non-NULL, and values
     *  will be NULL) For subsequent calls, iRow and jCol will be
     *  NULL. This matrix is symmetric - specify the lower diagonal
     *  only */
    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
        bool new_lambda, Ipopt::Index nele_hess,
        Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values)=0;
    /** Compute the value of a single constraint. The constraint
     *  number is i (starting counting from 0. */
    virtual bool eval_gi(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			 Ipopt::Index i, Ipopt::Number& gi)
    {
      std::cerr << "Method eval_gi not overloaded from TMINLP\n";
      throw -1;
    }
    /** Compute the structure or values of the gradient for one
     *  constraint. The constraint * number is i (starting counting
     *  from 0.  Other things are like with eval_jac_g. */
    virtual bool eval_grad_gi(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			      Ipopt::Index i, Ipopt::Index& nele_grad_gi, Ipopt::Index* jCol,
			      Ipopt::Number* values)
    {
      std::cerr << "Method eval_grad_gi not overloaded from TMINLP\n";
      throw -1;
    }
    //@}

    /** @name Solution Methods */
    //@{
    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution(TMINLP::SolverReturn status,
                                   Ipopt::Index n, const Ipopt::Number* x, Ipopt::Number obj_value) =0;
    //@}
    
    virtual const BranchingInfo * branchingInfo() const = 0;

    virtual const SosInfo * sosConstraints() const = 0;

    virtual const PerturbInfo* perturbInfo() const
    {
      return NULL;
    }

    /** Say if has a specific function to compute upper bounds*/
    virtual bool hasUpperBoundingObjective(){
      return false;}
    
    /** overload this method to return the value of an alternative objective function for
      upper bounding (to use it hasUpperBoundingObjective should return true).*/
    virtual bool eval_upper_bound_f(Ipopt::Index n, const Ipopt::Number* x,
                                    Ipopt::Number& obj_value){ return false; }

   /** Used to mark constraints of the problem.*/
   enum Convexity {
     Convex/** Constraint is convex.*/,
     NonConvex/** Constraint is non-convex.*/,
     SimpleConcave/** Constraint is concave of the simple form y >= F(x).*/};

   /** Structure for marked non-convex constraints. With possibility of
       storing index of a constraint relaxing the non-convex constraint*/
   struct MarkedNonConvex {
      /** Default constructor gives "safe" values.*/
	 MarkedNonConvex():
	 cIdx(-1), cRelaxIdx(-1){}
	 /** Index of the nonconvex constraint.*/
      int cIdx;
	 /** Index of constraint relaxing the nonconvex constraint.*/
	 int cRelaxIdx;};
   /** Structure which describes a constraints of the form
       $f[ y \gt F(x) \f]
	  with \f$ F(x) \f$ a concave function.*/
   struct SimpleConcaveConstraint{
      /** Default constructor gives "safe" values.*/
	 SimpleConcaveConstraint():
	   xIdx(-1), yIdx(-1), cIdx(-1){}
      /** Index of the variable x.*/
      int xIdx;
      /** Index of the variable y.*/
	 int yIdx;
      /** Index of the constraint.*/
	 int cIdx;};
    /** Get accest to constraint convexities.*/
    virtual bool get_constraint_convexities(int m, TMINLP::Convexity * constraints_convexities)const {
      CoinFillN(constraints_convexities, m, TMINLP::Convex);
      return true;}
  /** Get dimension information on nonconvex constraints.*/
  virtual bool get_number_nonconvex(int & number_non_conv, int & number_concave) const{
    number_non_conv = 0;
    number_concave = 0;
    return true;} 
  /** Get array describing the constraints marked nonconvex in the model.*/
  virtual bool get_constraint_convexities(int number_non_conv, MarkedNonConvex * non_convs) const{
    assert(number_non_conv == 0);
    return true;}
  /** Fill array containing indices of simple concave constraints.*/ 
  virtual bool get_simple_concave_constraints(int number_concave, SimpleConcaveConstraint * simple_concave) const{
    assert(number_concave == 0);
    return true;}

  /** Say if problem has a linear objective (for OA) */
  virtual bool hasLinearObjective(){return false;}

  /** Say if problem has general integer variables.*/
  bool hasGeneralInteger();

  /** Access array describing constraint to which perspectives should be applied.*/
  virtual const int * get_const_xtra_id() const{
    return NULL;
  }
  protected:
    /** Copy constructor */
    //@{
    /** Copy Constructor */
    TMINLP(const TMINLP&);

    /** Overloaded Equals Operator */
    void operator=(const TMINLP&);
    //@}

  private:
  };

} // namespace Ipopt

#endif

