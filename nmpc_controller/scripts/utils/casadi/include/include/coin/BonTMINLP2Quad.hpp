// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 10/06/2007

#ifndef __TMINLPQuad_HPP__
#define __TMINLPQuad_HPP__

#include "BonTMINLP2TNLP.hpp"
#include "BonQuadRow.hpp"

namespace Bonmin
{


  /** This is a derived class fro TMINLP2TNLP to handle adding quadratic cuts.
   */
  class TMINLP2TNLPQuadCuts : public Bonmin::TMINLP2TNLP
  {
  public:
    /**@name Constructors/Destructors */
    //@{
    TMINLP2TNLPQuadCuts(const Ipopt::SmartPtr<Bonmin::TMINLP> tminlp
#ifdef WARM_STARTER
        ,
        const OptionsList& options
#endif
        );


    /** Copy Constructor 
      * \warning source and copy point to the same tminlp_.
      */
    TMINLP2TNLPQuadCuts(const TMINLP2TNLPQuadCuts&);

    /** Virtual copy.*/
    virtual Bonmin::TMINLP2TNLP * clone() const{
      printf("Cloning TMINLP2TNLPQuadCuts.\n");
      return new TMINLP2TNLPQuadCuts(*this);}

    /** Destructor */
    virtual ~TMINLP2TNLPQuadCuts();
    //@}
    /**@name methods to gather information about the NLP */
    //@{
    /** This call is just passed onto parent class and add number of quadratic
        cuts*/
    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
        Ipopt::Index& nnz_h_lag,
        Ipopt::TNLP::IndexStyleEnum& index_style);

    /** This call is just passed onto parent class and add bounds of quadratic
        cuts*/
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
        Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    virtual bool get_constraints_linearity(Ipopt::Index m, Ipopt::TNLP::LinearityType* const_types);

    /** This call is just passed onto parent class and add 
        lambda for quadratic cuts*/
    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
        bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
        Ipopt::Index m, bool init_lambda,
        Ipopt::Number* lambda);

    /** Method that returns scaling parameters (passed to parent all quadratic
        not scaled). 
     */
    virtual bool get_scaling_parameters(Ipopt::Number& obj_scaling,
                                        bool& use_x_scaling, Ipopt::Index n,
                                        Ipopt::Number* x_scaling,
                                        bool& use_g_scaling, Ipopt::Index m,
                                        Ipopt::Number* g_scaling);


    /** Returns the value of the objective function in x*/
    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number& obj_value);

    /** Returns the vector of the gradient of
     *  the objective w.r.t. x */
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
        Ipopt::Number* grad_f);

    /** Returns the vector of constraint values in x (appends constraint values for quadratics).*/
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


    /** \name Cuts management. */
    //@{


    /** Add some linear or quadratic cuts to the problem formulation
        if some of the OsiRowCuts are quadratic they will be well understood as long as safe is true.*/
    void addCuts(const Cuts& cuts, bool safe);


    /** Add some cuts to the problem formulaiton (handles Quadratics).*/
    void addCuts(const OsiCuts &cuts);
 
    /** Add some linear cuts to the problem formulation.*/
   virtual void addCuts(unsigned int numberCuts, const OsiRowCut ** cuts);

 
    /** Remove some cuts from the formulation */
    void removeCuts(unsigned int number ,const int * toRemove);

    //@}
    //
    /** Change objective to a linear one whith given objective function.*/
    void set_linear_objective(int n_var, const double * obj, double c_0);

    /** Reset objective to original one */
    void reset_objective(){
      obj_.clear();
    }

  protected:
    /** Add some cuts to the problem formulaiton (handles Quadratics).*/
    void addRowCuts(const OsiCuts &cuts, bool safe);
    /**@name Default Compiler Generated Methods
     * (Hidden to avoid implicit creation/calling).
     * These methods are not implemented and
     * we do not want the compiler to implement
     * them for us, so we declare them private
     * and do not define them. This ensures that
     * they will not be implicitly created/called. */
    //@{
    /** Default Constructor */
    TMINLP2TNLPQuadCuts();

    /** Overloaded Equals Operator */
    TMINLP2TNLPQuadCuts& operator=(const TMINLP2TNLP&);
    //@}

  private:
  /** Some storage for quadratic cuts.*/
  vector<QuadRow *> quadRows_;

  /** Storage for the original hessian of the problem.*/
  AdjustableMat H_;

    /** print H_ for debug.*/
    void printH();
  /** Current umber of entries in the jacobian.*/
  int curr_nnz_jac_;

  /** Store user passed linear objective.*/
  vector<double> obj_;
  /** constant term in objective function.*/
  double c_;
  };

} // namespace Ipopt

#endif

