// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 10/16/2007
#ifndef BonminTMINLP2OsiLP_H
#define BonminTMINLP2OsiLP_H

#include <cmath>
#include <cstdio>
#include "IpSmartPtr.hpp"
#include "IpTNLP.hpp"
#include "BonTypes.hpp"

class OsiSolverInterface;
class OsiCuts;

namespace Bonmin {
  class TMINLP2TNLP;
  class BabSetupBase;

  /** A transformer class to build outer approximations i.e. transfomrs nonlinear programs into linear programs.*/
  class TMINLP2OsiLP: public Ipopt::ReferencedObject {

  public:

   /** Default constructor.*/
   TMINLP2OsiLP():
    tiny_(-0.),
    very_tiny_(-0.)
   {}

   /** Copy constructor.*/
   TMINLP2OsiLP(const TMINLP2OsiLP & other):
    tiny_(other.tiny_),
    very_tiny_(other.very_tiny_),
    model_(other.model_){
    }

    /** virtual copy constructor*/
    virtual TMINLP2OsiLP * clone() const = 0;

   void set_tols(double tiny, double very_tiny, double rhs_relax, double infty){
     tiny_ = tiny;
     very_tiny_ = very_tiny;
     rhs_relax_ = rhs_relax;
     infty_ = infty;
   }

   void set_model(Bonmin::TMINLP2TNLP * model){
     model_ = model;
     initialize_jac_storage();
   }

   /** Assignment operator.*/
   TMINLP2OsiLP & operator=(const TMINLP2OsiLP& rhs){
    if(this != & rhs){
      tiny_ = rhs.tiny_;
      very_tiny_ = rhs.very_tiny_;
      model_ = rhs.model_;
    }
    return (*this);
   }

   /** Destructor.*/
   ~TMINLP2OsiLP(){}

   /** Build the Outer approximation of model_ in x and put it in si.*/
   virtual void extract(OsiSolverInterface *si, 
                const double * x, bool getObj) = 0;

   
/** Get OAs of nonlinear constraints in x.*/
   virtual void get_refined_oa(OsiCuts & cs
                ) const = 0;

/** Get OAs of nonlinear constraints in x.*/
   virtual void get_oas(OsiCuts & cs, 
                const double * x, bool getObj, bool global) const = 0;


   
   protected:
   /** Facilitator to clean up coefficient.*/
  inline bool cleanNnz(double &value, double colLower, double colUpper,
    double rowLower, double rowUpper, double colsol,
    double & lb, double &ub, double tiny, double veryTiny) const;
   /** If constraint coefficient is below this, we try to remove it.*/
   double tiny_;
   /** If constraint coefficient is below this, we neglect it.*/
   double very_tiny_;
   /** Amount by which to relax OA constraints RHSes*/
   double rhs_relax_;
   /** infinity.*/
   double infty_;
   /** Count the number of linear outer approximations taken.*/
   static int nTimesCalled;

   /** Cache Jacobian matrix*/
   /** Columns of jacobian.*/
   mutable vector<int> jCol_;
   /** Rows of jacobian.*/
   mutable vector<int> iRow_;
   /** Values of jacobian.*/
   mutable vector<double> value_;

   vector<Ipopt::TNLP::LinearityType> const_types_;
 
   void initialize_jac_storage();

   Ipopt::SmartPtr<Bonmin::TMINLP2TNLP> model_;
  };

//A procedure to try to remove small coefficients in OA cuts (or make it non small
inline
bool 
TMINLP2OsiLP::cleanNnz(double &value, double colLower, double colUpper,
    double rowLower, double rowUpper, double colsol,
    double & lb, double &ub, double tiny, double veryTiny) const
{
  if(fabs(value)>= tiny) return 1;
  //fprintf(stderr, "Warning: small coefficient %g\n", tiny);

  if(fabs(value)<veryTiny) return 0;//Take the risk?

  //try and remove
  double infty = 1e20;
  bool colUpBounded = colUpper < 10000;
  bool colLoBounded = colLower > -10000;
  bool rowNotLoBounded =  rowLower <= - infty;
  bool rowNotUpBounded = rowUpper >= infty;
  bool pos =  value > 0;

  if(colLoBounded && !pos && rowNotUpBounded) {
    lb += value * (colsol - colLower);
    return 0;
  }
  else
    if(colLoBounded && pos && rowNotLoBounded) {
      ub += value * (colsol - colLower);
      return 0;
    }
    else
      if(colUpBounded && pos && rowNotUpBounded) {
        lb += value * (colsol - colUpper);
        return 0;
      }
      else
        if(colUpBounded && !pos && rowNotLoBounded) {
          ub += value * (colsol - colUpper);
          return 0;
        }
  //can not remove coefficient 
  return 1;
}


}

#endif

