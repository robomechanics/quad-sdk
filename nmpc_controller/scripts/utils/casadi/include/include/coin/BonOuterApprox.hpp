// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 10/16/2007
#ifndef BonminOuterApprox_H
#define BonminOuterApprox_H

#include <cmath>

namespace Bonmin{
   class OsiTMINLPInterface;
   class BabSetupBase;
}
class OsiSolverInterface;
namespace Bonmin {
  /** A class to build outer approximations.*/
  class OuterApprox{

  public:

   /** Default constructor.*/
   OuterApprox():
    tiny_(-0.),
    veryTiny_(-0.)
   {}

   /** Copy constructor.*/
   OuterApprox(const OuterApprox & other):
    tiny_(other.tiny_),
    veryTiny_(other.veryTiny_){
    }


   /** Assignment operator.*/
   OuterApprox & operator=(const OuterApprox& rhs){
    if(this != & rhs){
      tiny_ = rhs.tiny_;
      veryTiny_ = rhs.veryTiny_;}
    return (*this);
   }

   /** Destructor.*/
   ~OuterApprox(){}

   /** Initialize using options.*/
   void initialize(Bonmin::BabSetupBase &b);

   /** Build the Outer approximation in minlp and put it in si.*/
   void extractLinearRelaxation(Bonmin::OsiTMINLPInterface &minlp,
                                OsiSolverInterface *si, 
                                const double * x, bool getObj);
   /** Operator() calls extractLinearRelaxation*/
   void operator()(Bonmin::OsiTMINLPInterface &minlp,
                   OsiSolverInterface *si,
                   const double * x, bool getObj){
       extractLinearRelaxation(minlp, si, x, getObj);}

   private:
   /** Facilitator to clean up coefficient.*/
  inline bool cleanNnz(double &value, double colLower, double colUpper,
    double rowLower, double rowUpper, double colsol,
    double & lb, double &ub, double tiny, double veryTiny);
   /** If constraint coefficient is below this, we try to remove it.*/
   double tiny_;
   /** If constraint coefficient is below this, we neglect it.*/
   double veryTiny_;
   /** Count the number of linear outer approximations taken.*/
   static int nTimesCalled;
  };

//A procedure to try to remove small coefficients in OA cuts (or make it non small
inline
bool 
OuterApprox::cleanNnz(double &value, double colLower, double colUpper,
    double rowLower, double rowUpper, double colsol,
    double & lb, double &ub, double tiny, double veryTiny)
{
  if(fabs(value)>= tiny) return 1;

  if(fabs(value)<veryTiny) return 0;//Take the risk?

  //try and remove
  double infty = 1e20;
  bool colUpBounded = colUpper < 10000;
  bool colLoBounded = colLower > -10000;
  bool rowNotLoBounded =  rowLower <= - infty;
  bool rowNotUpBounded = rowUpper >= infty;
  bool pos =  value > 0;

  if(colLoBounded && pos && rowNotUpBounded) {
    lb += value * (colsol - colLower);
    return 0;
  }
  else
    if(colLoBounded && !pos && rowNotLoBounded) {
      ub += value * (colsol - colLower);
      return 0;
    }
    else
      if(colUpBounded && !pos && rowNotUpBounded) {
        lb += value * (colsol - colUpper);
        return 0;
      }
      else
        if(colUpBounded && pos && rowNotLoBounded) {
          ub += value * (colsol - colUpper);
          return 0;
        }
  //can not remove coefficient increase it to smallest non zero
  if(pos) value = tiny;
  else
    value = - tiny;
  return 1;
}

}

#endif

