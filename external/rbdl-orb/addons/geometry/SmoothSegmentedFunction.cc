/* -------------------------------------------------------------------------- *
 *           OpenSim:  SmoothSegmentedFunction.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
/*
 Update:
   This is a port of the original code so that it will work with
   the multibody code RBDL written by Martin Felis.

 Author:
   Matthew Millard

 Date:
   Nov 2015

*/

//=============================================================================
// INCLUDES
//=============================================================================
#include "SmoothSegmentedFunction.h"
#include <fstream>
#include <ostream>
#include <rbdl/rbdl_errors.h>

//=============================================================================
// STATICS
//=============================================================================
//using namespace SimTK;
//using namespace OpenSim;
using namespace std;
using namespace RigidBodyDynamics::Addons::Geometry;

static bool     DEBUG     = false;
static double   UTOL      = std::numeric_limits<double>::epsilon()*1e6;
static double   INTTOL    = std::numeric_limits<double>::epsilon()*1e2;
static double   SQRTEPS   = std::sqrt(numeric_limits<double>::epsilon());
static int      MAXITER   = 20;
static int      NUM_SAMPLE_PTS  = 100;
//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================
/*
 DETAILED COMPUTATIONAL COSTS:
 =========================================================================
     WITHOUT INTEGRAL
     _________________________________________________________________________
            Function  Comp  Div   Mult  Add   Assignments
     _________________________________________________________________________
    member assign                         M:2, 9
    curve gen:    m,m*100   m     m*100       m     m*100*(4)
                 +m*100(3)          +m*100*(3)

    Function detail
      Evaluations Function
      m       SimTK::SplineFitter<double>::
              fitForSmoothingParameter(3,x,u,0).getSpline();
      Cost:     ?

      m*100     SegmentedQuinticBezierToolkit::
              calcQuinticBezierCurveVal
      Cost:                   Mult   Add   Assignments
                          21     20  13

    Total       ~typically  > 2100*m multiplications, additions,
                  > 1000*m assignments
                  > 100*m divisions
     _________________________________________________________________________
          Comp    Div   Mult    Add     Assignments
    Total:    m+m*100(3)  m*100   m*100*21  m*100*20  m*100*13
                          +m+m*100*3  +m*100*4+9+M:2
          + m*Cost(SimTK::SplineFitter ...)
     =========================================================================
    ADDITIONAL COST OF COMPUTING THE INTEGRAL CURVE

                 Comp Div   Mult  Add    Assign
     RK45 Fn.Eval   m*100*(156  12    618   390    456)
     RK45 Overhead  m*100*(?  ?     ?   ?     ? )
     Spline cost    m*100*(?  ?     ?   ?     ? )

     Total:       ~typically > 100,000's mult, additions, assignments
                   > 40,000 comparisions
                   > 3000 divisions

     =========================================================================
    M: Matrix
    V: Vector

    N.B. These costs are dependent on SegmentedQuinticBezierToolkit
*/
SmoothSegmentedFunction::
SmoothSegmentedFunction(
  const RigidBodyDynamics::Math::MatrixNd& mX,
  const RigidBodyDynamics::Math::MatrixNd& mY,
  double x0, double x1,
  double y0, double y1,
  double dydx0, double dydx1,
  const std::string& name):
  _x0(x0),_x1(x1),_y0(y0),_y1(y1),_dydx0(dydx0),_dydx1(dydx1),
  _name(name)
{


  _numBezierSections = mX.cols();
  _mXVec.resize(_numBezierSections);
  _mYVec.resize(_numBezierSections);
  for(int s=0; s < _numBezierSections; s++) {
    _mXVec[s] = mX.col(s);
    _mYVec[s] = mY.col(s);
  }
}

//==============================================================================
SmoothSegmentedFunction::SmoothSegmentedFunction():
  _x0(NAN),_x1(NAN),
  _y0(NAN),_y1(NAN),
  _dydx0(NAN),_dydx1(NAN),_name("NOT_YET_SET")
{
  //_arraySplineUX.resize(0);
  _mXVec.resize(0);
  _mYVec.resize(0);
  //_splineYintX = SimTK::Spline();
  _numBezierSections = (int)NAN;
}

//==============================================================================
void SmoothSegmentedFunction::
updSmoothSegmentedFunction(
  const RigidBodyDynamics::Math::MatrixNd& mX,
  const RigidBodyDynamics::Math::MatrixNd& mY,
  double x0, double x1,
  double y0, double y1,
  double dydx0, double dydx1,
  const std::string& name)
{

  if(mX.rows() != 6 || mY.rows() != 6 || mX.cols() != mY.cols() ) {
    ostringstream errormsg;
    //! [Size Mismatch]
    errormsg << "SmoothSegmentedFunction::updSmoothSegmentedFunction "
             <<  _name.c_str()
             << ": matrices mX and mY must have 6 rows, and the same"
             << " number of columns."
             << endl;
    throw RigidBodyDynamics::Errors::RBDLSizeMismatchError(errormsg.str());
    //! [Size Mismatch]
  }

  _x0   =x0;
  _x1   =x1;
  _y0   =y0;
  _y1   =y1;
  _dydx0=dydx0;
  _dydx1=dydx1;

  if(mX.cols() != _mXVec.size()) {
    _mXVec.resize(mX.cols());
    _mYVec.resize(mY.cols());
  }

  _numBezierSections = mX.cols();
  for(int s=0; s < mX.cols(); s++) {
    _mXVec[s] = mX.col(s);
    _mYVec[s] = mY.col(s);
  }

  _name = name;
}
//==============================================================================
void SmoothSegmentedFunction::shift(double xShift, double yShift)
{
  _x0 += xShift;
  _x1 += xShift;
  _y0 += yShift;
  _y1 += yShift;

  for(int i=0; i<_mXVec.size(); ++i) {
    for(int j=0; j<_mXVec.at(i).rows(); ++j) {
      _mXVec.at(i)[j] += xShift;
      _mYVec.at(i)[j] += yShift;
    }
  }

}

void SmoothSegmentedFunction::scale(double xScale, double yScale)
{

  if( abs( xScale ) <= SQRTEPS) {
    ostringstream errormsg;
    //! [Invalid Parameter]
    errormsg << "SmoothSegmentedFunction::scale "
             <<  _name.c_str()
             << ": xScale must be greater than sqrt(eps). Setting xScale to such"
             << " a small value will cause the slope of the curve to approach "
             << " infinity, or become undefined."
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
    //! [Invalid Parameter]
  }

  _x0 *= xScale;
  _x1 *= xScale;
  _y0 *= yScale;
  _y1 *= yScale;
  _dydx0 *= yScale/xScale;
  _dydx1 *= yScale/xScale;

  for(int i=0; i<_mXVec.size(); ++i) {
    for(int j=0; j<_mXVec.at(i).rows(); ++j) {
      _mXVec.at(i)[j] *= xScale;
      _mYVec.at(i)[j] *= yScale;
    }
  }

}


//==============================================================================


/*Detailed Computational Costs
________________________________________________________________________
 If x is in the Bezier Curve
             Name   Comp.   Div.  Mult.   Add.  Assign.
_______________________________________________________________________
   SegmentedQuinticBezierToolkit::
           calcIndex   3*m+2           1*m   3
             *calcU   15    2    82    42    60
   calcQuinticBezierCurveVal           21    20    13
             total  15+3*m+2  2    103   62+1*m  76

   *Approximate. Uses iteration
________________________________________________________________________
If x is in the linear region

           Name   Comp.   Div.  Mult.   Add.  Assign.
                 1         1    2   1
________________________________________________________________________

*/

double SmoothSegmentedFunction::calcValue(double x) const
{
  double yVal = 0;
  if(x >= _x0 && x <= _x1 ) {
    int idx  = SegmentedQuinticBezierToolkit::calcIndex(x,_mXVec);
    double u = SegmentedQuinticBezierToolkit::
               calcU(x,_mXVec[idx], UTOL, MAXITER);
    yVal = SegmentedQuinticBezierToolkit::
           calcQuinticBezierCurveVal(u,_mYVec[idx]);
  } else {
    if(x < _x0) {
      yVal = _y0 + _dydx0*(x-_x0);
    } else {
      yVal = _y1 + _dydx1*(x-_x1);
    }
  }

  return yVal;
}


double SmoothSegmentedFunction::calcInverseValue(double y,
    double xGuess) const
{

  double xVal = 0;

  int idx = -1;
  double yLeft  = 0.;
  double yRight = 0;
  double xLeft  = 0.;
  double xRight = 0;
  double xDist  = 0;
  double xDistBest  = numeric_limits<double>::infinity();

  for(unsigned int i=0; i < _numBezierSections; ++i) {

    yLeft = y - _mYVec[i][0];
    yRight=     _mYVec[i][5] - y;

    xLeft = xGuess - _mXVec[i][0];
    xRight=        _mXVec[i][5] - xGuess;
    xDist = fabs(xLeft)+fabs(xRight);

    //If the y value is in the spline interval and the
    //x interval is closer to the guess, update the interval
    if(yLeft*yRight >= 0 && xDist < xDistBest) {
      idx = i;
      xDistBest = xDist;
    }

  }

  //y value is in the linear region
  if(idx == -1) {
    if( (y-_y1)*_dydx1 >= 0
        && fabs(_dydx1) > numeric_limits< double >::epsilon() ) {
      xVal = (y-_y1)/_dydx1 + _x1;
    } else if( (_y0-y)*_dydx0 >= 0
               && fabs(_dydx0) > numeric_limits< double >::epsilon() ) {
      xVal = (y-_y0)/_dydx0 + _x0;
    } else {
      xVal = numeric_limits<double>::signaling_NaN();
    }

  } else {
    //y is in an interval
    double u = SegmentedQuinticBezierToolkit::
               calcU(y,_mYVec[idx], UTOL, MAXITER);
    xVal = SegmentedQuinticBezierToolkit::
           calcQuinticBezierCurveVal(u,_mXVec[idx]);
  }

  return xVal;

}

double SmoothSegmentedFunction::calcValue(
  const RigidBodyDynamics::Math::VectorNd& ax) const
{

  if( !(ax.size() == 1) ) {
    ostringstream errormsg;
    errormsg << "SmoothSegmentedFunction::calcValue " << _name.c_str()
             << ": Argument x must have only 1 element, as this function is "
             <<  "designed only for 1D functions, but a function with "
             << ax.size() << " elements was entered"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLDofMismatchError(errormsg.str());
  }

  return calcValue(ax[0]);
}

/*Detailed Computational Costs
________________________________________________________________________
If x is in the Bezier Curve, and dy/dx is being evaluated
            Name   Comp.   Div.  Mult.   Add.  Assign.
_______________________________________________________________________
Overhead:
  SegmentedQuinticBezierToolkit::
          calcIndex   3*m+2           1*m   3
            *calcU   15    2    82    42   60
  Derivative Evaluation:
  **calcQuinticBezierCurveDYDX          21    20    13
            dy/du           20    19    11
            dx/du           20    19    11
            dy/dx       1

            total  17+3*m   3     143   m+100   98

*Approximate. Uses iteration
**Higher order derivatives cost more
________________________________________________________________________
If x is in the linear region

            Name   Comp.   Div.  Mult.   Add.  Assign.
                  1              1
________________________________________________________________________
  */

double SmoothSegmentedFunction::calcDerivative(double x, int order) const
{
  //return calcDerivative( SimTK::Array_<int>(order,0),
  //           RigidBodyDynamics::Math::VectorNd(1,x));
  double yVal = 0;

  //QUINTIC SPLINE


  if(order==0) {
    yVal = calcValue(x);
  } else {
    if(x >= _x0 && x <= _x1) {
      int idx  = SegmentedQuinticBezierToolkit::calcIndex(x,_mXVec);
      double u = SegmentedQuinticBezierToolkit::
                 calcU(x,_mXVec[idx],UTOL,MAXITER);
      yVal = SegmentedQuinticBezierToolkit::
             calcQuinticBezierCurveDerivDYDX(u, _mXVec[idx],
                                             _mYVec[idx], order);
      /*
                    std::cout << _mX(3, idx) << std::endl;
                    std::cout << _mX(idx) << std::endl;*/
    } else {
      if(order == 1) {
        if(x < _x0) {
          yVal = _dydx0;
        } else {
          yVal = _dydx1;
        }
      } else {
        yVal = 0;
      }
    }
  }

  return yVal;
}



double SmoothSegmentedFunction::
calcDerivative( const std::vector<int>& derivComponents,
                const RigidBodyDynamics::Math::VectorNd& ax) const
{
  /*
  for(int i=0; i < (signed)derivComponents.size(); i++){
    SimTK_ERRCHK2_ALWAYS( derivComponents[i] == 0,
    "SmoothSegmentedFunction::calcDerivative",
    "%s: derivComponents can only be populated with 0's because "
    "SmoothSegmentedFunction is only valid for a 1D function, but "
    "derivComponents had a value of %i in it",
    _name.c_str(), derivComponents[i]);
  }
  SimTK_ERRCHK2_ALWAYS( derivComponents.size() <= 6,
    "SmoothSegmentedFunction::calcDerivative",
    "%s: calcDerivative is only valid up to a 6th order derivative"
    " but derivComponents had a size of %i",
    _name.c_str(), derivComponents.size());

  SimTK_ERRCHK2_ALWAYS( ax.size() == 1,
    "SmoothSegmentedFunction::calcValue",
    "%s: Argument x must have only 1 element, as this function is "
    "designed only for 1D functions, but ax had a size of %i",
    _name.c_str(), ax.size());
  */

  for(int i=0; i < (signed)derivComponents.size(); i++) {
    if( !(derivComponents[i] == 0)) {
      ostringstream errormsg;
      errormsg << "SmoothSegmentedFunction::calcDerivative "
               << _name.c_str()
               << ": derivComponents can only be populated with 0's because "
               << "SmoothSegmentedFunction is only valid for a 1D function,"
               << " but derivComponents had a value of "
               << derivComponents[i] << " in it"
               << endl;
      throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
    }
  }
  //! [Dof Mismatch]
  if( !(derivComponents.size() <= 6)) {
    ostringstream errormsg;
    errormsg << "SmoothSegmentedFunction::calcDerivative " << _name.c_str()
             << ": calcDerivative is only valid up to a 6th order derivative"
             << " but derivComponents had a size of "
             << derivComponents.size()
             << endl;
    throw RigidBodyDynamics::Errors::RBDLDofMismatchError(errormsg.str());
  }
  //! [Dof Mismatch]

  if( !(ax.size() == 1) ) {
    ostringstream errormsg;
    errormsg << "SmoothSegmentedFunction::calcValue " << _name.c_str()
             << ": Argument x must have only 1 element, as this function is "
             << "designed only for 1D functions, but ax had a size of "
             << ax.size()
             << endl;
    throw RigidBodyDynamics::Errors::RBDLDofMismatchError(errormsg.str());
  }


  return calcDerivative(ax[0], derivComponents.size());
}

/*Detailed Computational Costs
________________________________________________________________________
If x is in the Bezier Curve, and dy/dx is being evaluated
            Name   Comp.   Div.  Mult.   Add.  Assign.
_______________________________________________________________________
      *spline.calcValue   7         2     3     1

*Approximate cost of evaluating a cubic spline with 100 knots, where
the bisection method is used to find the correct index
________________________________________________________________________
If x is in the linear region

        Name        Comp.   Div.  Mult.   Add.  Assign.
        *spline.calcValue  1         2     3     1
        integral eval    2         4     5     1
        total        3         6     8     2

*Approximate cost of evaluating a cubic spline at its last knot point
________________________________________________________________________

*/
/*
double SmoothSegmentedFunction::calcIntegral(double x) const
{
  SimTK_ERRCHK1_ALWAYS(_computeIntegral,
    "SmoothSegmentedFunction::calcIntegral",
    "%s: This curve was not constructed with its integral because"
    "computeIntegral was false",_name.c_str());

  double yVal = 0;
  if(x >= _x0 && x <= _x1){
    yVal = _splineYintX.calcValue(RigidBodyDynamics::Math::VectorNd(1,x));
  }else{
    //LINEAR EXTRAPOLATION
    if(x < _x0){
      RigidBodyDynamics::Math::VectorNd tmp(1);
      tmp(0) = _x0;
      double ic = _splineYintX.calcValue(tmp);
      if(_intx0x1){//Integrating left to right
        yVal = _y0*(x-_x0)
          + _dydx0*(x-_x0)*(x-_x0)*0.5
          + ic;
      }else{//Integrating right to left
        yVal = -_y0*(x-_x0)
          - _dydx0*(x-_x0)*(x-_x0)*0.5
          + ic;
      }
    }else{
      RigidBodyDynamics::Math::VectorNd tmp(1);
      tmp(0) = _x1;
      double ic = _splineYintX.calcValue(tmp);
      if(_intx0x1){
        yVal = _y1*(x-_x1)
          + _dydx1*(x-_x1)*(x-_x1)*0.5
          + ic;
      }else{
        yVal = -_y1*(x-_x1)
          - _dydx1*(x-_x1)*(x-_x1)*0.5
          + ic;
      }
    }
  }

  return yVal;
}
*/
/*
bool SmoothSegmentedFunction::isIntegralAvailable() const
{
  return _computeIntegral;
}
*/

/*
bool SmoothSegmentedFunction::isIntegralComputedLeftToRight() const
{
  return _intx0x1;
}
*/

int SmoothSegmentedFunction::getArgumentSize() const
{
  return 1;
}

int SmoothSegmentedFunction::getMaxDerivativeOrder() const
{
  return 6;
}

std::string SmoothSegmentedFunction::getName() const
{
  return _name;
}

void SmoothSegmentedFunction::setName(const std::string &name)
{
  _name = name;
}

RigidBodyDynamics::Math::VectorNd
SmoothSegmentedFunction::getCurveDomain() const
{
  RigidBodyDynamics::Math::VectorNd domain(2);

  domain[0] = 0;
  domain[1] = 0;
  if (!_mXVec.empty()) {
    domain[0] = _mXVec[0][0];
    domain[1] = _mXVec[_mXVec.size()-1][_mXVec[0].size()-1];
  }
  return domain;
}

///////////////////////////////////////////////////////////////////////////////
// Utility functions
///////////////////////////////////////////////////////////////////////////////

/*Detailed Computational Costs

_______________________________________________________________________
            Name   Comp.   Div.  Mult.   Add.  Assign.
_______________________________________________________________________

        *overhead  (17+3*m   2  82    42+m  63)*7
                119+21*m  14  574   294+7m  441

        calcValue           21    20    13
  calcDerivative: dy/dx        1  40    38    22
          : d2y/dx2        2  78    73    23
          : d3y/dx3        4  118   105   58
          : d4y/dx4        5  168   137   71
          : d5y/dx5        7  236   170   88
          : d6y/dx6        9  334   209   106

        **calcIntegral  7         2     3     1

      total per point   126+21*m  42   1571  1049  823
      total per elbow   126k+21k*m  42k  1571k   1049k   823k

  *Approximate. Overhead associated with finding the correct Bezier
          spline section, and evaluating u(x).
          Assumes 2 Newton iterations in calcU

  **Approximate. Includes estimated cost of evaluating a cubic spline
          with 100 knots
*/
RigidBodyDynamics::Math::MatrixNd
SmoothSegmentedFunction::calcSampledCurve(int maxOrder,
    double domainMin,
    double domainMax) const
{

  int pts = 1; //Number of points between each of the spline points used
  //to fit u(x), and also the integral spline
  if( !(maxOrder <= getMaxDerivativeOrder()) ) {
    ostringstream errormsg;
    errormsg << "SmoothSegmentedFunction::calcSampledCurve "
             << "Derivative order past the maximum computed order requested"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }

  double x0,x1,delta;
  //y,dy,d1y,d2y,d3y,d4y,d5y,d6y,iy
  RigidBodyDynamics::Math::VectorNd
  midX(NUM_SAMPLE_PTS*_numBezierSections-(_numBezierSections-1));
  RigidBodyDynamics::Math::VectorNd x(NUM_SAMPLE_PTS);

  //Generate a sample of X values inside of the curve that is denser where
  //the curve is more curvy.
  double u;
  int idx = 0;
  for(int s=0; s < _numBezierSections; s++) {
    //Sample the local set for u and x
    for(int i=0; i<NUM_SAMPLE_PTS; i++) {
      u = ( (double)i )/( (double)(NUM_SAMPLE_PTS-1) );
      x[i] = SegmentedQuinticBezierToolkit::
             calcQuinticBezierCurveVal(u,_mXVec[s]);
      if(_numBezierSections > 1) {
        //Skip the last point of a set that has another set of points
        //after it. Why? The last point and the starting point of the
        //next set are identical in value.
        if(i<(NUM_SAMPLE_PTS-1) || s == (_numBezierSections-1)) {
          midX[idx] = x[i];
          idx++;
        }
      } else {
        midX[idx] = x[i];
        idx++;
      }
    }
  }


  RigidBodyDynamics::Math::VectorNd xsmpl(pts*(midX.size()-1)+2*10*pts);

  RigidBodyDynamics::Math::MatrixNd results;

  /*
  if(_computeIntegral){
    results.resize(pts*(midX.size()-1)+2*10*pts,maxOrder+2+1);
  }else{

  }
  */
  results.resize(pts*(midX.size()-1)+2*10*pts,maxOrder+2);

  //Array initialization is so ugly ...
  std::vector<int> d1y(1),d2y(2),d3y(3),d4y(4),d5y(5),d6y(6);
  d1y[0]=0;
  d2y[0] = 0;
  d2y[1] = 0;
  for(int i=0; i<3; i++) {
    d3y[i]=0;
  }
  for(int i=0; i<4; i++) {
    d4y[i]=0;
  }
  for(int i=0; i<5; i++) {
    d5y[i]=0;
  }
  for(int i=0; i<6; i++) {
    d6y[i]=0;
  }

  //generate some sample points in the extrapolated region
  idx = 0;
  x0  = _x0 - 0.1*(_x1-_x0);
  if(domainMin < x0) {
    x0 = domainMin;
  }

  x1  = _x0;
  delta = (0.1)*(x1-x0)/(pts);

  for(int j=0; j<pts*10; j++) {
    xsmpl[idx] = x0 + delta*j;
    idx++;
  }


  //generate some points in the mid region
  for(int i=0; i< midX.size()-1; i++) {
    x0 = midX[i];
    x1 = midX[i+1];
    delta = (x1-x0)/pts;
    for(int j=0; j<pts; j++) {
      xsmpl[idx] = x0 + delta*j;
      idx++;
    }
  }

  //generate some points in the extrapolated region
  x0 = _x1;
  x1 = _x1 + 0.1*(_x1-_x0);
  if(domainMax > x1) {
    x1 = domainMax;
  }

  delta = (1.0/9.0)*(x1-x0)/(pts);

  for(int j=0; j<pts*10; j++) {
    xsmpl[idx] = x0 + delta*j;
    idx++;
  }

  //Populate the results matrix at the sample points
  RigidBodyDynamics::Math::VectorNd ax(1);
  for(int i=0; i < xsmpl.size(); i++) {
    ax[0] = xsmpl[i];
    results(i,0) = ax[0];
    if(i==48) {
      double here = 1.0;
    }
    results(i,1) = calcValue(ax);
    if(maxOrder>=1) {
      results(i,2) = calcDerivative(d1y,ax);
    }

    if(maxOrder>=2) {
      results(i,3) = calcDerivative(d2y,ax);
    }

    if(maxOrder>=3) {
      results(i,4) = calcDerivative(d3y,ax);
    }

    if(maxOrder>=4) {
      results(i,5) = calcDerivative(d4y,ax);
    }

    if(maxOrder>=5) {
      results(i,6) = calcDerivative(d5y,ax);
    }

    if(maxOrder>=6) {
      results(i,7) = calcDerivative(d6y,ax);
    }

    /*
    if(_computeIntegral){
      results(i,maxOrder+2) = calcIntegral(ax(0));
    }
    */
  }
  return results;
}

void SmoothSegmentedFunction::getXControlPoints(
  RigidBodyDynamics::Math::MatrixNd& mat) const
{
  mat.resize(_mXVec.size(), _mXVec.at(0).size());

  for(int i=0; i<_mXVec.size(); ++i) {
    for(int j=0; j<_mXVec.size(); ++j) {
      mat(i,j) = _mXVec.at(i)[j];
    }
  }

}
void SmoothSegmentedFunction::getYControlPoints(
  RigidBodyDynamics::Math::MatrixNd& mat) const
{
  mat.resize(_mYVec.size(), _mYVec.at(0).size());

  for(int i=0; i<_mYVec.size(); ++i) {
    for(int j=0; j<_mYVec.size(); ++j) {
      mat(i,j) = _mYVec.at(i)[j];
    }
  }

}


/*Detailed Computational Costs

_______________________________________________________________________
            Name   Comp.   Div.  Mult.   Add.  Assign.
_______________________________________________________________________

         *overhead   (17+3*m   2  82    42+m  63)*3
                51+9m    6  246   126+3m   189

        calcValue             21    20    13
  calcDerivative  : dy/dx        1  40    38    22
          : d2y/dx2        2  78    73    23

        **calcIntegral  7        2     3     1

      total per point    58+9m   9  387  260+3m   248
      total per elbow  5.8k+900m   900  38.7k  26k+300m 24.8k

  *Approximate. Overhead associated with finding the correct Bezier
          spline section, and evaluating u(x).
          Assumes 2 Newton iterations in calcU

  **Approximate. Includes estimated cost of evaluating a cubic spline
          with 100 knots
*/
void SmoothSegmentedFunction::printCurveToCSVFile(
  const std::string& path,
  const std::string& fileNameWithoutExtension,
  double domainMin,
  double domainMax) const
{
  //Only compute up to the 2nd derivative
  RigidBodyDynamics::Math::MatrixNd results =
    calcSampledCurve(2,domainMin,domainMax);
  std::vector<std::string> colNames(results.cols());
  colNames[0] = "x";
  colNames[1] = "y";
  colNames[2] = "dy/dx";
  colNames[3] = "d2y/dx2";
  /*
  if(results.cols() == 5){
    colNames[4] = "int_y(x)";
  }
  */
  std::string fname = fileNameWithoutExtension;
  fname.append(".csv");
  printMatrixToFile(results,colNames,path,fname);
}
/*
This function will print cvs file of the column vector col0 and the matrix data
*/
void SmoothSegmentedFunction::
printMatrixToFile(  RigidBodyDynamics::Math::MatrixNd& data,
                    std::vector<std::string>& colNames,
                    const std::string& path,
                    const std::string& filename) const
{

  ofstream datafile;
  std::string fullpath = path;

  if(fullpath.length() > 0) {
    fullpath.append("/");
  }

  fullpath.append(filename);

  datafile.open(fullpath.c_str(),std::ios::out);

  if(!datafile) {
    datafile.close();
    ostringstream errormsg;
    errormsg << "SmoothSegmentedFunction::printMatrixToFile "
             << _name.c_str() << ": Failed to open the file path: "
             << fullpath.c_str()
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidFileError(errormsg.str());
  }


  for(int i = 0; i < (signed)colNames.size(); i++) {
    if(i < (signed)colNames.size()-1) {
      datafile << colNames[i] << ",";
    } else {
      datafile << colNames[i] << "\n";
    }
  }

  for(int i = 0; i < data.rows(); i++) {
    for(int j = 0; j < data.cols(); j++) {
      if(j<data.cols()-1) {
        datafile << data(i,j) << ",";
      } else {
        datafile << data(i,j) << "\n";
      }
    }
  }
  datafile.close();
}


