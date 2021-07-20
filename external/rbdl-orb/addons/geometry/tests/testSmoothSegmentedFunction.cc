/* -------------------------------------------------------------------------- *
 *        OpenSim:  testSmoothSegmentedFunctionFactory.cpp                    *
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
    
     This also includes additional curves (the Anderson2007 curves)
     which are not presently in OpenSim.

    Author:
     Matthew Millard
    
    Date:
     Nov 2015

*/
/* 
Below is a basic bench mark simulation for the SmoothSegmentedFunctionFactory
class, a class that enables the easy generation of C2 continuous curves 
that define the various characteristic curves required in a muscle model
 */

// Author:  Matthew Millard

//==============================================================================
// INCLUDES
//==============================================================================
#define CATCH_CONFIG_MAIN
#include "geometry.h"
#include "numericalTestFunctions.h"
#include <rbdl/rbdl_math.h>
#include <ctime>
#include <string>
#include <stdio.h>
#include <exception>
#include <cassert>
#include <fstream>

#include "rbdl_tests.h"

using namespace RigidBodyDynamics::Addons::Geometry;

using namespace std;


/**
    This function will create a quintic Bezier curve y(x) and sample it, its 
    first derivative w.r.t. U (dx(u)/du and dy(u)/du), and its first derivative
    w.r.t. to X and print it to the screen.
*/
void testSegmentedQuinticBezierDerivatives(
      int maximumNumberOfToleranceViolations)
{
    //cout <<"**************************************************"<<endl;
    //cout << "   TEST: Bezier Curve Derivative DU" << endl;
    string name  = "testSegmentedQuinticBezierDerivatives()";
     RigidBodyDynamics::Math::VectorNd xPts(6);
     RigidBodyDynamics::Math::VectorNd yPts(6);
     xPts[0] = 0;
     xPts[1] = 0.5;
     xPts[2] = 0.5;
     xPts[3] = 0.75;
     xPts[4] = 0.75;
     xPts[5] = 1;

     yPts[0] = 0;
     yPts[1] = 0.125;
     yPts[2] = 0.125;
     yPts[3] = 0.5;
     yPts[4] = 0.5;
     yPts[5] = 1;

     double val = 0;
     double d1 = 0;
     double d2 = 0;
     double d3 = 0;
     double d4 = 0;
     double d5 = 0;
     double d6 = 0;

     double u = 0;

     int steps = 100;

     RigidBodyDynamics::Math::MatrixNd analyticDerXU(steps,8);
     RigidBodyDynamics::Math::MatrixNd analyticDerYU(steps,8);
     RigidBodyDynamics::Math::VectorNd uV(steps);
     for(int i = 0; i<steps; i++){
      //int i = 10;
      u = (double)i/(steps-1);
      uV[i] = u;

      val= SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveVal(u,xPts);
      d1 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,1);
      d2 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,2);
      d3 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,3);
      d4 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,4);
      d5 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,5);
      d6 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,6);

      analyticDerXU(i,0) = u;
      analyticDerXU(i,1) = val;
      analyticDerXU(i,2) = d1;
      analyticDerXU(i,3) = d2;
      analyticDerXU(i,4) = d3;
      analyticDerXU(i,5) = d4;
      analyticDerXU(i,6) = d5;
      analyticDerXU(i,7) = d6;

      val= SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveVal(u,yPts);
      d1 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,1);
      d2 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,2);
      d3 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,3);
      d4 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,4);
      d5 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,5);
      d6 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,6);

      analyticDerYU(i,0) = u;
      analyticDerYU(i,1) = val;
      analyticDerYU(i,2) = d1;
      analyticDerYU(i,3) = d2;
      analyticDerYU(i,4) = d3;
      analyticDerYU(i,5) = d4;
      analyticDerYU(i,6) = d5;
      analyticDerYU(i,7) = d6;

     }

     int mxDU = 6-1;
     RigidBodyDynamics::Math::MatrixNd numericDer(analyticDerXU.rows(), mxDU);
     RigidBodyDynamics::Math::MatrixNd errorDer(analyticDerXU.rows(), mxDU);

     double tol = (double)(1.0/steps);
     tol = tol*tol*50; 
     //Numerical error in a central difference increases with the 
     //square of h. 
     //http://en.wikipedia.org/wiki/Finite_difference

     RigidBodyDynamics::Math::VectorNd domainX = 
      RigidBodyDynamics::Math::VectorNd::Zero(analyticDerXU.rows());
     RigidBodyDynamics::Math::VectorNd rangeY = 
      RigidBodyDynamics::Math::VectorNd::Zero(analyticDerXU.rows());

     RigidBodyDynamics::Math::VectorNd analyticDerYX = 
      RigidBodyDynamics::Math::VectorNd::Zero(analyticDerXU.rows());
     for(int j=0; j<analyticDerXU.rows(); j++){
      domainX[j] = analyticDerXU(j,0);
     }


     for(int i=0;i<mxDU;i++){

      for(int j=0; j<analyticDerXU.rows(); j++){
       rangeY[j]        = analyticDerXU(j,i+1);
       analyticDerYX[j] = analyticDerXU(j,i+2);
      }

      numericDer.col(i) = calcCentralDifference(domainX,
                                                rangeY,true); 
      for(int j=0; j<analyticDerYX.rows();++j){
        errorDer(j,i)  =  analyticDerYX[j]-numericDer(j,i);
      }

      //errorDer(i)= abs( errorDer(i).elementwiseDivide(numericDer(i)) );
      //The end points can't be tested because a central difference
      //cannot be accurately calculated at these locations
      for(int j=1; j<analyticDerXU.rows()-1; j++){
       assert( abs(errorDer(j,i))<tol );
       //if(errorDer(j,i)>tol)
       //printf("Error > Tol: (%i,%i): %f > %f\n",j,i,errorDer(j,i),tol);
      }
     }     
     //errorDer.cwiseAbs();
     //cout << errorDer << endl;

    //printf("...absolute tolerance of %f met\n", tol);

     //cout << "   TEST: Bezier Curve Derivative DYDX to d6y/dx6" << endl;
     RigidBodyDynamics::Math::MatrixNd numericDerXY(analyticDerXU.rows(), 6);
     RigidBodyDynamics::Math::MatrixNd analyticDerXY(analyticDerXU.rows(),6);

     for(int i=0; i< analyticDerXU.rows(); i++)
     {
      analyticDerXY(i,0) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,1);
      analyticDerXY(i,1) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,2);
      analyticDerXY(i,2) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,3);
      analyticDerXY(i,3) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,4);
      analyticDerXY(i,4) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,5);
      analyticDerXY(i,5) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,6);
     }


     for(int j=0; j<numericDerXY.cols();j++){

      for(int k=0; k<numericDerXY.rows(); k++){
       domainX[k] = analyticDerXU(k,1);
       if(j == 0){
        rangeY[k]  = analyticDerYU(k,1);
       }else{
        rangeY[k]  = analyticDerXY(k,j-1);
       }
      }
      numericDerXY.col(j) = calcCentralDifference(domainX,
                       rangeY,true);
     
     }


     //Generate numerical derivative curves for the first 3 derivatives
     /*
     numericDerXY.col(0) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerYU.col(1),true);
     numericDerXY.col(1) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerXY.col(0),true);
     numericDerXY.col(2) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerXY.col(1),true);
     numericDerXY.col(3) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerXY.col(2),true);
     numericDerXY.col(4) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerXY.col(3),true);
     numericDerXY.col(5) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerXY.col(4),true);
     */

     //Create the matrix of errors
     RigidBodyDynamics::Math::MatrixNd errorDerXYNum(analyticDerXU.rows(), 6);
     RigidBodyDynamics::Math::MatrixNd errorDerXYDen(analyticDerXU.rows(), 6);
     RigidBodyDynamics::Math::MatrixNd errorDerXY(analyticDerXU.rows(), 6);

     for(int i = 0; i < errorDerXYNum.rows(); ++i){
        for(int j = 0; j < errorDerXYNum.cols(); ++j){
            errorDerXYNum(i,j) = abs( analyticDerXY(i,j)-numericDerXY(i,j));
            errorDerXYDen(i,j) = abs( analyticDerXY(i,j)+numericDerXY(i,j));
            errorDerXY(i,j)    = errorDerXYNum(i,j)/errorDerXYDen(i,j);
        }
     }

     double relTol = 5e-2;

     int relTolExceeded = 0;

     for(int j=0;j<6;j++){
      //can't test the first and last entries because a central diff.
      //cannot calculate these values accurately.
      for(int i=1;i<analyticDerXU.rows()-1;i++){
       if(errorDerXY(i,j)>relTol){
        //printf("Error > Tol: (%i,%i): %f > %f\n",i,j,
        //          errorDerXY(i,j),relTol);
        relTolExceeded++;
       }
      }
     }
     //cout << relTolExceeded << endl;

     //The relative tolerance gets exceeded occasionally in locations of
     //rapid change in the curve. Provided there are only a few locations
     //where the relative tolerance of 5% is broken, the curves should be
     //regarded as being good. Ten errors out of a possible 100*6 data points
     //seems relatively small.
     CHECK(relTolExceeded < maximumNumberOfToleranceViolations);


     //std::string fname = "analyticDerXY.csv";
     //printMatrixToFile(analyticDerXY,fname);
     //fname = "numericDerXY.csv";
     //printMatrixToFile(numericDerXY,fname);
     //fname = "errorDerXY.csv";
     //printMatrixToFile(errorDerXY,fname);
     //printf("   ...relative tolerance of %f not exceeded more than %i times\n"
     //    "   across all 6 derivatives, with 100 samples each\n",
     //        relTol, 10);
     //cout <<"**************************************************"<<endl;


}


   
TEST_CASE(__FILE__"_QuinticBezierToolKitDerivatives", "")
{
    int maximumNumberOfToleranceViolations = 10;
    testSegmentedQuinticBezierDerivatives(10);
}

TEST_CASE(__FILE__"_SmoothSegmentedFunctionProperties", "")
{
  //1. Make a fake monotonic curve
  RigidBodyDynamics::Math::VectorNd x(5);
  RigidBodyDynamics::Math::VectorNd y(5);
  RigidBodyDynamics::Math::VectorNd dydx(5);
  for(int i=0; i<x.size();++i){
    x[i]      = i*0.5*M_PI/(x.size()-1);  
    y[i]      = sin(x[i]) + x[i];
    dydx[i]   = cos(x[i]) + 1.0; 
  }
  double c = 0.5;
  
  RigidBodyDynamics::Math::MatrixNd mX(6,4), mY(6,4);
  RigidBodyDynamics::Math::MatrixNd p0(6,2);

  for(int i=0; i < 4; ++i){
    p0 = SegmentedQuinticBezierToolkit::
          calcQuinticBezierCornerControlPoints(  x[i],  y[i],  dydx[i],
                                               x[i+1],y[i+1],dydx[i+1],c);             
    mX.col(i)  = p0.col(0);
    mY.col(i)  = p0.col(1);
  }
  SmoothSegmentedFunction testCurve = SmoothSegmentedFunction();
  testCurve.updSmoothSegmentedFunction(   mX,     mY, 
                                        x[0],   x[4],
                                        y[0],   y[4],
                                     dydx[0],dydx[4],
                                     "testCurve");


  //2. Test key points.
  RigidBodyDynamics::Math::VectorNd yErr(5);
  RigidBodyDynamics::Math::VectorNd dydxErr(5);
  for(int i=0; i<x.size(); ++i){
    yErr[i]    = testCurve.calcValue(x[i]) - y[i];
    dydxErr[i] = testCurve.calcDerivative(x[i],1) - dydx[i];

    CHECK( abs(yErr[i])    < TOL_SMALL );
    CHECK( abs(dydxErr[i]) < TOL_SMALL );    
  }  

  //3. Test derivatives for numerical consistency
  RigidBodyDynamics::Math::MatrixNd testCurveSample
    = testCurve.calcSampledCurve( 6, x[0]-0.1, x[4]+0.1);

  bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      testCurve,  
      testCurveSample, 
      TOL_DX);

  CHECK(areCurveDerivativesGood);

  //4. Test C2 continuity
  bool curveIsContinuous = isCurveC2Continuous(testCurve,
                                               testCurveSample,
                                               TOL_BIG);
  CHECK(curveIsContinuous);

  //5. Test monotonicity
  bool curveIsMonotonic = isCurveMontonic(testCurveSample);
  CHECK(curveIsMonotonic);

}

TEST_CASE(__FILE__"_ShiftScale", "")
{
  //1. Make a curve
  RigidBodyDynamics::Math::VectorNd xV(5);
  RigidBodyDynamics::Math::VectorNd yV(5);
  RigidBodyDynamics::Math::VectorNd dydxV(5);
  for(int i=0; i<xV.size();++i){
    xV[i]      = i*0.5*M_PI/(xV.size()-1);
    yV[i]      = sin(xV[i]) + xV[i];
    dydxV[i]   = cos(xV[i]) + 1.0;
  }
  double c = 0.5;
  
  RigidBodyDynamics::Math::MatrixNd mX(6,4), mY(6,4);
  RigidBodyDynamics::Math::MatrixNd p0(6,2);

  for(int i=0; i < 4; ++i){
    p0 = SegmentedQuinticBezierToolkit::
          calcQuinticBezierCornerControlPoints(  xV[i],  yV[i],  dydxV[i],
                                               xV[i+1],yV[i+1],dydxV[i+1],c);
    mX.col(i)  = p0.col(0);
    mY.col(i)  = p0.col(1);
  }
  SmoothSegmentedFunction curve = SmoothSegmentedFunction();
  curve.updSmoothSegmentedFunction(       mX,     mY, 
                                        xV[0],   xV[4],
                                        yV[0],   yV[4],
                                     dydxV[0],dydxV[4],
                                     "testCurve"); 
  SmoothSegmentedFunction shiftedCurve = SmoothSegmentedFunction();
  shiftedCurve = curve;
  double xShift = 1.0/3.0;
  double yShift = 2.0/3.0;

  shiftedCurve.shift(xShift,yShift);

  //Test shift
  double x ,y, dydx, d2ydx2;
  double xS ,yS, dydxS, d2ydx2S;

  double xmin = -0.1;
  double xmax = 0.5*M_PI+0.1;

  for(int i=0; i<xV.size();++i){
    x      = xmin + i*(xmax-xmin)/(xV.size()-1);
    y      = curve.calcValue(x);    
    dydx   = curve.calcDerivative(x,1);
    d2ydx2 = curve.calcDerivative(x,2);

    xS      = x + xShift;  
    yS      = shiftedCurve.calcValue(xS) - yShift;
    dydxS   = shiftedCurve.calcDerivative(xS,1);
    d2ydx2S = shiftedCurve.calcDerivative(xS,2);

    CHECK(      abs(y-yS)       < TOL_SMALL );
    CHECK( abs(  dydx-dydxS)    < TOL_SMALL );
    CHECK( abs(d2ydx2-d2ydx2S)  < TOL_SMALL );

  }

  //Test scale
  SmoothSegmentedFunction scaledCurve  = SmoothSegmentedFunction();
  scaledCurve = curve;
  double xScale = 1.0/2.0;
  double yScale = 5.0/3.0;
  double dydxScale = yScale/xScale;

  scaledCurve.scale(xScale,yScale);

  for(int i=0; i<xV.size();++i){
    x      = xmin + i*(xmax-xmin)/(xV.size()-1);
    y      = curve.calcValue(x);
    dydx   = curve.calcDerivative(x,1);
    d2ydx2 = curve.calcDerivative(x,2);

    xS      = x*xScale;  
    yS      = scaledCurve.calcValue(xS)/yScale;
    dydxS   = scaledCurve.calcDerivative(xS,1)/dydxScale ;
    d2ydx2S = scaledCurve.calcDerivative(xS,2)*xScale/(dydxScale);

    CHECK(      abs(y-yS)       < TOL_SMALL );
    CHECK( abs(  dydx-dydxS)    < TOL_SMALL );
    CHECK( abs(d2ydx2-d2ydx2S)  < TOL_SMALL );

  }

}

TEST_CASE(__FILE__"_CalcFunctionInverseValue", "")
{
  //1. Make a curve with multiple maxima
  unsigned int n = 10;
  RigidBodyDynamics::Math::VectorNd xV(n);
  RigidBodyDynamics::Math::VectorNd yV(n);
  RigidBodyDynamics::Math::VectorNd dydxV(n);
  for(int i=0; i<xV.size();++i){
    xV[i]      = i*2*M_PI/(xV.size()-1);
    yV[i]      = sin(xV[i]);
    dydxV[i]   = cos(xV[i]);
  }
  double c = 0.5;

  unsigned int m =n-1;
  RigidBodyDynamics::Math::MatrixNd mX(6,m), mY(6,m);
  RigidBodyDynamics::Math::MatrixNd p0(6,2);

  for(int i=0; i < m; ++i){
    p0 = SegmentedQuinticBezierToolkit::
          calcQuinticBezierCornerControlPoints(  xV[i],  yV[i],  dydxV[i],
                                               xV[i+1],yV[i+1],dydxV[i+1],c);
    mX.col(i)  = p0.col(0);
    mY.col(i)  = p0.col(1);
  }
  SmoothSegmentedFunction curve = SmoothSegmentedFunction();
  curve.updSmoothSegmentedFunction(       mX,     mY,
                                        xV[0],   xV[n-1],
                                        yV[0],   yV[n-1],
                                     dydxV[0],dydxV[n-1],
                                     "testCurve");

   //Testing within the curve: make sure it returns an x value that
   //evaluates to the desired y and that it is the correct x.
   double xGuess = M_PI*0.25;
   double y0     = sin(xGuess);
   double x0     = curve.calcInverseValue(y0,xGuess);
   double err    = curve.calcValue(x0)-y0;
   CHECK( abs(err) < TOL_SMALL);
   CHECK( abs(x0-xGuess) < 0.1);

   //Testing within the curve: test the other x.
   xGuess = M_PI*(0.25) + M_PI*(0.5);
   y0     = sin(xGuess);
   x0     = curve.calcInverseValue(y0,xGuess);
   err    = curve.calcValue(x0)-y0;
   CHECK( abs(err) < TOL_SMALL);
   CHECK( abs(x0-xGuess) < 0.1);

   //Test outside of the interval.
   xGuess = -1.0;
   y0     = dydxV[0]*xGuess;
   x0     = curve.calcInverseValue(y0,xGuess);
   err    = curve.calcValue(x0)-y0;
   CHECK( abs(err) < TOL_SMALL);
   CHECK( abs(x0-xGuess) < 0.1);

   //Test outside of the interval.
   xGuess = M_PI*2.0 + 1.0;
   y0     = dydxV[n-1]*1.0;
   x0     = curve.calcInverseValue(y0,xGuess);
   err    = curve.calcValue(x0)-y0;
   CHECK( abs(err) < TOL_SMALL);
   CHECK( abs(x0-xGuess) < 0.1);
}
