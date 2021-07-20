/* -------------------------------------------------------------------------- *
 *        OpenSim:  SmoothSegmentedFunctionFactory.cpp                        *
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

#include "MuscleFunctionFactory.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>

#include <rbdl/rbdl_errors.h>

using namespace std;
using namespace RigidBodyDynamics::Addons::Muscle;
using namespace RigidBodyDynamics::Addons::Geometry;
//=============================================================================
// STATICS
//=============================================================================
//using namespace std;


static int NUM_SAMPLE_PTS = 100; //The number of knot points to use to sample
//each Bezier corner section

static double SMOOTHING = 0;   //The amount of smoothing to use when fitting
//3rd order splines to the quintic Bezier
//functions
static bool DEBUG = true;  //When this is set to true, each function's debug
//routine will be called, which ususally results
//in a text file of its output being produced

static double UTOL = (double)std::numeric_limits<double>::epsilon()*1e2;

static double INTTOL = (double)std::numeric_limits<double>::epsilon()*1e4;

static int MAXITER = 20;
//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

//=============================================================================
// MUSCLE CURVE FITTING FUNCTIONS
//=============================================================================
void  MuscleFunctionFactory::createFiberActiveForceLengthCurve(
  double x0,
  double x1,
  double x2,
  double x3,
  double ylow,
  double dydx,
  double curviness,
  const std::string& curveName,
  SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{
  //Ensure that the inputs are within a valid range
  double rootEPS = sqrt(std::numeric_limits<double>::epsilon());

  if( (!(x0>=0 && x1>x0+rootEPS && x2>x1+rootEPS && x3>x2+rootEPS) ) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberActiveForceLengthCurve: "
             << curveName
             << ": This must be true: 0 < lce0 < lce1 < lce2 < lce3"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }


  if( !(ylow >= 0) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberActiveForceLengthCurve:"
             << curveName
             << ": shoulderVal must be greater than, or equal to 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  double dydxUpperBound = (1-ylow)/(x2-x1);


  if( !(dydx >= 0 && dydx < dydxUpperBound) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberActiveForceLengthCurve:"
             << curveName
             << ": plateauSlope must be greater than 0 and less than "
             << dydxUpperBound
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(curviness >= 0 && curviness <= 1) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberActiveForceLengthCurve:"
             << curveName
             << ": curviness must be between 0 and 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  std::string name = curveName;
  name.append(".createFiberActiveForceLengthCurve");



  //Translate the users parameters into Bezier curves
  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);

  //The active force length curve is made up of 5 elbow shaped sections.
  //Compute the locations of the joining point of each elbow section.

  //Calculate the location of the shoulder
  double xDelta = 0.05*x2; //half the width of the sarcomere 0.0259,
  //but TM.Winter's data has a wider shoulder than
  //this

  double xs  = (x2-xDelta);//x1 + 0.75*(x2-x1);

  //Calculate the intermediate points located on the ascending limb
  double y0  = 0;
  double dydx0 = 0;

  double y1  = 1 - dydx*(xs-x1);
  double dydx01= 1.25*(y1-y0)/(x1-x0);//(y1-y0)/(x1-(x0+xDelta));

  double x01   = x0 + 0.5*(x1-x0); //x0 + xDelta + 0.5*(x1-(x0+xDelta));
  double y01   = y0 + 0.5*(y1-y0);

  //Calculate the intermediate points of the shallow ascending plateau
  double x1s   = x1 + 0.5*(xs-x1);
  double y1s   = y1 + 0.5*(1-y1);
  double dydx1s= dydx;

  //double dydx01c0 = 0.5*(y1s-y01)/(x1s-x01) + 0.5*(y01-y0)/(x01-x0);
  //double dydx01c1 = 2*( (y1-y0)/(x1-x0));
  //double dydx01(1-c)*dydx01c0 + c*dydx01c1;

  //x2 entered
  double y2 = 1;
  double dydx2 = 0;

  //Descending limb
  //x3 entered
  double y3 = 0;
  double dydx3 = 0;

  double x23 = (x2+xDelta) + 0.5*(x3-(x2+xDelta)); //x2 + 0.5*(x3-x2);
  double y23 = y2 + 0.5*(y3-y2);

  //double dydx23c0 = 0.5*((y23-y2)/(x23-x2)) + 0.5*((y3-y23)/(x3-x23));
  //double dydx23c1 = 2*(y3-y2)/(x3-x2);
  double dydx23   = (y3-y2)/((x3-xDelta)-(x2+xDelta));
  //(1-c)*dydx23c0 + c*dydx23c1;

  //Compute the locations of the control points
  RigidBodyDynamics::Math::MatrixNd p0 = SegmentedQuinticBezierToolkit::
                                         calcQuinticBezierCornerControlPoints(x0,ylow,dydx0,x01,y01,dydx01,c);
  RigidBodyDynamics::Math::MatrixNd p1 = SegmentedQuinticBezierToolkit::
                                         calcQuinticBezierCornerControlPoints(x01,y01,dydx01,x1s,y1s,dydx1s,c);
  RigidBodyDynamics::Math::MatrixNd p2 = SegmentedQuinticBezierToolkit::
                                         calcQuinticBezierCornerControlPoints(x1s,y1s,dydx1s,x2, y2, dydx2,c);
  RigidBodyDynamics::Math::MatrixNd p3 = SegmentedQuinticBezierToolkit::
                                         calcQuinticBezierCornerControlPoints(x2, y2, dydx2,x23,y23,dydx23,c);
  RigidBodyDynamics::Math::MatrixNd p4 = SegmentedQuinticBezierToolkit::
                                         calcQuinticBezierCornerControlPoints(x23,y23,dydx23,x3,ylow,dydx3,c);

  RigidBodyDynamics::Math::MatrixNd mX(6,5), mY(6,5);
  mX.col(0) = p0.col(0);
  mX.col(1) = p1.col(0);
  mX.col(2) = p2.col(0);
  mX.col(3) = p3.col(0);
  mX.col(4) = p4.col(0);

  mY.col(0) = p0.col(1);
  mY.col(1) = p1.col(1);
  mY.col(2) = p2.col(1);
  mY.col(3) = p3.col(1);
  mY.col(4) = p4.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    mX,mY,x0,x3,ylow,ylow,0,0,curveName);
}

void MuscleFunctionFactory::createFiberForceVelocityCurve(
  double fmaxE,
  double dydxC,
  double dydxNearC,
  double dydxIso,
  double dydxE,
  double dydxNearE,
  double concCurviness,
  double eccCurviness,
  const std::string& curveName,
  SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{
  //Ensure that the inputs are within a valid range

  if( !(fmaxE > 1.0) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityCurve: "
             << curveName
             <<": fmaxE must be greater than 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(dydxC >= 0.0 && dydxC < 1) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityCurve: "
             << curveName
             << ": dydxC must be greater than or equal to 0 "
             <<" and less than 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(dydxNearC > dydxC && dydxNearC <= 1) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityCurve: "
             << curveName
             << ": dydxNearC must be greater than or equal to 0 "
             << "and less than 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(dydxIso > 1) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityCurve: "
             << curveName
             << ": dydxIso must be greater than (fmaxE-1)/1 ("
             << ((fmaxE-1.0)/1.0)
             << ")"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(dydxE >= 0.0 && dydxE < (fmaxE-1)) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityCurve: "
             <<  curveName
             <<": dydxE must be greater than or equal to 0 "
             << "and less than fmaxE-1 ("
             << (fmaxE-1) << ")"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if(!(dydxNearE >= dydxE && dydxNearE < (fmaxE-1))) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityCurve"
             << curveName
             << ": dydxNearE must be greater than or equal to dydxE "
             << "and less than fmaxE-1 (" << (fmaxE-1)
             << ")"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if(! (concCurviness <= 1.0 && concCurviness >= 0)) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityCurve "
             << curveName
             << ": concCurviness must be between 0 and 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if(! (eccCurviness <= 1.0 && eccCurviness >= 0)) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityCurve "
             << curveName
             << ": eccCurviness must be between 0 and 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  std::string name = curveName;
  name.append(".createFiberForceVelocityCurve");

  //Translate the users parameters into Bezier point locations
  double cC = SegmentedQuinticBezierToolkit::scaleCurviness(concCurviness);
  double cE = SegmentedQuinticBezierToolkit::scaleCurviness(eccCurviness);

  //Compute the concentric control point locations
  double xC   = -1;
  double yC   = 0;

  double xNearC = -0.9;
  double yNearC = yC + 0.5*dydxNearC*(xNearC-xC) + 0.5*dydxC*(xNearC-xC);

  double xIso = 0;
  double yIso = 1;

  double xE   = 1;
  double yE   = fmaxE;

  double xNearE = 0.9;
  double yNearE = yE + 0.5*dydxNearE*(xNearE-xE) + 0.5*dydxE*(xNearE-xE);


  RigidBodyDynamics::Math::MatrixNd concPts1 = SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(  xC,   yC,  dydxC,
          xNearC, yNearC,dydxNearC,cC);
  RigidBodyDynamics::Math::MatrixNd concPts2 = SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(xNearC,yNearC,dydxNearC,
          xIso,  yIso,  dydxIso,  cC);
  RigidBodyDynamics::Math::MatrixNd eccPts1 = SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(  xIso,  yIso,  dydxIso,
          xNearE,  yNearE,  dydxNearE, cE);
  RigidBodyDynamics::Math::MatrixNd eccPts2 = SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(xNearE, yNearE, dydxNearE,
          xE,   yE,   dydxE, cE);

  RigidBodyDynamics::Math::MatrixNd mX(6,4), mY(6,4);
  mX.col(0) = concPts1.col(0);
  mX.col(1) = concPts2.col(0);
  mX.col(2) = eccPts1.col(0);
  mX.col(3) = eccPts2.col(0);

  mY.col(0) = concPts1.col(1);
  mY.col(1) = concPts2.col(1);
  mY.col(2) = eccPts1.col(1);
  mY.col(3) = eccPts2.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    mX,mY,xC,xE,yC,yE,dydxC,dydxE,curveName);
}


void MuscleFunctionFactory::createFiberForceVelocityInverseCurve(
  double fmaxE,
  double dydxC,
  double dydxNearC,
  double dydxIso,
  double dydxE,
  double dydxNearE,
  double concCurviness,
  double eccCurviness,
  const std::string& curveName,
  SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{
  //Ensure that the inputs are within a valid range
  if(! (fmaxE > 1.0 )) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityInverseCurve: "
             << curveName
             << ": fmaxE must be greater than 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  double SimTKSignificantReal =
    pow((double)std::numeric_limits<double>::epsilon(), 7.0/8.0);

  if(! (dydxC > SimTKSignificantReal && dydxC < 1 )) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityInverseCurve "
             << curveName
             << ": dydxC must be greater than 0"
             << "and less than 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if(! (dydxNearC > dydxC && dydxNearC < 1 )) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityInverseCurve "
             << ": dydxNearC must be greater than 0 "
             << curveName
             << " and less than 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if(! (dydxIso > 1)) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityInverseCurve "
             << curveName
             << ": dydxIso must be greater than or equal to 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  //double SimTKSignificantReal =
  //  pow(std::numeric_limits<double>::epsilon(), 7.0/8.0);

  if(! (dydxE > SimTKSignificantReal && dydxE < (fmaxE-1)) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityInverseCurve "
             << curveName
             << ": dydxE must be greater than or equal to 0"
             << " and less than fmaxE-1 (" << (fmaxE-1) << ")"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if(! (dydxNearE >= dydxE && dydxNearE < (fmaxE-1)) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityInverseCurve "
             << curveName
             << ": dydxNearE must be greater than or equal to dydxE"
             << "and less than fmaxE-1 ("<< (fmaxE-1) << ")"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if(! (concCurviness <= 1.0 && concCurviness >= 0) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityInverseCurve "
             << curveName
             << ": concCurviness must be between 0 and 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if(! (eccCurviness <= 1.0 && eccCurviness >= 0) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceVelocityInverseCurve "
             << curveName
             << ": eccCurviness must be between 0 and 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  std::string name = curveName;
  name.append(".createFiberForceVelocityInverseCurve");

  //Translate the users parameters into Bezier point locations
  double cC = SegmentedQuinticBezierToolkit::scaleCurviness(concCurviness);
  double cE = SegmentedQuinticBezierToolkit::scaleCurviness(eccCurviness);

  //Compute the concentric control point locations
  double xC   = -1;
  double yC   = 0;

  double xNearC = -0.9;
  double yNearC = yC + 0.5*dydxNearC*(xNearC-xC) + 0.5*dydxC*(xNearC-xC);

  double xIso = 0;
  double yIso = 1;

  double xE   = 1;
  double yE   = fmaxE;

  double xNearE = 0.9;
  double yNearE = yE + 0.5*dydxNearE*(xNearE-xE) + 0.5*dydxE*(xNearE-xE);


  RigidBodyDynamics::Math::MatrixNd concPts1 = SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(   xC,   yC,  dydxC,
          xNearC, yNearC,dydxNearC,cC);
  RigidBodyDynamics::Math::MatrixNd concPts2 = SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(xNearC,yNearC,dydxNearC,
          xIso,  yIso,  dydxIso,  cC);
  RigidBodyDynamics::Math::MatrixNd eccPts1 = SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(  xIso,  yIso,  dydxIso,
          xNearE,  yNearE,  dydxNearE, cE);
  RigidBodyDynamics::Math::MatrixNd eccPts2 = SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(xNearE, yNearE, dydxNearE,
          xE,   yE,   dydxE, cE);

  RigidBodyDynamics::Math::MatrixNd mX(6,4), mY(6,4);
  mX.col(0) = concPts1.col(0);
  mX.col(1) = concPts2.col(0);
  mX.col(2) =  eccPts1.col(0);
  mX.col(3) =  eccPts2.col(0);

  mY.col(0) = concPts1.col(1);
  mY.col(1) = concPts2.col(1);
  mY.col(2) =  eccPts1.col(1);
  mY.col(3) =  eccPts2.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    mY,mX,yC,yE,xC,xE,1/dydxC,1/dydxE, curveName);

}

void MuscleFunctionFactory::createFiberCompressiveForcePennationCurve(
  double phi0,
  double k,
  double curviness,
  const std::string& curveName,
  SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{
  //Check the input arguments
  if( !(phi0>0 && phi0<(M_PI/2.0)) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberCompressiveForcePennationCurve "
             << curveName
             << ": phi0 must be greater than 0, and less than Pi/2"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(k > (1.0/(M_PI/2.0-phi0))) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberCompressiveForcePennationCurve "
             << curveName
             << ": k must be greater than " << (1.0/(M_PI/2.0-phi0))
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(curviness>=0 && curviness <= 1)  ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberCompressiveForcePennationCurve "
             << curveName
             << ": curviness must be between 0.0 and 1.0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  std::string name=curveName;
  name.append(".createFiberCompressiveForcePennationCurve");

  //Translate the user parameters to quintic Bezier points
  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);
  double x0 = phi0;
  double y0 = 0;
  double dydx0 = 0;
  double x1 = M_PI/2.0;
  double y1 = 1;
  double dydx1 = k;

  RigidBodyDynamics::Math::MatrixNd ctrlPts = SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c);

  RigidBodyDynamics::Math::MatrixNd mX(6,1), mY(6,1);
  mX.col(0) = ctrlPts.col(0);
  mY.col(0) = ctrlPts.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    mX,mY,x0,x1,y0,y1,dydx0,dydx1,curveName);
}

void MuscleFunctionFactory::
createFiberCompressiveForceCosPennationCurve(
  double cosPhi0,
  double k,
  double curviness,
  const std::string& curveName,
  SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{
  //Check the input arguments
  if( !(cosPhi0>0 && cosPhi0 < 1) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberCompressiveForceCosPennationCurve "
             << curveName
             << ": cosPhi0 must be greater than 0, and less than 1"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(k < 1/cosPhi0) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberCompressiveForceCosPennationCurve "
             << curveName
             << ": k must be less than 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(curviness>=0 && curviness <= 1) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberCompressiveForceCosPennationCurve"
             << curveName
             << ": curviness must be between 0.0 and 1.0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  std::string name=curveName;
  name.append(".createFiberCompressiveForceCosPennationCurve");

  //Translate the user parameters to quintic Bezier points
  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);
  double x0 = 0;
  double y0 = 1;
  double dydx0 = k;
  double x1 = cosPhi0;
  double y1 = 0;
  double dydx1 = 0;

  RigidBodyDynamics::Math::MatrixNd ctrlPts = SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c);

  RigidBodyDynamics::Math::MatrixNd mX(6,1), mY(6,1);
  mX.col(0) = ctrlPts.col(0);
  mY.col(0) = ctrlPts.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    mX,mY,x0,x1,y0,y1,dydx0,dydx1,curveName);
}

void MuscleFunctionFactory::createFiberCompressiveForceLengthCurve(
  double lmax,
  double k,
  double curviness,
  const std::string& curveName,
  SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{

  if( !(lmax>0) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberCompressiveForceLength "
             << curveName
             << ": l0 must be greater than 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(k < -(1.0/lmax)) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberCompressiveForceLength "
             << curveName
             << ": k must be less than "
             << -(1.0/lmax)
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(curviness>=0 && curviness <= 1) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberCompressiveForceLength "
             << curveName
             << ": curviness must be between 0.0 and 1.0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  std::string caller = curveName;
  caller.append(".createFiberCompressiveForceLength");

  //Translate the user parameters to quintic Bezier points
  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);
  double x0 = 0.0;
  double y0 = 1;
  double dydx0 = k;
  double x1 = lmax;
  double y1 = 0;
  double dydx1 = 0;

  RigidBodyDynamics::Math::MatrixNd ctrlPts = SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c);

  RigidBodyDynamics::Math::MatrixNd mX(6,1), mY(6,1);
  mX.col(0) = ctrlPts.col(0);
  mY.col(0) = ctrlPts.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    mX,mY,x0,x1,y0,y1,dydx0,dydx1,curveName);
}


void  MuscleFunctionFactory::createFiberForceLengthCurve(
  double eZero,
  double eIso,
  double kLow,
  double kIso,
  double curviness,
  const std::string& curveName,
  SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{

  if( !(eIso > eZero) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceLength "
             << curveName
             << ": The following must hold: eIso  > eZero"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(kIso > (1.0/(eIso-eZero))) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceLength "
             << curveName
             << ": kiso must be greater than 1/(eIso-eZero) ("
             << (1.0/(eIso-eZero)) << ")"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(kLow > 0.0 && kLow < 1/(eIso-eZero)) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceLength "
             << curveName
             << ": kLow must be greater than 0 and less than"
             << 1.0/(eIso-eZero)
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(curviness>=0 && curviness <= 1) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createFiberForceLength "
             << curveName
             << ": curviness must be between 0.0 and 1.0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  std::string callerName = curveName;
  callerName.append(".createFiberForceLength");


  //Translate the user parameters to quintic Bezier points
  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);
  double xZero = 1+eZero;
  double yZero = 0;

  double xIso = 1 + eIso;
  double yIso = 1;

  double deltaX = std::min(0.1*(1.0/kIso), 0.1*(xIso-xZero));

  double xLow   = xZero + deltaX;
  double xfoot  = xZero + 0.5*(xLow-xZero);
  double yfoot  = 0;
  double yLow   = yfoot + kLow*(xLow-xfoot);

  //Compute the Quintic Bezier control points
  RigidBodyDynamics::Math::MatrixNd p0 = SegmentedQuinticBezierToolkit::
                                         calcQuinticBezierCornerControlPoints(xZero, yZero, 0,
                                             xLow, yLow,  kLow,c);

  RigidBodyDynamics::Math::MatrixNd p1 = SegmentedQuinticBezierToolkit::
                                         calcQuinticBezierCornerControlPoints(xLow, yLow, kLow,
                                             xIso, yIso, kIso, c);
  RigidBodyDynamics::Math::MatrixNd mX(6,2);
  RigidBodyDynamics::Math::MatrixNd mY(6,2);

  mX.col(0) = p0.col(0);
  mY.col(0) = p0.col(1);

  mX.col(1) = p1.col(0);
  mY.col(1) = p1.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    mX, mY, xZero, xIso, yZero, yIso, 0.0, kIso, curveName);
}




void MuscleFunctionFactory::
createTendonForceLengthCurve( double eIso, double kIso,
                              double fToe, double curviness,
                              const std::string& curveName,
                              SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{

  if( !(eIso>0) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createTendonForceLengthCurve "
             << curveName
             << ": eIso must be greater than 0, but "
             << eIso << " was entered"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(fToe>0 && fToe < 1) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createTendonForceLengthCurve "
             << curveName
             << ": fToe must be greater than 0 and less than 1, but "
             << fToe
             << " was entered"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(kIso > (1/eIso)) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createTendonForceLengthCurve "
             << curveName
             << ": kIso must be greater than 1/eIso, ("
             << (1/eIso) << "), but kIso ("
             << kIso << ") was entered"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }


  if( !(curviness>=0 && curviness <= 1) ) {
    ostringstream errormsg;
    errormsg << "MuscleFunctionFactory::"
             << "createTendonForceLengthCurve "
             << curveName
             << " : curviness must be between 0.0 and 1.0, but "
             << curviness << " was entered"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  std::string callerName = curveName;
  callerName.append(".createTendonForceLengthCurve");

  //Translate the user parameters to quintic Bezier points
  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);
  double x0 = 1.0;
  double y0 = 0;
  double dydx0 = 0;

  double xIso = 1.0 + eIso;
  double yIso = 1;
  double dydxIso = kIso;

  //Location where the curved section becomes linear
  double yToe = fToe;
  double xToe = (yToe-1)/kIso + xIso;


  //To limit the 2nd derivative of the toe region the line it tends to
  //has to intersect the x axis to the right of the origin
  double xFoot = 1.0+(xToe-1.0)/10.0;
  double yFoot = 0;
  double dydxToe = (yToe-yFoot)/(xToe-xFoot);

  //Compute the location of the corner formed by the average slope of the
  //toe and the slope of the linear section
  double yToeMid = yToe*0.5;
  double xToeMid = (yToeMid-yIso)/kIso + xIso;
  double dydxToeMid = (yToeMid-yFoot)/(xToeMid-xFoot);

  //Compute the location of the control point to the left of the corner
  double xToeCtrl = xFoot + 0.5*(xToeMid-xFoot);
  double yToeCtrl = yFoot + dydxToeMid*(xToeCtrl-xFoot);



  //Compute the Quintic Bezier control points
  RigidBodyDynamics::Math::MatrixNd p0 = SegmentedQuinticBezierToolkit::
                                         calcQuinticBezierCornerControlPoints(x0,y0,dydx0,
                                             xToeCtrl,yToeCtrl,dydxToeMid,c);
  RigidBodyDynamics::Math::MatrixNd p1 = SegmentedQuinticBezierToolkit::
                                         calcQuinticBezierCornerControlPoints(xToeCtrl, yToeCtrl, dydxToeMid,
                                             xToe,   yToe,  dydxIso, c);
  RigidBodyDynamics::Math::MatrixNd mX(6,2);
  RigidBodyDynamics::Math::MatrixNd mY(6,2);

  mX.col(0) = p0.col(0);
  mY.col(0) = p0.col(1);

  mX.col(1) = p1.col(0);
  mY.col(1) = p1.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    mX, mY, x0, xToe, y0,  yToe, dydx0, dydxIso, curveName);

}
