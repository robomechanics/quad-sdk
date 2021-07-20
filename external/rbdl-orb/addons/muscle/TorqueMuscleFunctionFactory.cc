/*-------------------------------------------------------------------------
                OpenSim:  SmoothSegmentedFunctionFactory.cpp
 --------------------------------------------------------------------------
 The OpenSim API is a toolkit for musculoskeletal modeling and simulation.
 See http:%opensim.stanford.edu and the NOTICE file for more information.
 OpenSim is developed at Stanford University and supported by the US
 National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA
 through the Warrior Web program.

 Copyright (c) 2005-2012 Stanford University and the Authors
 Author(s): Matthew Millard

 Licensed under the Apache License, Version 2.0 (the 'License'); you may
 not use this file except in compliance with the License. You may obtain a
 copy of the License at http:%www.apache.org/licenses/LICENSE-2.0.

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an 'AS IS' BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 --------------------------------------------------------------------------

 Derivative work
 Date      : September 2016
 Authors(s): Millard
 Updates   : Made active torque-angle, passive-torque-angle, torque-velocity
             and tendon-torque-angle curves based on the equivalent line-type
             curves in OpenSim.
*/

#include "TorqueMuscleFunctionFactory.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>

#include <rbdl/rbdl_errors.h>

using namespace std;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons::Muscle;
using namespace RigidBodyDynamics::Addons::Geometry;

static double SQRTEPS = sqrt( (double)std::numeric_limits<double>::epsilon());

//=============================================================================
// Anderson 2007 Active Torque Angle Curve
//=============================================================================

void TorqueMuscleFunctionFactory::
createAnderson2007ActiveTorqueAngleCurve(
  double c2,
  double c3,
  const std::string& curveName,
  SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{
  //Check the input arguments
  if( !(c2 > 0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createAnderson2007ActiveTorqueAngleCurve "
             << curveName
             << ": c2 must be greater than 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }


  std::string name=curveName;
  name.append(".createAnderson2007ActiveTorqueAngleCurve");

  //For now these advanced paramters are hidden. They will only be
  //uncovered if absolutely necessary.
  double minValueAtShoulders = 0;
  double minShoulderSlopeMagnitude = 0;

  double curviness = 0.5;
  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);

  //Translate the user parameters to quintic Bezier points
  double x0 = c3 - 1.05*(0.5*(M_PI/c2));
  double x1 = c3 - 0.95*(0.5*(M_PI/c2));
  double x2 = c3;
  double x3 = c3 + 0.95*(0.5*(M_PI/c2));
  double x4 = c3 + 1.05*(0.5*(M_PI/c2));

  double y0 = minValueAtShoulders;
  double y1 = cos(c2*(x1-c3));
  double y2 = cos(c2*(x2-c3));
  double y3 = cos(c2*(x3-c3));
  double y4 = minValueAtShoulders;

  double dydx0 =  minShoulderSlopeMagnitude;
  double dydx1 = -sin(c2*(x1-c3))*c2;
  double dydx2 = -sin(c2*(x2-c3))*c2;
  double dydx3 = -sin(c2*(x3-c3))*c2;
  double dydx4 = -minShoulderSlopeMagnitude;


  //Compute the Quintic Bezier control points
  RigidBodyDynamics::Math::MatrixNd p0 =
    SegmentedQuinticBezierToolkit::
    calcQuinticBezierCornerControlPoints(x0,y0,dydx0,
                                         x1,y1,dydx1,c);

  RigidBodyDynamics::Math::MatrixNd p1 =
    SegmentedQuinticBezierToolkit::
    calcQuinticBezierCornerControlPoints(x1,y1,dydx1,
                                         x2,y2,dydx2,c);

  RigidBodyDynamics::Math::MatrixNd p2 =
    SegmentedQuinticBezierToolkit::
    calcQuinticBezierCornerControlPoints(x2,y2,dydx2,
                                         x3,y3,dydx3,c);

  RigidBodyDynamics::Math::MatrixNd p3 =
    SegmentedQuinticBezierToolkit::
    calcQuinticBezierCornerControlPoints(x3,y3,dydx3,
                                         x4,y4,dydx4,c);

  RigidBodyDynamics::Math::MatrixNd mX(6,4);
  RigidBodyDynamics::Math::MatrixNd mY(6,4);

  mX.col(0) = p0.col(0);
  mY.col(0) = p0.col(1);
  mX.col(1) = p1.col(0);
  mY.col(1) = p1.col(1);
  mX.col(2) = p2.col(0);
  mY.col(2) = p2.col(1);
  mX.col(3) = p3.col(0);
  mY.col(3) = p3.col(1);


  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    mX, mY, x0, x4, y0, y4, dydx0, dydx4, curveName);
}

//=============================================================================
// ANDERSON 2007 Active Torque Angular Velocity Curve
//=============================================================================
void TorqueMuscleFunctionFactory::
createAnderson2007ActiveTorqueVelocityCurve(
  double c4,
  double c5,
  double c6,
  double minEccentricMultiplier,
  double maxEccentricMultiplier,
  const std::string& curveName,
  SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{
  //Check the input arguments
  if( !(c4 < c5) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createAndersonActiveTorqueVelocityCurve "
             << curveName
             << ": c4 must be greater than c5"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !((c4 > 0)) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createAndersonActiveTorqueVelocityCurve "
             << curveName
             << ": c4 must be greater than 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(c6 > 0.0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createAndersonActiveTorqueVelocityCurve "
             << curveName
             << ": c6 must be greater than 1.0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(minEccentricMultiplier > 1.0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createAndersonActiveTorqueVelocityCurve "
             << curveName
             << ": minEccentricMultiplier must be greater than 1.0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(maxEccentricMultiplier > minEccentricMultiplier) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createAndersonActiveTorqueVelocityCurve "
             << curveName
             << ": maxEccentricMultiplier must be greater than "
             << " minEccentricMultiplier"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  //Advanced settings that we'll hide for now
  double minShoulderSlopeMagnitude  = 0;
  double curviness = 0.75;
  double c         = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);

  //Go and get the value of the curve that is closest to
  //the maximum contraction velocity by setting rhs of Eqn. 9
  //to 0 and solving
  double dthMaxConc = abs( 2.0*c4*c5/(c5-3.0*c4) );

  //Go and evaluate the concentric side of the Anderson curve
  //at 1/2 of omega max - we need this to use the updated
  //torque-velocity curve.
  double wMid     =   dthMaxConc*0.50;
  double tvMidDen =  (2*c4*c5 + wMid*(2*c5-4*c4));
  double tvMid    =  (2*c4*c5 + wMid*(c5-3*c4))/tvMidDen;

  tvMid = min(tvMid,0.45);
  double tvMaxEcc = 1.1 + c6*0.2;

  createTorqueVelocityCurve(tvMaxEcc,
                            tvMid,
                            curveName,
                            smoothSegmentedFunctionToUpdate);


}
//=============================================================================
// ANDERSON 2007 Passive Torque Angle Curve
//=============================================================================
void TorqueMuscleFunctionFactory::
createAnderson2007PassiveTorqueAngleCurve(
  double scale,
  double c1,
  double b1,
  double k1,
  double b2,
  double k2,
  const std::string& curveName,
  SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{

  if( !(scale > 0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createAnderson2007PassiveTorqueAngleCurve "
             << curveName
             << ": scale must be greater than 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( !(c1 > 0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createAnderson2007PassiveTorqueAngleCurve "
             << curveName
             << ": c1 must be greater than 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  //Advanced settings that we'll hide for now
  double curviness = 0.75;
  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);
  double minShoulderSlopeMagnitude = 0;

  //Zero out the coefficients associated with a
  //the side of the curve that goes negative.
  bool flag_oneSided = true;

  if(flag_oneSided) {
    if(fabs(b1) > 0) {
      if(b1 > 0) {
        b2 = 0;
        k2 = 0;
      } else {
        b1 = 0;
        k1 = 0;
      }

    } else if(fabs(b2) > 0) {
      if(b2 > 0) {
        b1 = 0;
        k1 = 0;
      } else {
        b2 = 0;
        k2 = 0;
      }
    }
  }
  //Divide up the curve into a left half
  //and a right half, rather than 1 and 2.
  //Why? These two different halves require different
  //   Bezier curves.

  double c1Scale = c1*scale;
  double thL    = 0.; //left
  double thR    = 0.; //right
  double DtauDthL = 0.;
  double DtauDthR = 0.;
  double bL     = 0.;
  double kL     = 0.;
  double bR     = 0.;
  double kR     = 0.;

  int curveType   = 0; //flat curve
  int flag_thL  = 0;
  int flag_thR  = 0;

  if(fabs(k1)>0 && fabs(b1)>0) {
    //The value of theta where the passive force generated by the
    //muscle is equal to 1 maximum isometric contraction.
    thL     = (1/k1)*log(fabs( c1Scale/b1 ));
    DtauDthL  = b1*k1*exp(thL*k1);
    bL      = b1;
    kL      = k1;
    flag_thL  = 1;
  }

  if(fabs(k2)>0 && fabs(b2)>0) {
    //The value of theta where the passive force generated by the
    //muscle is equal to 1 maximum isometric contraction.
    thR     = (1/k2)*log(fabs( c1Scale/b2 ));
    DtauDthR  = b2*k2*exp(thR*k2);
    bR      = b2;
    kR      = k2;
    flag_thR  = 1;
  }

  //A 'left' curve has a negative slope,
  //A 'right' curve has a positive slope.
  if(DtauDthL > DtauDthR) {
    double tmpD = thL;
    thL   = thR;
    thR   = tmpD;

    tmpD  =  bR;
    bR    = bL;
    bL    = tmpD;

    tmpD  = kR;
    kR    = kL;
    kL    = tmpD;

    tmpD    = DtauDthL;
    DtauDthL  = DtauDthR;
    DtauDthR  = tmpD;

    int tmpI = flag_thL;
    flag_thL = flag_thR;
    flag_thR = tmpI;
  }


  if(flag_thL) {
    curveType   = curveType + 1;
  }
  if(flag_thR) {
    curveType   = curveType + 2;
  }

  RigidBodyDynamics::Math::MatrixNd mX(6,1);
  RigidBodyDynamics::Math::MatrixNd mY(6,1);

  double xStart  = 0;
  double xEnd    = 0;
  double yStart  = 0;
  double yEnd    = 0;
  double dydxStart = 0;
  double dydxEnd   = 0;


  switch(curveType) {

  case 0: {
    //No curve - it's a flat line
    RigidBodyDynamics::Math::MatrixNd p0 =
      SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(0.,0.,0.,
                                           1.,0.,0.,c);

    mX.col(0) = p0.col(0);
    mY.col(0) = p0.col(1);

  }
  break;
  case 1: {
    //Get a point on the curve that is close to 0.
    double x1  = (1/kL)*log(fabs(0.01*c1Scale/bL) );
    double y1  = bL*exp(kL*x1);
    double dydx1 = bL*kL*exp(kL*x1);

    //Get a point that is at 1 maximum isometric torque
    double x3  = thL;
    double y3  = bL*exp(kL*x3);
    double dydx3 = bL*kL*exp(kL*x3);

    //Get a mid-point
    double x2  = 0.5*(x1+x3);
    double y2  = bL*exp(kL*x2);
    double dydx2 = bL*kL*exp(kL*x2);

    //Past the crossing point of the linear extrapolation
    double x0   = x1 - 2*y1/dydx1;
    double y0   = 0;
    double dydx0  = minShoulderSlopeMagnitude*copysign(1.0,dydx1);

    xStart    = x3;
    xEnd      = x0;
    yStart    = y3;
    yEnd      = y0;
    dydxStart = dydx3;
    dydxEnd   = dydx0;

    RigidBodyDynamics::Math::MatrixNd p0 =
      SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x3,y3,dydx3,
                                           x2,y2,dydx2,c);
    RigidBodyDynamics::Math::MatrixNd p1 =
      SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x2,y2,dydx2,
                                           x1,y1,dydx1,c);
    RigidBodyDynamics::Math::MatrixNd p2 =
      SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x1,y1,dydx1,
                                           x0,y0,dydx0,c);

    mX.resize(6,3);
    mY.resize(6,3);

    mX.col(0) = p0.col(0);
    mY.col(0) = p0.col(1);
    mX.col(1) = p1.col(0);
    mY.col(1) = p1.col(1);
    mX.col(2) = p2.col(0);
    mY.col(2) = p2.col(1);

  }
  break;
  case 2: {
    //Get a point on the curve that is close to 0.
    double x1  = (1/kR)*log(fabs(0.01*c1Scale/bR) );
    double y1  = bR*exp(kR*x1);
    double dydx1 = bR*kR*exp(kR*x1);

    //Go just past the crossing point of the linear extrapolation
    double x0   = x1 - 2*y1/dydx1;
    double y0   = 0;
    double dydx0 = minShoulderSlopeMagnitude*copysign(1.0,dydx1);

    //Get a point close to 1 maximum isometric torque
    double x3  = thR;
    double y3  = bR*exp(kR*x3);
    double dydx3 = bR*kR*exp(kR*x3);

    //Get a mid point.
    double x2   = 0.5*(x1+x3);
    double y2   = bR*exp(kR*x2);
    double dydx2  = bR*kR*exp(kR*x2);

    xStart    = x0;
    xEnd    = x3;
    yStart    = y0;
    yEnd    = y3;
    dydxStart   = dydx0;
    dydxEnd   = dydx3;

    RigidBodyDynamics::Math::MatrixNd p0 =
      SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x0,y0,dydx0,
                                           x1,y1,dydx1,c);
    RigidBodyDynamics::Math::MatrixNd p1 =
      SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x1,y1,dydx1,
                                           x2,y2,dydx2,c);
    RigidBodyDynamics::Math::MatrixNd p2 =
      SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x2,y2,dydx2,
                                           x3,y3,dydx3,c);
    mX.resize(6,3);
    mY.resize(6,3);

    mX.col(0) = p0.col(0);
    mY.col(0) = p0.col(1);
    mX.col(1) = p1.col(0);
    mY.col(1) = p1.col(1);
    mX.col(2) = p2.col(0);
    mY.col(2) = p2.col(1);

  }
  break;
  case 3: {
    //Only when flag_oneSided = false;
    double x0 = thL;
    double x4 = thR;

    double x2 = 0.5*(x0 + x4);
    double x1 = 0.5*(x0 + x2);
    double x3 = 0.5*(x2 + x4);

    double y0 = b1*exp(k1*x0)
                + b2*exp(k2*x0);
    double y1 = b1*exp(k1*x1)
                + b2*exp(k2*x1);
    double y2 = b1*exp(k1*x2)
                + b2*exp(k2*x2);
    double y3 = b1*exp(k1*x3)
                + b2*exp(k2*x3);
    double y4 = b1*exp(k1*x4)
                + b2*exp(k2*x4);

    double dydx0 =   b1*k1*exp(k1*x0)
                     + b2*k2*exp(k2*x0);
    double dydx1 =   b1*k1*exp(k1*x1)
                     + b2*k2*exp(k2*x1);
    double dydx2 =   b1*k1*exp(k1*x2)
                     + b2*k2*exp(k2*x2);
    double dydx3 =   b1*k1*exp(k1*x3)
                     + b2*k2*exp(k2*x3);
    double dydx4 =   b1*k1*exp(k1*x4)
                     + b2*k2*exp(k2*x4);

    xStart    = x0;
    xEnd    = x4;
    yStart    = y0;
    yEnd    = y4;
    dydxStart   = dydx0;
    dydxEnd   = dydx4;

    RigidBodyDynamics::Math::MatrixNd p0 =
      SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x0,y0,dydx0,
                                           x1,y1,dydx1,c);
    RigidBodyDynamics::Math::MatrixNd p1 =
      SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x1,y1,dydx1,
                                           x2,y2,dydx2,c);
    RigidBodyDynamics::Math::MatrixNd p2 =
      SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x2,y2,dydx2,
                                           x3,y3,dydx3,c);
    RigidBodyDynamics::Math::MatrixNd p3 =
      SegmentedQuinticBezierToolkit::
      calcQuinticBezierCornerControlPoints(x3,y3,dydx3,
                                           x4,y4,dydx4,c);

    mX.resize(6,4);
    mY.resize(6,4);

    mX.col(0) = p0.col(0);
    mY.col(0) = p0.col(1);
    mX.col(1) = p1.col(0);
    mY.col(1) = p1.col(1);
    mX.col(2) = p2.col(0);
    mY.col(2) = p2.col(1);
    mX.col(3) = p3.col(0);
    mY.col(3) = p3.col(1);

  }
  break;
  default: {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createAnderson2007PassiveTorqueAngleCurve "
             << curveName
             << ": undefined curveType"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }

  };

  //Normalize the y values.
  mY = mY*(1/c1Scale);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    mX,                mY,
    xStart,            xEnd,
    yStart/c1Scale,    yEnd/c1Scale,
    dydxStart/c1Scale, dydxEnd/c1Scale,
    curveName);
}

//=============================================================================
//
// New torque-muscle characteristic curves
//
//=============================================================================


//=============================================================================
// torque-velocity curves
//=============================================================================

void TorqueMuscleFunctionFactory::createTorqueVelocityCurve(
  double tvAtEccentricOmegaMax,
  double tvAtHalfConcentricOmegaMax,
  const std::string& curveName,
  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
  smoothSegmentedFunctionToUpdate )
{

  double slopeAtConcentricOmegaMax  =  0.0;
  double slopeNearEccentricOmegaMax = -0.025;
  double slopeAtEccentricOmegaMax   =  0.0;
  double eccentricCurviness         =  0.75;

  createTorqueVelocityCurve(
    tvAtEccentricOmegaMax,
    tvAtHalfConcentricOmegaMax,
    slopeAtConcentricOmegaMax,
    slopeNearEccentricOmegaMax,
    slopeAtEccentricOmegaMax,
    eccentricCurviness,
    curveName,
    smoothSegmentedFunctionToUpdate);

}

void  TorqueMuscleFunctionFactory::createTorqueVelocityCurve(
  double tvAtEccentricOmegaMax,
  double tvAtHalfConcentricOmegaMax,
  double slopeAtConcentricOmegaMax,
  double slopeNearEccentricOmegaMax,
  double slopeAtEccentricOmegaMax,
  double eccentricCurviness,
  const std::string& curveName,
  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
  smoothSegmentedFunctionToUpdate )
{

  if( (tvAtEccentricOmegaMax < 1.05) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTorqueVelocityCurve "
             << curveName
             << ": tvAtEccentricOmegaMax must be greater than 1.05"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if(    (tvAtHalfConcentricOmegaMax < 0.05
          || tvAtHalfConcentricOmegaMax > 0.45) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTorqueVelocityCurve "
             << curveName
             << ": tvAtHalfOmegaMax must be in the interval [0.05,0.45]"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( (slopeAtConcentricOmegaMax > 0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTorqueVelocityCurve "
             << curveName
             << ": slopeAtConcentricOmegaMax cannot be less than 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( (slopeNearEccentricOmegaMax > 0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTorqueVelocityCurve "
             << curveName
             << ": slopeNearEccentricOmegaMax cannot be less than 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( (slopeAtEccentricOmegaMax > 0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTorqueVelocityCurve "
             << curveName
             << ": slopeAtEccentricOmegaMax cannot be less than 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( (eccentricCurviness < 0 || eccentricCurviness > 1.0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTorqueVelocityCurve "
             << curveName
             << ": eccentricCurviness must be in the interval [0,1]"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }


  double omegaMax = 1.0;
  double wmaxC =  omegaMax; //In biomechanics the concentric side gets a
  //a +'ve signed velocity
  double wmaxE = -omegaMax;

  //-----------------------------------------------------------------------
  // 1. Fit concentric Bezier curves to a Hill-type hyperbolic curve
  //-----------------------------------------------------------------------

  /*
  First we compute the terms that are consistent with a Hill-type concentric
  contraction. Starting from Hill's hyperbolic equation

  f(w)     = (fiso*b - a*w) / (b+w)


  Taking a derivative w.r.t omega yields

  df(w)/dw = [ (fiso*b - a*w + a*(b+w))] / (b+w)^2

  at w = wmaxC the numerator goes to 0

  (fiso*b - a*wmaxC) / (b + wmaxC) = 0
  (fiso*b - a*wmaxC) = 0;
   b =  a*wmaxC/fiso;

  Subtituting this expression for b into the expression when
  f(wHalf) = tvAtHalfOmegaMax yields this expression for parameter a

  a = tvAtHalfOmegaMax*w*fiso ...
      / (wmaxC*tvAtHalfOmegaMax + fiso*wmaxC - fiso*w);

  */

  double fiso = 1.0;
  double w = 0.5*wmaxC;
  double a = -tvAtHalfConcentricOmegaMax*w*fiso
             / (wmaxC*tvAtHalfConcentricOmegaMax - fiso*wmaxC + fiso*w);

  double b = a*wmaxC/fiso;
  double yCheck  = (b*fiso-a*w)/(b+w);

  if( abs(yCheck-tvAtHalfConcentricOmegaMax) > SQRTEPS ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTorqueVelocityCurve "
             << curveName
             << ": Internal error fitting the concentric curve to Hill's "
             << "hyperbolic curve. This error condition was true: "
             << "abs(yCheck-tvAtHalfOmegaMax) > sqrt(eps)"
             << "Consult the maintainers of this addon."
             << endl;
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }


  w = 0*wmaxC;
  double dydxIso = (-(a)*(b+w) - (b*fiso-a*w)) / ((b+w)*(b+w));


  w = 0.9*wmaxC;
  double dydxNearC = (-(a)*(b+w) - (b*fiso-a*w)) / ((b+w)*(b+w));


  if( dydxNearC > slopeAtConcentricOmegaMax || abs(dydxNearC) > abs(1/wmaxC) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTorqueVelocityCurve "
             << curveName
             << ": Internal error fitting the concentric curve to Hill's "
             << "hyperbolic curve. This error condition was true: "
             << " dydxNearC < dydxC || dydxNearC > abs(1/wmaxC)"
             << "Consult the maintainers of this addon."
             << endl;
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }

  //----------------------------------------------------------------------------
  // Iterate over the curviness value to get a good match between Hill's
  // hyperbolic curve and the Bezier curve
  //----------------------------------------------------------------------------

  double xNearC =  0.9*wmaxC;
  double yNearC = (b*fiso-a*w)/(b+w);
  double xIso = 0;
  double yIso = 1.0;

  double cC = 0.5;

  MatrixNd pts = SegmentedQuinticBezierToolkit::
                 calcQuinticBezierCornerControlPoints(xIso,  yIso,   dydxIso,
                     xNearC,yNearC, dydxNearC,
                     cC);
  MatrixNd xpts(6,1);
  xpts.col(0) = pts.col(0);
  MatrixNd ypts(6,1);
  ypts.col(0) = pts.col(1);

  SmoothSegmentedFunction fvCFcn = SmoothSegmentedFunction(
                                     xpts,ypts,
                                     xIso,xNearC,
                                     yIso,yNearC,
                                     dydxIso,dydxNearC,
                                     "tvFcn");
  SmoothSegmentedFunction fvCFcnLeft = SmoothSegmentedFunction();
  SmoothSegmentedFunction fvCFcnRight = SmoothSegmentedFunction();

  int nSample   = 10;
  double f      = 0;
  double yHill  = 0;

  //Calculate the error of the Bezier curve with the starting curviness value
  //of 0.5
  for(int j=0; j<nSample; ++j) {
    w     = ((double)(j-1))*wmaxC/((double)nSample-1.0);
    yHill = (b*fiso-a*w)/(b+w);
    f     += abs( fvCFcn.calcValue(w)-yHill);
  }

  double fBest  = f;
  double fLeft  = 0.;
  double fRight = 0.;
  double cCBest = cC;

  double cCLeft   = 0;
  double cCRight  = 0;
  double h        = 0.25;

  std::string fvLeftName("fvLeft");
  std::string fvRightName("fvRight");

  //Use the bisection method to solve for the curviness value that
  //results in the best fit to Hill's hyperbola. Here we are using
  //the bisection method rather than Newton's method because I had
  //convergence problems with Newton's method in the prototype code.
  for(int i=0; i<10; ++i) {

    //Form the left-approaching curve and evaluate its error
    cCLeft = cCBest - h;

    pts = SegmentedQuinticBezierToolkit::
          calcQuinticBezierCornerControlPoints(
            xIso,  yIso,   dydxIso,
            xNearC,yNearC, dydxNearC,
            cCLeft);
    xpts.col(0) = pts.col(0);
    ypts.col(0) = pts.col(1);
    fvCFcnLeft.updSmoothSegmentedFunction(xpts,      ypts,
                                          xIso,      xNearC,
                                          yIso,      yNearC,
                                          dydxIso,   dydxNearC,
                                          fvLeftName);
    fLeft = 0;
    for(int j=0; j<nSample; ++j) {
      w     = ((double)(j-1))*wmaxC/((double)nSample-1.0);
      yHill = (b*fiso-a*w)/(b+w);
      fLeft += abs( fvCFcnLeft.calcValue(w)-yHill);
    }

    //Form the right-approaching curve and evaluate its error
    cCRight = cCBest + h;
    pts = SegmentedQuinticBezierToolkit::
          calcQuinticBezierCornerControlPoints(
            xIso,  yIso,   dydxIso,
            xNearC,yNearC, dydxNearC, cCRight);
    xpts.col(0) = pts.col(0);
    ypts.col(0) = pts.col(1);
    fvCFcnRight.updSmoothSegmentedFunction(xpts,     ypts,
                                           xIso,     xNearC,
                                           yIso,     yNearC,
                                           dydxIso,  dydxNearC,
                                           fvRightName);
    fRight = 0;
    for(int j=0; j<nSample; ++j) {
      w     = ((double)(j-1))*wmaxC/((double)nSample-1.0);
      yHill = (b*fiso-a*w)/(b+w);
      fRight += abs( fvCFcnRight.calcValue(w)-yHill);
    }

    if(abs(fLeft)<abs(fBest)) {
      fBest  = fLeft;
      cCBest = cCLeft;
    }
    if(abs(fRight)<abs(fBest)) {
      fBest  = fRight;
      cCBest = cCRight;
    }

    h = h/2.0;
  }

  cC = cCBest;
  double cE = SegmentedQuinticBezierToolkit::scaleCurviness(eccentricCurviness);

  MatrixNd xM(6,4);
  MatrixNd yM(6,4);

  double xC = wmaxC;
  double yC = 0.0;

  double xE = wmaxE;
  double yE = tvAtEccentricOmegaMax;

  double xNearE     = 0.9*wmaxE;
  double dydxNearE  = slopeAtEccentricOmegaMax;
  double yNearE     = yE  + 0.5*dydxNearE*(xNearE-xE)
                      + 0.5*slopeAtEccentricOmegaMax*(xNearE-xE);


  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(
          xE,yE, slopeAtEccentricOmegaMax,
          xNearE,yNearE, dydxNearE,
          cE);

  xM.col(0) = pts.col(0);
  yM.col(0) = pts.col(1);

  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(
          xNearE,yNearE, dydxNearE,
          xIso,yIso,dydxIso,
          cE);

  xM.col(1) = pts.col(0);
  yM.col(1) = pts.col(1);


  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(
          xIso,yIso,dydxIso,
          xNearC,yNearC, dydxNearC,
          cC);

  xM.col(2) = pts.col(0);
  yM.col(2) = pts.col(1);


  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(
          xNearC,yNearC, dydxNearC,
          xC,yC, slopeAtConcentricOmegaMax,
          cC);

  xM.col(3) = pts.col(0);
  yM.col(3) = pts.col(1);


  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    xM,yM,
    xE,xC,
    yE,yC,
    slopeAtEccentricOmegaMax,
    slopeAtConcentricOmegaMax,
    curveName);

}


//=============================================================================
// passive-torque angle curve
//=============================================================================
void TorqueMuscleFunctionFactory::createPassiveTorqueAngleCurve(
  double angleAtZeroTorque,
  double angleAtOneNormTorque,
  const std::string& curveName,
  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
  smoothSegmentedFunctionToUpdate )
{

  double stiffnessAtOneNormTorque =
    4.6/(angleAtOneNormTorque-angleAtZeroTorque);
  double stiffnessAtLowTorque = 0.01*stiffnessAtOneNormTorque;
  double curviness = 0.5;

  createPassiveTorqueAngleCurve(
    angleAtZeroTorque,
    angleAtOneNormTorque,
    stiffnessAtLowTorque,
    stiffnessAtOneNormTorque,
    curviness,
    curveName,
    smoothSegmentedFunctionToUpdate);

}

void TorqueMuscleFunctionFactory::createPassiveTorqueAngleCurve(
  double angleAtZeroTorque,
  double angleAtOneNormTorque,
  double stiffnessAtLowTorque,
  double stiffnessAtOneNormTorque,
  double curviness,
  const std::string& curveName,
  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
  smoothSegmentedFunctionToUpdate )
{

  if( abs(angleAtOneNormTorque - angleAtZeroTorque) <= SQRTEPS) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createPassiveTorqueAngleCurve "
             << curveName
             << ": abs(angleAtOneNormTorque - angleAtZeroTorque) must be "
             << "greater than sqrt(eps)"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( abs(stiffnessAtLowTorque) > (0.9/abs(angleAtOneNormTorque
                                   -angleAtZeroTorque)) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createPassiveTorqueAngleCurve "
             << curveName
             << ": stiffnessAtLowTorque has a magnitude that is too "
             << "large, it exceeds 0.9/abs(angleAtOneNormTorque-angleAtZeroTorque)"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( abs(stiffnessAtOneNormTorque) < (1.1/abs(angleAtOneNormTorque
                                       -angleAtZeroTorque)) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createPassiveTorqueAngleCurve "
             << curveName
             << ": stiffnessAtOneNormTorque has a magnitude that is too "
             << "small, it is less than 1.1/abs(angleAtOneNormTorque-angleAtZeroTorque)"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( stiffnessAtOneNormTorque*stiffnessAtLowTorque < 0.0 ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createPassiveTorqueAngleCurve "
             << curveName
             << ": stiffnessAtLowTorque and  stiffnessAtOneNormTorque must have the"
             << " same sign."
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( stiffnessAtOneNormTorque*(angleAtOneNormTorque-angleAtZeroTorque) < 0.0) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createPassiveTorqueAngleCurve "
             << curveName
             << ": stiffnessAtOneNormTorque must have the same sign as "
             << "(angleAtOneNormTorque-angleAtZeroTorque)"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( (curviness < 0 || curviness > 1.0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createPassiveTorqueAngleCurve "
             << curveName
             << ": curviness must be in the interval [0,1]"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  MatrixNd xM(6,2);
  MatrixNd yM(6,2);
  MatrixNd pts(6,2);

  double x0,x1,y0,y1,dydx0,dydx1;
  if(angleAtZeroTorque < angleAtOneNormTorque) {
    x0    = angleAtZeroTorque;
    x1    = angleAtOneNormTorque;
    y0    = 0.;
    y1    = 1.;
    dydx0 = 0.0;
    dydx1 = stiffnessAtOneNormTorque;
  } else {
    x0    = angleAtOneNormTorque;
    x1    = angleAtZeroTorque;
    y0    = 1.0;
    y1    = 0.0;
    dydx0 = stiffnessAtOneNormTorque;
    dydx1 = 0.0;
  }

  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);

  if(abs(stiffnessAtOneNormTorque) > SQRTEPS) {

    double delta = min( 0.1*(1.0-abs(1.0/stiffnessAtOneNormTorque)),
                        0.05*abs(x1-x0));
    if(stiffnessAtOneNormTorque < 0.) {
      delta *= -1.0;
    }

    double xLow   = angleAtZeroTorque + delta;
    double xFoot  = angleAtZeroTorque + 0.5*(xLow-angleAtZeroTorque);
    double yFoot  = 0.0;
    double yLow   = yFoot + stiffnessAtLowTorque*(xLow-xFoot);

    pts = SegmentedQuinticBezierToolkit::calcQuinticBezierCornerControlPoints(
            x0,  y0,               dydx0,
            xLow,yLow,stiffnessAtLowTorque, c);
    xM.col(0) = pts.col(0);
    yM.col(0) = pts.col(1);

    pts = SegmentedQuinticBezierToolkit::calcQuinticBezierCornerControlPoints(
            xLow,yLow,stiffnessAtLowTorque,
            x1,  y1,               dydx1, c);
    xM.col(1) = pts.col(0);
    yM.col(1) = pts.col(1);

  } else {
    pts = SegmentedQuinticBezierToolkit::
          calcQuinticBezierCornerControlPoints(
            angleAtZeroTorque,0, 0,
            angleAtOneNormTorque,0, 0,
            c);
    xM.col(0) = pts.col(0);
    yM.col(1) = pts.col(1);
  }

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    xM,    yM,
    x0,    x1,
    y0,    y1,
    dydx0, dydx1,
    curveName);


}

//=============================================================================
// active-torque angle curve
//=============================================================================


void TorqueMuscleFunctionFactory::createGaussianShapedActiveTorqueAngleCurve(
  double angleAtOneNormTorque,
  double angularStandardDeviation,
  const std::string& curveName,
  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
  smoothSegmentedFunctionToUpdate )
{

  createGaussianShapedActiveTorqueAngleCurve(
    angleAtOneNormTorque,
    angularStandardDeviation,
    0.0,
    0.0,
    0.5,
    curveName,
    smoothSegmentedFunctionToUpdate );

}


void TorqueMuscleFunctionFactory::createGaussianShapedActiveTorqueAngleCurve(
  double angleAtOneNormTorque,
  double angularStandardDeviation,
  double minSlopeAtShoulders,
  double minValueAtShoulders,
  double curviness,
  const std::string& curveName,
  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
  smoothSegmentedFunctionToUpdate )
{

  if( (angularStandardDeviation < SQRTEPS) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createGaussianShapedActiveTorqueAngleCurve "
             << curveName
             << ": angularStandardDeviation is less than sqrt(eps)"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( (minValueAtShoulders < 0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createGaussianShapedActiveTorqueAngleCurve "
             << curveName
             << ": minValueAtShoulders is less than 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }


  if( (minSlopeAtShoulders < 0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createGaussianShapedActiveTorqueAngleCurve "
             << curveName
             << ": minSlopeAtShoulders is less than 0"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( (curviness < 0 || curviness > 1.0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createGaussianShapedActiveTorqueAngleCurve "
             << curveName
             << ": curviness must be in the interval [0,1]"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  double taCutoff = 1e-3;
  if(taCutoff < minValueAtShoulders ) {
    taCutoff = minValueAtShoulders;
  }
  double angularStandardDeviationSq =
    angularStandardDeviation*angularStandardDeviation;
  double thetaWidth = sqrt(-log(taCutoff)*2*angularStandardDeviationSq);
  double thetaMin   = -thetaWidth + angleAtOneNormTorque;
  double thetaMax   =  thetaWidth + angleAtOneNormTorque;

  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);

  double x0 =  thetaMin;

  double x1 =  angleAtOneNormTorque - angularStandardDeviation;
  double x2 =  angleAtOneNormTorque;
  double x3 =  angleAtOneNormTorque + angularStandardDeviation;

  if( (angleAtOneNormTorque-thetaMin) <
      (angleAtOneNormTorque-angularStandardDeviation)) {
    x1 =  angleAtOneNormTorque - 0.5*thetaMin;
    x3 =  angleAtOneNormTorque + 0.5*thetaMin;
  }

  double x4 =  thetaMax;

  double y0 = minValueAtShoulders;
  double y1 = exp(-(x1-angleAtOneNormTorque)*(x1-angleAtOneNormTorque)
                  / (2*angularStandardDeviationSq) );
  double y2 = exp(-(x2-angleAtOneNormTorque)*(x2-angleAtOneNormTorque)
                  / (2*angularStandardDeviationSq) );
  double y3 = exp(-(x3-angleAtOneNormTorque)*(x3-angleAtOneNormTorque)
                  / (2*angularStandardDeviationSq) );
  double y4 = minValueAtShoulders;


  double dydx1 = -(2*(x1-angleAtOneNormTorque)
                   / (2*angularStandardDeviationSq)) * y1;
  double dydx2 = -(2*(x2-angleAtOneNormTorque)
                   / (2*angularStandardDeviationSq)) * y2;
  double dydx3 = -(2*(x3-angleAtOneNormTorque)
                   / (2*angularStandardDeviationSq)) * y3;

  double dydx0 =   minSlopeAtShoulders;
  double dydx4 =   minSlopeAtShoulders;

  if(dydx1 < 0) {
    dydx0 *= -1.0;
  }
  if(dydx3 < 0) {
    dydx4 *= -1.0;
  }


  MatrixNd xM(6,4);
  MatrixNd yM(6,4);
  MatrixNd pts(6,2);

  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(
          x0,y0, dydx0,
          x1,y1, dydx1, c);

  xM.col(0) = pts.col(0);
  yM.col(0) = pts.col(1);

  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(
          x1,y1, dydx1,
          x2,y2, dydx2, c);

  xM.col(1) = pts.col(0);
  yM.col(1) = pts.col(1);

  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(
          x2,y2, dydx2,
          x3,y3, dydx3, c);

  xM.col(2) = pts.col(0);
  yM.col(2) = pts.col(1);

  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(
          x3,y3, dydx3,
          x4,y4, dydx4, c);

  xM.col(3) = pts.col(0);
  yM.col(3) = pts.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    xM,   yM,
    x0,   x4,
    y0,   y4,
    dydx0,dydx4,
    curveName);

}


//=============================================================================
// tendon-torque angle curve
//=============================================================================

void TorqueMuscleFunctionFactory::createTendonTorqueAngleCurve(
  double angularStretchAtOneNormTorque,
  const std::string& curveName,
  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
  smoothSegmentedFunctionToUpdate )
{

  createTendonTorqueAngleCurve(
    angularStretchAtOneNormTorque,
    1.5/angularStretchAtOneNormTorque,
    1.0/3.0,
    0.5,
    curveName,
    smoothSegmentedFunctionToUpdate );

}


void TorqueMuscleFunctionFactory::createTendonTorqueAngleCurve(
  double angularStretchAtOneNormTorque,
  double stiffnessAtOneNormTorque,
  double normTorqueAtToeEnd,
  double curviness,
  const std::string& curveName,
  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
  smoothSegmentedFunctionToUpdate )
{


  if( angularStretchAtOneNormTorque < SQRTEPS ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTendonTorqueAngleCurve "
             << curveName
             << ": angularStretchAtOneNormTorque should be greater than sqrt(eps)"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( stiffnessAtOneNormTorque < 1.1/angularStretchAtOneNormTorque) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTendonTorqueAngleCurve "
             << curveName
             << ": stiffnessAtOneNormTorque should be greater "
             << " than 1.1/angularStretchAtOneNormTorque"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( normTorqueAtToeEnd < SQRTEPS || normTorqueAtToeEnd > 0.99) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTendonTorqueAngleCurve "
             << curveName
             << ": normTorqueAtToeEnd must be in the inteval [sqrt(eps), 0.99]"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  if( (curviness < 0 || curviness > 1.0) ) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createTendonTorqueAngleCurve "
             << curveName
             << ": curviness must be in the interval [0,1]"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);
  double x0    = 0;
  double y0    = 0;
  double dydx0 = 0;

  double xIso    = angularStretchAtOneNormTorque;
  double yIso    = 1;
  double dydxIso = stiffnessAtOneNormTorque;

  //Location where the curved section becomes linear
  double yToe = normTorqueAtToeEnd;
  double xToe = (yToe-1)/stiffnessAtOneNormTorque + xIso;


  //To limit the 2nd derivative of the toe region the line it tends to
  //has to intersect the x axis to the right of the origin
  double xFoot = (xToe)/10.0;
  double yFoot = 0;


  //Compute the location of the corner formed by the average slope of the
  //toe and the slope of the linear section
  double yToeMid = yToe*0.5;
  double xToeMid = (yToeMid-yIso)/stiffnessAtOneNormTorque + xIso;
  double dydxToeMid = (yToeMid-yFoot)/(xToeMid-xFoot);

  //Compute the location of the control point to the left of the corner
  double xToeCtrl = xFoot + 0.5*(xToeMid-xFoot);
  double yToeCtrl = yFoot + dydxToeMid*(xToeCtrl-xFoot);

  MatrixNd xM(6,2);
  MatrixNd yM(6,2);
  MatrixNd pts(6,2);

  //Compute the Quintic Bezier control points
  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(x0,      y0,     dydx0,
            xToeCtrl,yToeCtrl,dydxToeMid, c);
  xM.col(0) = pts.col(0);
  yM.col(0) = pts.col(1);

  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(xToeCtrl, yToeCtrl, dydxToeMid,
            xToe,     yToe,    dydxIso, c);
  xM.col(1) = pts.col(0);
  yM.col(1) = pts.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(xM,yM,
      x0,xToe,
      y0,yToe,
      dydx0,dydxIso,
      curveName);

}


//=============================================================================
// damping blending curve
//=============================================================================

void TorqueMuscleFunctionFactory::createDampingBlendingCurve(
  double normAngularVelocityAtMaximumDamping,
  const std::string& curveName,
  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
  smoothSegmentedFunctionToUpdate)
{
  if( abs(normAngularVelocityAtMaximumDamping) < SQRTEPS) {
    ostringstream errormsg;
    errormsg << "TorqueMuscleFunctionFactory::"
             << "createDampingBlendingCurve "
             << curveName
             << ": |normAngularVelocityAtMaximumDamping| < SQRTEPS"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  double x0,x1,x2,y0,y1,y2,dydx0,dydx1,dydx2;

  if(normAngularVelocityAtMaximumDamping > 0) {
    x0 = 0.;
    x2 = normAngularVelocityAtMaximumDamping;
    y0 = 0.;
    y2 = 1.;
  } else {
    x0 = normAngularVelocityAtMaximumDamping;
    x2 = 0.;
    y0 = 1.0;
    y2 = 0.0;
  }
  x1 = 0.5*(x0+x2);
  y1 = 0.5*(y0+y2);

  dydx0 = 0.;
  dydx2 = 0.;
  dydx1 = 2.0*(y2-y0)/(x2-x0);

  double c = SegmentedQuinticBezierToolkit::scaleCurviness(0.5);

  MatrixNd xM(6,2);
  MatrixNd yM(6,2);
  MatrixNd pts(6,2);

  //Compute the Quintic Bezier control points
  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(x0,      y0,     dydx0,
            x1,      y1,     dydx1, c);
  xM.col(0) = pts.col(0);
  yM.col(0) = pts.col(1);

  pts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(x1,      y1,     dydx1,
            x2,      y2,     dydx2, c);
  xM.col(1) = pts.col(0);
  yM.col(1) = pts.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(xM,yM,
      x0,x2,
      y0,y2,
      dydx0,dydx2,
      curveName);

}

