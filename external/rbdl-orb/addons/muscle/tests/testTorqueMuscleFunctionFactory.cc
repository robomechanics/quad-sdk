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

#include "../TorqueMuscleFunctionFactory.h"
#include "../../geometry/geometry.h"
#include "../../geometry/tests/numericalTestFunctions.h"

#include <rbdl/rbdl_math.h>
#include <ctime>
#include <string>
#include <stdio.h>
#include <exception>
#include <cassert>

#include "rbdl_tests.h"

using namespace RigidBodyDynamics::Addons::Geometry;
using namespace RigidBodyDynamics::Addons::Muscle;
using namespace std;

/*
static double EPSILON = numeric_limits<double>::epsilon();

static bool FLAG_PLOT_CURVES    = false;
static string FILE_PATH         = "";
static double TOL_DX            = 5e-3;
static double TOL_DX_BIG        = 1e-2;
static double TOL_BIG           = 1e-6;
static double TOL_SMALL         = 1e-12;
*/

TEST_CASE(__FILE__"_Anderson2007ActiveTorqueAngleCurve", "")
{
    double subjectWeight    = 75.0*9.81;
    double subjectHeight    = 1.75;
    double scale      = subjectHeight*subjectWeight; 

    //These parameters are taken from table 3 for hip extension for
    //men between the ages of 18-25
    double c1      = 0.161; //normalized maximum hip joint torque
    double c2      = 0.958; // pi/(theta_max - theta_min)
    double c3      = 0.932; //theta_max_iso_torque
    double c4      = 1.578; //omega_1: angular velocity at 75% tq_iso_max
    double c5      = 3.190; //omega_2: angular velocity at 50% tq_iso_max
    double c6      = 0.242; //E, where eccentric slope = (1+E)*concentricSlope
                //Passive torque angle curve parameters
    double b1      =-1.210; // torque_passive = b1*exp(k1*theta) 
    double k1      =-6.351; //        +b2*exp(k2*theta)
    double b2      = 0.476;            
    double k2      = 5.910;



    //cout <<endl;      
    //cout <<endl;
    //cout <<"**************************************************"<<endl;
    //cout <<"ANDERSON 2007 ACTIVE TORQUE ANGLE CURVE TESTING   "<<endl;

    SmoothSegmentedFunction andersonTaCurve = SmoothSegmentedFunction();
    TorqueMuscleFunctionFactory::
     createAnderson2007ActiveTorqueAngleCurve(c2,c3,
          "test_Anderson2007TorqueAngleCurve",
          andersonTaCurve);      

    double angleRange    = (M_PI/c4); 
    double angleActiveMin   = c3 - angleRange*0.75;
    double angleActiveMax   = c3 + angleRange*0.75;

    RigidBodyDynamics::Math::MatrixNd andersonTaCurveSample 
        = andersonTaCurve.calcSampledCurve( 6,
                       angleActiveMin,
                       angleActiveMax);    

    //cout << "   Keypoint Testing" << endl;

    CHECK(abs(andersonTaCurve.calcValue(c3) - 1.0) < TOL_SMALL);  
    CHECK(abs(andersonTaCurve.calcDerivative(c3,1))     < TOL_BIG);
    CHECK(abs(andersonTaCurve.calcDerivative(c3,2))     < TOL_BIG);

    RigidBodyDynamics::Math::VectorNd curveDomain 
     = andersonTaCurve.getCurveDomain();

    CHECK(abs(andersonTaCurve.calcValue(curveDomain[0]))     < TOL_SMALL);
    CHECK(abs(andersonTaCurve.calcDerivative(curveDomain[0],1))    < TOL_BIG);
    CHECK(abs(andersonTaCurve.calcValue(curveDomain[1]))     < TOL_SMALL);
    CHECK(abs(andersonTaCurve.calcDerivative(curveDomain[1],1))    < TOL_BIG);
    //cout << "   passed " << endl;

    //cout << "    Continuity and Smoothness Testing" << endl;
    bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      andersonTaCurve,  
      andersonTaCurveSample, 
      TOL_DX);
    CHECK(areCurveDerivativesGood);

    bool curveIsContinuous = isCurveC2Continuous( andersonTaCurve, 
                                                  andersonTaCurveSample,
                                                  TOL_BIG);
    CHECK(curveIsContinuous);

    if(FLAG_PLOT_CURVES){
     andersonTaCurve.printCurveToCSVFile(
        FILE_PATH,
        "anderson2007ActiveTorqueAngleCurve",
        angleActiveMin,
        angleActiveMax);
    }


}   

TEST_CASE(__FILE__"_Anderson2007PassiveTorqueAngleCurve", "")
{
    double subjectWeight    = 75.0*9.81;
    double subjectHeight    = 1.75;
    double scale      = subjectHeight*subjectWeight; 

    //These parameters are taken from table 3 for hip extension for
    //men between the ages of 18-25
    double c1      = 0.161; //normalized maximum hip joint torque
    double c2      = 0.958; // pi/(theta_max - theta_min)
    double c3      = 0.932; //theta_max_iso_torque
    double c4      = 1.578; //omega_1: angular velocity at 75% tq_iso_max
    double c5      = 3.190; //omega_2: angular velocity at 50% tq_iso_max
    double c6      = 0.242; //E, where eccentric slope = (1+E)*concentricSlope
                //Passive torque angle curve parameters
    double b1      =-1.210; // torque_passive = b1*exp(k1*theta) 
    double k1      =-6.351; //        +b2*exp(k2*theta)
    double b2      = 0.476;            
    double k2      = 5.910;

    
    //cout <<endl;
    //cout <<endl;
    //cout <<"**************************************************"<<endl;
    //cout <<"ANDERSON 2007 PASSIVE TORQUE ANGLE CURVE TESTING   "<<endl;

    double curveSign = 1.0;



    for(int z = 0; z<2; ++z){

     if(z == 0){
      curveSign = 1.0;
      //cout <<"    TESTING SIDE 1"<<endl;
     }else{
      curveSign = -1.0;
      //cout <<"    TESTING SIDE 2"<<endl;

     }
     SmoothSegmentedFunction andersonTpCurve = SmoothSegmentedFunction();
     TorqueMuscleFunctionFactory::
       createAnderson2007PassiveTorqueAngleCurve(
            scale,
            c1,
            b1*curveSign,
            k1,
            b2*curveSign,
            k2,
            "test_passiveTorqueAngleCurve",
            andersonTpCurve);

     RigidBodyDynamics::Math::VectorNd curveDomain 
      = andersonTpCurve.getCurveDomain();

     double angleMin = curveDomain[0];
     double angleMax = curveDomain[1];

     RigidBodyDynamics::Math::MatrixNd andersonTpCurveSample
            = andersonTpCurve.calcSampledCurve( 6,
                        angleMin-0.1,
                        angleMax+0.1);

     //cout << "   Keypoint Testing" << endl;

     double tauMin = andersonTpCurve.calcValue(angleMin);
     double tauMax = andersonTpCurve.calcValue(angleMax);
     double tauMinAngle = angleMin;

     if(tauMin > tauMax){
      double tmp  = tauMin;
      tauMin   = tauMax;
      tauMax   = tmp;
      tauMinAngle = angleMax;
     }

     CHECK( abs(tauMin) < TOL_SMALL);
     CHECK( abs(tauMax - 1.0) < TOL_SMALL);
     CHECK( abs(andersonTpCurve.calcDerivative(tauMinAngle,1)) < TOL_SMALL);

     //cout << "   passed " << endl;

     //cout << "   Continuity and Smoothness Testing " << endl;
     bool areCurveDerivativesGood = 
      areCurveDerivativesCloseToNumericDerivatives( 
       andersonTpCurve,  
       andersonTpCurveSample, 
       TOL_DX);
     CHECK(areCurveDerivativesGood);

     bool curveIsContinuous = isCurveC2Continuous(andersonTpCurve,  
                                                  andersonTpCurveSample,
                                                  TOL_BIG);
     CHECK(curveIsContinuous);

     bool curveIsMonotonic = isCurveMontonic(andersonTpCurveSample);
     CHECK(curveIsMonotonic);
     //cout << "   passed " << endl;


    }

    SmoothSegmentedFunction andersonTpCurve = SmoothSegmentedFunction();
    TorqueMuscleFunctionFactory::
      createAnderson2007PassiveTorqueAngleCurve(
        scale,
        c1,
        b1,
        k1,
        b2,
        k2,
        "test_passiveTorqueAngleCurve",
        andersonTpCurve);

    if(FLAG_PLOT_CURVES){
     andersonTpCurve.printCurveToCSVFile(
        FILE_PATH,
        "anderson2007PassiveTorqueAngleCurve",
        andersonTpCurve.getCurveDomain()[0]-0.1,
        andersonTpCurve.getCurveDomain()[1]+0.1);
    }

}

TEST_CASE(__FILE__"_Anderson2007ActiveTorqueVelocityCurve", "")
{
    double subjectWeight    = 75.0*9.81;
    double subjectHeight    = 1.75;
    double scale      = subjectHeight*subjectWeight; 

    //These parameters are taken from table 3 for hip extension for
    //men between the ages of 18-25
    double c1      = 0.161; //normalized maximum hip joint torque
    double c2      = 0.958; // pi/(theta_max - theta_min)
    double c3      = 0.932; //theta_max_iso_torque
    double c4      = 1.578; //omega_1: angular velocity at 75% tq_iso_max
    double c5      = 3.190; //omega_2: angular velocity at 50% tq_iso_max
    double c6      = 0.242; //E, where eccentric slope = (1+E)*concentricSlope
                //Passive torque angle curve parameters
    double b1      =-1.210; // torque_passive = b1*exp(k1*theta) 
    double k1      =-6.351; //        +b2*exp(k2*theta)
    double b2      = 0.476;            
    double k2      = 5.910;

    //cout <<endl;
    //cout <<endl;
    //cout <<"**************************************************"<<endl;
    //cout <<"ANDERSON 2007 ACTIVE TORQUE VELOCITY CURVE TESTING"<<endl;

    double minEccentricMultiplier = 1.1;
    double maxEccentricMultiplier = 1.4;

    SmoothSegmentedFunction andersonTvCurve = SmoothSegmentedFunction();
    TorqueMuscleFunctionFactory::
    createAnderson2007ActiveTorqueVelocityCurve(
     c4,c5,c6,minEccentricMultiplier,maxEccentricMultiplier,
     "test_anderson2007ActiveTorqueVelocityCurve",
     andersonTvCurve);

    RigidBodyDynamics::Math::VectorNd curveDomain 
     = andersonTvCurve.getCurveDomain();

    double angularVelocityMin = curveDomain[0];
    double angularVelocityMax = curveDomain[1];


    RigidBodyDynamics::Math::MatrixNd andersonTvCurveSample
        = andersonTvCurve.calcSampledCurve( 6,
                       angularVelocityMin-0.1,
                       angularVelocityMax+0.1);

    //cout << "   Keypoint Testing" << endl;

    CHECK(abs(andersonTvCurve.calcValue(0) - 1.0)        < TOL_SMALL);
    //CHECK(abs(andersonTvCurve.calcValue(c4) - 0.75)    < TOL_BIG);
    //CHECK(abs(andersonTvCurve.calcValue(c5) - 0.5)     < TOL_BIG);
    double val = abs(andersonTvCurve.calcValue(
                     angularVelocityMax/angularVelocityMax));
    CHECK(val   < TOL_BIG);

    double maxTv = andersonTvCurve.calcValue(
          angularVelocityMin/angularVelocityMax);
    CHECK(maxTv >= minEccentricMultiplier);
    CHECK(maxTv <= maxEccentricMultiplier);

    CHECK(abs(andersonTvCurve.calcDerivative
              (angularVelocityMax/angularVelocityMax,1))<TOL_SMALL);
    CHECK(abs(andersonTvCurve.calcDerivative
              (angularVelocityMin/angularVelocityMax,1))<TOL_SMALL);

    //cout << "   passed " << endl;
    //cout << "    Continuity and Smoothness Testing" << endl;

    bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      andersonTvCurve,  
      andersonTvCurveSample, 
      TOL_DX);

    CHECK(areCurveDerivativesGood);

    bool curveIsContinuous = isCurveC2Continuous( andersonTvCurve,
                                                  andersonTvCurveSample,
                                                  TOL_BIG);
    CHECK(curveIsContinuous);

    bool curveIsMonotonic = isCurveMontonic(andersonTvCurveSample);
    CHECK(curveIsMonotonic);


    if(FLAG_PLOT_CURVES){
     andersonTvCurve.printCurveToCSVFile(
        FILE_PATH,
        "anderson2007ActiveTorqueVelocityCurve",
        angularVelocityMin,
        angularVelocityMax);
    }

}
//==============================================================================
TEST_CASE(__FILE__"_TorqueAngularVelocityCurve", "")
{
    SmoothSegmentedFunction tv = SmoothSegmentedFunction();
    SmoothSegmentedFunction tvX = SmoothSegmentedFunction();
    double tvAtEccentricOmegaMax       =  1.3;
    double tvAtHalfConcentricOmegaMax  = 0.3;
    std::string curveName0("tvTest0");
    std::string curveName1("tvTest1");

    TorqueMuscleFunctionFactory::createTorqueVelocityCurve(
                                    tvAtEccentricOmegaMax,
                                    tvAtHalfConcentricOmegaMax,
                                    curveName0,
                                    tv);
    double slopeAtConcentricOmegaMax = -0.1;
    double slopeAtEccentricOmegaMax = -0.1;
    double slopeNearEccentricOmegaMax = -0.01;
    double eccentricCurviness = 0.75;
    TorqueMuscleFunctionFactory::createTorqueVelocityCurve(
                                    tvAtEccentricOmegaMax,
                                    tvAtHalfConcentricOmegaMax,
                                    slopeAtConcentricOmegaMax,
                                    slopeNearEccentricOmegaMax,
                                    slopeAtEccentricOmegaMax,
                                    eccentricCurviness,
                                    curveName1,
                                    tvX);

    double wmin = -1.1;
    double wmax =  1.1;
    int    npts =  100;
    double delta = (wmax-wmin)/((double)npts-1.0);


    //Get the parameters for the Hill type curve
    double wmaxC =  1.0;
    double wmaxE = -1.0;
    double fiso  = 1.0;
    double w     = 0.5*wmaxC;
    double a = -tvAtHalfConcentricOmegaMax*w*fiso
                / (wmaxC*tvAtHalfConcentricOmegaMax
                   - fiso*wmaxC + fiso*w);
    double b =  a*wmaxC/fiso;
    double tvHill = 0;
    double errHill = 0;
    for(int i=0; i<npts; ++i){
      w = wmin + i*delta;
      if(w > wmaxC){
        CHECK( abs( tv.calcValue(w) ) < TOL_SMALL);
        CHECK( abs( tv.calcDerivative(w,1) ) < TOL_SMALL);
        CHECK( abs(tvX.calcDerivative(w,1)
                   - slopeAtConcentricOmegaMax ) < TOL_BIG);
      }else if(w > 0 && w <= wmaxC){
        tvHill = (b*fiso-a*w)/(b+w);
        errHill = abs(tv.calcValue(w)-tvHill);
        //printf("%i. Err %f, ",i,errHill);
        CHECK( errHill < 0.02);
        errHill = abs(tvX.calcValue(w)-tvHill);
        //printf(" Err %f\n",errHill);
        CHECK( errHill < 0.02);
      }else if(w < 0 & w > wmaxE){
        CHECK(tv.calcValue(w) > 1.0);
      }else if(w < wmaxE){
        CHECK(abs( tv.calcValue(w)        - tvAtEccentricOmegaMax ) < TOL_SMALL);
        CHECK(abs( tv.calcDerivative(w,1) - 0.0 ) <  TOL_SMALL);
        //CHECK(abs( tvX.calcValue(w) - tvAtEccentricOmegaMax ) );
        CHECK(abs( tvX.calcDerivative(w,1)
                   - slopeAtEccentricOmegaMax ) < TOL_SMALL );
      }
    }

    RigidBodyDynamics::Math::VectorNd curveDomain = tv.getCurveDomain();

    double angularVelocityMin = curveDomain[0];
    double angularVelocityMax = curveDomain[1];


    RigidBodyDynamics::Math::MatrixNd tvCurveSample
        = tv.calcSampledCurve( 6,
              angularVelocityMin-0.1,
              angularVelocityMax+0.1);

    bool areCurveDerivativesGood =
     areCurveDerivativesCloseToNumericDerivatives(
      tv,
      tvCurveSample,
      TOL_DX);

    CHECK(areCurveDerivativesGood);

    bool curveIsContinuous = isCurveC2Continuous( tv,
                                                  tvCurveSample,
                                                  TOL_BIG);
    CHECK(curveIsContinuous);

    bool curveIsMonotonic = isCurveMontonic(tvCurveSample);
    CHECK(curveIsMonotonic);

    if(FLAG_PLOT_CURVES){
        tv.printCurveToCSVFile(
        FILE_PATH,
        "millard2016TorqueVelocityCurve",
        -1.1,
         1.1);
    }

}

//==============================================================================
TEST_CASE(__FILE__"_PassiveTorqueAngleCurve", "")
{
  SmoothSegmentedFunction tp     = SmoothSegmentedFunction();
  SmoothSegmentedFunction tpX    = SmoothSegmentedFunction();
  double angleAtZeroTorque0      = 0;
  double angleAtOneNormTorque0   = -M_PI;

  double angleAtZeroTorque1       = 0;
  double angleAtOneNormTorque1    = M_PI;

  double stiffnessAtOneNormTorque1 =
        5.6/(angleAtOneNormTorque1-angleAtZeroTorque1);
  double stiffnessAtLowTorque1     =
        0.05*stiffnessAtOneNormTorque1;
  double curviness1 = 0.75;

  std::string curveName0("tpTest0");
  std::string curveName1("tpTest1");


  TorqueMuscleFunctionFactory::createPassiveTorqueAngleCurve(
                                  angleAtZeroTorque0,
                                  angleAtOneNormTorque0,
                                  curveName0,
                                  tp);

  TorqueMuscleFunctionFactory::createPassiveTorqueAngleCurve(
                                angleAtZeroTorque1,
                                angleAtOneNormTorque1,
                                stiffnessAtLowTorque1,
                                stiffnessAtOneNormTorque1,
                                curviness1,
                                curveName1,
                                tpX);

  CHECK( abs(tp.calcValue(angleAtZeroTorque0))        < TOL_SMALL);
  CHECK( abs(tp.calcValue(angleAtOneNormTorque0)-1.0) < TOL_SMALL);

  CHECK( abs(tpX.calcValue(angleAtZeroTorque1))          < TOL_SMALL);
  CHECK( abs(tpX.calcValue(angleAtOneNormTorque1) - 1.0) < TOL_SMALL);
  CHECK( abs(tpX.calcDerivative(angleAtZeroTorque1,1)) < TOL_SMALL);
  CHECK( abs(tpX.calcDerivative(angleAtOneNormTorque1,1)
             -stiffnessAtOneNormTorque1) < TOL_SMALL);

  RigidBodyDynamics::Math::VectorNd curveDomain0 = tp.getCurveDomain();
  RigidBodyDynamics::Math::VectorNd curveDomain1 = tpX.getCurveDomain();

  RigidBodyDynamics::Math::MatrixNd tpSample0
      = tp.calcSampledCurve( 6,
            curveDomain0[0]-0.1,
            curveDomain0[1]+0.1);

  RigidBodyDynamics::Math::MatrixNd tpSample1
      = tpX.calcSampledCurve( 6,
            curveDomain1[0]-0.1,
            curveDomain1[1]+0.1);

  bool areCurveDerivativesGood =
   areCurveDerivativesCloseToNumericDerivatives(
    tp,
    tpSample0,
    TOL_DX);

  CHECK(areCurveDerivativesGood);

  areCurveDerivativesGood =
     areCurveDerivativesCloseToNumericDerivatives(
      tpX,
      tpSample1,
      TOL_DX);

  CHECK(areCurveDerivativesGood);
  bool curveIsContinuous = isCurveC2Continuous( tp,
                                                tpSample0,
                                                TOL_BIG);
  CHECK(curveIsContinuous);
  curveIsContinuous = isCurveC2Continuous(tpX,
                                          tpSample1,
                                          TOL_BIG);
  CHECK(curveIsContinuous);

  bool curveIsMonotonic = isCurveMontonic(tpSample0);
  CHECK(curveIsMonotonic);

  curveIsMonotonic = isCurveMontonic(tpSample1);
  CHECK(curveIsMonotonic);


  if(FLAG_PLOT_CURVES){
      tp.printCurveToCSVFile(
      FILE_PATH,
      "millard2016PassiveTorqueAngleCurve",
      curveDomain0[0]-0.1,
      curveDomain0[1]+0.1);
  }
}

//==============================================================================
TEST_CASE(__FILE__"_ActiveTorqueAngleCurve", "")
{
  SmoothSegmentedFunction ta     = SmoothSegmentedFunction();
  SmoothSegmentedFunction taX    = SmoothSegmentedFunction();
  double angleAtOneNormTorque   = 1.5;
  double angularStandardDeviation = 1.0;
  double angularStandardDeviationSq =
      angularStandardDeviation*angularStandardDeviation;

  double minSlopeAtShoulders = 0.2;
  double minValueAtShoulders = 0.1;
  double xTrans = sqrt(-log(minValueAtShoulders)*2*angularStandardDeviationSq);
  double delta = abs(xTrans+angleAtOneNormTorque);
  double curviness = 0.75;

  std::string curveName0("tpTest0");
  std::string curveName1("tpTest1");


  TorqueMuscleFunctionFactory::createGaussianShapedActiveTorqueAngleCurve(
                                  angleAtOneNormTorque,
                                  angularStandardDeviation,
                                  curveName0,
                                  ta);

  TorqueMuscleFunctionFactory::createGaussianShapedActiveTorqueAngleCurve(
                                angleAtOneNormTorque,
                                angularStandardDeviation,
                                minSlopeAtShoulders,
                                minValueAtShoulders,
                                curviness,
                                curveName1,
                                taX);

  CHECK(abs(ta.calcValue(angleAtOneNormTorque)-1.0) < TOL_SMALL);

  CHECK(abs(ta.calcValue(angleAtOneNormTorque
                         +10.0*angularStandardDeviation)) < TOL_SMALL);
  CHECK(abs(ta.calcValue(angleAtOneNormTorque
                         -10.0*angularStandardDeviation)) < TOL_SMALL);

  CHECK(abs(taX.calcValue(angleAtOneNormTorque)-1.0) < TOL_SMALL);

  double err = abs(taX.calcDerivative(angleAtOneNormTorque + delta,1)
                   + minSlopeAtShoulders);
  CHECK(err < TOL_SMALL);
  err = abs(taX.calcDerivative(angleAtOneNormTorque - delta,1)
            - minSlopeAtShoulders);
  CHECK(err < TOL_SMALL);


  RigidBodyDynamics::Math::VectorNd curveDomain0 = ta.getCurveDomain();
  RigidBodyDynamics::Math::VectorNd curveDomain1 = taX.getCurveDomain();

  RigidBodyDynamics::Math::MatrixNd taSample0
      = ta.calcSampledCurve( 6,
            curveDomain0[0]-0.1,
            curveDomain0[1]+0.1);

  RigidBodyDynamics::Math::MatrixNd taSample1
      = taX.calcSampledCurve( 6,
            curveDomain1[0]-0.1,
            curveDomain1[1]+0.1);

  bool areCurveDerivativesGood =
   areCurveDerivativesCloseToNumericDerivatives(
      ta,
      taSample0,
      TOL_DX);


  CHECK(areCurveDerivativesGood);

  areCurveDerivativesGood =
     areCurveDerivativesCloseToNumericDerivatives(
      taX,
      taSample1,
      TOL_DX);

  CHECK(areCurveDerivativesGood);
  bool curveIsContinuous = isCurveC2Continuous( ta,
                                                taSample0,
                                                TOL_BIG);
  CHECK(curveIsContinuous);
  curveIsContinuous = isCurveC2Continuous(taX,
                                          taSample1,
                                          TOL_BIG);
  CHECK(curveIsContinuous);

  if(FLAG_PLOT_CURVES){
      ta.printCurveToCSVFile(
      FILE_PATH,
      "millard2016ActiveTorqueAngleCurve",
      curveDomain0[0]-0.1,
      curveDomain0[1]+0.1);
  }

}

//==============================================================================
TEST_CASE(__FILE__"_TendonTorqueAngleCurve", "")
{
  SmoothSegmentedFunction tt     = SmoothSegmentedFunction();
  SmoothSegmentedFunction ttX    = SmoothSegmentedFunction();

  double angularStretchAtOneNormTorque = M_PI/2.0;
  double stiffnessAtOneNormTorque = 2.5/angularStretchAtOneNormTorque;
  double normTorqueAtToeEnd = 2.0/3.0;
  double curviness = 0.5;

  std::string curveName0("ttTest0");
  std::string curveName1("ttTest1");


  TorqueMuscleFunctionFactory::createTendonTorqueAngleCurve(
                                  angularStretchAtOneNormTorque,
                                  curveName0,
                                  tt);

  TorqueMuscleFunctionFactory::createTendonTorqueAngleCurve(
                                  angularStretchAtOneNormTorque,
                                  stiffnessAtOneNormTorque,
                                  normTorqueAtToeEnd,
                                  curviness,
                                  curveName1,
                                  ttX);

  CHECK(abs(tt.calcValue(0)) < TOL_SMALL);
  CHECK(abs(ttX.calcValue(0)) < TOL_SMALL);

  CHECK(abs(tt.calcValue(angularStretchAtOneNormTorque)-1.0) < TOL_SMALL);
  CHECK(abs(ttX.calcValue(angularStretchAtOneNormTorque)-1.0) < TOL_SMALL);

  CHECK(abs(ttX.calcDerivative(angularStretchAtOneNormTorque,1)
               - stiffnessAtOneNormTorque) < TOL_SMALL);

  RigidBodyDynamics::Math::VectorNd curveDomain0 = tt.getCurveDomain();
  RigidBodyDynamics::Math::VectorNd curveDomain1 = ttX.getCurveDomain();

  RigidBodyDynamics::Math::MatrixNd ttSample0
      = tt.calcSampledCurve( 6,
            curveDomain0[0]-0.1,
            curveDomain0[1]+0.1);

  RigidBodyDynamics::Math::MatrixNd ttSample1
      = ttX.calcSampledCurve( 6,
            curveDomain1[0]-0.1,
            curveDomain1[1]+0.1);

  bool areCurveDerivativesGood =
   areCurveDerivativesCloseToNumericDerivatives(
    tt,
    ttSample0,
    TOL_DX);

  CHECK(areCurveDerivativesGood);

  areCurveDerivativesGood =
     areCurveDerivativesCloseToNumericDerivatives(
      ttX,
      ttSample1,
      TOL_DX);

  CHECK(areCurveDerivativesGood);
  bool curveIsContinuous = isCurveC2Continuous( tt,
                                                ttSample0,
                                                TOL_BIG);
  CHECK(curveIsContinuous);
  curveIsContinuous = isCurveC2Continuous(ttX,
                                          ttSample1,
                                          TOL_BIG);
  CHECK(curveIsContinuous);

  bool curveIsMonotonic = isCurveMontonic(ttSample0);
  CHECK(curveIsMonotonic);

  curveIsMonotonic = isCurveMontonic(ttSample1);
  CHECK(curveIsMonotonic);


  if(FLAG_PLOT_CURVES){
      tt.printCurveToCSVFile(
      FILE_PATH,
      "millard2016ActiveTorqueAngleCurve",
      curveDomain0[0]-0.1,
      angularStretchAtOneNormTorque);
  }

}

//==============================================================================
TEST_CASE(__FILE__"_DampingBlendingCurve", "")
{
  SmoothSegmentedFunction negOne     = SmoothSegmentedFunction();
  SmoothSegmentedFunction posOne     = SmoothSegmentedFunction();

  std::string curveName0("neg1");
  std::string curveName1("pos1");


  TorqueMuscleFunctionFactory::createDampingBlendingCurve(
                                -1.0,curveName0,negOne);

  TorqueMuscleFunctionFactory::createDampingBlendingCurve(
                                 1.0,curveName0,posOne);


  CHECK(abs(negOne.calcValue(0)) < TOL_SMALL);
  CHECK(abs(posOne.calcValue(0)) < TOL_SMALL);

  CHECK(abs(negOne.calcValue(-1.0)-1.0) < TOL_SMALL);
  CHECK(abs(posOne.calcValue(1.0)-1.0) < TOL_SMALL);

  CHECK(abs(negOne.calcDerivative( 0,1)) < TOL_SMALL);
  CHECK(abs(negOne.calcDerivative(-1,1)) < TOL_SMALL);

  CHECK(abs(posOne.calcDerivative( 0,1)) < TOL_SMALL);
  CHECK(abs(posOne.calcDerivative( 1,1)) < TOL_SMALL);

  RigidBodyDynamics::Math::VectorNd curveDomainNegOne =  negOne.getCurveDomain();
  RigidBodyDynamics::Math::VectorNd curveDomainPosOne = posOne.getCurveDomain();

  RigidBodyDynamics::Math::MatrixNd sampleNegOne
      = negOne.calcSampledCurve( 6,
            curveDomainNegOne[0]-0.1,
            curveDomainNegOne[1]+0.1);

  RigidBodyDynamics::Math::MatrixNd samplePosOne
      = posOne.calcSampledCurve( 6,
            curveDomainPosOne[0]-0.1,
            curveDomainPosOne[1]+0.1);

  bool areCurveDerivativesGood =
   areCurveDerivativesCloseToNumericDerivatives(
    negOne,
    sampleNegOne,
    TOL_DX);

  CHECK(areCurveDerivativesGood);

  areCurveDerivativesGood =
     areCurveDerivativesCloseToNumericDerivatives(
      posOne,
      samplePosOne,
      TOL_DX);

  CHECK(areCurveDerivativesGood);
  bool curveIsContinuous = isCurveC2Continuous( negOne,
                                                sampleNegOne,
                                                TOL_BIG);
  CHECK(curveIsContinuous);
  curveIsContinuous = isCurveC2Continuous(posOne,
                                          samplePosOne,
                                          TOL_BIG);
  CHECK(curveIsContinuous);

  bool curveIsMonotonic = isCurveMontonic(sampleNegOne);
  CHECK(curveIsMonotonic);

  curveIsMonotonic = isCurveMontonic(samplePosOne);
  CHECK(curveIsMonotonic);


  if(FLAG_PLOT_CURVES){
      negOne.printCurveToCSVFile(
      FILE_PATH,
      "millard2016DampingBlendingCurve",
      curveDomainNegOne[0]-0.1,
      curveDomainNegOne[1]+0.1);
  }

}
