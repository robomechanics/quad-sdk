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
Below is a basic bench mark simulation for the MuscleFunctionFactory
class, a class that enables the easy generation of C2 continuous curves 
that define the various characteristic curves required in a muscle model
 */

// Author:  Matthew Millard

//==============================================================================
// INCLUDES
//==============================================================================

#include "../../geometry/geometry.h"
#include "../MuscleFunctionFactory.h"
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
//using namespace OpenSim;
//using namespace SimTK;

/*
static double EPSILON = numeric_limits<double>::epsilon();

static bool FLAG_PLOT_CURVES    = false;
static string FILE_PATH      = "";
static double TOL_DX      = 5e-3;
static double TOL_DX_BIG     = 1e-2;
static double TOL_BIG     = 1e-6;
static double TOL_SMALL      = 1e-12;
*/



TEST_CASE(__FILE__"_tendonCurve", "")
{
    //cout <<"**************************************************"<<endl;
    //cout <<"TENDON CURVE TESTING            "<<endl;
    double e0   = 0.04;
    double kiso = 1.5/e0;
    double c    = 0.5;//0.75;    
    double ftoe = 1.0/3.0;

    SmoothSegmentedFunction tendonCurve = SmoothSegmentedFunction();
    MuscleFunctionFactory::createTendonForceLengthCurve(
         e0,kiso,ftoe,c, "test_tendonCurve", tendonCurve);
    

    RigidBodyDynamics::Math::MatrixNd tendonCurveSample
     =tendonCurve.calcSampledCurve(6,1.0,1+e0);
    //tendonCurve.printMuscleCurveToCSVFile(FILE_PATH);

//0. Test that each curve fulfills its contract at the end points.
    //cout << "   Keypoint Testing" << endl;
    RigidBodyDynamics::Math::VectorNd tendonCurveDomain = 
     tendonCurve.getCurveDomain();
    CHECK(abs(tendonCurve.calcValue(tendonCurveDomain[0]))<TOL_SMALL);
    CHECK(abs(tendonCurve.calcValue(tendonCurveDomain[1])-ftoe)<TOL_SMALL);

    CHECK(abs(tendonCurve.calcValue(1.0)    )< TOL_SMALL);  
    CHECK(abs(tendonCurve.calcDerivative(1.0,1))< TOL_BIG);
    CHECK(abs(tendonCurve.calcDerivative(1.0,2))< TOL_BIG);

    CHECK(abs(tendonCurve.calcValue(1+e0)    -1.0 ) < TOL_SMALL);  
    CHECK(abs(tendonCurve.calcDerivative(1+e0,1)-kiso) < TOL_BIG);
    CHECK(abs(tendonCurve.calcDerivative(1+e0,2)-0   ) < TOL_BIG);
    //cout << "   passed" << endl;
    //cout << endl;
//1. Test each derivative sample for correctness against a numerically
//   computed version
    bool areCurveDerivativesGood = 
      areCurveDerivativesCloseToNumericDerivatives(
       tendonCurve,
       tendonCurveSample,TOL_DX_BIG);

    CHECK(areCurveDerivativesGood);
//2. Test each integral, where computed for correctness.
    //testMuscleCurveIntegral(tendonCurve, tendonCurveSample);

//3. Test numerically to see if the curve is C2 continuous
    bool curveIsContinuous = isCurveC2Continuous(tendonCurve,
                                                 tendonCurveSample,
                                                 TOL_BIG);

    CHECK(curveIsContinuous);
//4. Test for montonicity where appropriate
    bool curveIsMonotonic = isCurveMontonic(tendonCurveSample);
    CHECK(curveIsMonotonic);

    if(FLAG_PLOT_CURVES){
     tendonCurve.printCurveToCSVFile(FILE_PATH,
                 "tendonCurve",
                 1.0-(e0/10),
                 1+e0);
    }
    //cout << "    passed" << endl;

}

TEST_CASE(__FILE__"_activeForceLengthCurve", "")
{
    //cout << endl;
    //cout << endl;
    //cout <<"**************************************************"<<endl;
    //cout <<"FIBER ACTIVE FORCE LENGTH CURVE TESTING     "<<endl;
    double lce0 = 0.4;
    double lce1 = 0.75;
    double lce2 = 1;
    double lce3 = 1.6;
    double shoulderVal  = 0.05;
    double plateauSlope = 0.75;//0.75;
    double curviness    = 0.75;
    SmoothSegmentedFunction fiberfalCurve = SmoothSegmentedFunction();

    MuscleFunctionFactory::
     createFiberActiveForceLengthCurve(lce0, lce1, lce2, lce3, 
          shoulderVal, plateauSlope, curviness,
          "test_fiberActiveForceLengthCurve", fiberfalCurve);
    //fiberfalCurve.printMuscleCurveToCSVFile(FILE_PATH);

    RigidBodyDynamics::Math::MatrixNd fiberfalCurveSample 
        = fiberfalCurve.calcSampledCurve(6,0,lce3);

    //0. Test that each curve fulfills its contract.
    //cout << "   Keypoint Testing" << endl;

    CHECK(abs(fiberfalCurve.calcValue(lce0) - shoulderVal) < TOL_SMALL);  
    CHECK(abs(fiberfalCurve.calcDerivative(lce0,1))     < TOL_BIG);
    CHECK(abs(fiberfalCurve.calcDerivative(lce0,2))     < TOL_BIG);
   
    //lce2 isn't the location of the end of a quintic Bezier curve
    //so I can't actually do any testing on this point.
    //SimTK_TEST_EQ_TOL(fiberfalCurve.calcValue(lce0),shoulderVal,TOL_SMALL);  
    //SimTK_TEST_EQ_TOL(
    //fiberfalCurve.calcDerivative(lce2,1),plateauSlope,TOL_BIG);
    //SimTK_TEST_EQ_TOL(fiberfalCurve.calcDerivative(lce2,2),0.0,TOL_BIG);

    CHECK(abs(fiberfalCurve.calcValue(lce2) - 1.0)    <  TOL_SMALL);  
    CHECK(abs(fiberfalCurve.calcDerivative(lce2,1))   <  TOL_BIG);
    CHECK(abs(fiberfalCurve.calcDerivative(lce2,2))   <  TOL_BIG);
    CHECK(abs(fiberfalCurve.calcValue(lce3)-shoulderVal) <  TOL_SMALL);
    CHECK(abs(fiberfalCurve.calcDerivative(lce3,1))   <  TOL_BIG);
    CHECK(abs(fiberfalCurve.calcDerivative(lce3,2))   <  TOL_BIG);

    //cout << "   passed" << endl;
    //cout << endl;
//1. Test each derivative sample for correctness against a numerically
//   computed version
    bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      fiberfalCurve,
      fiberfalCurveSample,
      TOL_DX);
    CHECK(areCurveDerivativesGood);
//2. Test each integral, where computed for correctness.
    //testMuscleCurveIntegral(fiberfalCurve,fiberfalCurveSample);

//3. Test numerically to see if the curve is C2 continuous
    bool curveIsContinuous = isCurveC2Continuous(fiberfalCurve,
                                                 fiberfalCurveSample,
                                                 TOL_BIG);
    CHECK(curveIsContinuous);

    //fiberfalCurve.MuscleCurveToCSVFile("C:/mjhmilla/Stanford/dev");

    if(FLAG_PLOT_CURVES){
     fiberfalCurve.printCurveToCSVFile(FILE_PATH,
                  "fiberFalCurve",
                  0.3,
                  1.8);
    }


}


TEST_CASE(__FILE__"_ForceVelocityCurve", "")
{
    //cout <<"**************************************************"<<endl;
    //cout <<"FIBER FORCE VELOCITY CURVE TESTING       "<<endl;

    double fmaxE     = 1.8;
    double dydxC     = 0.1;
    double dydxNearC    = 0.15;
    double dydxE     = 0.1;
    double dydxNearE    = 0.1+0.0001;
    double dydxIso   = 5;
    double concCurviness= 0.1;
    double eccCurviness = 0.75;

    SmoothSegmentedFunction fiberFVCurve = SmoothSegmentedFunction();
    MuscleFunctionFactory::
     createFiberForceVelocityCurve(fmaxE, 
            dydxC,
            dydxNearC,
            dydxIso,
            dydxE,
            dydxNearE,
            concCurviness,
            eccCurviness,
            "test_fiberForceVelocityCurve",
            fiberFVCurve);
    //fiberFVCurve.printMuscleCurveToCSVFile(FILE_PATH);

    RigidBodyDynamics::Math::MatrixNd fiberFVCurveSample 
        = fiberFVCurve.calcSampledCurve(6,-1.0,1.0);

//0. Test that each curve fulfills its contract.
    //cout << "   Keypoint Testing" << endl;

    assert(abs(fiberFVCurve.calcValue(-1)      ) < TOL_SMALL);  
    assert(abs(fiberFVCurve.calcDerivative(-1,1)-dydxC  ) < TOL_BIG  );
    assert(abs(fiberFVCurve.calcDerivative(-1,2)     ) < TOL_BIG  );
    assert(abs(fiberFVCurve.calcValue(0)    -1.0  ) < TOL_SMALL);  
    assert(abs(fiberFVCurve.calcDerivative(0,1)-dydxIso ) < TOL_BIG  );
    assert(abs(fiberFVCurve.calcDerivative(0,2)      ) < TOL_BIG  );
    assert(abs(fiberFVCurve.calcValue(1)    -fmaxE   ) < TOL_SMALL);  
    assert(abs(fiberFVCurve.calcDerivative(1,1)-dydxE   ) < TOL_BIG  );
    assert(abs(fiberFVCurve.calcDerivative(1,2)      ) < TOL_BIG  );

    //cout << "   passed" << endl;
    //cout << endl;
//1. Test each derivative sample for correctness against a numerically
//   computed version
    bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      fiberFVCurve,
      fiberFVCurveSample,
      TOL_DX);
    CHECK(areCurveDerivativesGood);

//2. Test each integral, where computed for correctness.
    //testMuscleCurveIntegral(fiberFVCurve,fiberFVCurveSample);

//3. Test numerically to see if the curve is C2 continuous
    bool curveIsContinuous = isCurveC2Continuous(
                                fiberFVCurve,
                                fiberFVCurveSample,
                                TOL_BIG);
    CHECK(curveIsContinuous);
//4. Test for montonicity where appropriate

    isCurveMontonic(fiberFVCurveSample);
    CHECK(curveIsContinuous);

    if(FLAG_PLOT_CURVES){
     fiberFVCurve.printCurveToCSVFile(FILE_PATH,
                 "fiberFvCurve",
                 -1.1,
                  1.1);
    }

    //cout << "    passed" << endl;

}

TEST_CASE(__FILE__"_ForceVelocityInverseCurve", "")
{
    //cout <<"**************************************************"<<endl;
    //cout <<"FIBER FORCE VELOCITY INVERSE CURVE TESTING     "<<endl;
    double fmaxE     = 1.8;
    double dydxC     = 0.1;
    double dydxNearC    = 0.15;
    double dydxE     = 0.1;
    double dydxNearE    = 0.1+0.0001;
    double dydxIso   = 5;
    double concCurviness= 0.1;
    double eccCurviness = 0.75;

    SmoothSegmentedFunction fiberFVInvCurve = SmoothSegmentedFunction();    
    MuscleFunctionFactory::
      createFiberForceVelocityInverseCurve( 
        fmaxE, 
        dydxC, 
        dydxNearC, 
        dydxIso, 
        dydxE, 
        dydxNearE,
        concCurviness,  
        eccCurviness,
        "test_fiberForceVelocityInverseCurve",
        fiberFVInvCurve);

    SmoothSegmentedFunction fiberFVCurve = SmoothSegmentedFunction();
    MuscleFunctionFactory::
     createFiberForceVelocityCurve(
      fmaxE, 
      dydxC,
      dydxNearC,
      dydxIso,
      dydxE,
      dydxNearE,
      concCurviness,
      eccCurviness,
      "test_fiberForceVelocityCurve",
      fiberFVCurve);    
    //fiberFVInvCurve.printMuscleCurveToFile(FILE_PATH);
    

    RigidBodyDynamics::Math::MatrixNd fiberFVCurveSample
     = fiberFVCurve.calcSampledCurve(6, -1.0, 1.0);

    RigidBodyDynamics::Math::MatrixNd fiberFVInvCurveSample 
        = fiberFVInvCurve.calcSampledCurve(6,0,fmaxE);

//0. Test that each curve fulfills its contract.
    //cout << "   Keypoint Testing" << endl;

    CHECK(abs( fiberFVInvCurve.calcValue(0)    +1    ) < TOL_SMALL);
    CHECK(abs( fiberFVInvCurve.calcDerivative(0,1)-1/dydxC    ) < TOL_BIG);
    CHECK(abs( fiberFVInvCurve.calcDerivative(0,2)      ) < TOL_BIG);               
    CHECK(abs( fiberFVInvCurve.calcValue(1)          ) < TOL_SMALL);  
    CHECK(abs( fiberFVInvCurve.calcDerivative(1,1)-1/dydxIso  ) < TOL_BIG);
    CHECK(abs( fiberFVInvCurve.calcDerivative(1,2)      ) < TOL_BIG);                
    CHECK(abs( fiberFVInvCurve.calcValue(fmaxE)   -1    ) < TOL_SMALL);  
    CHECK(abs( fiberFVInvCurve.calcDerivative(fmaxE,1)-1/dydxE) < TOL_BIG);
    CHECK(abs( fiberFVInvCurve.calcDerivative(fmaxE,2)     ) <  TOL_BIG);

    //cout << "   passed" << endl;
    //cout << endl;
//1. Test each derivative sample for correctness against a numerically
//   computed version
    bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      fiberFVInvCurve,
      fiberFVInvCurveSample,
      TOL_DX);
    CHECK(areCurveDerivativesGood);

//2. Test each integral, where computed for correctness.
    //testMuscleCurveIntegral(fiberFVInvCurve,fiberFVInvCurveSample);

//3. Test numerically to see if the curve is C2 continuous
    bool curveIsContinuous = isCurveC2Continuous(
                                fiberFVInvCurve,
                                fiberFVInvCurveSample,
                                TOL_BIG);
    CHECK(curveIsContinuous);
//4. Test for montonicity where appropriate

    bool curveIsMonotonic = isCurveMontonic(fiberFVInvCurveSample);
    CHECK(curveIsMonotonic);

//5. Testing the inverse of the curve - is it really an inverse?
    //cout << endl;
    //cout << "   TEST: Inverse correctness:fv(fvinv(fv)) = fv" << endl;
    double fv = 0;
    double dlce = 0;
    double fvCalc = 0;
    double fvErr = 0;
    double fvErrMax = 0;
    for(int i = 0; i < fiberFVInvCurveSample.rows(); i++){
     fv   = fiberFVCurveSample(i,0);
     dlce    = fiberFVInvCurve.calcValue(fv);
     fvCalc  = fiberFVCurve.calcValue(dlce);
     fvErr   = abs(fv-fvCalc);
     if(fvErrMax < fvErr)
      fvErrMax = fvErr;

     CHECK( fvErr < TOL_BIG);
    }

    if(FLAG_PLOT_CURVES){
     fiberFVInvCurve.printCurveToCSVFile(FILE_PATH,
                  "fiberFvInvCurve",
                  -0.1,
                  fmaxE+0.1);
    }

    //printf("   passed with a maximum error of %fe-12",fvErrMax*1e12);

}

TEST_CASE(__FILE__"_passiveForceLengthCurve", "")
{
    double e0f   = 0.6;
    double kisof    = 8.389863790885878;
    double cf    = 0.65;
    double klow  = 0.5*(1.0/e0f);

    SmoothSegmentedFunction fiberFLCurve = SmoothSegmentedFunction();
    MuscleFunctionFactory::
             createFiberForceLengthCurve(
              0.0, e0f,klow,kisof,cf,
             "test_fiberForceLength",
             fiberFLCurve);

    RigidBodyDynamics::Math::MatrixNd fiberFLCurveSample 
        = fiberFLCurve.calcSampledCurve(6,1.0,1.0+e0f);

//0. Test that each curve fulfills its contract.
    //cout << "   Keypoint Testing" << endl;

    RigidBodyDynamics::Math::VectorNd fiberFLCurveDomain 
    = fiberFLCurve.getCurveDomain();

    CHECK(abs(fiberFLCurveDomain[0] -1)     < TOL_SMALL);
    CHECK(abs(fiberFLCurveDomain[1] - (1+e0f)) < TOL_SMALL);
    CHECK(abs(fiberFLCurve.calcValue(1.0)  )   < TOL_SMALL);  
    CHECK(abs(fiberFLCurve.calcDerivative(1.0,1)) < TOL_BIG);
    CHECK(abs(fiberFLCurve.calcDerivative(1.0,2)) < TOL_BIG);
    CHECK(abs(fiberFLCurve.calcValue(1+e0f) -1.0)      < TOL_SMALL);  
    CHECK(abs(fiberFLCurve.calcDerivative(1+e0f,1)-kisof) < TOL_BIG);
    CHECK(abs(fiberFLCurve.calcDerivative(1+e0f,2))    < TOL_BIG);
    //cout << "   passed" << endl;
    //cout << endl;
//1. Test each derivative sample for correctness against a numerically
//   computed version
    bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      fiberFLCurve,
      fiberFLCurveSample,
      TOL_DX);
    CHECK(areCurveDerivativesGood);
//2. Test each integral, where computed for correctness.
    //testMuscleCurveIntegral(fiberFLCurve,fiberFLCurveSample);

//3. Test numerically to see if the curve is C2 continuous
    bool curveIsContinuous = isCurveC2Continuous(
                              fiberFLCurve,
                              fiberFLCurveSample,
                              TOL_BIG);
    CHECK(curveIsContinuous);
//4. Test for montonicity where appropriate

    bool curveIsMonotonic = isCurveMontonic(fiberFLCurveSample);
    CHECK(curveIsMonotonic);

    if(FLAG_PLOT_CURVES){
     fiberFLCurve.printCurveToCSVFile(FILE_PATH,
                 "fiberFLCurve",
                 1.0-0.1,
                 1.0+e0f+0.1);
    }
}

TEST_CASE(__FILE__"_compressiveForceLengthCurve", "")
{
///////////////////////////////////////
//FIBER COMPRESSIVE FORCE LENGTH
///////////////////////////////////////
    //cout <<"**************************************************"<<endl;
    //cout <<"FIBER COMPRESSIVE FORCE LENGTH CURVE TESTING   "<<endl;


    double lmax = 0.6;
    double kce  = -8.389863790885878;
    double cce  = 0.5;//0.0;

    SmoothSegmentedFunction fiberCECurve = SmoothSegmentedFunction();
    MuscleFunctionFactory::
    createFiberCompressiveForceLengthCurve(
      lmax,
      kce,
      cce,
      "test_fiberCompressiveForceLengthCurve",
      fiberCECurve);

    //fiberCECurve.printMuscleCurveToFile("C:/mjhmilla/Stanford/dev"
    //    "/OpenSim_LOCALPROJECTS/MuscleLibrary_Bench_20120210/build");
    RigidBodyDynamics::Math::MatrixNd fiberCECurveSample 
        = fiberCECurve.calcSampledCurve(6,0,lmax);

//0. Test that each curve fulfills its contract.
    //cout << "   Keypoint Testing" << endl;

    RigidBodyDynamics::Math::VectorNd fiberCECurveDomain 
     = fiberCECurve.getCurveDomain();
    CHECK(abs(fiberCECurveDomain[0])          < TOL_SMALL);
    CHECK(abs(fiberCECurveDomain[1]- lmax)       < TOL_SMALL);
    CHECK(abs(fiberCECurve.calcValue(lmax))      < TOL_SMALL);  
    CHECK(abs(fiberCECurve.calcDerivative(lmax,1))  < TOL_BIG);
    CHECK(abs(fiberCECurve.calcDerivative(lmax,2))  < TOL_BIG);
    CHECK(abs(fiberCECurve.calcValue(0) - 1.0)      < TOL_SMALL);  
    CHECK(abs(fiberCECurve.calcDerivative(0,1)-kce)    < TOL_BIG);
    CHECK(abs(fiberCECurve.calcDerivative(0,2))     < TOL_BIG);
    //cout << "   passed" << endl;
    //cout << endl;
//1. Test each derivative sample for correctness against a numerically
//   computed version
    bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      fiberCECurve,
      fiberCECurveSample,
      TOL_DX);
    CHECK(areCurveDerivativesGood);

//2. Test each integral, where computed for correctness.
    //testMuscleCurveIntegral(fiberCECurve,fiberCECurveSample);

//3. Test numerically to see if the curve is C2 continuous
    bool curveIsContinuous = isCurveC2Continuous(
                              fiberCECurve,
                              fiberCECurveSample,
                              TOL_BIG);
    CHECK(curveIsContinuous);
//4. Test for montonicity where appropriate

    bool curveIsMonotonic = isCurveMontonic(fiberCECurveSample);
    CHECK(curveIsMonotonic);

//5. Testing Exceptions
    //cout << endl;
    //cout << "   Exception Testing" << endl;

    if(FLAG_PLOT_CURVES){
     fiberCECurve.printCurveToCSVFile(FILE_PATH,
                 "fiberCECurve",
                 -0.1,
                 lmax + 0.1);
    }

    //cout << "    passed" << endl;

}

TEST_CASE(__FILE__"_compressivePhiCurve", "")
{
    //cout <<"**************************************************"<<endl;
    //cout <<"FIBER COMPRESSIVE FORCE PHI CURVE TESTING   "<<endl;

    double phi0 = (M_PI/2)*(1.0/2.0);
    double phi1 = M_PI/2;
    double kphi  = 8.389863790885878;
    double cphi  = 0.0;  

    SmoothSegmentedFunction fiberCEPhiCurve = SmoothSegmentedFunction();
    MuscleFunctionFactory::
     createFiberCompressiveForcePennationCurve(phi0,kphi,cphi,
         "test_fiberCompressiveForcePennationCurve", fiberCEPhiCurve);    


    RigidBodyDynamics::Math::MatrixNd fiberCEPhiCurveSample 
        = fiberCEPhiCurve.calcSampledCurve(6,phi0,phi1);

//0. Test that each curve fulfills its contract.
    //cout << "   Keypoint Testing" << endl;

    CHECK(abs(fiberCEPhiCurve.calcValue(phi0))     < TOL_SMALL);  
    CHECK(abs(fiberCEPhiCurve.calcDerivative(phi0,1)) < TOL_BIG);
    CHECK(abs(fiberCEPhiCurve.calcDerivative(phi0,2)) < TOL_BIG);
    CHECK(abs(fiberCEPhiCurve.calcValue(phi1)) -1     < TOL_SMALL);
    CHECK(abs(fiberCEPhiCurve.calcDerivative(phi1,1)-kphi)  < TOL_BIG);
    CHECK(abs(fiberCEPhiCurve.calcDerivative(phi1,2))    < TOL_BIG);
    //cout << "   passed" << endl;
    //cout << endl;
//1. Test each derivative sample for correctness against a numerically
//   computed version
    bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      fiberCEPhiCurve,
      fiberCEPhiCurveSample,
      TOL_DX);
    CHECK(areCurveDerivativesGood);
//2. Test each integral, where computed for correctness.
    //testMuscleCurveIntegral(fiberCEPhiCurve,fiberCEPhiCurveSample);

//3. Test numerically to see if the curve is C2 continuous
    //cout << "Attention: Removed test for the Cos Phi Compressive Curve"<<endl;
    bool curveIsContinuous = isCurveC2Continuous(
                                fiberCEPhiCurve,
                                fiberCEPhiCurveSample,
                                TOL_BIG);
    CHECK(curveIsContinuous);
//4. Test for montonicity where appropriate
    bool curveIsMonotonic = isCurveMontonic(fiberCEPhiCurveSample);
    CHECK(curveIsMonotonic);


    if(FLAG_PLOT_CURVES){
     fiberCEPhiCurve.printCurveToCSVFile(FILE_PATH,
                 "fiberCEPhiCurve",
                 phi0-0.1,
                 phi1+0.1);
    }

    //cout << "    passed" << endl;

}

TEST_CASE(__FILE__"_compressiveCosPhiCurve", "")
{
    //cout <<"**************************************************"<<endl;
    //cout <<"FIBER COMPRESSIVE FORCE COSPHI CURVE TESTING   "<<endl;

    double cosPhi0 = cos( (80.0/90.0)*M_PI/2);
    double kcosPhi  = -1.2/(cosPhi0);
    double ccosPhi  = 0.5;
    SmoothSegmentedFunction fiberCECosPhiCurve = SmoothSegmentedFunction();

    MuscleFunctionFactory::
        createFiberCompressiveForceCosPennationCurve(
           cosPhi0,kcosPhi,ccosPhi,
           "test_fiberCompressiveForceCosPennationCurve",
           fiberCECosPhiCurve);

    RigidBodyDynamics::Math::MatrixNd fiberCECosPhiCurveSample 
        = fiberCECosPhiCurve.calcSampledCurve(6,0,cosPhi0);

//0. Test that each curve fulfills its contract.
    //cout << "   Keypoint Testing" << endl;

    CHECK(abs(fiberCECosPhiCurve.calcValue(cosPhi0)    )  < TOL_SMALL);  
    CHECK(abs(fiberCECosPhiCurve.calcDerivative(cosPhi0,1)   )  < TOL_BIG);
    CHECK(abs(fiberCECosPhiCurve.calcDerivative(cosPhi0,2)   )  < TOL_BIG);
    CHECK(abs(fiberCECosPhiCurve.calcValue(0)    - 1.0    )  < TOL_SMALL);  
    CHECK(abs(fiberCECosPhiCurve.calcDerivative(0,1) -kcosPhi)  < TOL_BIG);
    CHECK(abs(fiberCECosPhiCurve.calcDerivative(0,2)      )  < TOL_BIG);
    //cout << "   passed" << endl;
    //cout << endl;
//1. Test each derivative sample for correctness against a numerically
//   computed version
    bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      fiberCECosPhiCurve,
      fiberCECosPhiCurveSample,
      TOL_DX);
    CHECK(areCurveDerivativesGood);
//2. Test each integral, where computed for correctness.
    //testMuscleCurveIntegral(fiberCECosPhiCurve,fiberCECosPhiCurveSample);

//3. Test numerically to see if the curve is C2 continuous

    bool curveIsContinuous = isCurveC2Continuous(
                                fiberCECosPhiCurve,
                                fiberCECosPhiCurveSample,
                                TOL_BIG);
    CHECK(curveIsContinuous);
//4. Test for montonicity where appropriate

    bool curveIsMonotonic = isCurveMontonic(fiberCECosPhiCurveSample);
    CHECK(curveIsMonotonic);


    //cout << "    passed" << endl;
    if(FLAG_PLOT_CURVES){
     fiberCECosPhiCurve.printCurveToCSVFile(FILE_PATH,
                 "fiberCECosPhiCurve",
                 -0.1,
                 cosPhi0+0.1);
    }

}


