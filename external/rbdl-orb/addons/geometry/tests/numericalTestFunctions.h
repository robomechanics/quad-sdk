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

#include "../geometry.h"
#include <rbdl/rbdl_math.h>
#include <ctime>
#include <string>
#include <stdio.h>
#include <exception>
#include <cassert>

using namespace RigidBodyDynamics::Addons::Geometry;

using namespace std;

static double EPSILON = numeric_limits<double>::epsilon();
static double SQRTEPSILON = sqrt(EPSILON);
static double TOL = std::numeric_limits<double>::epsilon()*1e4;

static bool FLAG_PLOT_CURVES    = false;
static string FILE_PATH         = "";
static double TOL_DX            = 5e-3;
static double TOL_DX_BIG        = 1e-2;
static double TOL_BIG           = 1e-6;
static double TOL_SMALL         = 1e-12;

/**
This function will print cvs file of the column vector col0 and the matrix 
 data

@params col0: A vector that must have the same number of rows as the data matrix
       This column vector is printed as the first column
@params data: A matrix of data
@params filename: The name of the file to print
*/
void printMatrixToFile(
    const RigidBodyDynamics::Math::VectorNd& col0, 
    const RigidBodyDynamics::Math::MatrixNd& data, 
    string& filename);


/**
This function will print cvs file of the matrix 
 data

@params data: A matrix of data
@params filename: The name of the file to print
*/
void printMatrixToFile( 
    const RigidBodyDynamics::Math::MatrixNd& data, 
    string& filename);


/**
    This function computes a standard central difference dy/dx. If 
    extrap_endpoints is set to 1, then the derivative at the end points is 
    estimated by linearly extrapolating the dy/dx values beside the end points

 @param x domain vector
 @param y range vector
 @param extrap_endpoints: (false)   Endpoints of the returned vector will be 
               zero, because a central difference
               is undefined at these endpoints
             (true)  Endpoints are computed by linearly 
               extrapolating using a first difference from 
               the neighboring 2 points
 @returns dy/dx computed using central differences
*/
RigidBodyDynamics::Math::VectorNd 
    calcCentralDifference(  RigidBodyDynamics::Math::VectorNd& x, 
             RigidBodyDynamics::Math::VectorNd& y, 
             bool extrap_endpoints);

/**
    This function computes a standard central difference dy/dx at each point in
    a vector x, for a SmoothSegmentedFunction mcf, to a desired tolerance. This 
    function will take the best step size at each point to minimize the 
    error caused by taking a numerical derivative, and the error caused by
    numerical rounding error:

    For a step size of h/2 to the left and to the right of the point of 
    interest the error is
    error = 1/4*h^2*c3 + r*f(x)/h,         (1)
      
    Where c3 is the coefficient of the 3rd order Taylor series expansion
    about point x. Thus c3 can be computed if the order + 2 derivative is
    known
     
     c3 = (d^3f(x)/dx^3)/(6)            (2)
     
    And r*f(x)/h is the rounding error that occurs due to the central 
    difference.

    Taking a first derivative of 1 and solving for h yields

    h = (r*f(x)*2/c3)^(1/3)

    Where r is EPSILON

  @param x domain vector
  @param mcf the SmoothSegmentedFunction of interest
  @param order the order of the numerical derivative
  @param tolerance desired tolerance on the returned result
  @returns dy/dx computed using central differences
*/
RigidBodyDynamics::Math::VectorNd 
calcCentralDifference(  RigidBodyDynamics::Math::VectorNd& x, 
            SmoothSegmentedFunction& mcf,
            double tol, int order);

/**
    This function tests numerically for continuity of a curve. The test is 
    performed by taking a point on the curve, and then two points (called the 
    shoulder points) to the left and right of the point in question. The 
    shoulder points are located half way between the sample points in xV.

    The value of the function's derivative is evaluated at each of the shoulder 
    points and used to linearly extrapolate from the shoulder points back to the 
    original point. If the original point and the linear extrapolations of each 
    of the shoulder points agree within a tolerance, then the curve is assumed 
    to be continuous. This tolerance is evaluated to be the smaller of the 2nd 
    derivative times a multiplier (taylorErrorMult) or minValueSecondDerivative


    @param x     Values to test for continuity

    @param yx    The SmoothSegmentedFunction to test

    @param order The order of the curve of SmoothSegmentedFunction to test
        for continuity

    @param minValueSecondDerivative the minimum value allowed for the 2nd 
           term in the Taylor series. Here we need to define a minimum because
           this 2nd term is used to define a point specific tolerance for the
           continuity test. If the 2nd derivative happens to go to zero, we 
           still cannot let the minimum error tolerance to go to zero - else
           the test will fail on otherwise good curves. 

    @param taylorErrorMult  This scales the error tolerance. The default error
             tolerance is the the 2nd order Taylor series term. This term is
             dependent on the size of the higher-order-terms and the sample 
             spacing used for xV (since the shoulder points are picked half-
             way between the sampled points)
*/
bool isFunctionContinuous(  RigidBodyDynamics::Math::VectorNd& xV, 
                            SmoothSegmentedFunction& yV, 
                            int order, 
                            double minValueSecondDerivative,
                            double taylorErrorMult);


/**
This function will scan through a vector and determine if it is monotonic or
not

@param y the vector of interest
@param multEPS The tolerance on the monotoncity check, expressed as a scaling of
       EPSILON
@return true if the vector is monotonic, false if it is not
*/
bool isVectorMonotonic( RigidBodyDynamics::Math::VectorNd& y, 
                        int multEPS);


/**
This function will compute the numerical integral of y(x) using the trapezoidal
method

@param x the domain vector
@param y the range vector, of y(x), evaluated at x
@param flag_TrueIntForward_FalseIntBackward 
    When this flag is set to true, the integral of y(x) will be evaluated from
    left to right, starting with int(y(0)) = 0. When this flag is false, then
    y(x) will be evaluated from right to left with int(y(n)) = 0, where n is 
    the maximum number of elements.                    
@return the integral of y(x)
*/
RigidBodyDynamics::Math::VectorNd calcTrapzIntegral(
            RigidBodyDynamics::Math::VectorNd& x, 
            RigidBodyDynamics::Math::VectorNd& y, 
            bool flag_TrueIntForward_FalseIntBackward);


/**
   @param a The first vector
   @param b The second vector
   @return Returns the maximum absolute difference between vectors a and b
*/
double calcMaximumVectorError(RigidBodyDynamics::Math::VectorNd& a, 
                              RigidBodyDynamics::Math::VectorNd& b);


/*
This function tests the SmoothSegmentedFunction to see if it is C2 continuous. 
This function works by using the applying the function isFunctionContinuous 
multiple times. For details of the method used please refer to the doxygen for
isFunctionContinuous.

@param mcf - a SmoothSegmentedFunction
@param mcfSample: 
  A n-by-m matrix of values where the first column is the domain (x) of interest
  and the remaining columns are the curve value (y), and its derivatives (dy/dx,
  d2y/dx2, d3y/dx3, etc).  This matrix is returned by the function 
  calcSampledCurve in SmoothSegmented Function.  
@param continuityTol  
@return bool: true if the curve is C2 continuous to the desired tolernace
              
*/ 
bool isCurveC2Continuous(SmoothSegmentedFunction& mcf,
              RigidBodyDynamics::Math::MatrixNd& mcfSample,
              double continuityTol);

/*
 4. The MuscleCurveFunctions which are supposed to be monotonic will be
    tested for monotonicity.
*/
bool isCurveMontonic(RigidBodyDynamics::Math::MatrixNd mcfSample);

/**
This function compares the i^th derivative return by a SmoothSegmented function 
against a numerical derivative computed using a central difference applied to 
the (i-1)^th derivative of the function. This function first checks the 
1st derivative and continues checking derivatives until the 4th derivative. 

@param mcf - a SmoothSegmentedFunction
@param mcfSample: 
  A n-by-m matrix of values where the first column is the domain (x) of interest
  and the remaining columns are the curve value (y), and its derivatives (dy/dx,
  d2y/dx2, d3y/dx3, etc). This matrix is returned by the function 
  calcSampledCurve in SmoothSegmented Function.

@param tol : the tolerance used to assess the relative error between the 
             numerically computed derivatives and the derivatives returned by
             the SmoothSegmentedFunction
@return bool: true if all of the derivatives up to the 4th (hard coded) are 
              within a relative tolerance of tol w.r.t. to the numerically 
              computed derivatives
*/
bool areCurveDerivativesCloseToNumericDerivatives(
     SmoothSegmentedFunction& mcf,
     RigidBodyDynamics::Math::MatrixNd& mcfSample,
     double tol);
