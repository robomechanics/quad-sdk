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


#include "numericalTestFunctions.h"

#include <rbdl/rbdl_math.h>
#include <ctime>
#include <string>
#include <stdio.h>
#include <exception>
#include <cassert>
#include <fstream>

using namespace RigidBodyDynamics::Addons::Geometry;
using namespace std;


void printMatrixToFile(
    const RigidBodyDynamics::Math::VectorNd& col0, 
    const RigidBodyDynamics::Math::MatrixNd& data, 
    string& filename)
{
    
    ofstream datafile;
    datafile.open(filename.c_str());

    for(int i = 0; i < data.rows(); i++){
     datafile << col0[i] << ",";
     for(int j = 0; j < data.cols(); j++){
      if(j<data.cols()-1)
       datafile << data(i,j) << ",";
      else
       datafile << data(i,j) << "\n";
     }   
    }
    datafile.close();
} 


void printMatrixToFile( 
    const RigidBodyDynamics::Math::MatrixNd& data, 
    string& filename)
{
    ofstream datafile;
    datafile.open(filename.c_str());

    for(int i = 0; i < data.rows(); i++){
     for(int j = 0; j < data.cols(); j++){
      if(j<data.cols()-1)
       datafile << data(i,j) << ",";
      else
       datafile << data(i,j) << "\n";
     }   
    }
    datafile.close();
}


RigidBodyDynamics::Math::VectorNd 
    calcCentralDifference(  RigidBodyDynamics::Math::VectorNd& x, 
             RigidBodyDynamics::Math::VectorNd& y, 
             bool extrap_endpoints){
 
    RigidBodyDynamics::Math::VectorNd dy(x.size());
    double dx1,dx2;
    double dy1,dy2;
    int size = x.size();
    for(int i=1; i<size-1; i++){
     dx1 = x[i]   - x[i-1];
     dx2 = x[i+1] - x[i];
     dy1 = y[i]   - y[i-1];
     dy2 = y[i+1] - y[i];
     dy[i]= 0.5*dy1/dx1 + 0.5*dy2/dx2;
    }

    if(extrap_endpoints == true){
     dy1   = dy[2] - dy[1];
     dx1   = x[2]  - x[1];
     dy[0] = dy[1] + (dy1/dx1)*(x[0]-x[1]);

     dy2 = dy[size-2] - dy[size-3];
     dx2 = x[size-2]  - x[size-3];
     dy[size-1] = dy[size-2] + (dy2/dx2)*(x[size-1]-x[size-2]);
    }
    return dy;
}


RigidBodyDynamics::Math::VectorNd 
calcCentralDifference(  RigidBodyDynamics::Math::VectorNd& x, 
            SmoothSegmentedFunction& mcf,
            double tol, int order){
 

    RigidBodyDynamics::Math::VectorNd dyV(x.size());
    RigidBodyDynamics::Math::VectorNd yV(x.size());

    double y = 0;
    double dy = 0;
    double dyNUM = 0;
    double err= 0;
    double h = 0;
    double xL = 0;
    double xR = 0;

    double c3 = 0;
    double fL = 0;
    double fR = 0;
    double rootEPS = sqrt(EPSILON);

    double y_C3min = 1e-10;
    double y_C3max = 1e1;


    for(int i=0; i<x.size(); i++){
     yV[i] = mcf.calcDerivative(x[i],order-1);
    }
   

    for(int i=0; i< x.size(); i++){
     
     c3 = abs(mcf.calcDerivative(x[i],order+2));
     
     //singularity prevention
     if(abs(c3) < y_C3min)
      c3 = y_C3min;
     //Compute h
     y  = abs(mcf.calcDerivative(x[i], order-1));
     //preventing 0 from being assigned to y
     if(y < y_C3min)
      y = y_C3min;

     //Dumb check
     if(y/c3 < y_C3min){
      c3 = 1;
      y = y_C3min;
     }
     if(y/c3 > y_C3max){
      c3 = 1;
      y = y_C3max;
     }

     h  = pow( ( (EPSILON*y*2.0)/(c3) ) , 1.0/3.0);
    
     //Now check that h to the left and right are at least similar
     //If not, take the smallest one.
     xL = x[i]-h/2;
     xR = x[i]+h/2;

     fL = mcf.calcDerivative(xL, order-1);
     fR = mcf.calcDerivative(xR, order-1);

     //Just for convenience checking ...
     dyNUM = (fR-fL)/h;
     dy    = mcf.calcDerivative(x[i],order);
     err   = abs(dy-dyNUM);

     /*if(err > tol && abs(dy) > rootEPS && order <= 2){
      err = err/abs(dy);
      if(err > tol)
       cout << "rel tol exceeded" << endl;     
     }*/

     dyV[i] = dyNUM;

    }


    return dyV;
}

bool isFunctionContinuous(   RigidBodyDynamics::Math::VectorNd& xV, 
                             SmoothSegmentedFunction& yV, 
                             int order, 
                             double minValueSecondDerivative,
                             double taylorErrorMult)
{
    bool flag_continuous = true;

    double xL = 0;   // left shoulder point
    double xR = 0;   // right shoulder point
    double yL = 0;   // left shoulder point function value
    double yR = 0;   // right shoulder point function value
    double dydxL = 0;   // left shoulder point derivative value
    double dydxR = 0;   // right shoulder point derivative value

    double xVal = 0;    //x value to test
    double yVal = 0;    //Y(x) value to test

    double yValEL = 0;  //Extrapolation to yVal from the left
    double yValER = 0;  //Extrapolation to yVal from the right

    double errL = 0;
    double errR = 0;

    double errLMX = 0;
    double errRMX = 0;


    for(int i =1; i < xV.size()-1; i++){
     xVal = xV[i];
     yVal = yV.calcDerivative(xVal, order);

     xL = 0.5*(xV[i]+xV[i-1]);
     xR = 0.5*(xV[i]+xV[i+1]);

     yL = yV.calcDerivative(xL,order);
     yR = yV.calcDerivative(xR,order);

     dydxL = yV.calcDerivative(xL,order+1);
     dydxR = yV.calcDerivative(xR,order+1);

     
     yValEL = yL + dydxL*(xVal-xL);
     yValER = yR - dydxR*(xR-xVal);

     errL = abs(yValEL-yVal);
     errR = abs(yValER-yVal);

     errLMX = abs(yV.calcDerivative(xL,order+2)*0.5*(xVal-xL)*(xVal-xL));
     errRMX = abs(yV.calcDerivative(xR,order+2)*0.5*(xR-xVal)*(xR-xVal));

     errLMX*=taylorErrorMult;
     errRMX*=taylorErrorMult;

     if(errLMX < minValueSecondDerivative)
      errLMX = minValueSecondDerivative;

     if(errRMX < minValueSecondDerivative)
      errRMX = minValueSecondDerivative; // to accomodate numerical
              //error in errL

     if(errL > errLMX || errR > errRMX){      
      flag_continuous = false;
     }
    }

    return flag_continuous;
}

bool isVectorMonotonic( RigidBodyDynamics::Math::VectorNd& y, 
            int multEPS)
{
    double dir = y[y.size()-1]-y[0];
    bool isMonotonic = true;

    if(dir < 0){
     for(int i =1; i <y.size(); i++){
      if(y[i] > y[i-1]+EPSILON*multEPS){
       isMonotonic = false;
      //printf("Monotonicity broken at idx %i, since %fe-16 > %fe-16\n",
       //     i,y(i)*1e16,y(i-1)*1e16);
       printf("Monotonicity broken at idx %i, since "
        "y(i)-y(i-1) < tol, (%f*EPSILON < EPSILON*%i) \n",
            i,((y[i]-y[i-1])/EPSILON), multEPS);
      }
     }
    }
    if(dir > 0){
     for(int i =1; i <y.size(); i++){
      if(y[i] < y[i-1]-EPSILON*multEPS){
       isMonotonic = false;
       printf("Monotonicity broken at idx %i, since "
        "y(i)-y(i-1) < -tol, (%f*EPSILON < -EPSILON*%i) \n",
            i,((y[i]-y[i-1])/EPSILON), multEPS);
      }
     }
    }
    if(dir == 0){
     isMonotonic = false;
    }

    return isMonotonic;
}

RigidBodyDynamics::Math::VectorNd calcTrapzIntegral(
            RigidBodyDynamics::Math::VectorNd& x, 
            RigidBodyDynamics::Math::VectorNd& y, 
            bool flag_TrueIntForward_FalseIntBackward)
{
    RigidBodyDynamics::Math::VectorNd inty 
     = RigidBodyDynamics::Math::VectorNd::Zero(y.size());
    //inty = 0;


    int startIdx = 1;
    int endIdx = y.size()-1;

    if(flag_TrueIntForward_FalseIntBackward == true){
      
     double width = 0;
     for(int i = 1; i <= endIdx; i=i+1){
      width = abs(x[i]-x[i-1]);
      inty[i] = inty[i-1] +  width*(0.5)*(y[i]+y[i-1]);
     }

    }else{
     
     double width = 0;      
     for(int i = endIdx-1; i >= 0; i=i-1){
      width = abs(x[i]-x[i+1]);
      inty[i] = inty[i+1] +  width*(0.5)*(y[i]+y[i+1]);
     }
    }

  
    return inty;
}


double calcMaximumVectorError(RigidBodyDynamics::Math::VectorNd& a, 
            RigidBodyDynamics::Math::VectorNd& b)
{
    double error = 0;
    double cerror=0;
    for(int i = 0; i< a.size(); i++)
    {
     cerror = abs(a[i]-b[i]);
     if(cerror > error){
      error = cerror;
     }     
    }
    return error;
}



bool isCurveC2Continuous(SmoothSegmentedFunction& mcf,
              RigidBodyDynamics::Math::MatrixNd& mcfSample,
              double continuityTol)
{
    //cout << "   TEST: C2 Continuity " << endl;

    int multC0 = 5;
    int multC1 = 50;
    int multC2 = 200;

    RigidBodyDynamics::Math::VectorNd fcnSample = 
     RigidBodyDynamics::Math::VectorNd::Zero(mcfSample.rows());

    for(int i=0; i < mcfSample.rows(); i++){
     fcnSample[i] = mcfSample(i,0);
    }


    bool c0 = isFunctionContinuous(fcnSample, mcf, 0, continuityTol, multC0);
    bool c1 = isFunctionContinuous(fcnSample, mcf, 1, continuityTol, multC1);
    bool c2 = isFunctionContinuous(fcnSample, mcf, 2, continuityTol, multC2);



    return (c0 && c1 && c2);
    //printf( "   passed: C2 continuity established to a multiple\n"
    //     "     of the next Taylor series error term.\n "
    //     "     C0,C1, and C2 multiples: %i,%i and %i\n",
    //        multC0,multC1,multC2);
    //cout << endl;
}


bool isCurveMontonic(RigidBodyDynamics::Math::MatrixNd mcfSample)
{
    //cout << "   TEST: Monotonicity " << endl;
    int multEps = 10;

    RigidBodyDynamics::Math::VectorNd fcnSample = 
     RigidBodyDynamics::Math::VectorNd::Zero(mcfSample.rows());

    for(int i=0; i < mcfSample.rows(); i++){
     fcnSample[i] = mcfSample(i,1);
    }

    bool monotonic = isVectorMonotonic(fcnSample,10);
    return monotonic;
    //printf("   passed: curve is monotonic to %i*EPSILON",multEps);
    //cout << endl;
}


bool areCurveDerivativesCloseToNumericDerivatives(
     SmoothSegmentedFunction& mcf,
     RigidBodyDynamics::Math::MatrixNd& mcfSample,
     double tol)
{
    //cout << "   TEST: Derivative correctness " << endl;
    int maxDer = 4;//mcf.getMaxDerivativeOrder() - 2;
   
    RigidBodyDynamics::Math::MatrixNd numSample(mcfSample.rows(),maxDer);  
    RigidBodyDynamics::Math::MatrixNd relError(mcfSample.rows(),maxDer);
    

    RigidBodyDynamics::Math::VectorNd domainX = 
     RigidBodyDynamics::Math::VectorNd::Zero(mcfSample.rows());

    for(int j=0; j<mcfSample.rows(); j++)
     domainX[j] = mcfSample(j,0);

    for(int i=0; i < maxDer; i++){
     //Compute the relative error
     numSample.col(i)=calcCentralDifference(domainX,mcf,tol,i+1);
     for(int j=0; j<mcfSample.rows();++j ){
        relError(j,i)= mcfSample(j,i+2)-numSample(j,i);  
      } 

     //compute a relative error where possible
     for(int j=0; j < relError.rows(); j++){
      if(abs(mcfSample(j,i+2)) > tol){
       relError(j,i) = relError(j,i)/mcfSample(j,i+2);
      }
     }

    }

    RigidBodyDynamics::Math::VectorNd errRelMax =
     RigidBodyDynamics::Math::VectorNd::Zero(6);
    RigidBodyDynamics::Math::VectorNd errAbsMax = 
     RigidBodyDynamics::Math::VectorNd::Zero(6);

    double absTol = 5*tol;

    bool flagError12=false;
    RigidBodyDynamics::Math::VectorNd tolExceeded12V =
     RigidBodyDynamics::Math::VectorNd::Zero(mcfSample.rows());
    
    int tolExceeded12 = 0;
    int tolExceeded34 = 0;
    for(int j=0;j<maxDer;j++){
     
     for(int i=0; i<mcfSample.rows(); i++){
      if(relError(i,j) > tol && mcfSample(i,j+2) > tol){
       if(j <= 1){
        tolExceeded12++;
        tolExceeded12V[i]=1;
        flagError12=true;
       }
       if(j>=2)
        tolExceeded34++;       
      }
      if(mcfSample(i,j+2) > tol)
      if(errRelMax[j] < abs(relError(i,j)))
        errRelMax[j] = abs(relError(i,j));

      //This is a harder test: here we're comparing absolute error
      //so the tolerance margin is a little higher
      if(relError(i,j) > absTol && mcfSample(i,j+2) <= tol){
       if(j <= 1){
        tolExceeded12++;
        tolExceeded12V[i]=1;
        flagError12=true;
       }
       if(j>=2)
        tolExceeded34++;            
      }

      if(mcfSample(i,j+2) < tol)
      if(errAbsMax[j] < abs(relError(i,j)))
        errAbsMax[j] = abs(relError(i,j));
      
     
     }

     /*
     if(flagError12 == true){
      printf("Derivative %i Rel Error Exceeded:\n",j);
      printf("x     dx_relErr dx_calcVal dx_sample"
        " dx2_relErr dx2_calcVal dx2_sample\n");
      for(int i=0; i<mcfSample.rows(); i++){
       if(tolExceeded12V(i) == 1){
          printf("%f %f %f  %f %f   %f    %f",
        mcfSample(i,0),relError(i,0),mcfSample(i,2),numSample(i,0),
                relError(i,1),mcfSample(i,3),numSample(i,1));
        
       }
      }
     }
     flagError12=false;*/
     //tolExceeded12V = 
     //RigidBodyDynamics::Math::VectorNd::Zero(mcfSample.rows());
    }
    
   return (tolExceeded12 == 0);
    
}



