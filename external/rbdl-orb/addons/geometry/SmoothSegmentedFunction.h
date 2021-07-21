#ifndef SMOOTHSEGMENTEDFUNCTION_H_
#define SMOOTHSEGMENTEDFUNCTION_H_
/* -------------------------------------------------------------------------- *
 *      OpenSim:  SmoothSegmentedFunction.h                                   *
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

#include "Function.h"
#include "SegmentedQuinticBezierToolkit.h"
#include <vector>
#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_config.h>

  /**
  This class contains the quintic Bezier curves, x(u) and y(u), that have been
  created by SmoothSegmentedFunctionFactory to follow a physiologically 
  meaningful muscle characteristic. A SmoothSegmentedFunction cannot be 
  created directly,you must use SmoothSegmentedFunctionFactory to create the 
  muscle curve of interest.

   <B>Computational Cost Details</B>
  All computational costs assume the following operation costs:

  \verbatim
  Operation Type   : #flops
  +,-,=,Boolean Op : 1 
       / : 10
       sqrt: 20
       trig: 40
  \endverbatim

  These relative weightings will vary processor to processor, and so any 
  of the quoted computational costs are approximate.


<B> RBDL Port Notes </B>
The port of this code from OpenSim has been accompanied by a few changes:

1.  The 'calcIntegral' method has been removed. Why? This function 
  relied on having access to a variable-step error controlled 
  integrator. There is no such integrator built into RBDL. Rather 
  than add a dependency (by using Boost perhaps) this functionality
  has been removed. 

2.  The function name .printMuscleCurveToFile(...) has been changed
  to .printCurveToFile(). 


  @author Matt Millard
  @version 0.0


  */

namespace RigidBodyDynamics {
  namespace Addons {
  namespace Geometry{

  class RBDL_ADDON_DLLAPI SmoothSegmentedFunction : public Function_<double>
  {
   

     public:

    ///The default constructor, which populates the member data fields with
    ///NaN's
    SmoothSegmentedFunction();

    //SmoothSegmentedFunction();
     /**
     Creates a set of quintic Bezier curves.

     @param mX   The matrix of quintic Bezier x point locations (6xn).
         Each column vector is the 6 control points required
         for each quintic Bezier curve. For C0 continuity 
         adjacent columns must share the last and first control
         points. For C1 continuity the last 2 and first two
         control points of adjacent curves should be on the same
         curve.

     @param mY   The matrix of quintic Bezier y point locations (6xn).
        
     @param x0   The minimum x value. This is used for the linear 
         extrapolation of the Bezier curve. This parameter is
         explicitly asked for, rather than computed, to prevent
         rounding error from reducing the accuracy of the 
         linear extrapolation.
     @param x1   The maximum x value. This is used for the linear 
         extrapolation and is required for the same reasons 
         as x0. 
     @param y0   The value of y(x) at x=x0. This is used for the linear 
         extrapolation and is required for the same reasons 
         as x0.
     @param y1   The value of y(x) at x=x1.  This is used for the linear 
         extrapolation and is required for the same reasons 
         as x0.
     @param dydx0  The value of dy/dx at x=x0.  This is used for the linear 
         extrapolation and is required for the same reasons 
         as x0.
     @param dydx1  The value of dy/dx at x=x1. This is used for the linear 
         extrapolation and is required for the same reasons 
         as x0.
     @param name    The name of the data this SmoothSegmentedFunction. This name
                    is used to make human-readable error messages and to 
                    generate sensible names when printing the curve to file. 

     <B>Computational Costs</B>
     Generating the integral curve is not cheap, and so should only be used 
     when if it will be evaluated during a simulation. 
     \verbatim   
    Computatonal Cost Per Bezier Section:
      Without Integral :   4,100 flops
     \endverbatim

      */
     SmoothSegmentedFunction(
      const RigidBodyDynamics::Math::MatrixNd& mX, 
      const RigidBodyDynamics::Math::MatrixNd& mY, 
      double x0, double x1,
      double y0, double y1,
      double dydx0, double dydx1,
      const std::string& name); 


     /**
      Updates the Bezier curve conrol points. The updated curve must have
      the same number of control points as the original curve.

       @param mX   The matrix of quintic Bezier x point locations (6xn).
           Each column vector is the 6 control points required
           for each quintic Bezier curve. For C0 continuity 
           adjacent columns must share the last and first control
           points. For C1 continuity the last 2 and first two
           control points of adjacent curves should be on the same
           curve.

       @param mY   The matrix of quintic Bezier y point locations (6xn).
          
       @param x0   The minimum x value. This is used for the linear 
           extrapolation of the Bezier curve. This parameter is
           explicitly asked for, rather than computed, to prevent
           rounding error from reducing the accuracy of the 
           linear extrapolation.
       @param x1   The maximum x value. This is used for the linear 
           extrapolation and is required for the same reasons 
           as x0. 
       @param y0   The value of y(x) at x=x0. This is used for the linear 
           extrapolation and is required for the same reasons 
           as x0.
       @param y1   The value of y(x) at x=x1.  This is used for the linear 
           extrapolation and is required for the same reasons 
           as x0.
       @param dydx0  The value of dy/dx at x=x0.  This is used for the linear 
           extrapolation and is required for the same reasons 
           as x0.
       @param dydx1  The value of dy/dx at x=x1. This is used for the linear 
           extrapolation and is required for the same reasons 
           as x0.
        @param name The name of the data this SmoothSegmentedFunction. This name
                    is used to make human-readable error messages and to 
                    generate sensible names when printing the curve to file.

     */
     void updSmoothSegmentedFunction(
      const RigidBodyDynamics::Math::MatrixNd& mX, 
      const RigidBodyDynamics::Math::MatrixNd& mY, 
      double x0, double x1,
      double y0, double y1,
      double dydx0, double dydx1,
      const std::string& name);

     /**
      This function will shift the entire SmoothSegmentedFunction by xShift
      and yShift. Setting xShift = yShift = 0.0 will leave the curve unchanged.

      @param xShift - the amount to shift the curve in the x-direction. 
      @param yShift - the amount to shift the curve in the y-direction
     */
     void shift(double xShift, double yShift);

     /**
      This function will scale the curve in the x and y directions. Setting 
      xScale=yScale=1.0 will leave the curve unchanged.

      \b aborts \b
    -If abs(xScale) < sqrt(eps) 

      @param xScale: the amount to scale the curve in the x direction
      @param yScale: the amount to scale the curve in the y direction
     */
     void scale(double xScale, double yScale);



     /**Calculates the value of the curve this object represents.

     @param x The domain point of interest
     
     \b aborts \b
    -If ax does not have a size of 1
     @returns The value of the curve


     The curve is parameterized as a set of Bezier curves. If x is within the
     domain of these Bezier curves they will be evaluated. If x is outside
     of the domain of these Bezier curves a linear extrapolation will be 
     evalulated


     <B>Computational Costs</B>
     \verbatim
      x in curve domain  : ~282 flops
      x in linear section:   ~5 flops
     \endverbatim
     */
     double calcValue(double x) const;

     /**
      This function solves for the x value of the curve given its y value.
      Since a SmoothSegmentedFunction is not necessarily monotonic lower
      and an upper bound on the argument must be specified so that the
      desired root is found.

      @param y: The range value of interest
      @param xGuess: guess for the value of x
      @return x: the value of x which corresponds to y = f(x) between
                 xLowerBound, and xUpperBound
     */
     double calcInverseValue( double y,
                              double xGuess) const;

     /**Calculates the value of the derivative of the curve this object 
     represents. 

     @param x   The domain point of interest.
     
     @param order The order of the derivative to compute. Note that order must
        be between 0 and 2. Calling 0 just calls calcValue. 
     
     \b aborts \b
    -If anything but 0's are stored in derivComponents
    -If more than the 6th derivative is asked for
    -If ax has a size other than 1
     
     @return The value of the d^ny/dx^n th derivative evaluated at x   
         
    

     <B>Computational Costs</B>   
     \verbatim
      x in curve domain  : ~391 flops
      x in linear section:   ~2 flops   
     \endverbatim
    
     */
     double calcDerivative(double x, int order) const;   

     

     
     /*This will return the value of the integral of this objects curve 
     evaluated at x. 
     
     @param x the domain point of interest

     \b aborts \b
    -If the function does not have a pre-computed integral
     @return the value of the functions integral evaluated at x

     The integral is approximate, though its errors are small.
     The integral is computed by numerically integrating the function when
     the constructor for this class is called (if computeIntegral is true) and 
     then splining the result, thus the regions between the knot points may
     have some error in them. A very fine mesh of points is used to create the
     spline so the errors will be small

     <B>Computational Costs</B>
     \verbatim
      x in curve domain  : ~13 flops
      x in linear section: ~19 flops
     \endverbatim

     */
     //double calcIntegral(double x) const;
     
     /*
    Returns a bool that indicates if the integral curve has been computed.

    @return true if the integral of this function is available, false if
      it has not been computed.
     */
     //bool isIntegralAvailable() const;

     /*
     Returns a bool that indicates if the integral computed is compuated left
     to right, or right to left.

     @return true if the integral was computed left to right, and false if the
       integral was computed right to left. Note that the output of
       this function is only valid if isIntegralAvailable() returns
       true.
     */
     //bool isIntegralComputedLeftToRight() const;

     /**
     Returns a string that is the name for this curve, which is set at the 
     time of construction and cannot be changed after construction.

     @return The string name this object was given during construction*/
     std::string getName() const;

     /**
      Sets the name of the SmoothSegmentedFunction object.
      
      @param name The name of the data this SmoothSegmentedFunction. This name
                  is used to make human-readable error messages and to 
                  generate sensible names when printing the curve to file.
     */
     void setName(const std::string &name);

     /**
     This function returns a SimTK::Vec2 that contains in its 0th element
     the lowest value of the curve domain, and in its 1st element the highest
     value in the curve domain of the curve. Outside of this domain the curve
     is approximated using linear extrapolation.

     @return The minimum and maximum value of the domain, x, of the curve 
        y(x). Within this range y(x) is a curve, outside of this range
        the function y(x) is a C2 (continuous to the second 
        derivative) linear extrapolation*/
     RigidBodyDynamics::Math::VectorNd getCurveDomain() const;

     /**This function will generate a csv file (of 'name_curveName.csv', where 
     name is the one used in the constructor) of the muscle curve, and 
     'curveName' corresponds to the function that was called from
     SmoothSegmentedFunctionFactory to create the curve.
     
     @param path The full path to the location. Note '/' slashes must be used,
      and do not put a '/' after the last folder.
     @param fileNameWithoutExtension The name of the file to write, not 
            including the file extension    
     @param domainMin 
       the left most domain point of the curve to print. The curve
       will extend to at least this point.
     @param domainMax 
       the right most domain point of the curve to print. The 
       printed curve will extend at least to this point, perhaps
       beyond.
    \b aborts \b
      -If the filename is empty

     For example the tendon 
     curve for a muscle named 'glutmax' will be:
     
     'glutmax_tendonForceLengthCurve.csv'

     The file will contain the following columns:
     
     \verbatim
     Col# 1, 2,   3,   4,  5
      x, y, dy/dx, d2y/dx2,  iy
     \endverbatim

     Where iy is the integral of y(x). If the curve has been set not to have
     an integral, this column will not exist.
     
     The curve will be sampled from its linear extrapolation region, through 
     the curve, out to the other linear extrapolation region. The width of 
     each linear extrapolation region is 10% of the entire range of x, or 
     0.1*(x1-x0).

     The number of rows used will vary from curve to curve. Each quintic 
     Bezier curve section will have 100 samples. Each linearily extrapolated
     region will have 10 samples each. Some muscle curves (the tendon, 
     parallel elements, compressive elements) consist of only 1 elbow, and so 
     these matrices will have only 100+20 rows. The force velocity curve is 
     made up of 2 elbows and will have 200+20 rows. The active force length 
     curve has 5 elbows, and so its sampled matrix will have 500+20 rows  

     <B>Computational Costs</B>
     This varies depending on the curve (as mentioned above).
     \verbatim
      ~97,400 to 487,000 flops
     \endverbatim

     <B>Example</B>
     To read the csv file with a header in from Matlab, you need to use 
     csvread set so that it will ignore the header row. This is accomplished
     by using the extra two numerical arguments for csvread to tell the 
     function to begin reading from the 1st row, and the 0th index (csvread
     is 0 indexed).
     \verbatim
    data=csvread('test_tendonForceLengthCurve.csv',1,0);
     \endverbatim

     */
     void printCurveToCSVFile(const std::string& path,
          const std::string& fileNameWithoutExtension,
            double domainMin,
            double domainMax) const;
     

     /**
     @param maxOrder The maximum derivative order to compute
     @param domainMin
     @param domainMax
     \b aborts \b
    -If the requested derivatve order is greater than 
    getMaxDerivativeOrder()
     @returns a matrix populated with x,y,dy/dx ... d^ny/dx^n,iy


     This function will generate a RigidBodyDynamics::Math::MatrixNd populated 
     with samples of the muscle curves values, derivatives (up to 6) and its 
     first integral (if available). The matrix has the following columns:
     
     \verbatim
     Col# 1, 2,   3,   4,   5,   6,   7,   8, 9,
      x, y, dy/dx, d2y/dx2, d3y/dx3, d4y/dx4, d5y/dx5, d6y/dx6, iy
     \endverbatim

     Where iy is the integral of y(x). If the curve has been set not to have
     an integral, this column will not exist.
     
     The curve will be sampled from its 
     linear extrapolation region, through the curve, out to the other linear
     extrapolation region. The width of each linear extrapolation region is
     10% of the entire range of x, or 0.1*(x1-x0).

     The rows used will vary from curve to curve. Each quintic Bezier curve
     section will have 100 samples + 20 samples for the linear extrapolation
     region. Some muscle curves (the tendon, parallel elements, compressive 
     elements) consist of only 1 elbow, and so these matrices will have only 
     100+20 rows. The force velocity curve is made up of 2 elbows and will 
     have 200+20 rows. The active force length curve has 5 elbows, and so its
     sampled matrix will have 500+20 rows  

     */
     RigidBodyDynamics::Math::MatrixNd calcSampledCurve(int maxOrder,
                double domainMin,
                double domainMax) const;


     void getXControlPoints(RigidBodyDynamics::Math::MatrixNd& mat) const;
     void getYControlPoints(RigidBodyDynamics::Math::MatrixNd& mat) const;

    private:
     

    /**Nov 2015: Not needed in the RBDL port.
       Array of spline fit functions X(u) for each Bezier elbow*/
    //Nov 2015: Not needed in the RBDL port.
    //SimTK::Array_<SimTK::Spline> _arraySplineUX;  
    /**Nov 2015: Not included in the RBDL port (RBDL doesn't have an
       error controlled integrator to generate this curve)
       Spline fit of the integral of the curve y(x)*/
    //SimTK::Spline _splineYintX;
    
    /**Bezier X1,...,Xn control point locations. Control points are 
    stored in 6x1 vectors in the order above*/
    std::vector<RigidBodyDynamics::Math::VectorNd> _mXVec; 
    /**Bezier Y1,...,Yn control point locations. Control points are 
    stored in 6x1 vectors in the order above*/
    std::vector<RigidBodyDynamics::Math::VectorNd> _mYVec; 

    /**The number of quintic Bezier curves that describe the relation*/
    int _numBezierSections;

    /**The minimum value of the domain*/
    double _x0;
    /**The maximum value of the domain*/
    double _x1;
    /**The minimum value of the range*/
    double _y0;
    /**The maximum value of the range*/
    double _y1;
    /**The slope at _x0*/
    double _dydx0;
    /**The slope at _x1*/
    double _dydx1;
    /*This is the users */
    //bool _computeIntegral;

    /*This variable, when true, indicates that the user wants the integral
    from left to right (x0 to x1). If it is false, the integral from right
    to left (x1 to x0) is computed*/
    //bool _intx0x1;
    /**The name of the function**/
    std::string _name;
      
    /**No human should be constructing a SmoothSegmentedFunction, so the
    constructor is made private so that mere mortals cannot look at it. 
    SmoothSegmentedFunctionFactory should be used to create 
    MuscleCurveFunctions and that's why its a friend*/
    friend class SmoothSegmentedFunctionFactory;

     



    /**
    This function will print cvs file of the column vector col0 and the 
    matrix data
     
    @param data A matrix of data
    @param colnames Array of column headings
    @param path The desired path to the folder to write the file
    @param filename The name of the file to print
    \b aborts \b
      -If the desired file cannot be created and openened, perhaps 
       because the path doesn't exist.
    */
    void printMatrixToFile( RigidBodyDynamics::Math::MatrixNd& data,
          std::vector<std::string>& colnames,
          const std::string& path, 
          const std::string& filename) const;

     /**
     Refer to the documentation for calcValue(double x) 
     because this function is identical in function to 
     calcValue(double x), but requires different inputs. 
     This is a required virtual function required because this class extends 
     the Function interface.
     */
    double calcValue(const RigidBodyDynamics::Math::VectorNd& x) const; 
    /*virtual*/

    /** Refer to the documentation for calcDerivative(double x, int order) 
     because this function is identical in function to 
     calcDerivative(double x, int order), but requires different inputs. 
     This is a required virtual function required because this class extends 
     the Function interface.*/
     double calcDerivative(const std::vector<int>& derivComponents, 
           const RigidBodyDynamics::Math::VectorNd& x) const; 
           /*virtual*/ 

     /**This will return the size of the vector that the 
     calcValue(const RigidBodyDynamics::Math::VectorNd& x) require. This is a 
     required virtual function required because this class extends the 
     Function interface, though is only needed if you call 
     
     double calcValue(const RigidBodyDynamics::Math::VectorNd& x) const;

     or

     double calcDerivative( const SimTK::Array_<int>& derivComponents, 
          const RigidBodyDynamics::Math::VectorNd& x) const;

    Since this class is implementing strictly scalar functions you can use
    the simplified versions of calcValue(double x) and 
    calcDerivative(double x, int order) instead.

     */
     int getArgumentSize() const; /*virtual*/ 

     /**@return The maximum order derivative that this object is capable of 
     returning*/
     /*virtual*/ 
     int getMaxDerivativeOrder() const;

     
  };

}
}
}



#endif 
