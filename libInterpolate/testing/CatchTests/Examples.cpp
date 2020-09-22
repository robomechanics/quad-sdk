#include "catch.hpp"

// This file contains examples that demonstrate how to write simple tests.

#include<vector>
#include<string>

#include<libInterpolate/Interpolate.hpp>
#include<libInterpolate/AnyInterpolator.hpp>

unsigned int Factorial( unsigned int number ) {
    return number <= 1 ? number : Factorial(number-1)*number;
}






TEST_CASE("Runtime dispatch with AnyInterpolator")
{

  std::string method;
  std::vector<double> x,y;

  x.push_back(0);
  x.push_back(1);
  x.push_back(2);

  y.push_back(10);
  y.push_back(20);
  y.push_back(30);

  method = "linear";

  _1D::AnyInterpolator<double> interp;

  // select the interpolation method on user input
  if( method == "linear")
    interp = _1D::LinearInterpolator<double>();
  if( method == "cubicspline")
    interp = _1D::CubicSplineInterpolator<double>();
  if( method == "monotonic")
    interp = _1D::MonotonicInterpolator<double>();

  interp.setData( x.size(), x.data(), y.data() );

  // interpolation is done with the operator() method.
  double val = interp(1.5);

  CHECK(val == Approx(25));
}

