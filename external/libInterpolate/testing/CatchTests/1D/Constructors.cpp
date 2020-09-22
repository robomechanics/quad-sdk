#include "catch.hpp"

#include <libInterpolate/Interpolators/_1D/LinearInterpolator.hpp>
#include <libInterpolate/Interpolators/_1D/CubicSplineInterpolator.hpp>
#include <libInterpolate/Interpolators/_1D/MonotonicInterpolator.hpp>


TEST_CASE( "1D - Default construction with setData", "[construction]" )
{

  int N = 100;
  double xmin = 0, xmax = 2*M_PI;
  double dx = (xmax - xmin)/(N-1);

  std::vector<double> xx(N), yy(N);
  for( int i = 0; i < N; i++)
  {
    xx[i] = dx*i;
    yy[i] = sin(xx[i]);
  }

  SECTION("Linear Interpolator")
  {
    _1D::LinearInterpolator<double> interp;
    interp.setData(xx,yy);
    CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );

    _1D::LinearInterpolator<double> interp2(interp);
    CHECK( interp2( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    CHECK( interp2( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    CHECK( interp2( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );
  }
  SECTION("Cubic Spline Interpolator")
  {
    _1D::CubicSplineInterpolator<double> interp;
    interp.setData(xx,yy);
    CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );

    _1D::CubicSplineInterpolator<double> interp2(interp);
    CHECK( interp2( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    CHECK( interp2( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    CHECK( interp2( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );
  }
  SECTION("Monotonic Interpolator")
  {
    _1D::MonotonicInterpolator<double> interp;
    interp.setData(xx,yy);
    CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );

    _1D::MonotonicInterpolator<double> interp2(interp);
    CHECK( interp2( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    CHECK( interp2( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    CHECK( interp2( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );
  }


}


TEST_CASE( "1D - Construction with data", "[construction]" )
{

  int N = 100;
  double xmin = 0, xmax = 2*M_PI;
  double dx = (xmax - xmin)/(N-1);

  std::vector<double> xx(N), yy(N);
  for( int i = 0; i < N; i++)
  {
    xx[i] = dx*i;
    yy[i] = sin(xx[i]);
  }

  SECTION("Linear Interpolator")
  {
    _1D::LinearInterpolator<double> interp(xx,yy);
    CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );
  }
  SECTION("Cubic Spline Interpolator")
  {
    _1D::CubicSplineInterpolator<double> interp(xx,yy);
    CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );
  }
  SECTION("Monotonic Interpolator")
  {
    _1D::MonotonicInterpolator<double> interp(xx,yy);
    CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );
  }


}


