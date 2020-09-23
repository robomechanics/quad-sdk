#include "catch.hpp"

#include <libInterpolate/Interpolators/_1D/LinearInterpolator.hpp>
#include <libInterpolate/Interpolators/_1D/CubicSplineInterpolator.hpp>
#include <libInterpolate/Interpolators/_1D/MonotonicInterpolator.hpp>

TEST_CASE( "Runtime Binding w/ std::function Tests", "[polymorphism]" ) {

  std::function<double(double)> interp;

  SECTION("Linear Interpolation")
  {
    std::vector<double> x,y;

    x.push_back(0); y.push_back(-1);
    x.push_back(5); y.push_back(9);  // slope of 2
    x.push_back(10); y.push_back(14);  // slope of 1 

    interp = _1D::LinearInterpolator<double>();
    interp.target<_1D::LinearInterpolator<double>>()->setData(x,y);

    //CHECK( interp( -1 ) == Approx( -3 ) );
    CHECK( interp(  0 ) == Approx( -1 ) );
    CHECK( interp(  1 ) == Approx(  1 ) );
    CHECK( interp(  2 ) == Approx(  3 ) );
    CHECK( interp(  3 ) == Approx(  5 ) );
    CHECK( interp(  4 ) == Approx(  7 ) );
    CHECK( interp(  5 ) == Approx(  9 ) );
    CHECK( interp(  6 ) == Approx( 10 ) );
    CHECK( interp(  7 ) == Approx( 11 ) );
    CHECK( interp(  8 ) == Approx( 12 ) );
    CHECK( interp(  9 ) == Approx( 13 ) );
    CHECK( interp( 10 ) == Approx( 14 ) );
    //CHECK( interp( 11 ) == Approx(  0 ) );
  }

  SECTION("Cubic Spline Interpolation")
  {
    int N = 100;
    double xmin = 0, xmax = 2*M_PI;
    double dx = (xmax - xmin)/(N-1);

    interp = _1D::CubicSplineInterpolator<double>();

    CHECK_THROWS_AS( interp(1), std::logic_error );

    std::vector<double> xx(N), yy(N);
    for( int i = 0; i < N; i++)
    {
      xx[i] = dx*i;
      yy[i] = sin(xx[i]);
    }

    interp.target<_1D::CubicSplineInterpolator<double>>()->setData( xx, yy );

    REQUIRE_NOTHROW( interp(1) );


    CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ) );
    CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ) );
    CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ) );
  }


  SECTION("Monotonic Interpolation")
  {
    int N = 100;
    double xmin = 0, xmax = 2*M_PI;
    double dx = (xmax - xmin)/(N-1);

    interp = _1D::MonotonicInterpolator<double>();

    CHECK_THROWS_AS( interp(1), std::logic_error );

    std::vector<double> xx(N), yy(N);
    for( int i = 0; i < N; i++)
    {
      xx[i] = dx*i;
      yy[i] = sin(xx[i]);
    }
    interp.target<_1D::MonotonicInterpolator<double>>()->setData( xx, yy );

    REQUIRE_NOTHROW( interp(1) );

    CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );

  }


}

