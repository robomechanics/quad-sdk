#include "catch.hpp"

#include <libInterpolate/Interpolators/_1D/LinearInterpolator.hpp>
#include <libInterpolate/Interpolators/_1D/CubicSplineInterpolator.hpp>
#include <libInterpolate/Interpolators/_1D/MonotonicInterpolator.hpp>
#include <libInterpolate/Interpolators/_1D/AnyInterpolator.hpp>

TEST_CASE( "Runtime Binding w/ _1D::AnyInterpolator Tests", "[polymorphism]" )
{
  SECTION("AnyInterpolator can store a LinearInterpolator")
  {

    std::vector<double> x,y;

    x.push_back(0); y.push_back(-1);
    x.push_back(5); y.push_back(9);  // slope of 2
    x.push_back(10); y.push_back(14);  // slope of 1 

    SECTION("With default setData Signature")
    {
      _1D::AnyInterpolator<double> interp = _1D::LinearInterpolator<double>();
      interp.setData(x.size(),x.data(),y.data());

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
    }

    SECTION("With user-defiend setData Signature")
    {
      _1D::AnyInterpolator<double, void(std::vector<double>,std::vector<double>)> interp = _1D::LinearInterpolator<double>();
      interp.setData(x,y);

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
    }
  }


  SECTION("AnyInterpolator can store a CubicSplineInterpolation")
  {
    int N = 100;
    double xmin = 0, xmax = 2*M_PI;
    double dx = (xmax - xmin)/(N-1);

    SECTION("With default setData Signature")
    {
      _1D::AnyInterpolator<double> interp = _1D::CubicSplineInterpolator<double>();

      CHECK_THROWS_AS( interp(1), std::logic_error );

      std::vector<double> xx(N), yy(N);
      for( int i = 0; i < N; i++)
      {
        xx[i] = dx*i;
        yy[i] = sin(xx[i]);
      }

      interp.setData( xx.size(), xx.data(), yy.data() );

      REQUIRE_NOTHROW( interp(1) );


      CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ) );
      CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ) );
      CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ) );
    }

    SECTION("With user-defined setData Signature")
    {
      _1D::AnyInterpolator<double, void(std::vector<double>,std::vector<double> )> interp = _1D::CubicSplineInterpolator<double>();

      CHECK_THROWS_AS( interp(1), std::logic_error );

      std::vector<double> xx(N), yy(N);
      for( int i = 0; i < N; i++)
      {
        xx[i] = dx*i;
        yy[i] = sin(xx[i]);
      }

      interp.setData( xx, yy );

      REQUIRE_NOTHROW( interp(1) );


      CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ) );
      CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ) );
      CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ) );
    }
  }


  SECTION("AnyInterpolator can store a MonotonicInterpolation")
  {
    int N = 100;
    double xmin = 0, xmax = 2*M_PI;
    double dx = (xmax - xmin)/(N-1);

    SECTION("With default setData Signature")
    {
      _1D::AnyInterpolator<double> interp = _1D::MonotonicInterpolator<double>();

      CHECK_THROWS_AS( interp(1), std::logic_error );

      std::vector<double> xx(N), yy(N);
      for( int i = 0; i < N; i++)
      {
        xx[i] = dx*i;
        yy[i] = sin(xx[i]);
      }
      interp.setData( xx.size(), xx.data(), yy.data() );

      REQUIRE_NOTHROW( interp(1) );

      CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
      CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
      CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );
    }

    SECTION("With user-defined setData Signature")
    {
      _1D::AnyInterpolator<double, void(std::vector<double>,std::vector<double>)> interp = _1D::MonotonicInterpolator<double>();

      CHECK_THROWS_AS( interp(1), std::logic_error );

      std::vector<double> xx(N), yy(N);
      for( int i = 0; i < N; i++)
      {
        xx[i] = dx*i;
        yy[i] = sin(xx[i]);
      }
      interp.setData( xx, yy );

      REQUIRE_NOTHROW( interp(1) );

      CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
      CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
      CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );
    }

  }

  SECTION("AnyInterpolator implementation can be changed at runtime")
  {

    std::vector<double> x,y;

    x.push_back(0); y.push_back(-1);
    x.push_back(5); y.push_back(9);  // slope of 2
    x.push_back(10); y.push_back(14);  // slope of 1 


    _1D::AnyInterpolator<double> interp = _1D::MonotonicInterpolator<double>();

    interp = _1D::CubicSplineInterpolator<double>();

    interp = _1D::LinearInterpolator<double>();

    interp.setData(x.size(), x.data(), y.data());

    CHECK( interp(  1 ) == Approx(  1 ) );
    CHECK( interp(  2 ) == Approx(  3 ) );


  }


}
