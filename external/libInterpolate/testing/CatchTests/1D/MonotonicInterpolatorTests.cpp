#include "catch.hpp"

#include <libInterpolate/Interpolators/_1D/MonotonicInterpolator.hpp>


TEST_CASE( "MonotonicInterpolator Tests", "[monotonic]" ) {

  SECTION("Double Precision")
  {
  _1D::MonotonicInterpolator<double> interp;

  int N = 100;
  double xmin = 0, xmax = 2*M_PI;
  double dx = (xmax - xmin)/(N-1);

  _1D::MonotonicInterpolator<double>::VectorType xx(N), yy(N);

  CHECK_THROWS_AS( interp(1), std::logic_error );

  for( int i = 0; i < N; i++)
  {
    xx(i) = dx*i;
    yy(i) = sin(xx(i));
  }
  interp.setData( xx, yy );

  REQUIRE_NOTHROW( interp(1) );


  SECTION("Interpolation")
  {
    CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );
  }
  }


  //SECTION("Single Precision")
  //{
  //_1D::MonotonicInterpolator<float> interp;

  //int N = 100;
  //float xmin = 0, xmax = 2*M_PI;
  //float dx = (xmax - xmin)/(N-1);

  //_1D::MonotonicInterpolator<float>::VectorType xx(N), yy(N);

  //CHECK_THROWS_AS( interp(1), std::logic_error );

  //for( int i = 0; i < N; i++)
  //{
    //xx(i) = dx*i;
    //yy(i) = sin(xx(i));
  //}
  //interp.setData( xx, yy );

  //REQUIRE_NOTHROW( interp(1) );


  //SECTION("Interpolation")
  //{
    //CHECK( interp( dx/2          ) == Approx( sin( dx/2          ) ).epsilon(0.001) );
    //CHECK( interp( M_PI/2 - dx/2 ) == Approx( sin( M_PI/2 - dx/2 ) ).epsilon(0.001) );
    //CHECK( interp( M_PI/4 - dx/4 ) == Approx( sin( M_PI/4 - dx/4 ) ).epsilon(0.001) );
  //}

  //}

}

TEST_CASE("MonotonicInterpolator Edge Case Tests")
{
  _1D::MonotonicInterpolator<double> interp;

  std::vector<double> x, y;
  x.push_back( 1.0 ); y.push_back( 2.0 );
  x.push_back( 2.0 ); y.push_back( 8.0 );
  x.push_back( 3.0 ); y.push_back(16.0 );
  x.push_back( 4.0 ); y.push_back(64.0 );

  interp.setData(x,y);

  REQUIRE_NOTHROW( interp(1) );

  CHECK( interp(1.0) == Approx( 2.0));
  CHECK( interp(2.0) == Approx( 8.0));
  CHECK( interp(3.0) == Approx(16.0));
  CHECK( interp(4.0) == Approx(64.0));

}
