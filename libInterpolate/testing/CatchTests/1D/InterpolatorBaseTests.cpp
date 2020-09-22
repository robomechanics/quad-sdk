#include "catch.hpp"

#include <libInterpolate/Interpolators/_1D/InterpolatorBase.hpp>


namespace _1D {

class TestInterp : public InterpolatorBase<TestInterp>
{
  public:
    double operator()( double x ) const
    {
      return x + 10;
    }

    VectorType getX() { return *(this->xView); }
    VectorType getY() { return *(this->yView); }

};

}


TEST_CASE( "InterpolatorBase Setup Tests", "[plumbing]" ) {

  _1D::TestInterp interp;

  // check that the interpolator works as expected
  REQUIRE( interp(1) == Approx(11) );
  REQUIRE( interp(10) == Approx(20) );

  size_t N = 10;

  SECTION("Eigen Vector Initialization")
  {
    Eigen::Matrix<double,Eigen::Dynamic,1> xx(N), yy(N);

    for( size_t i = 0; i < N; i++)
    {
      xx(i) = 0.1*i;
      yy(i) = xx(i)*xx(i);
    }

    SECTION("Deep Copy")
    {
      interp.setData( xx, yy );

      // clear the original data to make sure deep copy worked
      for(size_t i = 0; i < N; i++)
        xx(i) = yy(i) = 0;

      auto x = interp.getX();
      auto y = interp.getY();

      for( size_t i = 0; i < N; i++)
      {
        REQUIRE( x(i) == Approx( 0.1*i ) );
        REQUIRE( y(i) == Approx( x(i)*x(i) ) );
      }
    }

    // No longer supported. Is "trickier" than one might think...
    //SECTION("Shallow Copy")
    //{
      //interp.setData( xx, yy, false );

      //// clear the original data to check that shallow copy worked
      //for(int i = 0; i < N; i++)
        //xx(i) = yy(i) = 0;

      //auto x = interp.getX();
      //auto y = interp.getY();

      //for( int i = 0; i < N; i++)
      //{
        //REQUIRE( x(i) == Approx( 0.0 ) );
        //REQUIRE( y(i) == Approx( 0.0 ) );
      //}
    //}

  }

  SECTION("Std Vector Initialization")
  {
    std::vector<double> xx(N), yy(N);

    for( size_t i = 0; i < N; i++)
    {
      xx[i] = 0.1*i;
      yy[i] = xx[i]*xx[i];
    }

    SECTION("Deep Copy")
    {
      interp.setData( xx, yy );

      // clear the original data to make sure deep copy worked
      for(size_t i = 0; i < N; i++)
        xx[i] = yy[i] = 0;

      auto x = interp.getX();
      auto y = interp.getY();

      for( size_t i = 0; i < N; i++)
      {
        REQUIRE( x(i) == Approx( 0.1*i ) );
        REQUIRE( y(i) == Approx( x(i)*x(i) ) );
      }
    }

    //SECTION("Shallow Copy")
    //{
      //interp.setData( xx, yy, false );

      //// clear the original data to check that shallow copy worked
      //for(int i = 0; i < N; i++)
        //xx[i] = yy[i] = 0;

      //auto x = interp.getX();
      //auto y = interp.getY();

      //for( int i = 0; i < N; i++)
      //{
        //REQUIRE( x(i) == Approx( 0.0 ) );
        //REQUIRE( y(i) == Approx( 0.0 ) );
      //}
    //}

  }

  SECTION("Raw Pointer Initialization")
  {
    double *xx, *yy;
    xx = new double[N];
    yy = new double[N];


    for( size_t i = 0; i < N; i++)
    {
      xx[i] = 0.1*i;
      yy[i] = xx[i]*xx[i];
    }

    SECTION("Deep Copy")
    {
      interp.setData( N, xx, yy );

      // clear the original data to make sure deep copy worked
      for(size_t i = 0; i < N; i++)
        xx[i] = yy[i] = 0;

      auto x = interp.getX();
      auto y = interp.getY();

      for( size_t i = 0; i < N; i++)
      {
        REQUIRE( x(i) == Approx( 0.1*i ) );
        REQUIRE( y(i) == Approx( x(i)*x(i) ) );
      }
    }

    //SECTION("Shallow Copy")
    //{
      //interp.setData( N, xx, yy, false );

      //// clear the original data to check that shallow copy worked
      //for(int i = 0; i < N; i++)
      //for(int i = 0; i < N; i++)
        //xx[i] = yy[i] = 0;

      //auto x = interp.getX();
      //auto y = interp.getY();

      //for( int i = 0; i < N; i++)
      //{
        //REQUIRE( x(i) == Approx( 0.0 ) );
        //REQUIRE( y(i) == Approx( 0.0 ) );
      //}
    //}

    delete[] xx;
    delete[] yy;
  }

  SECTION("Data Get Functions")
  {
    Eigen::Matrix<double,Eigen::Dynamic,1> xx(N), yy(N);

    for( size_t i = 0; i < N; i++)
    {
      xx(i) = 0.1*i;
      yy(i) = xx(i)*xx(i);
    }

    interp.setData( xx, yy );

    auto x = interp.getXData();
    auto y = interp.getYData();

    REQUIRE( x.size() > 0 );
    CHECK( x[0] == Approx(0) );
    REQUIRE( x.size() == N );
    CHECK( x[N-1] == Approx( xx(N-1) ) );

    REQUIRE( y.size() > 0 );
    CHECK( y[0] == Approx(0) );
    REQUIRE( y.size() == N );
    CHECK( y[N-1] == Approx( pow(xx(N-1),2) ) );

  }

}


