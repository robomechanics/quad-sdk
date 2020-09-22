#include "catch.hpp"

#include <libInterpolate/Interpolators/_2D/InterpolatorBase.hpp>


namespace _2D {

class TestInterp : public InterpolatorBase<TestInterp>
{
  public:
    virtual double operator()( double x, double y ) const
    {
      return x + 2*y + 10;
    }

    VectorType getX() { return *(this->xView); }
    VectorType getY() { return *(this->yView); }
    VectorType getZ() { return *(this->zView); }

};

}


TEST_CASE( "2D InterpolatorBase Setup Tests", "[plumbing]" ) {

  _2D::TestInterp interp;

  // make sure interpolator works the way we expect
  REQUIRE( interp(1,1) == Approx(13) );
  REQUIRE( interp(10,20) == Approx(60) );

  size_t N = 10;

  SECTION("Eigen Vector Initialization")
  {
    _2D::InterpolatorBase<double>::VectorType xx(N), yy(N), zz(N);

    for( size_t i = 0; i < N; i++)
    {
      xx(i) = 0.1*i;
      yy(i) = 0.2*i;
      zz(i) = xx(i) + yy(i);
    }

    SECTION("Deep Copy")
    {
      interp.setData( xx, yy, zz );

      // clear the original data to make sure deep copy worked
      for(size_t i = 0; i < N; i++)
        xx(i) = yy(i) = 0;

      auto x = interp.getX();
      auto y = interp.getY();
      auto z = interp.getZ();

      for( size_t i = 0; i < N; i++)
      {
        REQUIRE( x(i) == Approx( 0.1*i ) );
        REQUIRE( y(i) == Approx( 0.2*i ) );
        REQUIRE( z(i) == Approx( x(i)+y(i) ) );
      }
    }

    //SECTION("Shallow Copy")
    //{
      //interp.setData( xx, yy, zz, false );

      //// clear the original data to check that shallow copy worked
      //for(int i = 0; i < N; i++)
        //xx(i) = yy(i) = zz(i) = 0;

      //auto x = interp.getX();
      //auto y = interp.getY();
      //auto z = interp.getZ();

      //for( int i = 0; i < N; i++)
      //{
        //REQUIRE( x(i) == Approx( 0.0 ) );
        //REQUIRE( y(i) == Approx( 0.0 ) );
        //REQUIRE( z(i) == Approx( 0.0 ) );
      //}
    //}

  }

  SECTION("Std Vector Initialization")
  {
    std::vector<double> xx(N), yy(N), zz(N);

    for( size_t i = 0; i < N; i++)
    {
      xx[i] = 0.1*i;
      yy[i] = 0.2*i;
      zz[i] = xx[i] + yy[i];
    }

    SECTION("Deep Copy")
    {
      interp.setData( xx, yy, zz );

      // clear the original data to make sure deep copy worked
      for(size_t i = 0; i < N; i++)
        xx[i] = yy[i] = zz[i] =  0;

      auto x = interp.getX();
      auto y = interp.getY();
      auto z = interp.getZ();

      for( size_t i = 0; i < N; i++)
      {
        REQUIRE( x(i) == Approx( 0.1*i ) );
        REQUIRE( y(i) == Approx( 0.2*i ) );
        REQUIRE( z(i) == Approx( x(i) + y(i) ) );
      }
    }

    //SECTION("Shallow Copy")
    //{
      //interp.setData( xx, yy, zz, false );

      //// clear the original data to check that shallow copy worked
      //for(int i = 0; i < N; i++)
        //xx[i] = yy[i] = zz[i] = 0;

      //auto x = interp.getX();
      //auto y = interp.getY();
      //auto z = interp.getZ();

      //for( int i = 0; i < N; i++)
      //{
        //REQUIRE( x(i) == Approx( 0.0 ) );
        //REQUIRE( y(i) == Approx( 0.0 ) );
        //REQUIRE( z(i) == Approx( 0.0 ) );
      //}
    //}

  }

  SECTION("Raw Pointer Initialization")
  {
    double *xx, *yy, *zz;
    xx = new double[N];
    yy = new double[N];
    zz = new double[N];


    for( size_t i = 0; i < N; i++)
    {
      xx[i] = 0.1*i;
      yy[i] = 0.2*i;
      zz[i] = xx[i] + yy[i];
    }

    SECTION("Deep Copy")
    {
      interp.setData( N, xx, yy, zz );

      // clear the original data to make sure deep copy worked
      for(size_t i = 0; i < N; i++)
        xx[i] = yy[i] = zz[i] = 0;

      auto x = interp.getX();
      auto y = interp.getY();
      auto z = interp.getZ();

      for( size_t i = 0; i < N; i++)
      {
        REQUIRE( x(i) == Approx( 0.1*i ) );
        REQUIRE( y(i) == Approx( 0.2*i ) );
        REQUIRE( z(i) == Approx( x(i)+y(i) ) );
      }
    }

    //SECTION("Shallow Copy")
    //{
      //interp.setData( N, xx, yy, zz, false );

      //// clear the original data to check that shallow copy worked
      //for(int i = 0; i < N; i++)
      //for(int i = 0; i < N; i++)
        //xx[i] = yy[i] = zz[i] = 0;

      //auto x = interp.getX();
      //auto y = interp.getY();
      //auto z = interp.getZ();

      //for( int i = 0; i < N; i++)
      //{
        //REQUIRE( x(i) == Approx( 0.0 ) );
        //REQUIRE( y(i) == Approx( 0.0 ) );
        //REQUIRE( z(i) == Approx( 0.0 ) );
      //}
    //}

    delete[] xx;
    delete[] yy;
    delete[] zz;
  }

  SECTION("Eigen Vector Initialization")
  {
    _2D::InterpolatorBase<double>::VectorType xx(N), yy(N), zz(N);

    for( size_t i = 0; i < N; i++)
    {
      xx(i) = 0.1*i;
      yy(i) = 0.2*i;
      zz(i) = xx(i) + yy(i);
    }

    interp.setData( xx, yy, zz );

    auto x = interp.getXData();
    auto y = interp.getYData();
    auto z = interp.getZData();

    REQUIRE( x.size() > 0 );
    CHECK( x[0] == Approx(0) );
    REQUIRE( x.size() == N );
    CHECK( x[N-1] == Approx( xx(N-1) ) );

    REQUIRE( y.size() > 0 );
    CHECK( y[0] == Approx(0) );
    REQUIRE( y.size() == N );
    CHECK( y[N-1] == Approx( yy(N-1) ) );

    REQUIRE( z.size() > 0 );
    CHECK( z[0] == Approx(0) );
    REQUIRE( z.size() == N );
    CHECK( z[N-1] == Approx( xx(N-1) + yy(N-1) ) );



  }

}


