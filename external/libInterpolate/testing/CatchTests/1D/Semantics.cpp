#include "catch.hpp"

#include <libInterpolate/Interpolators/_1D/LinearInterpolator.hpp>
#include <libInterpolate/Interpolators/_1D/CubicSplineInterpolator.hpp>
#include <libInterpolate/Interpolators/_1D/MonotonicInterpolator.hpp>

namespace {
  template<class T>
  T
  makeInterpolator(size_t N)
  {
    double dx = 20. / N;
    std::vector<double> *xx, *yy;
    xx = new std::vector<double>(N);
    yy = new std::vector<double>(N);
    for( size_t i = 0; i < N; i++)
    {
      (*xx)[i] = dx*i;
      (*yy)[i] = 2*(*xx)[i];
    }
    T interp = T(*xx,*yy);
    delete xx;
    delete yy;
    return interp;
  }

  template<typename T>
  void
  testFactoryFunctions(std::string testName)
  {
    SECTION(testName)
    {
      SECTION("Small")
      {
        auto interp1 = makeInterpolator<T>(10);
        for( int i = 0; i < 10; i++)
          CHECK( interp1(2*i) == Approx(2*(2*i)) );
      }
      SECTION("Large")
      {
        auto interp2 = makeInterpolator<T>(100);
        for( int i = 0; i < 100; i++)
          CHECK( interp2(0.2*i) == Approx(2*(0.2*i)) );
      }
    }
  }
}

TEST_CASE( "1D - Factory Functions", "[semantics]" )
{


  testFactoryFunctions<_1D::LinearInterpolator<double>>("Linear");
  testFactoryFunctions<_1D::CubicSplineInterpolator<double>>("Cubic Spline");
  testFactoryFunctions<_1D::MonotonicInterpolator<double>>("Monotonic");


}


namespace {
  template<typename T>
  void
  testCopy1D(std::string testName)
  {
    SECTION(testName)
    {
      std::vector<double> x{1,2,3};
      std::vector<double> y{2,3,4};
      for(int i = 0; i < 3; i++)
      {
        CHECK( x[i] == Approx(i+1) );
        CHECK( y[i] == Approx(i+2) );
      }

      // check that copy constructor and copy assignment operator work.
      T interp1;
      interp1.setData(x,y);

      T interp2(interp1);
      T interp3 = interp1;

      for(int i = 0; i < 3; i++)
      {
        CHECK( interp1.getXData()[i] == Approx(i+1) );
        CHECK( interp1.getYData()[i] == Approx(i+2) );
        CHECK( interp2.getXData()[i] == Approx(i+1) );
        CHECK( interp2.getYData()[i] == Approx(i+2) );
        CHECK( interp3.getXData()[i] == Approx(i+1) );
        CHECK( interp3.getYData()[i] == Approx(i+2) );

        CHECK( interp1(i+1) == Approx(i+2) );
        CHECK( interp2(i+1) == Approx(i+2) );
        CHECK( interp3(i+1) == Approx(i+2) );
      }


      // check that changing the data does not affect
      // interpolators
      for(int i = 0; i < 3; i++)
      {
        x[i] = i+3;
        y[i] = 2*i+4;
      }

      for(int i = 0; i < 3; i++)
      {
        CHECK( interp1.getXData()[i] == Approx(i+1) );
        CHECK( interp1.getYData()[i] == Approx(i+2) );
        CHECK( interp2.getXData()[i] == Approx(i+1) );
        CHECK( interp2.getYData()[i] == Approx(i+2) );
        CHECK( interp3.getXData()[i] == Approx(i+1) );
        CHECK( interp3.getYData()[i] == Approx(i+2) );

        CHECK( interp1(i+1) == Approx(i+2) );
        CHECK( interp2(i+1) == Approx(i+2) );
        CHECK( interp3(i+1) == Approx(i+2) );
      }

      // check that modifying one does not affect the others
      interp2.setData(x,y);

      for(int i = 0; i < 3; i++)
      {
        CHECK( interp1.getXData()[i] == Approx(i+1) );
        CHECK( interp1.getYData()[i] == Approx(i+2) );
        CHECK( interp2.getXData()[i] == Approx(i+3) );
        CHECK( interp2.getYData()[i] == Approx(2*i+4) );
        CHECK( interp3.getXData()[i] == Approx(i+1) );
        CHECK( interp3.getYData()[i] == Approx(i+2) );

        CHECK( interp1(i+1) == Approx(i+2) );
        CHECK( interp2(i+3) == Approx(2*i+4) );
        CHECK( interp3(i+1) == Approx(i+2) );
      }

      // check copy assignment again
      interp1 = interp2;
      for(int i = 0; i < 3; i++)
      {
        CHECK( interp1.getXData()[i] == Approx(i+3) );
        CHECK( interp1.getYData()[i] == Approx(2*i+4) );
        CHECK( interp2.getXData()[i] == Approx(i+3) );
        CHECK( interp2.getYData()[i] == Approx(2*i+4) );
        CHECK( interp3.getXData()[i] == Approx(i+1) );
        CHECK( interp3.getYData()[i] == Approx(i+2) );

        CHECK( interp1(i+3) == Approx(2*i+4) );
        CHECK( interp2(i+3) == Approx(2*i+4) );
        CHECK( interp3(i+1) == Approx(i+2) );
      }

      // check interpolation
      CHECK( interp1(0+0.5+3) == Approx(2*(0+0.5)+4) );
      CHECK( interp2(0+0.5+3) == Approx(2*(0+0.5)+4) );
      CHECK( interp3(0+0.5+1) == Approx((0+0.5)+2) );

      CHECK( interp1(1+0.5+3) == Approx(2*(1+0.5)+4) );
      CHECK( interp2(1+0.5+3) == Approx(2*(1+0.5)+4) );
      CHECK( interp3(1+0.5+1) == Approx((1+0.5)+2) );


    }
  }
}

TEST_CASE( "Copy Constructor and Copy Assignment", "[semantics]" )
{

  testCopy1D<_1D::LinearInterpolator<double>>("1D Linear");
  testCopy1D<_1D::CubicSplineInterpolator<double>>("1D Cubic Spline");

}


