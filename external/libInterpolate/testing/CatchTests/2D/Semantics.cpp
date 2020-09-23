#include "catch.hpp"

#include <libInterpolate/Interpolators/_2D/BilinearInterpolator.hpp>
#include <libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp>
#include <libInterpolate/Interpolators/_2D/ThinPlateSplineInterpolator.hpp>

namespace {
  template<class T>
  T
  makeInterpolator(size_t N)
  {
    double dx = 20. / N;
    double dy = 40. / N;
    std::vector<double> xx(N*N), yy(N*N), zz(N*N);
    for( size_t i = 0; i < N*N; i++)
    {
      xx[i] = dx*(i/N);
      yy[i] = dy*(i%N);
      zz[i] = xx[i]*yy[i];
    }
    T interp = T(xx,yy,zz);
    // no copies required here. return value optimization.
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
        for( size_t i = 0; i < 10; i++)
        {
          for( size_t j = 0; j < 10; j++)
          {
            CHECK( interp1(2*i,4*j) == Approx((2*i)*(4*j)) );
          }
        }
      }
      SECTION("Large")
      {
        auto interp2 = makeInterpolator<T>(20);
        for( size_t i = 0; i < 20; i++)
        {
          for( size_t j = 0; j < 20; j++)
          {
            CHECK( interp2(0.2*i,0.4*j) == Approx((0.2*i)*(0.4*j)) );
          }
        }
      }
    }
  }
}

TEST_CASE( "2D - Factory Functions", "[semantics]" )
{


  testFactoryFunctions<_2D::BilinearInterpolator<double>>("Bilinear");
  testFactoryFunctions<_2D::BicubicInterpolator<double>>("Biubic Spline");
  //testFactoryFunctions<_2D::ThinPlateSplineInterpolator<double>>("Thin Plate Spline");


}


namespace {
  template<typename T>
  void
  testCopy1D(std::string testName)
  {
    SECTION(testName)
    {
      std::vector<double> x{1,1,1,2,2,2,3,3,3};
      std::vector<double> y{2,3,4,2,3,4,2,3,4};
      std::vector<double> z{1,2,3,4,5,6,7,8,9};
      for(int i = 0; i < 3*3; i++)
      {
        CHECK( x[i] == Approx((i/3)+1) );
        CHECK( y[i] == Approx((i%3)+2) );
        CHECK( z[i] == Approx(i+1    ) );
      }

      // check that copy constructor and copy assignment operator work.
      T interp1;
      interp1.setData(x,y,z);

      T interp2(interp1);
      T interp3 = interp1;

      for(int i = 0; i < 3; i++)
      {
      for(int j = 0; j < 3; j++)
      {
        CHECK( interp1.getXData()[i] == Approx((i/3)+1) );
        CHECK( interp1.getYData()[i] == Approx((i%3)+2) );
        CHECK( interp1.getZData()[i] == Approx(i+1    ) );
        CHECK( interp2.getXData()[i] == Approx((i/3)+1) );
        CHECK( interp2.getYData()[i] == Approx((i%3)+2) );
        CHECK( interp2.getZData()[i] == Approx(i+1    ) );
        CHECK( interp3.getXData()[i] == Approx((i/3)+1) );
        CHECK( interp3.getYData()[i] == Approx((i%3)+2) );
        CHECK( interp3.getZData()[i] == Approx(i+1    ) );

        CHECK( interp1(i+1,j+2) == Approx(i*3+j+1) );
        CHECK( interp2(i+1,j+2) == Approx(i*3+j+1) );
        CHECK( interp3(i+1,j+2) == Approx(i*3+j+1) );
      }
      }


      // check that changing the data does not affect
      // interpolators
      for(int i = 0; i < 3*3; i++)
      {
        x[i] += 1;
        y[i] += 1;
        z[i] += 1;
      }

      for(int i = 0; i < 3; i++)
      {
      for(int j = 0; j < 3; j++)
      {
        CHECK( interp1.getXData()[i] == Approx((i/3)+1) );
        CHECK( interp1.getYData()[i] == Approx((i%3)+2) );
        CHECK( interp1.getZData()[i] == Approx(i+1    ) );
        CHECK( interp2.getXData()[i] == Approx((i/3)+1) );
        CHECK( interp2.getYData()[i] == Approx((i%3)+2) );
        CHECK( interp2.getZData()[i] == Approx(i+1    ) );
        CHECK( interp3.getXData()[i] == Approx((i/3)+1) );
        CHECK( interp3.getYData()[i] == Approx((i%3)+2) );
        CHECK( interp3.getZData()[i] == Approx(i+1    ) );

        CHECK( interp1(i+1,j+2) == Approx(i*3+j+1) );
        CHECK( interp2(i+1,j+2) == Approx(i*3+j+1) );
        CHECK( interp3(i+1,j+2) == Approx(i*3+j+1) );
      }
      }

      // check that modifying one does not affect the others
      interp2.setData(x,y,z);

      for(int i = 0; i < 3; i++)
      {
      for(int j = 0; j < 3; j++)
      {
        CHECK( interp1.getXData()[i] == Approx((i/3)+1) );
        CHECK( interp1.getYData()[i] == Approx((i%3)+2) );
        CHECK( interp1.getZData()[i] == Approx(i+1    ) );
        CHECK( interp2.getXData()[i] == Approx((i/3)+2) );
        CHECK( interp2.getYData()[i] == Approx((i%3)+3) );
        CHECK( interp2.getZData()[i] == Approx(i+2    ) );
        CHECK( interp3.getXData()[i] == Approx((i/3)+1) );
        CHECK( interp3.getYData()[i] == Approx((i%3)+2) );
        CHECK( interp3.getZData()[i] == Approx(i+1    ) );

        CHECK( interp1(i+1,j+2) == Approx(i*3+j+1) );
        CHECK( interp2(i+2,j+3) == Approx(i*3+j+2) );
        CHECK( interp3(i+1,j+2) == Approx(i*3+j+1) );
      }
      }

      // check copy assignment again
      interp1 = interp2;
      for(int i = 0; i < 3; i++)
      {
      for(int j = 0; j < 3; j++)
      {
        CHECK( interp1.getXData()[i] == Approx((i/3)+2) );
        CHECK( interp1.getYData()[i] == Approx((i%3)+3) );
        CHECK( interp1.getZData()[i] == Approx(i+2    ) );
        CHECK( interp2.getXData()[i] == Approx((i/3)+2) );
        CHECK( interp2.getYData()[i] == Approx((i%3)+3) );
        CHECK( interp2.getZData()[i] == Approx(i+2    ) );
        CHECK( interp3.getXData()[i] == Approx((i/3)+1) );
        CHECK( interp3.getYData()[i] == Approx((i%3)+2) );
        CHECK( interp3.getZData()[i] == Approx(i+1    ) );

        CHECK( interp1(i+2,j+3) == Approx(i*3+j+2) );
        CHECK( interp2(i+2,j+3) == Approx(i*3+j+2) );
        CHECK( interp3(i+1,j+2) == Approx(i*3+j+1) );
      }
      }

      //x{1,1,1,2,2,2,3,3,3};
      //y{2,3,4,2,3,4,2,3,4};
      //z{1,2,3,4,5,6,7,8,9};
      // check interpolation
      CHECK( interp1(2.5,4) == Approx(1+(5+2)/2.) );
      CHECK( interp2(2.5,4) == Approx(1+(5+2)/2.) );
      CHECK( interp3(1.5,3) == Approx(  (5+2)/2.) );

    }
  }
}

TEST_CASE( "2D - Copy Constructor and Copy Assignment", "[semantics]" )
{

  testCopy1D<_2D::BilinearInterpolator<double>>("Bilinear");
  testCopy1D<_2D::BicubicInterpolator<double>>("Bicubic");
  testCopy1D<_2D::ThinPlateSplineInterpolator<double>>("Thin Plate Spline");

}


