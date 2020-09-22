#include "catch.hpp"

#include <libInterpolate/Interpolators/_2D/BilinearInterpolator.hpp>
#include <libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp>
#include <libInterpolate/Interpolators/_2D/ThinPlateSplineInterpolator.hpp>
#include <libInterpolate/Interpolators/_2D/AnyInterpolator.hpp>

TEST_CASE( "Runtime Binding w/ _2D::AnyInterpolator Tests", "[polymorphism]" )
{
  _2D::AnyInterpolator<double> interp;

  int nx, ny;
  double xmin, xmax, dx;
  double ymin, ymax, dy;

  SECTION("AnyInterpolator can store a BilinearInterpolator")
  {
    nx = 10;
    ny = 5;

    xmin = -1;
    xmax = 8;

    ymin = -1;
    ymax = 3;

    dx = (xmax - xmin)/(nx - 1);
    dy = (ymax - ymin)/(ny - 1);


    Eigen::Matrix<double,Eigen::Dynamic,1> xx(nx*ny), yy(nx*ny), zz(nx*ny);

    auto f  = [](double x, double y){return x*y + 2*x + 3*y;};

    for( int i = 0; i < nx*ny; i++)
    {
      // gnuplot format is essentially row-major
      xx(i) = xmin+dx*(i/ny);
      yy(i) = ymin+dy*(i%ny);
      zz(i) = f(xx(i),yy(i));
    }

    interp = _2D::BilinearInterpolator<double>();
    interp.setData( xx.size(), xx.data(), yy.data(), zz.data() );

    REQUIRE( interp(0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
    REQUIRE( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
    REQUIRE( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
    REQUIRE( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
    REQUIRE( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

    REQUIRE( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
    REQUIRE( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;
  }

  SECTION("AnyInterpolator can store a BicubicInterpolator")
  {
    nx = 10;
    ny = 5;

    xmin = -1;
    xmax = 8;

    ymin = -1;
    ymax = 3;

    dx = (xmax - xmin)/(nx - 1);
    dy = (ymax - ymin)/(ny - 1);

    Eigen::Matrix<double,Eigen::Dynamic,1> xx(nx*ny), yy(nx*ny), zz(nx*ny);

    auto f  = [](double x, double y){return x*y + 2*x + 3*y;};

    for( int i = 0; i < nx*ny; i++)
    {
      // gnuplot format is essentially row-major
      xx(i) = xmin+dx*(i/ny);
      yy(i) = ymin+dy*(i%ny);
      zz(i) = f(xx(i),yy(i));
    }

    interp = _2D::BicubicInterpolator<double>();
    interp.setData( xx.size(), xx.data(), yy.data(), zz.data() );

    CHECK( interp(0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
    CHECK( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
    CHECK( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
    CHECK( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
    CHECK( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

    CHECK( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
    CHECK( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;








  }
  SECTION("AnyInterpolator can store a ThinPlateSplineInterpolator")
  {
      nx = 10;
      ny = 5;

      xmin = -1;
      xmax = 8;

      ymin = -1;
      ymax = 3;

      dx = (xmax - xmin)/(nx - 1);
      dy = (ymax - ymin)/(ny - 1);

      Eigen::Matrix<double,Eigen::Dynamic,1> xx(nx*ny), yy(nx*ny), zz(nx*ny);

      auto f  = [](double x, double y){return x*y + 2*x + 3*y;};

      for( int i = 0; i < nx*ny; i++)
      {
        // gnuplot format is essentially row-major
        xx(i) = xmin+dx*(i/ny);
        yy(i) = ymin+dy*(i%ny);
        zz(i) = f(xx(i),yy(i));
      }

      interp = _2D::ThinPlateSplineInterpolator<double>();
      interp.setData( xx.size(), xx.data(), yy.data(), zz.data() );

      CHECK( interp(0,0)+1 == Approx(1+f(0,0)).epsilon(0.00002 )) ;
      CHECK( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      CHECK( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      CHECK( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      CHECK( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      CHECK( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
      CHECK( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;

  }


}
