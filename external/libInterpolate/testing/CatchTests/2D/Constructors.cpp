#include "catch.hpp"

#include <libInterpolate/Interpolators/_2D/BilinearInterpolator.hpp>
#include <libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp>
#include <libInterpolate/Interpolators/_2D/ThinPlateSplineInterpolator.hpp>


TEST_CASE( "2D - Default construction with setData", "[construction]" )
{

}

TEST_CASE( "2D - Construction with data", "[construction]" )
{

    int nx, ny;
    double xmin, xmax, dx;
    double ymin, ymax, dy;

    nx = 10;
    ny = 5;

    xmin = -1;
    xmax = 8;

    ymin = -1;
    ymax = 3;

    dx = (xmax - xmin)/(nx - 1);
    dy = (ymax - ymin)/(ny - 1);

    std::vector<double> xx(nx*ny), yy(nx*ny), zz(nx*ny);

    auto f  = [](double x, double y){return x*y + 2*x + 3*y;};

    for( int i = 0; i < nx*ny; i++)
    {
      // gnuplot format is essentially row-major
      xx[i] = xmin+dx*(i/ny);
      yy[i] = ymin+dy*(i%ny);
      zz[i] = f(xx[i],yy[i]);
    }

    SECTION("Bilinear Interpolation")
    {
      _2D::BilinearInterpolator<double> interp(xx, yy, zz);

      REQUIRE( interp(0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
      REQUIRE( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      REQUIRE( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      REQUIRE( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      REQUIRE( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      REQUIRE( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
      REQUIRE( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;


      _2D::BilinearInterpolator<double> interp2(interp);

      REQUIRE( interp2(0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
      REQUIRE( interp2(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      REQUIRE( interp2(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      REQUIRE( interp2(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      REQUIRE( interp2(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      REQUIRE( interp2(-2,-1) == Approx(0).epsilon(0.00002 )) ;
      REQUIRE( interp2(10,3)  == Approx(0).epsilon(0.00002 )) ;
    }

    SECTION("Bicubic Interpolation")
    {
      _2D::BicubicInterpolator<double> interp(xx, yy, zz);

      REQUIRE( interp(0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
      REQUIRE( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      REQUIRE( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      REQUIRE( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      REQUIRE( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      REQUIRE( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
      REQUIRE( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;

      _2D::BicubicInterpolator<double> interp2(interp);

      REQUIRE( interp2(0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
      REQUIRE( interp2(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      REQUIRE( interp2(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      REQUIRE( interp2(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      REQUIRE( interp2(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      REQUIRE( interp2(-2,-1) == Approx(0).epsilon(0.00002 )) ;
      REQUIRE( interp2(10,3)  == Approx(0).epsilon(0.00002 )) ;
    }

    SECTION("Thin Plate Spline Interpolation")
    {
      _2D::ThinPlateSplineInterpolator<double> interp(xx, yy, zz);

      REQUIRE( interp(0,0)+1 == Approx(1+f(0,0)).epsilon(0.00002 )) ;
      REQUIRE( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      REQUIRE( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      REQUIRE( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      REQUIRE( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      REQUIRE( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
      REQUIRE( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;

      _2D::ThinPlateSplineInterpolator<double> interp2(interp);

      REQUIRE( interp2(0,0)+1 == Approx(1+f(0,0)).epsilon(0.00002 )) ;
      REQUIRE( interp2(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      REQUIRE( interp2(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      REQUIRE( interp2(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      REQUIRE( interp2(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      REQUIRE( interp2(-2,-1) == Approx(0).epsilon(0.00002 )) ;
      REQUIRE( interp2(10,3)  == Approx(0).epsilon(0.00002 )) ;
    }

}


