#include "catch.hpp"

#include <libInterpolate/Interpolators/_2D/BilinearInterpolator.hpp>
#include <libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp>
#include <libInterpolate/Interpolators/_2D/ThinPlateSplineInterpolator.hpp>

namespace {
  double call( std::function<double(double,double)> f, double x, double y)
  { return f(x,y); }
}

TEST_CASE( "2D Runtime Binding w/ std::function Tests", "[polymorphism]" ) {

  std::function<double(double,double)> interp;

  SECTION("Bilinear Interpolation")
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
    interp.target<_2D::BilinearInterpolator<double>>()->setData( xx, yy, zz );

    REQUIRE( interp(0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
    REQUIRE( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
    REQUIRE( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
    REQUIRE( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
    REQUIRE( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

    REQUIRE( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
    REQUIRE( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;

    SECTION("via copied std::function passed to func")
    {
      REQUIRE( call( interp,0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
      REQUIRE( call( interp,1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      REQUIRE( call( interp,2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      REQUIRE( call( interp,2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      REQUIRE( call( interp,8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      REQUIRE( call( interp,-2,-1) == Approx(0).epsilon(0.00002 )) ;
      REQUIRE( call( interp,10,3)  == Approx(0).epsilon(0.00002 )) ;
    }

  }

  SECTION("Bicubic Interpolation")
  {

    int nx, ny;
    double xmin, xmax, dx;
    double ymin, ymax, dy;

    SECTION("Monotomic Data")
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
      interp.target<_2D::BicubicInterpolator<double>>()->setData( xx, yy, zz );

      CHECK( interp(0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
      CHECK( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      CHECK( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      CHECK( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      CHECK( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      CHECK( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
      CHECK( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;

      SECTION("via copied std::function passed to func")
      {
        REQUIRE( call( interp,0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
        REQUIRE( call( interp,1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
        REQUIRE( call( interp,2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
        REQUIRE( call( interp,2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
        REQUIRE( call( interp,8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

        REQUIRE( call( interp,-2,-1) == Approx(0).epsilon(0.00002 )) ;
        REQUIRE( call( interp,10,3)  == Approx(0).epsilon(0.00002 )) ;
      }

    }

    SECTION("Oscillating Data")
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

      auto f  = [](double x, double y){return sin(x)*sin(y);};

      for( int i = 0; i < nx*ny; i++)
      {
        // gnuplot format is essentially row-major
        xx(i) = xmin+dx*(i/ny);
        yy(i) = ymin+dy*(i%ny);
        zz(i) = f(xx(i),yy(i));
      }


       
      interp = _2D::BicubicInterpolator<double>();
      interp.target<_2D::BicubicInterpolator<double>>()->setData( xx, yy, zz );

      CHECK( interp(0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
      CHECK( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      CHECK( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      CHECK( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      CHECK( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      CHECK( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
      CHECK( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;
    }


  }


  SECTION("Thin Plate Spline Interpolation")
  {

    int nx, ny;
    double xmin, xmax, dx;
    double ymin, ymax, dy;

    SECTION("Monotomic Data")
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
      interp.target<_2D::ThinPlateSplineInterpolator<double>>()->setData( xx, yy, zz );

      CHECK( interp(0,0)+1 == Approx(1+f(0,0)).epsilon(0.00002 )) ;
      CHECK( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      CHECK( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      CHECK( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      CHECK( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      CHECK( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
      CHECK( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;

      SECTION("via copied std::function passed to func")
      {
        REQUIRE( call( interp,0,0)+1 == Approx(1+f(0,0)).epsilon(0.00002 )) ;
        REQUIRE( call( interp,1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
        REQUIRE( call( interp,2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
        REQUIRE( call( interp,2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
        REQUIRE( call( interp,8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

        REQUIRE( call( interp,-2,-1) == Approx(0).epsilon(0.00002 )) ;
        REQUIRE( call( interp,10,3)  == Approx(0).epsilon(0.00002 )) ;
      }

    }

    SECTION("Oscillating Data")
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

      auto f  = [](double x, double y){return sin(x)*sin(y);};

      for( int i = 0; i < nx*ny; i++)
      {
        // gnuplot format is essentially row-major
        xx(i) = xmin+dx*(i/ny);
        yy(i) = ymin+dy*(i%ny);
        zz(i) = f(xx(i),yy(i));
      }


       
      interp = _2D::ThinPlateSplineInterpolator<double>();
      interp.target<_2D::ThinPlateSplineInterpolator<double>>()->setData( xx, yy, zz );

      CHECK( interp(0,0)+1 == Approx(1+f(0,0)).epsilon(0.00002 )) ;
      CHECK( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
      CHECK( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
      CHECK( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
      CHECK( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

      CHECK( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
      CHECK( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;
    }


  }



}

