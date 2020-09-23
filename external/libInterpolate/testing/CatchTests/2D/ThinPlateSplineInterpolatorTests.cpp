#include "catch.hpp"

#include <fstream>

#include <libInterpolate/Interpolators/_2D/ThinPlateSplineInterpolator.hpp>

namespace _2D {
  class TestThinPlateSplineInterp : public ThinPlateSplineInterpolator<double>
  {
  };
}


TEST_CASE( "ThinPlateSplineInterpolator Tests - Monotonic Data", "[thin plate spline]" ) {

  _2D::TestThinPlateSplineInterp interp;

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

  _2D::ThinPlateSplineInterpolator<double>::VectorType xx(nx*ny), yy(nx*ny), zz(nx*ny);

  auto f  = [](double x, double y){return x*y + 2*x + 3*y;};

  for( int i = 0; i < nx*ny; i++)
  {
    // gnuplot format is essentially row-major
    xx(i) = xmin+dx*(i/ny);
    yy(i) = ymin+dy*(i%ny);
    zz(i) = f(xx(i),yy(i));
  }
  interp.setData( xx, yy, zz );

  SECTION("Write Data")
  {
    std::ofstream out;

    out.open("TPS-input.txt");
    for(int i = 0; i < nx*ny; i++)
      out << xx(i) << " " << yy(i) << " " << zz(i) << "\n";
    out.close();

    out.open("TPS-output.txt");
    for(int i = 0; i < 5*nx; i++)
    {
      for(int j = 0; j < 5*ny; j++)
      {
        out << xmin + dx*i/5. << " " << ymin + dy*j/5. << " " << interp(xmin+dx*i/5., ymin+dy*j/5.) << "\n";
      }
      out << "\n";
    }
    out.close();

  }



  SECTION("Interpolation")
  {
    CHECK( interp(0,0)+1 == Approx(1+f(0,0)).epsilon(0.00002 )) ;
    CHECK( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
    CHECK( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
    CHECK( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
    CHECK( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

    CHECK( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
    CHECK( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;
  }







}

TEST_CASE( "ThinPlateSplineInterpolator Tests - Oscillating Data", "[thin plate spline]" ) {

  _2D::TestThinPlateSplineInterp interp;

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

  _2D::ThinPlateSplineInterpolator<double>::VectorType xx(nx*ny), yy(nx*ny), zz(nx*ny);

  auto f  = [](double x, double y){return sin(x)*sin(y);};

  for( int i = 0; i < nx*ny; i++)
  {
    // gnuplot format is essentially row-major
    xx(i) = xmin+dx*(i/ny);
    yy(i) = ymin+dy*(i%ny);
    zz(i) = f(xx(i),yy(i));
  }
  interp.setData( xx, yy, zz );

  SECTION("Write Data")
  {
    std::ofstream out;

    out.open("TPS-oscillations-input.txt");
    for(int i = 0; i < nx*ny; i++)
      out << xx(i) << " " << yy(i) << " " << zz(i) << "\n";
    out.close();

    out.open("TPS-oscillations-output.txt");
    for(int i = 0; i < 5*nx; i++)
    {
      for(int j = 0; j < 5*ny; j++)
      {
        out << xmin + dx*i/5. << " " << ymin + dy*j/5. << " " << interp(xmin+dx*i/5., ymin+dy*j/5.) << "\n";
      }
      out << "\n";
    }
    out.close();

  }

  SECTION("Interpolation")
  {
    CHECK( interp(0,0)+1 == Approx(1+f(0,0)).epsilon(0.00002 )) ;
    CHECK( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
    CHECK( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
    CHECK( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
    CHECK( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

    CHECK( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
    CHECK( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;

    CHECK( interp(M_PI/2,M_PI/2)  == Approx(sin(M_PI/2)*sin(M_PI/2)).epsilon(0.02)) ;
  }






}
