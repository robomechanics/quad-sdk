#include "catch.hpp"

#include <fstream>

#include <libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp>

namespace _2D {
  class TestBicubicInterp : public BicubicInterpolator<double>
  {
    public:
      VectorType getX() { return *(this->X); }
      VectorType getY() { return *(this->Y); }
      MatrixType getZ() { return *(this->Z); }
  };
}


TEST_CASE( "BicubicInterpolator Tests - Monotonic Data", "[bicubic]" ) {

  _2D::TestBicubicInterp interp;

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

  _2D::BicubicInterpolator<double>::VectorType xx(nx*ny), yy(nx*ny), zz(nx*ny);

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

    out.open("Bicubic-input.txt");
    for(int i = 0; i < nx*ny; i++)
      out << xx(i) << " " << yy(i) << " " << zz(i) << "\n";
    out.close();

    out.open("Bicubic-output.txt");
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

  SECTION("1D -> 2D Mappings")
  {
    auto X = interp.getX();
    auto Y = interp.getY();
    auto Z = interp.getZ();

    REQUIRE( X.size() == nx );
    CHECK( X(0) == Approx( xmin+0    ) );
    CHECK( X(2) == Approx( xmin+2*dx ) );
    CHECK( X(4) == Approx( xmin+4*dx ) );

    REQUIRE( Y.size() == ny );
    CHECK( Y(0) == Approx( ymin+0    ) );
    CHECK( Y(2) == Approx( ymin+2*dy ) );
    CHECK( Y(4) == Approx( ymin+4*dy ) );

    REQUIRE( Z.size() == nx*ny );
    CHECK( Z(0,0) == Approx( f(xmin+0*dx,ymin+0*dy) ) );
    CHECK( Z(0,2) == Approx( f(xmin+0*dx,ymin+2*dy) ) );
    CHECK( Z(2,0) == Approx( f(xmin+2*dx,ymin+0*dy) ) );
    CHECK( Z(2,2) == Approx( f(xmin+2*dx,ymin+2*dy) ) );
  }


  SECTION("Interpolation")
  {
    CHECK( interp(0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
    CHECK( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
    CHECK( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
    CHECK( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
    CHECK( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

    CHECK( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
    CHECK( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;
  }


}

TEST_CASE( "BicubicInterpolator Tests - Oscillating Data", "[thin plate spline]" ) {

  _2D::TestBicubicInterp interp;

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

  _2D::BicubicInterpolator<double>::VectorType xx(nx*ny), yy(nx*ny), zz(nx*ny);

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

    out.open("Bicubic-oscillations-input.txt");
    for(int i = 0; i < nx*ny; i++)
      out << xx(i) << " " << yy(i) << " " << zz(i) << "\n";
    out.close();

    out.open("Bicubic-oscillations-output.txt");
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
    CHECK( interp(0,0)   == Approx(f(0,0)).epsilon(0.00002 )) ;
    CHECK( interp(1,2)   == Approx(f(1,2)).epsilon(0.00002 )) ;
    CHECK( interp(2,1)   == Approx(f(2,1)).epsilon(0.00002 )) ;
    CHECK( interp(2,-1)  == Approx(f(2,-1)).epsilon(0.00002 )) ;
    CHECK( interp(8,3)   == Approx(f(8,3)).epsilon(0.00002 )) ;

    CHECK( interp(-2,-1) == Approx(0).epsilon(0.00002 )) ;
    CHECK( interp(10,3)  == Approx(0).epsilon(0.00002 )) ;
  }






}


