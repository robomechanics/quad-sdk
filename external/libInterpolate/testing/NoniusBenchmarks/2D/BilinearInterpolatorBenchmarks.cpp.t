#define NONIUS_RUNNER
#include "nonius/nonius.h++"
#include "nonius/main.h++"

#include "../Utils/DataSet.hpp"

#include "Interpolators/_2D/BilinearInterpolator.hpp"
<%doc>
  This is a mako template file that is used to generate several benchmarks for different
  sized data array. Rather than copy and pasting the same code to test interpolation
  of 4x4, 8x8, 16x16, etc sized data arrays, we just generate them with this template.
  That way, if we every need to change something, we can just do it once in the template
  and regenerate the 
  
</%doc>
<%
  Ns = [ 2**i for i in range(2,10) ]
%>

% for N in Ns:
NONIUS_BENCHMARK("BilinearInterpolator ${N}x${N} Data Set Construct",
[](nonius::chronometer meter)
{
  
  _2D::BilinearInterpolator<double> interp;
  _2D::DataSet data(${N},${N});

  meter.measure( [&](){ interp.setData( data.x, data.y, data.z ); } );

})

NONIUS_BENCHMARK("BilinearInterpolator ${N}x${N} Data Set 1000 Point Interpolation",
[](nonius::chronometer meter)
{
  _2D::BilinearInterpolator<double> interp;
  _2D::DataSet data(${N},${N});

  interp.setData( data.x, data.y, data.z );

  double dx = ${N}*1./100;
  double dy = ${N}*1./10;
  meter.measure( [&](){
  for(double i = 0; i < 100; i++)
    for(double j = 0; j < 10; j++)
      interp(i*dx,j*dy);
  } );

})

% endfor




















