#define NONIUS_RUNNER
#include "nonius/nonius.h++"
#include "nonius/main.h++"

#include "../Utils/DataSet.hpp"

#include "Interpolators/_2D/BicubicInterpolator.hpp"



NONIUS_BENCHMARK("BicubicInterpolator 4x4 Data Set Construct",
[](nonius::chronometer meter)
{
  
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(4,4);

  meter.measure( [&](){ interp.setData( data.x, data.y, data.z ); } );

})

NONIUS_BENCHMARK("BicubicInterpolator 4x4 Data Set 1000 Point Interpolation",
[](nonius::chronometer meter)
{
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(4,4);

  interp.setData( data.x, data.y, data.z );

  double dx = 4*1./100;
  double dy = 4*1./10;
  meter.measure( [&](){
  for(double i = 0; i < 100; i++)
    for(double j = 0; j < 10; j++)
      interp(i*dx,j*dy);
  } );

})

NONIUS_BENCHMARK("BicubicInterpolator 8x8 Data Set Construct",
[](nonius::chronometer meter)
{
  
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(8,8);

  meter.measure( [&](){ interp.setData( data.x, data.y, data.z ); } );

})

NONIUS_BENCHMARK("BicubicInterpolator 8x8 Data Set 1000 Point Interpolation",
[](nonius::chronometer meter)
{
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(8,8);

  interp.setData( data.x, data.y, data.z );

  double dx = 8*1./100;
  double dy = 8*1./10;
  meter.measure( [&](){
  for(double i = 0; i < 100; i++)
    for(double j = 0; j < 10; j++)
      interp(i*dx,j*dy);
  } );

})

NONIUS_BENCHMARK("BicubicInterpolator 16x16 Data Set Construct",
[](nonius::chronometer meter)
{
  
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(16,16);

  meter.measure( [&](){ interp.setData( data.x, data.y, data.z ); } );

})

NONIUS_BENCHMARK("BicubicInterpolator 16x16 Data Set 1000 Point Interpolation",
[](nonius::chronometer meter)
{
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(16,16);

  interp.setData( data.x, data.y, data.z );

  double dx = 16*1./100;
  double dy = 16*1./10;
  meter.measure( [&](){
  for(double i = 0; i < 100; i++)
    for(double j = 0; j < 10; j++)
      interp(i*dx,j*dy);
  } );

})

NONIUS_BENCHMARK("BicubicInterpolator 32x32 Data Set Construct",
[](nonius::chronometer meter)
{
  
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(32,32);

  meter.measure( [&](){ interp.setData( data.x, data.y, data.z ); } );

})

NONIUS_BENCHMARK("BicubicInterpolator 32x32 Data Set 1000 Point Interpolation",
[](nonius::chronometer meter)
{
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(32,32);

  interp.setData( data.x, data.y, data.z );

  double dx = 32*1./100;
  double dy = 32*1./10;
  meter.measure( [&](){
  for(double i = 0; i < 100; i++)
    for(double j = 0; j < 10; j++)
      interp(i*dx,j*dy);
  } );

})

NONIUS_BENCHMARK("BicubicInterpolator 64x64 Data Set Construct",
[](nonius::chronometer meter)
{
  
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(64,64);

  meter.measure( [&](){ interp.setData( data.x, data.y, data.z ); } );

})

NONIUS_BENCHMARK("BicubicInterpolator 64x64 Data Set 1000 Point Interpolation",
[](nonius::chronometer meter)
{
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(64,64);

  interp.setData( data.x, data.y, data.z );

  double dx = 64*1./100;
  double dy = 64*1./10;
  meter.measure( [&](){
  for(double i = 0; i < 100; i++)
    for(double j = 0; j < 10; j++)
      interp(i*dx,j*dy);
  } );

})

NONIUS_BENCHMARK("BicubicInterpolator 128x128 Data Set Construct",
[](nonius::chronometer meter)
{
  
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(128,128);

  meter.measure( [&](){ interp.setData( data.x, data.y, data.z ); } );

})

NONIUS_BENCHMARK("BicubicInterpolator 128x128 Data Set 1000 Point Interpolation",
[](nonius::chronometer meter)
{
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(128,128);

  interp.setData( data.x, data.y, data.z );

  double dx = 128*1./100;
  double dy = 128*1./10;
  meter.measure( [&](){
  for(double i = 0; i < 100; i++)
    for(double j = 0; j < 10; j++)
      interp(i*dx,j*dy);
  } );

})

NONIUS_BENCHMARK("BicubicInterpolator 256x256 Data Set Construct",
[](nonius::chronometer meter)
{
  
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(256,256);

  meter.measure( [&](){ interp.setData( data.x, data.y, data.z ); } );

})

NONIUS_BENCHMARK("BicubicInterpolator 256x256 Data Set 1000 Point Interpolation",
[](nonius::chronometer meter)
{
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(256,256);

  interp.setData( data.x, data.y, data.z );

  double dx = 256*1./100;
  double dy = 256*1./10;
  meter.measure( [&](){
  for(double i = 0; i < 100; i++)
    for(double j = 0; j < 10; j++)
      interp(i*dx,j*dy);
  } );

})

NONIUS_BENCHMARK("BicubicInterpolator 512x512 Data Set Construct",
[](nonius::chronometer meter)
{
  
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(512,512);

  meter.measure( [&](){ interp.setData( data.x, data.y, data.z ); } );

})

NONIUS_BENCHMARK("BicubicInterpolator 512x512 Data Set 1000 Point Interpolation",
[](nonius::chronometer meter)
{
  _2D::BicubicInterpolator<double> interp;
  _2D::DataSet data(512,512);

  interp.setData( data.x, data.y, data.z );

  double dx = 512*1./100;
  double dy = 512*1./10;
  meter.measure( [&](){
  for(double i = 0; i < 100; i++)
    for(double j = 0; j < 10; j++)
      interp(i*dx,j*dy);
  } );

})





















