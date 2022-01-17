#include <ros/ros.h>
#include <gtest/gtest.h>

#include "quad_utils/matplotlibcpp.h"
#include "quad_utils/ros_utils.h"
#include "quad_utils/math_utils.h"

namespace plt = matplotlibcpp;

TEST(MathTest, testWrap) {

  int N = 201;
  double amplitude = 10;
  double period  = 4*M_PI;
  std::vector<double> data(N), t(N);
  for (int i = 0; i < data.size(); i++) {
    t[i] = i*period/N;
    data[i] = amplitude*sin(t[i]);
  }
  
  std::vector<double> data_wrapped = math_utils::wrapToPi(data);
  std::vector<double> data_unwrapped = math_utils::unwrap(data_wrapped);

  double error = 0;
  for (int i = 0; i<data.size(); i++) {
    error += abs(data[i] - data_unwrapped[i]);
  }

  double tolerance = 1e-4;
  EXPECT_TRUE(error <= tolerance);

  // plt::figure();
  // plt::named_plot("original", t, data);
  // plt::named_plot("wrapped", t, data_wrapped);
  // plt::named_plot("unwrapped", t, data_unwrapped, "--");
  // plt::xlabel("t");
  // plt::ylabel("data");
  // plt::legend();
  // plt::show();
  // plt::pause(0.001);

}