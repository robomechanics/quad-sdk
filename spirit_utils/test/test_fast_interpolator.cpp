#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/fast_interpolator.h"

TEST(FastInterpolator, testSimpleLinearInterp) {
	FastInterpolator interp;

  std::vector<double> X = {0,1,2,3,4};
  std::vector<double> Y = {0,1,4,9,16};
	ASSERT_DOUBLE_EQ(interp.interp1(X,Y,1.5), 2.5);
}

TEST(FastInterpolator, testExtrapolateLow) {
  FastInterpolator interp;
  std::vector<double> X = {0,1,2,3,4};
  std::vector<double> Y = {0,1,4,9,16};

  ASSERT_THROW(interp.interp1(X,Y,-1), std::runtime_error);
}

TEST(FastInterpolator, testExtrapolateHigh) {
  FastInterpolator interp;
  std::vector<double> X = {0,1,2,3,4};
  std::vector<double> Y = {0,1,4,9,16};

  ASSERT_THROW(interp.interp1(X,Y,5), std::runtime_error);
}

TEST(FastInterpolator, testLinearInterpVec) {
  FastInterpolator interp;

  std::vector<double> X = {0,1,2,3,4};
  std::vector<double> Y = {0,1,4,9,16};
  std::vector<double> Xq = {0.5,1.5,2};
  std::vector<double> Yq = interp.interp1(X,Y,Xq);
  std::vector<double> Yq_sol = {0.5, 2.5, 4};
  ASSERT_EQ(Yq.size(), Yq_sol.size());
  for (size_t i = 0; i < Yq.size(); ++i)
  {
    ASSERT_DOUBLE_EQ(Yq.at(i), Yq_sol.at(i));
  }
}