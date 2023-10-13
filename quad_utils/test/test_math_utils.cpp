#include <gtest/gtest.h>
#include <ros/ros.h>

#include "quad_utils/math_utils.h"
#include "quad_utils/ros_utils.h"

TEST(MathTest, testWrap) {
    int N = 201;
    double amplitude = 10;
    double period = 4 * M_PI;
    std::vector<double> data(N), t(N);
    for (int i = 0; i < data.size(); i++) {
        t[i] = i * period / N;
        data[i] = amplitude * sin(t[i]);
    }

    std::vector<double> data_wrapped = math_utils::wrapToPi(data);
    std::vector<double> data_unwrapped =
        math_utils::getUnwrappedVector(data_wrapped);

    double error = 0;
    for (int i = 0; i < data.size(); i++) {
        error += abs(data[i] - data_unwrapped[i]);
    }

    double tolerance = 1e-4;
    EXPECT_TRUE(error <= tolerance);
}
