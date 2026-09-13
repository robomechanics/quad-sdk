#include <gtest/gtest.h>
#include "robot_driver/controllers/underbrush_torque_observation.hpp"

TEST(UnderbrushTorque, MatchesV90EnvelopeInBothDirections) {
  const auto evaluate = [](double requested, double velocity) {
    return underbrush::modelTorque((requested + 0.5 * velocity) / 25.0,
                                  0.0, velocity);
  };
  EXPECT_DOUBLE_EQ(evaluate(2.5, 0.0), 2.5);
  EXPECT_DOUBLE_EQ(evaluate(30.0, 0.0), 23.4);
  EXPECT_DOUBLE_EQ(evaluate(30.0, 10.0), 20.2);
  EXPECT_DOUBLE_EQ(evaluate(-30.0, 10.0), -23.4);
  EXPECT_DOUBLE_EQ(evaluate(-30.0, -10.0), -20.2);
  EXPECT_DOUBLE_EQ(evaluate(30.0, -10.0), 23.4);
  EXPECT_DOUBLE_EQ(evaluate(30.0, 13.5), 20.2);
  EXPECT_DOUBLE_EQ(evaluate(30.0, 21.75), 10.1);
  EXPECT_DOUBLE_EQ(evaluate(-30.0, 21.75), -11.7);
  EXPECT_DOUBLE_EQ(evaluate(30.0, 30.0), 0.0);
  EXPECT_DOUBLE_EQ(evaluate(-30.0, -35.0), 0.0);
}

