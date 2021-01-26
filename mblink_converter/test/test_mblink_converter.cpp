#include <ros/ros.h>
#include <gtest/gtest.h>

#include "mblink_converter/mblink_converter.h"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mblink_converter_tester");

  return RUN_ALL_TESTS();
}
