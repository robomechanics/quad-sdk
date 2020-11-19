#include <ros/ros.h>
#include <gtest/gtest.h>

#include "mblink_converter/mblink_converter.h"

TEST(MBLinkConverter, testConstructor) {

  ros::NodeHandle nh;
  std::shared_ptr<MBLink> mblink_ptr(new MBLink);
  mblink_ptr->start(1,"mbblink_converter_tester");
  mblink_ptr->rxstart();
  mblink_ptr->setRetry("UPST_ADDRESS", 5);

  MBLinkConverter mblink_converter(nh, mblink_ptr);
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mblink_converter_tester");

  return RUN_ALL_TESTS();
}
