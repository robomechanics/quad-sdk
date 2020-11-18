#include <ros/ros.h>
#include <iostream>

#include "mblink_converter/mblink_converter.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mblink_converter_node");
  ros::NodeHandle nh;

  /// Ghost MBLink interface class
  std::shared_ptr<MBLink> mblink_ptr(new MBLink);
  mblink_ptr->start(argc,argv);
  mblink_ptr->rxstart();
  mblink_ptr->setRetry("UPST_ADDRESS", 5);

  MBLinkConverter mblink_converter(nh, mblink_ptr);
  mblink_converter.spin();

  return 0;
}
