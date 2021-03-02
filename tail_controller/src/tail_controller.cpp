#include "tail_controller/tail_controller.h"

TailController::TailController(ros::NodeHandle nh) {
	nh_ = nh;

  // Get rosparams
  std::string imu_topic;
  spirit_utils::loadROSParam(nh_,"topics/imu",imu_topic);
  spirit_utils::loadROSParam(nh_,"tail_controller/update_rate",update_rate_);
  
  // Setup pubs and subs
  imu_sub_ = nh_.subscribe(imu_topic,1,&TailController::imuCallback, this);
}


void TailController::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  last_imu_msg_ = (*msg);
}

void TailController::sendTailCommands(const std_msgs::UInt8::ConstPtr& msg) {

}

void TailController::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    sendTailCommands();
    ros::spinOnce();
    r.sleep();
  }
}