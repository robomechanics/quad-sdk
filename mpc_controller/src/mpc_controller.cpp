#include "mpc_controller/mpc_controller.h"

MPCController::MPCController(ros::NodeHandle nh) {
  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);
	nh_ = nh;
}

void MPCController::spin() {
	ros::Rate r(update_rate_);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
}