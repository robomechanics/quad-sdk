#include "mpc_controller/mpc_controller.h"

MPCController::MPCController(ros::NodeHandle nh) {
	nh_ = nh;
}

void MPCController::spin() {
	ros::Rate r(update_rate_);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
}