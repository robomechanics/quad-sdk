#include "contact_detection/contact_detection.h"

ContactDetection::ContactDetection(ros::NodeHandle nh) {
	nh_ = nh;

	// Load rosparams from parameter server
	nh.param<double>("contact_detection/update_rate", update_rate_, 200); // add a param for your package instead of using the estimator one

	// Setup pubs and subs here

	// sample_sub = ...

	// sample_pub = ...
}

void ContactDetection::spin() {
	ros::Rate r(update_rate_);
	while (ros::ok()) {
		// Collect new messages on subscriber topics

		ros::spinOnce();
		// Enforce update rate
		r.sleep();
	}
}
