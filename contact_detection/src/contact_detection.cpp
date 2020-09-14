#include "contact_detection/contact_detection.h"

contact_detection::contact_detection(ros::NodeHandle nh) {
	nh_ = nh;

	// Load rosparams from parameter server
	nh.param<double>("ekf_estimator/update_rate", update_rate_, 200); // add a param for your package instead of using the estimator one

	// Setup pubs and subs here

	// sample_sub = ...

	// sample_pub = ...
}

void contact_detection::spin() {
	ros::Rate r(update_rate_);
	while (ros::ok()) {
		// Collect new messages on subscriber topics

		ros::spinOnce();
		// Enforce update rate
		r.sleep();
	}
}
