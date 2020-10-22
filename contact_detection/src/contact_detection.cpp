#include "contact_detection/contact_detection.h"

ContactDetection::ContactDetection(ros::NodeHandle nh) {
	nh_ = nh;

	// Load rosparams from parameter server
	nh.param<double>("contact_detection/update_rate", update_rate_, 200); // add a param for your package instead of using the estimator one

	// Setup pubs and subs here
	joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic,1,&ContactDetection::jointEncoderCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic,1,&ContactDetection::imuCallback, this);
  contact_detection_pub_ = nh_.advertise<spirit_msgs::ContactDetection>(contact_detection_topic,1);
}

void ContactDetection::jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  last_joint_state_msg_ = msg;
}

void ContactDetection::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  last_imu_msg_ = msg;
}

spirit_msgs::ContactDetection ContactDetection::updateStep() {
  spirit_msgs::ContactDetection new_contact_est;

  return new_contact_est;
}

void ContactDetection::spin() {
	ros::Rate r(update_rate_);
	while (ros::ok()) {
		// Collect new messages on subscriber topics
		ros::spinOnce();

		// Compute new state estimate
		spirit_msgs::ContactDetection new_contact_est = this->updateStep();

		// Publish new state estimate
		contact_detection_pub_.publish(new_contact_est);

		// Store new state estimate for next iteration
		last_contact_est_ = new_contact_est;
		// Enforce update rate
		r.sleep();
	}
}
