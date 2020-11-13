#include "contact_detection/contact_detection.h"

ContactDetection::ContactDetection(ros::NodeHandle nh) {
	nh_ = nh;

	// Load rosparams from parameter server
	std::string joint_encoder_topic, imu_topic, contact_detection_topic;
	nh.param<double>("contact_detection/update_rate", update_rate_, 200); // add a param for your package instead of using the estimator one
	nh.param<std::string>("topics/joint_encoder", joint_encoder_topic, "/joint_encoder");
	nh.param<std::string>("topics/imu", imu_topic, "/imu");
	nh.param<std::string>("topics/contact_detection", contact_detection_topic, "/contact_detection");

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

  new_contact_est.contact_prob = {0.8,0.2,0.2,0.8};
  new_contact_est.contact_state = {true,false,false,true};
  new_contact_est.header.stamp = ros::Time::now();
  return new_contact_est;
}

void ContactDetection::spin() {
	ros::Rate r(update_rate_);
	while (ros::ok()) {

		// Collect new messages on subscriber topics
		ros::spinOnce();

		// Compute new contact estimate
		spirit_msgs::ContactDetection new_contact_est = this->updateStep();

		// Publish new contact detection message
		contact_detection_pub_.publish(new_contact_est);

		// Store new state estimate for next iteration
		last_contact_est_ = new_contact_est;

		// Enforce update rate
		r.sleep();
	}
}