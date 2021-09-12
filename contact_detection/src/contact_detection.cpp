#include "contact_detection/contact_detection.h"

ContactDetection::ContactDetection(ros::NodeHandle nh) {
	nh_ = nh;

	// Load rosparams from parameter server
	std::string joint_encoder_topic, imu_topic, contact_mode_topic;
	nh.param<double>("contact_detection/update_rate", update_rate_, 200); // add a param for your package instead of using the estimator one
	nh.param<std::string>("topics/joint_encoder", joint_encoder_topic, "/joint_encoder");
	nh.param<std::string>("topics/imu", imu_topic, "/imu");
	nh.param<std::string>("topics/contact_mode", contact_mode_topic, "/contact_mode");

	// Setup pubs and subs here
	joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic,1,&ContactDetection::jointEncoderCallback, this);
	imu_sub_ = nh_.subscribe(imu_topic,1,&ContactDetection::imuCallback, this);
	contact_mode_pub_ = nh_.advertise<spirit_msgs::ContactMode>(contact_mode_topic,1);
}

void ContactDetection::jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  last_joint_state_msg_ = msg;
}

void ContactDetection::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  last_imu_msg_ = msg;
}

spirit_msgs::ContactMode ContactDetection::updateStep() {
  spirit_msgs::ContactMode new_contact_est;

  double contact_prob[4] = {0.0,0.3,0.7,0.99};

  new_contact_est.leg_contacts.resize(4);
  for (int i = 0; i < 4; ++i)
  {
    new_contact_est.leg_contacts.at(i).contact_prob = contact_prob[i];
    new_contact_est.leg_contacts.at(i).contact_state = contact_prob[i] > 0.5;
    new_contact_est.leg_contacts.at(i).contact_forces.x = 0;
    new_contact_est.leg_contacts.at(i).contact_forces.y = 0;
    new_contact_est.leg_contacts.at(i).contact_forces.z = 0;
  }

  new_contact_est.header.stamp = ros::Time::now();
  return new_contact_est;
}

void ContactDetection::spin() {
	ros::Rate r(update_rate_);
	while (ros::ok()) {

		// Collect new messages on subscriber topics
		ros::spinOnce();

		// Compute new contact estimate
		spirit_msgs::ContactMode new_contact_est = this->updateStep();

		// Publish new contact detection message
		contact_mode_pub_.publish(new_contact_est);

		// Store new state estimate for next iteration
		last_contact_est_ = new_contact_est;

		// Enforce update rate
		r.sleep();
	}
}
