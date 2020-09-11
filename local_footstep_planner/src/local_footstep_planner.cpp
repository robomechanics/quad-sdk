#include "local_footstep_planner/local_footstep_planner.h"

LocalFootstepPlanner::LocalFootstepPlanner(ros::NodeHandle nh) {
	nh_ = nh;

	// Load rosparams from parameter server
	std::string joint_encoder_topic, imu_topic, state_estimate_topic;
	nh.param<std::string>("topics/terrain_map", terrain_map_topic, "/terrain_map");
	nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
	nh.param<std::string>("topics/footstep_plan", footstep_plan_topic, "/footstep_plan");
	nh.param<double>("local_footstep_planner/update_rate", update_rate_, 1);

	// Setup pubs and subs
	joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic,1,&EKFEstimator::jointEncoderCallback, this);
	imu_sub_ = nh_.subscribe(imu_topic,1,&EKFEstimator::imuCallback, this);
	state_estimate_pub_ = nh_.advertise<std_msgs::String>(state_estimate_topic,1);
}

void LocalFootstepPlanner::spin() {
	ros::Rate r(update_rate_);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
}