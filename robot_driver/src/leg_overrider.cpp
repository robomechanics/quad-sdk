#include "robot_driver/leg_overrider.h"

LegOverrider::LegOverrider(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string body_force_topic, leg_override_topic;
  nh.param<std::string>("topics/body_force/joint_torques", body_force_topic,
                        "/body_force/joint_torques");
  nh.param<std::string>("topics/control/leg_override", leg_override_topic,
                        "/control/leg_override");
  nh.param<double>(
      "body_force_estimator/update_rate", update_rate_,
      100);  // add a param for your package instead of using the estimator one

  // Setup pubs and subs here
  body_force_sub_ = nh_.subscribe(body_force_topic, 1,
                                  &LegOverrider::bodyForceCallback, this);
  leg_override_pub_ =
      nh_.advertise<quad_msgs::LegOverride>(leg_override_topic, 1);
}

void LegOverrider::bodyForceCallback(
    const quad_msgs::BodyForceEstimate::ConstPtr& msg) {
  // ROS_INFO("In robotStateCallback");
  last_body_force_estimate_msg_ = msg;
}

void LegOverrider::publishLegOverride() {
  // ROS_INFO("In BodyForce");
  quad_msgs::LegOverride msg;
  quad_msgs::LegCommand leg_command;
  quad_msgs::MotorCommand motor_command;

  double legPos[3] = {0.0, 0.4, 0.8};
  for (int i = 0; i < 3; i++) {
    motor_command.pos_setpoint = legPos[i];
    motor_command.vel_setpoint = 0;
    motor_command.kp = 50;
    motor_command.kd = 2;
    motor_command.torque_ff = 0;

    leg_command.motor_commands.push_back(motor_command);
  }

  // Disable a leg
  // msg.leg_index.push_back(0);
  // msg.leg_commands.push_back(leg_command);

  msg.header.stamp = ros::Time::now();

  leg_override_pub_.publish(msg);
}

void LegOverrider::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Collect new messages on subscriber topics

    // Publish new leg override
    publishLegOverride();

    ros::spinOnce();
    // Enforce update rate
    r.sleep();
  }
}
