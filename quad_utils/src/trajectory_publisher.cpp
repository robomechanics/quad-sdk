#include "quad_utils/trajectory_publisher.h"

TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string body_plan_topic, trajectory_state_topic;

  nh.param<std::string>("topics/global_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/state/trajectory", trajectory_state_topic,
                        "/state/trajectory");

  nh.param<std::string>("map_frame", map_frame_, "map");
  nh.param<std::string>("trajectory_publisher/traj_source", traj_source_,
                        "topic");
  nh.param<double>("trajectory_publisher/update_rate", update_rate_, 30);

  // Setup subs and pubs
  body_plan_sub_ = nh_.subscribe(body_plan_topic, 1,
                                 &TrajectoryPublisher::robotPlanCallback, this);

  trajectory_state_pub_ =
      nh_.advertise<quad_msgs::RobotState>(trajectory_state_topic, 1);

  // Initialize kinematics object
  quadKD_ = std::make_shared<quad_utils::QuadKD>();
}

void TrajectoryPublisher::importTrajectory() {
  // Clear current trajectory message
  traj_msg_.states.clear();
  traj_msg_.header.frame_id = map_frame_;
  traj_msg_.header.stamp = ros::Time::now();

  // Load the desired values into traj_msg here
  return;
}

void TrajectoryPublisher::robotPlanCallback(
    const quad_msgs::RobotPlan::ConstPtr& msg) {
  // Save the most recent body plan
  body_plan_msg_ = (*msg);
}

void TrajectoryPublisher::publishTrajectoryState() {
  // Wait until we actually have data
  if (body_plan_msg_.states.empty()) {
    return;
  }

  // Get the current time in the trajectory since the beginning of the plan
  double traj_duration = (body_plan_msg_.states.back().header.stamp -
                          body_plan_msg_.states.front().header.stamp)
                             .toSec();
  double t =
      (ros::Time::now() - body_plan_msg_.states.front().header.stamp).toSec();

  // Ensure the trajectory remains valid
  t = std::min(t, traj_duration);

  // Interpolate to get the correct state and publish it
  quad_msgs::RobotState interp_state;
  int interp_primitive_id;
  quad_msgs::GRFArray interp_grf;

  quad_utils::interpRobotPlan(body_plan_msg_, t, interp_state,
                              interp_primitive_id, interp_grf);

  // Fill joints and feet with dummy data
  if (interp_state.joints.name.empty()) {
    interp_state.joints.name = {"8",  "0", "1", "9",  "2", "3",
                                "10", "4", "5", "11", "6", "7"};
    interp_state.joints.position = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    interp_state.joints.velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    interp_state.joints.effort = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  }
  quad_utils::fkRobotState(*quadKD_, interp_state.body, interp_state.joints,
                           interp_state.feet);

  trajectory_state_pub_.publish(interp_state);
}

void TrajectoryPublisher::spin() {
  ros::Rate r(update_rate_);
  if (traj_source_.compare("import") == 0) {
    importTrajectory();
  }
  while (ros::ok()) {
    // Publish the trajectory state
    publishTrajectoryState();

    // Collect new messages on subscriber topics
    ros::spinOnce();

    // Enforce update rate
    r.sleep();
  }
}
