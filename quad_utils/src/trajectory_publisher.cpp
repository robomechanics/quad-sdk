#include "quad_utils/trajectory_publisher.hpp"

TrajectoryPublisher::TrajectoryPublisher(rclcpp::Node::SharedPtr node)
    : node_(node) {
  // Load rosparams from parameter server
  std::string body_plan_topic, trajectory_state_topic;

  // Add Robot_ns, and robot_description
  quad_utils::loadROSParam(node_, "namespace", robot_ns_);
  quad_utils::loadROSParam(node_, "robot_description", robot_description_);
  quad_utils::loadROSParamDefault(node_, "topics.global_plan", body_plan_topic,
                                  std::string("global_plan"));
  quad_utils::loadROSParamDefault(node_, "topics.state.trajectory",
                                  trajectory_state_topic,
                                  std::string("state.trajectory"));

  quad_utils::loadROSParamDefault(node_, "map_frame", map_frame_,
                                  std::string("map"));
  quad_utils::loadROSParamDefault(node_, "trajectory_publisher.traj_source",
                                  traj_source_, std::string("topic"));
  quad_utils::loadROSParamDefault(node_, "trajectory_publisher.update_rate",
                                  update_rate_, 30.0);

  // Setup subs and pubs
  body_plan_sub_ = node_->create_subscription<quad_msgs::msg::RobotPlan>(
      body_plan_topic, 10,
      std::bind(&TrajectoryPublisher::robotPlanCallback, this,
                std::placeholders::_1));

  trajectory_state_pub_ = node_->create_publisher<quad_msgs::msg::RobotState>(
      trajectory_state_topic, 10);

  // Initialize kinematics object
  quadKD_ = std::make_shared<quad_utils::QuadKD>(node_, robot_ns_);
}

void TrajectoryPublisher::importTrajectory() {
  // Load the desired values into body_plan_msg_ here
  return;
}

void TrajectoryPublisher::robotPlanCallback(
    const quad_msgs::msg::RobotPlan::SharedPtr msg) {
  // Save the most recent body plan
  body_plan_msg_ = (*msg);
}

void TrajectoryPublisher::publishTrajectoryState() {
  // Wait until we actually have data
  if (body_plan_msg_.states.empty()) {
    return;
  }

  // Get the current time in the trajectory since the beginning of the plan
  double traj_duration =
      (rclcpp::Time(body_plan_msg_.states.back().header.stamp) -
       rclcpp::Time(body_plan_msg_.states.front().header.stamp))
          .seconds();
  double t =
      (node_->now() - rclcpp::Time(body_plan_msg_.states.front().header.stamp))
          .seconds();

  // Ensure the trajectory remains valid
  t = std::min(t, traj_duration);

  // Interpolate to get the correct state and publish it
  quad_msgs::msg::RobotState interp_state;
  int interp_primitive_id;
  quad_msgs::msg::GRFArray interp_grf;

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

  trajectory_state_pub_->publish(interp_state);
}

void TrajectoryPublisher::spin() {
  rclcpp::Rate r(update_rate_);
  if (traj_source_.compare("import") == 0) {
    importTrajectory();
  }
  while (rclcpp::ok()) {
    // Publish the trajectory state
    publishTrajectoryState();

    // Collect new messages on subscriber topics
    rclcpp::spin_some(node_);

    // Enforce update rate
    r.sleep();
  }
}
