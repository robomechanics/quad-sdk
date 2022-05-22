#include "quad_utils/rviz_interface.h"

RVizInterface::RVizInterface(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string global_plan_topic, local_plan_topic, discrete_global_plan_topic,
      foot_plan_discrete_topic, foot_plan_continuous_topic,
      state_estimate_topic, ground_truth_state_topic, trajectory_state_topic,
      grf_topic;

  // Load topic names from parameter server
  quad_utils::loadROSParam(nh_, "topics/global_plan", global_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/local_plan", local_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/control/grfs", grf_topic);
  quad_utils::loadROSParam(nh_, "topics/global_plan_discrete",
                           discrete_global_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/foot_plan_discrete",
                           foot_plan_discrete_topic);
  quad_utils::loadROSParam(nh_, "topics/foot_plan_continuous",
                           foot_plan_continuous_topic);
  quad_utils::loadROSParam(nh_, "topics/state/estimate", state_estimate_topic);
  quad_utils::loadROSParam(nh_, "topics/state/ground_truth",
                           ground_truth_state_topic);
  quad_utils::loadROSParam(nh_, "topics/state/trajectory",
                           trajectory_state_topic);
  quad_utils::loadROSParamDefault(nh_, "tf_prefix", tf_prefix_,
                                  std::string(""));

  std::string global_plan_viz_topic, local_plan_viz_topic,
      local_plan_ori_viz_topic, global_plan_grf_viz_topic,
      local_plan_grf_viz_topic, current_grf_viz_topic,
      discrete_body_plan_viz_topic, foot_plan_discrete_viz_topic,
      estimate_joint_states_viz_topic, ground_truth_joint_states_viz_topic,
      trajectory_joint_states_viz_topic, state_estimate_trace_viz_topic,
      ground_truth_trace_viz_topic, trajectory_state_trace_viz_topic;

  quad_utils::loadROSParam(nh_, "topics/visualization/global_plan",
                           global_plan_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/local_plan",
                           local_plan_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/local_plan_ori",
                           local_plan_ori_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/global_plan_grf",
                           global_plan_grf_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/local_plan_grf",
                           local_plan_grf_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/current_grf",
                           current_grf_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/global_plan_discrete",
                           discrete_body_plan_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/foot_plan_discrete",
                           foot_plan_discrete_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/joint_states/estimate",
                           estimate_joint_states_viz_topic);
  quad_utils::loadROSParam(nh_,
                           "topics/visualization/joint_states/ground_truth",
                           ground_truth_joint_states_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/joint_states/trajectory",
                           trajectory_joint_states_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/state/estimate_trace",
                           state_estimate_trace_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/state/ground_truth_trace",
                           ground_truth_trace_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/state/trajectory_trace",
                           trajectory_state_trace_viz_topic);

  // Setup rviz_interface parameters
  quad_utils::loadROSParam(nh_, "/map_frame", map_frame_);
  quad_utils::loadROSParam(nh_, "/rviz_interface/update_rate", update_rate_);
  quad_utils::loadROSParam(nh_, "/rviz_interface/colors/front_left",
                           front_left_color_);
  quad_utils::loadROSParam(nh_, "/rviz_interface/colors/back_left",
                           back_left_color_);
  quad_utils::loadROSParam(nh_, "/rviz_interface/colors/front_right",
                           front_right_color_);
  quad_utils::loadROSParam(nh_, "/rviz_interface/colors/back_right",
                           back_right_color_);
  quad_utils::loadROSParam(nh_, "/rviz_interface/colors/net_grf",
                           net_grf_color_);
  quad_utils::loadROSParam(nh_, "/rviz_interface/colors/individual_grf",
                           individual_grf_color_);

  double period, dt;
  quad_utils::loadROSParam(nh_, "/local_footstep_planner/period", period);
  quad_utils::loadROSParam(nh_, "/local_planner/timestep", dt);
  orientation_subsample_interval_ = int(period / dt);

  // Setup plan subs
  global_plan_sub_ = nh_.subscribe<quad_msgs::RobotPlan>(
      global_plan_topic, 1,
      boost::bind(&RVizInterface::robotPlanCallback, this, _1, GLOBAL));
  local_plan_sub_ = nh_.subscribe<quad_msgs::RobotPlan>(
      local_plan_topic, 1,
      boost::bind(&RVizInterface::robotPlanCallback, this, _1, LOCAL));
  grf_sub_ = nh_.subscribe(grf_topic, 1, &RVizInterface::grfCallback, this);
  foot_plan_discrete_sub_ =
      nh_.subscribe(foot_plan_discrete_topic, 1,
                    &RVizInterface::footPlanDiscreteCallback, this);
  foot_plan_continuous_sub_ =
      nh_.subscribe(foot_plan_continuous_topic, 1,
                    &RVizInterface::footPlanContinuousCallback, this);

  // Setup plan visual pubs
  global_plan_viz_pub_ =
      nh_.advertise<visualization_msgs::Marker>(global_plan_viz_topic, 1);
  local_plan_viz_pub_ =
      nh_.advertise<visualization_msgs::Marker>(local_plan_viz_topic, 1);
  global_plan_grf_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      global_plan_grf_viz_topic, 1);
  local_plan_grf_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      local_plan_grf_viz_topic, 1);
  current_grf_viz_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(current_grf_viz_topic, 1);
  discrete_body_plan_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(
      discrete_body_plan_viz_topic, 1);
  foot_plan_discrete_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(
      foot_plan_discrete_viz_topic, 1);
  local_plan_ori_viz_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>(local_plan_ori_viz_topic, 1);

  // Setup publishers for state traces
  state_estimate_trace_pub_ = nh_.advertise<visualization_msgs::Marker>(
      state_estimate_trace_viz_topic, 1);
  ground_truth_state_trace_pub_ = nh_.advertise<visualization_msgs::Marker>(
      ground_truth_trace_viz_topic, 1);
  trajectory_state_trace_pub_ = nh_.advertise<visualization_msgs::Marker>(
      trajectory_state_trace_viz_topic, 1);

  // Setup state subs to call the same callback but with pub ID included
  state_estimate_sub_ = nh_.subscribe<quad_msgs::RobotState>(
      state_estimate_topic, 1,
      boost::bind(&RVizInterface::robotStateCallback, this, _1, ESTIMATE));
  ground_truth_state_sub_ = nh_.subscribe<quad_msgs::RobotState>(
      ground_truth_state_topic, 1,
      boost::bind(&RVizInterface::robotStateCallback, this, _1, GROUND_TRUTH));
  trajectory_state_sub_ = nh_.subscribe<quad_msgs::RobotState>(
      trajectory_state_topic, 1,
      boost::bind(&RVizInterface::robotStateCallback, this, _1, TRAJECTORY));

  // Setup state visual pubs
  estimate_joint_states_viz_pub_ = nh_.advertise<sensor_msgs::JointState>(
      estimate_joint_states_viz_topic, 1);
  ground_truth_joint_states_viz_pub_ = nh_.advertise<sensor_msgs::JointState>(
      ground_truth_joint_states_viz_topic, 1);
  trajectory_joint_states_viz_pub_ = nh_.advertise<sensor_msgs::JointState>(
      trajectory_joint_states_viz_topic, 1);

  std::string foot_0_plan_continuous_viz_topic,
      foot_1_plan_continuous_viz_topic, foot_2_plan_continuous_viz_topic,
      foot_3_plan_continuous_viz_topic;

  quad_utils::loadROSParam(nh_, "topics/visualization/foot_0_plan_continuous",
                           foot_0_plan_continuous_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/foot_1_plan_continuous",
                           foot_1_plan_continuous_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/foot_2_plan_continuous",
                           foot_2_plan_continuous_viz_topic);
  quad_utils::loadROSParam(nh_, "topics/visualization/foot_3_plan_continuous",
                           foot_3_plan_continuous_viz_topic);

  foot_0_plan_continuous_viz_pub_ =
      nh_.advertise<nav_msgs::Path>(foot_0_plan_continuous_viz_topic, 1);
  foot_1_plan_continuous_viz_pub_ =
      nh_.advertise<nav_msgs::Path>(foot_1_plan_continuous_viz_topic, 1);
  foot_2_plan_continuous_viz_pub_ =
      nh_.advertise<nav_msgs::Path>(foot_2_plan_continuous_viz_topic, 1);
  foot_3_plan_continuous_viz_pub_ =
      nh_.advertise<nav_msgs::Path>(foot_3_plan_continuous_viz_topic, 1);

  // Initialize Path message to visualize body plan
  state_estimate_trace_msg_.action = visualization_msgs::Marker::ADD;
  state_estimate_trace_msg_.pose.orientation.w = 1;
  state_estimate_trace_msg_.type = visualization_msgs::Marker::LINE_STRIP;
  state_estimate_trace_msg_.scale.x = 0.02;
  state_estimate_trace_msg_.header.frame_id = map_frame_;
  geometry_msgs::Point dummy_point;
  state_estimate_trace_msg_.points.push_back(dummy_point);
  ground_truth_state_trace_msg_ = state_estimate_trace_msg_;
  trajectory_state_trace_msg_ = state_estimate_trace_msg_;

  // Define visual properties for traces
  state_estimate_trace_msg_.id = 5;
  state_estimate_trace_msg_.color.a = 1.0;
  state_estimate_trace_msg_.color.r = (float)front_left_color_[0] / 255.0;
  state_estimate_trace_msg_.color.g = (float)front_left_color_[1] / 255.0;
  state_estimate_trace_msg_.color.b = (float)front_left_color_[2] / 255.0;

  ground_truth_state_trace_msg_.id = 6;
  ground_truth_state_trace_msg_.color.a = 1.0;
  ground_truth_state_trace_msg_.color.r = (float)back_left_color_[0] / 255.0;
  ground_truth_state_trace_msg_.color.g = (float)back_left_color_[1] / 255.0;
  ground_truth_state_trace_msg_.color.b = (float)back_left_color_[2] / 255.0;

  trajectory_state_trace_msg_.id = 7;
  trajectory_state_trace_msg_.color.a = 1.0;
  trajectory_state_trace_msg_.color.r = (float)front_right_color_[0] / 255.0;
  trajectory_state_trace_msg_.color.g = (float)front_right_color_[1] / 255.0;
  trajectory_state_trace_msg_.color.b = (float)front_right_color_[2] / 255.0;
}

void RVizInterface::robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg,
                                      const int pub_id) {
  // Initialize Path message to visualize body plan
  visualization_msgs::Marker body_plan_viz;
  body_plan_viz.header = msg->header;
  body_plan_viz.action = visualization_msgs::Marker::ADD;
  body_plan_viz.pose.orientation.w = 1;
  body_plan_viz.id = 5;
  body_plan_viz.type = visualization_msgs::Marker::LINE_STRIP;
  body_plan_viz.scale.x = 0.03;

  // Construct MarkerArray for body plan orientation
  geometry_msgs::PoseArray body_plan_ori_viz;
  body_plan_ori_viz.header = msg->header;

  // Loop through the BodyPlan message to get the state info
  int length = msg->states.size();
  for (int i = 0; i < length; i++) {
    // Load in the pose data directly from the Odometry message
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->states[i].header;
    pose_stamped.pose = msg->states[i].body.pose;

    std_msgs::ColorRGBA color;
    color.a = 1;
    if (pub_id == LOCAL) {
      color.g = 1.0;
    } else {
      if (msg->primitive_ids[i] == FLIGHT) {
        color.r = (float)back_left_color_[0] / 255.0;
        color.g = (float)back_left_color_[1] / 255.0;
        color.b = (float)back_left_color_[2] / 255.0;
      } else if (msg->primitive_ids[i] == LEAP_STANCE ||
                 msg->primitive_ids[i] == LAND_STANCE) {
        color.r = (float)front_right_color_[0] / 255.0;
        color.g = (float)front_right_color_[1] / 255.0;
        color.b = (float)front_right_color_[2] / 255.0;
      } else if (msg->primitive_ids[i] == CONNECT) {
        color.r = (float)front_left_color_[0] / 255.0;
        color.g = (float)front_left_color_[1] / 255.0;
        color.b = (float)front_left_color_[2] / 255.0;
      } else {
        ROS_WARN_THROTTLE(1, "Invalid primitive ID received in RViz interface");
      }
    }
    body_plan_viz.colors.push_back(color);
    body_plan_viz.points.push_back(msg->states[i].body.pose.position);

    // Add poses to the orientation message
    if (i % orientation_subsample_interval_ == 0) {
      body_plan_ori_viz.poses.push_back(pose_stamped.pose);
    }
  }

  // Publish the full path
  if (pub_id == GLOBAL) {
    global_plan_viz_pub_.publish(body_plan_viz);
  } else if (pub_id == LOCAL) {
    local_plan_viz_pub_.publish(body_plan_viz);
    local_plan_ori_viz_pub_.publish(body_plan_ori_viz);
  }

  // Construct MarkerArray and Marker message for GRFs
  visualization_msgs::MarkerArray grfs_viz_msg;
  visualization_msgs::Marker marker;

  // Initialize the headers and types
  marker.header = msg->header;
  marker.type = visualization_msgs::Marker::ARROW;

  // Define the shape of the discrete states
  double arrow_diameter = 0.01;
  marker.scale.x = arrow_diameter;
  marker.scale.y = 4 * arrow_diameter;
  marker.color.g = 0.733f;
  marker.pose.orientation.w = 1.0;

  for (int i = 0; i < length; i++) {
    for (int j = 0; j < msg->grfs[i].vectors.size(); j++) {
      // Reset the marker message
      marker.points.clear();
      marker.color.a = 1.0;
      marker.id = i * msg->grfs[i].vectors.size() + j;

      if (msg->grfs[i].vectors.size() > 1) {
        marker.color.r = (float)individual_grf_color_[0] / 255.0;
        marker.color.g = (float)individual_grf_color_[1] / 255.0;
        marker.color.b = (float)individual_grf_color_[2] / 255.0;
      } else {
        marker.color.r = (float)net_grf_color_[0] / 255.0;
        marker.color.g = (float)net_grf_color_[1] / 255.0;
        marker.color.b = (float)net_grf_color_[2] / 255.0;
      }

      // Define point messages for the base and tip of each GRF arrow
      geometry_msgs::Point p_base, p_tip;
      p_base = msg->grfs[i].points[j];

      /// Define the endpoint of the GRF arrow
      double grf_length_scale = 0.002;
      p_tip.x = p_base.x + grf_length_scale * msg->grfs[i].vectors[j].x;
      p_tip.y = p_base.y + grf_length_scale * msg->grfs[i].vectors[j].y;
      p_tip.z = p_base.z + grf_length_scale * msg->grfs[i].vectors[j].z;

      // if GRF = 0, set alpha to zero
      if (msg->grfs[i].contact_states[j] == false) {
        marker.color.a = 0.0;
      }

      // Add the points to the marker and add the marker to the array
      marker.points.push_back(p_base);
      marker.points.push_back(p_tip);
      grfs_viz_msg.markers.push_back(marker);
    }
  }

  // Publish grfs
  if (pub_id == GLOBAL) {
    global_plan_grf_viz_pub_.publish(grfs_viz_msg);
  } else if (pub_id == LOCAL) {
    local_plan_grf_viz_pub_.publish(grfs_viz_msg);
  }
}

void RVizInterface::grfCallback(const quad_msgs::GRFArray::ConstPtr &msg) {
  if (msg->vectors.empty()) {
    return;
  }

  // Construct MarkerArray and Marker message for GRFs
  visualization_msgs::MarkerArray grfs_viz_msg;
  visualization_msgs::Marker marker;

  // Initialize the headers and types
  marker.header = msg->header;
  marker.type = visualization_msgs::Marker::ARROW;

  // Define the shape of the discrete states
  double arrow_diameter = 0.01;
  marker.scale.x = arrow_diameter;
  marker.scale.y = 4 * arrow_diameter;
  marker.color.g = 0.733f;
  marker.pose.orientation.w = 1.0;

  for (int i = 0; i < msg->vectors.size(); i++) {
    // Reset the marker message
    marker.points.clear();
    marker.color.a = 1.0;
    marker.id = i;

    marker.color.r = (float)individual_grf_color_[0] / 255.0;
    marker.color.g = (float)individual_grf_color_[1] / 255.0;
    marker.color.b = (float)individual_grf_color_[2] / 255.0;

    // Define point messages for the base and tip of each GRF arrow
    geometry_msgs::Point p_base, p_tip;
    p_base = msg->points[i];

    /// Define the endpoint of the GRF arrow
    double grf_length_scale = 0.002;
    p_tip.x = p_base.x + grf_length_scale * msg->vectors[i].x;
    p_tip.y = p_base.y + grf_length_scale * msg->vectors[i].y;
    p_tip.z = p_base.z + grf_length_scale * msg->vectors[i].z;

    // if GRF = 0, set alpha to zero
    if (msg->contact_states[i] == false) {
      marker.color.a = 0.0;
    }

    // Add the points to the marker and add the marker to the array
    marker.points.push_back(p_base);
    marker.points.push_back(p_tip);
    grfs_viz_msg.markers.push_back(marker);
  }

  current_grf_viz_pub_.publish(grfs_viz_msg);
}

void RVizInterface::discreteBodyPlanCallback(
    const quad_msgs::RobotPlan::ConstPtr &msg) {
  // Construct Marker message
  visualization_msgs::Marker discrete_body_plan;

  // Initialize the headers and types
  discrete_body_plan.header = msg->header;
  discrete_body_plan.id = 0;
  discrete_body_plan.type = visualization_msgs::Marker::POINTS;

  // Define the shape of the discrete states
  double scale = 0.2;
  discrete_body_plan.scale.x = scale;
  discrete_body_plan.scale.y = scale;
  discrete_body_plan.color.r = 0.733f;
  discrete_body_plan.color.a = 1.0;

  // Loop through the discrete states
  int length = msg->states.size();
  for (int i = 0; i < length; i++) {
    geometry_msgs::Point p;
    p.x = msg->states[i].body.pose.position.x;
    p.y = msg->states[i].body.pose.position.y;
    p.z = msg->states[i].body.pose.position.z;
    discrete_body_plan.points.push_back(p);
  }

  // Publish both interpolated body plan and discrete states
  discrete_body_plan_viz_pub_.publish(discrete_body_plan);
}

void RVizInterface::footPlanDiscreteCallback(
    const quad_msgs::MultiFootPlanDiscrete::ConstPtr &msg) {
  // Initialize Marker message to visualize footstep plan as points
  visualization_msgs::Marker points;
  points.header = msg->header;
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::SPHERE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.05;
  points.scale.y = 0.05;
  points.scale.z = 0.05;

  // Loop through each foot
  int num_feet = msg->feet.size();
  for (int i = 0; i < num_feet; ++i) {
    // Loop through footstep in the plan
    int num_steps = msg->feet[i].footholds.size();
    for (int j = 0; j < num_steps; ++j) {
      // Create point message from FootstepPlan message, adjust height
      geometry_msgs::Point p;
      p.x = msg->feet[i].footholds[j].position.x;
      p.y = msg->feet[i].footholds[j].position.y;
      p.z = msg->feet[i].footholds[j].position.z;

      // Set the color of each marker (green for front, blue for back)
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      if (i == 0) {
        color.r = (float)front_left_color_[0] / 255.0;
        color.g = (float)front_left_color_[1] / 255.0;
        color.b = (float)front_left_color_[2] / 255.0;
      } else if (i == 1) {
        color.r = (float)back_left_color_[0] / 255.0;
        color.g = (float)back_left_color_[1] / 255.0;
        color.b = (float)back_left_color_[2] / 255.0;
      } else if (i == 2) {
        color.r = (float)front_right_color_[0] / 255.0;
        color.g = (float)front_right_color_[1] / 255.0;
        color.b = (float)front_right_color_[2] / 255.0;
      } else if (i == 3) {
        color.r = (float)back_right_color_[0] / 255.0;
        color.g = (float)back_right_color_[1] / 255.0;
        color.b = (float)back_right_color_[2] / 255.0;
      }

      // Add to the Marker message
      points.colors.push_back(color);
      points.points.push_back(p);
    }
  }

  // Publish the full marker array
  foot_plan_discrete_viz_pub_.publish(points);
}

void RVizInterface::footPlanContinuousCallback(
    const quad_msgs::MultiFootPlanContinuous::ConstPtr &msg) {
  std::vector<nav_msgs::Path> foot_paths;
  foot_paths.resize(4);

  for (int j = 0; j < msg->states[0].feet.size(); j++) {
    foot_paths[j].header.frame_id = map_frame_;

    for (int i = 0; i < msg->states.size(); i++) {
      geometry_msgs::PoseStamped foot;
      foot.pose.position.x = msg->states[i].feet[j].position.x;
      foot.pose.position.y = msg->states[i].feet[j].position.y;
      foot.pose.position.z = msg->states[i].feet[j].position.z;

      foot_paths[j].poses.push_back(foot);
    }
  }

  foot_0_plan_continuous_viz_pub_.publish(foot_paths[0]);
  foot_1_plan_continuous_viz_pub_.publish(foot_paths[1]);
  foot_2_plan_continuous_viz_pub_.publish(foot_paths[2]);
  foot_3_plan_continuous_viz_pub_.publish(foot_paths[3]);
}

void RVizInterface::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr &msg, const int pub_id) {
  // Make a transform message for the body, populate with state estimate data
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header = msg->header;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = map_frame_;
  transformStamped.transform.translation.x = msg->body.pose.position.x;
  transformStamped.transform.translation.y = msg->body.pose.position.y;
  transformStamped.transform.translation.z = msg->body.pose.position.z;
  transformStamped.transform.rotation = msg->body.pose.orientation;

  // Copy the joint portion of the state estimate message to a new message
  sensor_msgs::JointState joint_msg;
  joint_msg = msg->joints;

  // Set the header to the main header of the state estimate message and publish
  joint_msg.header = msg->header;
  joint_msg.header.stamp = ros::Time::now();

  Eigen::Vector3d current_pos, last_pos;
  quad_utils::pointMsgToEigen(msg->body.pose.position, current_pos);

  if (pub_id == ESTIMATE) {
    transformStamped.child_frame_id = tf_prefix_ + "_estimate/body";
    estimate_base_tf_br_.sendTransform(transformStamped);
    estimate_joint_states_viz_pub_.publish(joint_msg);

    quad_utils::pointMsgToEigen(state_estimate_trace_msg_.points.back(),
                                last_pos);

    // Erase trace if state displacement exceeds threshold, otherwise show
    if ((current_pos - last_pos).norm() >= trace_reset_threshold_) {
      state_estimate_trace_msg_.action = visualization_msgs::Marker::DELETEALL;
      state_estimate_trace_msg_.points.clear();
    } else {
      state_estimate_trace_msg_.action = visualization_msgs::Marker::ADD;
    }

    state_estimate_trace_msg_.points.push_back(msg->body.pose.position);
    state_estimate_trace_msg_.header.stamp = joint_msg.header.stamp;
    state_estimate_trace_pub_.publish(state_estimate_trace_msg_);

  } else if (pub_id == GROUND_TRUTH) {
    transformStamped.child_frame_id = tf_prefix_ + "_ground_truth/body";

    ground_truth_base_tf_br_.sendTransform(transformStamped);
    ground_truth_joint_states_viz_pub_.publish(joint_msg);

    quad_utils::pointMsgToEigen(ground_truth_state_trace_msg_.points.back(),
                                last_pos);

    // Erase trace if state displacement exceeds threshold, otherwise show
    if ((current_pos - last_pos).norm() >= trace_reset_threshold_) {
      ground_truth_state_trace_msg_.action =
          visualization_msgs::Marker::DELETEALL;
      ground_truth_state_trace_msg_.points.clear();
    } else {
      ground_truth_state_trace_msg_.action = visualization_msgs::Marker::ADD;
    }

    ground_truth_state_trace_msg_.points.push_back(msg->body.pose.position);
    ground_truth_state_trace_msg_.header.stamp = joint_msg.header.stamp;
    ground_truth_state_trace_pub_.publish(ground_truth_state_trace_msg_);

  } else if (pub_id == TRAJECTORY) {
    transformStamped.child_frame_id = tf_prefix_ + "_trajectory/body";
    trajectory_base_tf_br_.sendTransform(transformStamped);
    trajectory_joint_states_viz_pub_.publish(joint_msg);

    quad_utils::pointMsgToEigen(trajectory_state_trace_msg_.points.back(),
                                last_pos);

    // Erase trace if state displacement exceeds threshold, otherwise show
    if ((current_pos - last_pos).norm() >= trace_reset_threshold_) {
      trajectory_state_trace_msg_.action =
          visualization_msgs::Marker::DELETEALL;
      trajectory_state_trace_msg_.points.clear();
    } else {
      trajectory_state_trace_msg_.action = visualization_msgs::Marker::ADD;
    }

    trajectory_state_trace_msg_.points.push_back(msg->body.pose.position);
    trajectory_state_trace_msg_.header.stamp = joint_msg.header.stamp;
    trajectory_state_trace_pub_.publish(trajectory_state_trace_msg_);
  } else {
    ROS_WARN_THROTTLE(
        0.5, "Invalid publisher id, not publishing robot state to rviz");
  }
}

void RVizInterface::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Collect new messages on subscriber topics
    ros::spinOnce();

    // Enforce update rate
    r.sleep();
  }
}
