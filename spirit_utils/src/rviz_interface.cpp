#include "spirit_utils/rviz_interface.h"

RVizInterface::RVizInterface(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string global_plan_topic, local_plan_topic, discrete_global_plan_topic,
    foot_plan_discrete_topic, foot_plan_continuous_topic, state_estimate_topic,
    ground_truth_state_topic, trajectory_state_topic;

  // Load topic names from parameter server
  spirit_utils::loadROSParam(nh_,"topics/global_plan", global_plan_topic);
  spirit_utils::loadROSParam(nh_,"topics/local_plan", local_plan_topic);
  spirit_utils::loadROSParam(nh_,"topics/global_plan_discrete", discrete_global_plan_topic);
  spirit_utils::loadROSParam(nh_,"topics/foot_plan_discrete", foot_plan_discrete_topic);
  spirit_utils::loadROSParam(nh_,"topics/foot_plan_continuous", foot_plan_continuous_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/estimate", state_estimate_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/ground_truth", ground_truth_state_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/trajectory", trajectory_state_topic);
  
  std::string global_plan_viz_topic, local_plan_viz_topic,
    global_plan_grf_viz_topic, local_plan_grf_viz_topic, discrete_body_plan_viz_topic,
    foot_plan_discrete_viz_topic, estimate_joint_states_viz_topic, 
    ground_truth_joint_states_viz_topic, trajectory_joint_states_viz_topic;

  spirit_utils::loadROSParam(nh_,"topics/visualization/global_plan", global_plan_viz_topic);
  spirit_utils::loadROSParam(nh_,"topics/visualization/local_plan", local_plan_viz_topic);
  spirit_utils::loadROSParam(nh_,"topics/visualization/global_plan_grf", global_plan_grf_viz_topic);
  spirit_utils::loadROSParam(nh_,"topics/visualization/local_plan_grf", local_plan_grf_viz_topic);
  spirit_utils::loadROSParam(nh_,"topics/visualization/global_plan_discrete", 
    discrete_body_plan_viz_topic);
  spirit_utils::loadROSParam(nh_,"topics/visualization/foot_plan_discrete", 
    foot_plan_discrete_viz_topic);
  spirit_utils::loadROSParam(nh_,"topics/visualization/joint_states/estimate", 
    estimate_joint_states_viz_topic);
  spirit_utils::loadROSParam(nh_,"topics/visualization/joint_states/ground_truth", 
    ground_truth_joint_states_viz_topic);
  spirit_utils::loadROSParam(nh_,"topics/visualization/joint_states/trajectory", 
    trajectory_joint_states_viz_topic);

  // Setup rviz_interface parameters
  spirit_utils::loadROSParam(nh_,"map_frame",map_frame_);
  nh.param<double>("rviz_interface/update_rate", update_rate_, 10);
  nh.param<std::vector<int> >("rviz_interface/colors/front_left",
    front_left_color_, {0,255,0});
  nh.param<std::vector<int> >("rviz_interface/colors/back_left",
    back_left_color_, {0,0,255});
  nh.param<std::vector<int> >("rviz_interface/colors/front_right",
    front_right_color_, {0,255,0});
  nh.param<std::vector<int> >("rviz_interface/colors/back_right",
    back_right_color_, {0,0,255});
  nh.param<std::vector<int> >("rviz_interface/colors/net_grf",
    net_grf_color_, {255,0,0});
  nh.param<std::vector<int> >("rviz_interface/colors/individual_grf",
    individual_grf_color_, {255,0,0});

  // Setup plan subs
  global_plan_sub_ = nh_.subscribe<spirit_msgs::RobotPlan>(global_plan_topic,1,
    boost::bind( &RVizInterface::robotPlanCallback, this, _1, GLOBAL));
  local_plan_sub_ = nh_.subscribe<spirit_msgs::RobotPlan>(local_plan_topic,1,
    boost::bind( &RVizInterface::robotPlanCallback, this, _1, LOCAL));
  discrete_body_plan_sub_ = nh_.subscribe(discrete_global_plan_topic,1,
    &RVizInterface::discreteBodyPlanCallback, this);
  foot_plan_discrete_sub_ = nh_.subscribe(foot_plan_discrete_topic,1,
    &RVizInterface::footPlanDiscreteCallback, this);
  foot_plan_continuous_sub_ = nh_.subscribe(foot_plan_continuous_topic,1,
    &RVizInterface::footPlanContinuousCallback, this);

  // Setup plan visual pubs
  global_plan_viz_pub_ = nh_.advertise<nav_msgs::Path>(global_plan_viz_topic,1);
  local_plan_viz_pub_ = nh_.advertise<nav_msgs::Path>(local_plan_viz_topic,1);
  global_plan_grf_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>
    (global_plan_grf_viz_topic,1);
  local_plan_grf_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>
    (local_plan_grf_viz_topic,1);
  discrete_body_plan_viz_pub_ = nh_.advertise<visualization_msgs::Marker>
    (discrete_body_plan_viz_topic,1);
  foot_plan_discrete_viz_pub_ = nh_.advertise<visualization_msgs::Marker>
    (foot_plan_discrete_viz_topic,1);

  // Setup state subs to call the same callback but with pub ID included
  state_estimate_sub_ = nh_.subscribe<spirit_msgs::RobotState>(state_estimate_topic,1,
    boost::bind( &RVizInterface::robotStateCallback, this, _1, ESTIMATE));
  ground_truth_state_sub_ = nh_.subscribe<spirit_msgs::RobotState>(ground_truth_state_topic,1,
    boost::bind( &RVizInterface::robotStateCallback, this, _1, GROUND_TRUTH));
  trajectory_state_sub_ = nh_.subscribe<spirit_msgs::RobotState>(trajectory_state_topic,1,
    boost::bind( &RVizInterface::robotStateCallback, this, _1, TRAJECTORY));
  
  // Setup state visual pubs
  estimate_joint_states_viz_pub_ = nh_.advertise<sensor_msgs::JointState>
    (estimate_joint_states_viz_topic,1);
  ground_truth_joint_states_viz_pub_ = nh_.advertise<sensor_msgs::JointState>
    (ground_truth_joint_states_viz_topic,1);
  trajectory_joint_states_viz_pub_ = nh_.advertise<sensor_msgs::JointState>
    (trajectory_joint_states_viz_topic,1);

  std::string foot_0_plan_continuous_viz_topic,foot_1_plan_continuous_viz_topic,
    foot_2_plan_continuous_viz_topic, foot_3_plan_continuous_viz_topic;

  spirit_utils::loadROSParam(nh_,"topics/visualization/foot_0_plan_continuous", 
    foot_0_plan_continuous_viz_topic);
  spirit_utils::loadROSParam(nh_,"topics/visualization/foot_1_plan_continuous", 
    foot_1_plan_continuous_viz_topic);
  spirit_utils::loadROSParam(nh_,"topics/visualization/foot_2_plan_continuous", 
    foot_2_plan_continuous_viz_topic);
  spirit_utils::loadROSParam(nh_,"topics/visualization/foot_3_plan_continuous", 
    foot_3_plan_continuous_viz_topic);

  foot_0_plan_continuous_viz_pub_ = nh_.advertise<nav_msgs::Path>
    (foot_0_plan_continuous_viz_topic,1);
  foot_1_plan_continuous_viz_pub_ = nh_.advertise<nav_msgs::Path>
    (foot_1_plan_continuous_viz_topic,1);
  foot_2_plan_continuous_viz_pub_ = nh_.advertise<nav_msgs::Path>
    (foot_2_plan_continuous_viz_topic,1);
  foot_3_plan_continuous_viz_pub_ = nh_.advertise<nav_msgs::Path>
    (foot_3_plan_continuous_viz_topic,1);

}

void RVizInterface::robotPlanCallback(const spirit_msgs::RobotPlan::ConstPtr& msg,
  const int pub_id)
{

  // Initialize Path message to visualize body plan
  nav_msgs::Path body_plan_viz;
  body_plan_viz.header = msg->header;

  // Loop through the BodyPlan message to get the state info
  int length = msg->states.size();
  for (int i=0; i < length; i++) {

    // Load in the pose data directly from the Odometry message
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->states[i].header;
    pose_stamped.pose = msg->states[i].body.pose.pose;

    // Add to the path message
    body_plan_viz.poses.push_back(pose_stamped);
  }

  // Publish the full path
  if (pub_id == GLOBAL) {
    global_plan_viz_pub_.publish(body_plan_viz);
  } else if (pub_id == LOCAL) {
    local_plan_viz_pub_.publish(body_plan_viz);
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
  marker.scale.y = 4*arrow_diameter;
  marker.color.g = 0.733f;
  marker.pose.orientation.w = 1.0;

  for (int i=0; i < length; i++) {
    for (int j = 0; j < msg->grfs[i].vectors.size(); j++) {

      // Reset the marker message
      marker.points.clear();
      marker.color.a = 1.0;
      marker.id = i*msg->grfs[i].vectors.size() + j;

      if (msg->grfs[i].vectors.size() > 1) {
        marker.color.r = (float) individual_grf_color_[0]/255.0;
        marker.color.g = (float) individual_grf_color_[1]/255.0;
        marker.color.b = (float) individual_grf_color_[2]/255.0;
      } else {
        marker.color.r = (float) net_grf_color_[0]/255.0;
        marker.color.g = (float) net_grf_color_[1]/255.0;
        marker.color.b = (float) net_grf_color_[2]/255.0;
      }

      // Define point messages for the base and tip of each GRF arrow
      geometry_msgs::Point p_base, p_tip;
      p_base = msg->grfs[i].points[j];

      /// Define the endpoint of the GRF arrow
      double grf_length_scale = 0.002;
      p_tip.x = p_base.x + grf_length_scale*msg->grfs[i].vectors[j].x;
      p_tip.y = p_base.y + grf_length_scale*msg->grfs[i].vectors[j].y;
      p_tip.z = p_base.z + grf_length_scale*msg->grfs[i].vectors[j].z;

      // if GRF = 0, set alpha to zero
      if (msg->grfs[i].contact_states[j] == false) {
        marker.color.a = 0.0;
        continue;
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

void RVizInterface::discreteBodyPlanCallback(
  const spirit_msgs::RobotPlan::ConstPtr& msg) {

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
  for (int i=0; i < length; i++) {
    geometry_msgs::Point p;
    p.x = msg->states[i].body.pose.pose.position.x;
    p.y = msg->states[i].body.pose.pose.position.y;
    p.z = msg->states[i].body.pose.pose.position.z;
    discrete_body_plan.points.push_back(p);
  }

  // Publish both interpolated body plan and discrete states
  discrete_body_plan_viz_pub_.publish(discrete_body_plan);
}

void RVizInterface::footPlanDiscreteCallback(
  const spirit_msgs::MultiFootPlanDiscrete::ConstPtr& msg) {

  // Initialize Marker message to visualize footstep plan as points
  visualization_msgs::Marker points;
  points.header = msg->header;
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;

  // Loop through each foot
  int num_feet = msg->feet.size();
  for (int i=0;i<num_feet; ++i) {

    // Loop through footstep in the plan
    int num_steps = msg->feet[i].footholds.size();
    for (int j=0;j<num_steps; ++j) {

      // Create point message from FootstepPlan message, adjust height
      geometry_msgs::Point p;
      p.x = msg->feet[i].footholds[j].position.x;
      p.y = msg->feet[i].footholds[j].position.y;
      p.z = msg->feet[i].footholds[j].position.z;

      // Set the color of each marker (green for front, blue for back)
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      if (i == 0) {
        color.r = (float) front_left_color_[0]/255.0;
        color.g = (float) front_left_color_[1]/255.0;
        color.b = (float) front_left_color_[2]/255.0;
      } else if (i == 1) {
        color.r = (float) back_left_color_[0]/255.0;
        color.g = (float) back_left_color_[1]/255.0;
        color.b = (float) back_left_color_[2]/255.0;
      } else if (i == 2) {
        color.r = (float) front_right_color_[0]/255.0;
        color.g = (float) front_right_color_[1]/255.0;
        color.b = (float) front_right_color_[2]/255.0;
      } else if (i == 3) {
        color.r = (float) back_right_color_[0]/255.0;
        color.g = (float) back_right_color_[1]/255.0;
        color.b = (float) back_right_color_[2]/255.0;
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
  const spirit_msgs::MultiFootPlanContinuous::ConstPtr& msg) {

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
  const spirit_msgs::RobotState::ConstPtr& msg, const int pub_id) {

  // Make a transform message for the body, populate with state estimate data
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header = msg->header;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = map_frame_;
  transformStamped.transform.translation.x = msg->body.pose.pose.position.x;
  transformStamped.transform.translation.y = msg->body.pose.pose.position.y;
  transformStamped.transform.translation.z = msg->body.pose.pose.position.z;
  transformStamped.transform.rotation = msg->body.pose.pose.orientation;

  // Copy the joint portion of the state estimate message to a new message
  sensor_msgs::JointState joint_msg;
  joint_msg.name.reserve(msg->joints.name.size() + msg->tail_joints.name.size());
  joint_msg.name.insert(joint_msg.name.end(), msg->joints.name.begin(), msg->joints.name.end());
  joint_msg.name.insert(joint_msg.name.end(), msg->tail_joints.name.begin(), msg->tail_joints.name.end());
  joint_msg.position.reserve(msg->joints.position.size() + msg->tail_joints.position.size());
  joint_msg.position.insert(joint_msg.position.end(), msg->joints.position.begin(), msg->joints.position.end());
  joint_msg.position.insert(joint_msg.position.end(), msg->tail_joints.position.begin(), msg->tail_joints.position.end());
  joint_msg.velocity.reserve(msg->joints.velocity.size() + msg->tail_joints.velocity.size());
  joint_msg.velocity.insert(joint_msg.velocity.end(), msg->joints.velocity.begin(), msg->joints.velocity.end());
  joint_msg.velocity.insert(joint_msg.velocity.end(), msg->tail_joints.velocity.begin(), msg->tail_joints.velocity.end());
  joint_msg.effort.reserve(msg->joints.effort.size() + msg->tail_joints.effort.size());
  joint_msg.effort.insert(joint_msg.effort.end(), msg->joints.effort.begin(), msg->joints.effort.end());
  joint_msg.effort.insert(joint_msg.effort.end(), msg->tail_joints.effort.begin(), msg->tail_joints.effort.end());

  // Set the header to the main header of the state estimate message and publish
  joint_msg.header = msg->header;
  joint_msg.header.stamp = ros::Time::now();

  if (pub_id == ESTIMATE) {
    transformStamped.child_frame_id = "/estimate/body";
    estimate_base_tf_br_.sendTransform(transformStamped);
    estimate_joint_states_viz_pub_.publish(joint_msg);
  } else if (pub_id == GROUND_TRUTH) {
    transformStamped.child_frame_id = "/ground_truth/body";
    ground_truth_base_tf_br_.sendTransform(transformStamped);
    ground_truth_joint_states_viz_pub_.publish(joint_msg);
  } else if (pub_id == TRAJECTORY) {
    transformStamped.child_frame_id = "/trajectory/body";
    trajectory_base_tf_br_.sendTransform(transformStamped);
    trajectory_joint_states_viz_pub_.publish(joint_msg);
  } else {
    ROS_WARN_THROTTLE(0.5, "Invalid publisher id, not publishing robot state to rviz");
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
