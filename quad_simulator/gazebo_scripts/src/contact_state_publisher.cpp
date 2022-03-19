#include "contact_state_publisher.h"

ContactStatePublisher::ContactStatePublisher(ros::NodeHandle nh)
    : listener_(buffer_) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string grf_topic, toe0_contact_state_topic, toe1_contact_state_topic,
      toe2_contact_state_topic, toe3_contact_state_topic;
  quad_utils::loadROSParam(nh_, "topics/state/grfs", grf_topic);
  quad_utils::loadROSParam(nh_, "topics/gazebo/toe0_contact_state",
                           toe0_contact_state_topic);
  quad_utils::loadROSParam(nh_, "topics/gazebo/toe1_contact_state",
                           toe1_contact_state_topic);
  quad_utils::loadROSParam(nh_, "topics/gazebo/toe2_contact_state",
                           toe2_contact_state_topic);
  quad_utils::loadROSParam(nh_, "topics/gazebo/toe3_contact_state",
                           toe3_contact_state_topic);

  quad_utils::loadROSParam(nh_, "contact_state_publisher/update_rate",
                           update_rate_);

  // Setup pubs and subs
  toe0_contact_state_sub = nh_.subscribe<gazebo_msgs::ContactsState>(
      toe0_contact_state_topic, 1,
      boost::bind(&ContactStatePublisher::contactStateCallback, this, _1, 0),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay(true));
  toe1_contact_state_sub = nh_.subscribe<gazebo_msgs::ContactsState>(
      toe1_contact_state_topic, 1,
      boost::bind(&ContactStatePublisher::contactStateCallback, this, _1, 1),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay(true));
  toe2_contact_state_sub = nh_.subscribe<gazebo_msgs::ContactsState>(
      toe2_contact_state_topic, 1,
      boost::bind(&ContactStatePublisher::contactStateCallback, this, _1, 2),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay(true));
  toe3_contact_state_sub = nh_.subscribe<gazebo_msgs::ContactsState>(
      toe3_contact_state_topic, 1,
      boost::bind(&ContactStatePublisher::contactStateCallback, this, _1, 3),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay(true));

  grf_pub_ = nh_.advertise<quad_msgs::GRFArray>(grf_topic, 1);

  grf_array_msg_.vectors.resize(num_feet_);
  grf_array_msg_.points.resize(num_feet_);
  grf_array_msg_.contact_states.resize(num_feet_);
  transformsStamped_.resize(num_feet_);
}

void ContactStatePublisher::contactStateCallback(
    const gazebo_msgs::ContactsState::ConstPtr& msg, const int toe_idx) {
  std::string terrain_name = "mesh_terrain";
  std::string toe_collision_names[4] = {"toe0_collision", "toe1_collision",
                                        "toe2_collision", "toe3_collision"};
  std::string toe_string = toe_collision_names[toe_idx];

  int count = 0;

  grf_array_msg_.vectors[toe_idx].x = 0.0;
  grf_array_msg_.vectors[toe_idx].y = 0.0;
  grf_array_msg_.vectors[toe_idx].z = 0.0;

  grf_array_msg_.contact_states[toe_idx] = false;

  for (int i = 0; i < msg->states.size(); i++) {
    std::string str_toe = msg->states[i].collision1_name;
    std::string str_terrain = msg->states[i].collision2_name;
    std::size_t found_toe = str_toe.find(toe_string);
    std::size_t found_terrain = str_terrain.find(terrain_name);

    if ((found_toe != std::string::npos) &&
        (found_terrain != std::string::npos)) {
      grf_array_msg_.vectors[toe_idx].x +=
          msg->states[i].wrenches[0].force.x / msg->states.size();
      grf_array_msg_.vectors[toe_idx].y +=
          msg->states[i].wrenches[0].force.y / msg->states.size();
      grf_array_msg_.vectors[toe_idx].z +=
          msg->states[i].wrenches[0].force.z / msg->states.size();

      grf_array_msg_.points[toe_idx].x +=
          msg->states[i].contact_positions[0].x / msg->states.size();
      grf_array_msg_.points[toe_idx].y +=
          msg->states[i].contact_positions[0].y / msg->states.size();
      grf_array_msg_.points[toe_idx].z +=
          msg->states[i].contact_positions[0].z / msg->states.size();

      grf_array_msg_.contact_states[toe_idx] = true;
      count++;
    }
  }

  // Rotate into the world frame
  tf2::doTransform(grf_array_msg_.vectors[toe_idx],
                   grf_array_msg_.vectors[toe_idx],
                   transformsStamped_[toe_idx]);

  if (count != msg->states.size()) {
    ROS_WARN(
        "Contacts with objects other than terrain detected, force readings "
        "inaccurate!");
  }
}

void ContactStatePublisher::publishContactState() {
  grf_array_msg_.header.stamp = ros::Time::now();
  grf_pub_.publish(grf_array_msg_);
}

void ContactStatePublisher::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Collect new messages on subscriber topics
    ros::spinOnce();

    // Get the new toe frame transforms
    try {
      transformsStamped_[0] =
          buffer_.lookupTransform("map", "ground_truth/toe0", ros::Time(0));
      transformsStamped_[1] =
          buffer_.lookupTransform("map", "ground_truth/toe1", ros::Time(0));
      transformsStamped_[2] =
          buffer_.lookupTransform("map", "ground_truth/toe2", ros::Time(0));
      transformsStamped_[3] =
          buffer_.lookupTransform("map", "ground_truth/toe3", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1, "%s", ex.what());
      continue;
    }

    // Publish the contact state
    publishContactState();

    // Enforce update rate
    r.sleep();
  }
}
