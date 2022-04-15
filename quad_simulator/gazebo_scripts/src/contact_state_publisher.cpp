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

  // Setup subs
  toe0_contact_state_sub = nh_.subscribe<gazebo_msgs::ContactsState>(
      toe0_contact_state_topic, 1,
      boost::bind(&ContactStatePublisher::contactStateCallback, this, _1, 0),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  toe1_contact_state_sub = nh_.subscribe<gazebo_msgs::ContactsState>(
      toe1_contact_state_topic, 1,
      boost::bind(&ContactStatePublisher::contactStateCallback, this, _1, 1),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  toe2_contact_state_sub = nh_.subscribe<gazebo_msgs::ContactsState>(
      toe2_contact_state_topic, 1,
      boost::bind(&ContactStatePublisher::contactStateCallback, this, _1, 2),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  toe3_contact_state_sub = nh_.subscribe<gazebo_msgs::ContactsState>(
      toe3_contact_state_topic, 1,
      boost::bind(&ContactStatePublisher::contactStateCallback, this, _1, 3),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  // Setup pubs
  grf_pub_ = nh_.advertise<quad_msgs::GRFArray>(grf_topic, 1);

  // Init messgaes
  grf_array_msg_.vectors.resize(num_feet_);
  grf_array_msg_.points.resize(num_feet_);
  grf_array_msg_.contact_states.resize(num_feet_);

  // Init indicator
  ready_to_publish_ = false;
}

void ContactStatePublisher::contactStateCallback(
    const gazebo_msgs::ContactsState::ConstPtr& msg, const int toe_idx) {
  // Link names for search
  std::string terrain_name = "mesh_terrain";
  std::string toe_collision_names[4] = {"toe0_collision", "toe1_collision",
                                        "toe2_collision", "toe3_collision"};
  std::string toe_string = toe_collision_names[toe_idx];

  // Toe frame names
  std::string toe_transform_names[4] = {
      "ground_truth/toe0", "ground_truth/toe1", "ground_truth/toe2",
      "ground_truth/toe3"};

  // Init grf
  grf_array_msg_.vectors[toe_idx].x = 0.0;
  grf_array_msg_.vectors[toe_idx].y = 0.0;
  grf_array_msg_.vectors[toe_idx].z = 0.0;

  // Init contact position
  grf_array_msg_.points[toe_idx].x = 0.0;
  grf_array_msg_.points[toe_idx].y = 0.0;
  grf_array_msg_.points[toe_idx].z = 0.0;

  // Init contact state
  grf_array_msg_.contact_states[toe_idx] = false;

  // Loop through contact pairs to find the contact with ground
  for (int i = 0; i < msg->states.size(); i++) {
    // Search desired contact pair
    std::string str_toe = msg->states[i].collision1_name;
    std::string str_terrain = msg->states[i].collision2_name;
    std::size_t found_toe = str_toe.find(toe_string);
    std::size_t found_terrain = str_terrain.find(terrain_name);

    if ((found_toe != std::string::npos) &&
        (found_terrain != std::string::npos)) {
      // Get total wrench
      grf_array_msg_.vectors[toe_idx].x = msg->states[i].total_wrench.force.x;
      grf_array_msg_.vectors[toe_idx].y = msg->states[i].total_wrench.force.y;
      grf_array_msg_.vectors[toe_idx].z = msg->states[i].total_wrench.force.z;

      // Add up position - there might be multiple contact points for one
      // contact pair
      for (size_t j = 0; j < msg->states[i].contact_positions.size(); j++) {
        grf_array_msg_.points[toe_idx].x +=
            msg->states[i].contact_positions[j].x;
        grf_array_msg_.points[toe_idx].y +=
            msg->states[i].contact_positions[j].y;
        grf_array_msg_.points[toe_idx].z +=
            msg->states[i].contact_positions[j].z;
      }

      // Compute averaged contact position
      grf_array_msg_.points[toe_idx].x /=
          msg->states[i].contact_positions.size();
      grf_array_msg_.points[toe_idx].y /=
          msg->states[i].contact_positions.size();
      grf_array_msg_.points[toe_idx].z /=
          msg->states[i].contact_positions.size();

      // Assign contact state
      grf_array_msg_.contact_states[toe_idx] = true;

      // We only want the contact pair with ground
      break;
    }
  }

  // Get the toe frame transforms
  geometry_msgs::TransformStamped transformsStamped;
  try {
    transformsStamped = buffer_.lookupTransform(
        "map", toe_transform_names[toe_idx], ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1, "%s", ex.what());
    ready_to_publish_ = false;
    return;
  }

  // Set tranformation to pure rotation
  transformsStamped.transform.translation.x = 0;
  transformsStamped.transform.translation.y = 0;
  transformsStamped.transform.translation.z = 0;

  // Rotate into the world frame (the bumper source codes said it's in world
  // frame but actually in body frame)
  tf2::doTransform(grf_array_msg_.vectors[toe_idx],
                   grf_array_msg_.vectors[toe_idx], transformsStamped);
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

    // Publish the contact state
    if (ready_to_publish_) {
      publishContactState();
    }

    ready_to_publish_ = true;

    // Enforce update rate
    r.sleep();
  }
}
