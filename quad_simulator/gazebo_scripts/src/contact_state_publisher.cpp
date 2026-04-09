#include "gazebo_scripts/contact_state_publisher.hpp"

ContactStatePublisher::ContactStatePublisher(rclcpp::Node::SharedPtr node)
    : node_(node),
      tf_buffer_(node_->get_clock(), tf2::durationFromSec(10.0), node_),
      tf_listener_(tf_buffer_, node_) {
  // Load rosparams from parameter server
  std::string grf_topic;

  // Load Rosparams from Node Specific yaml File
  quad_utils::loadROSParam(node_, "topics.state.grfs", grf_topic);
  quad_utils::loadROSParam(node_, "namespace", ns);
  quad_utils::loadROSParam(node_, "world", world_name);

  std::array<std::string, 4> toe_contact_state_topics;
  for (int toe_idx = 0; toe_idx < num_feet_; ++toe_idx) {
    const std::string leg_ns = "leg_" + std::to_string(toe_idx);
    quad_utils::loadROSParam(node_, leg_ns + ".frames.toe",
                             toe_frame_names_[toe_idx]);
    toe_collision_names_[toe_idx] = toe_frame_names_[toe_idx] + "_collision";
    toe_contact_state_topics[toe_idx] =
        "gazebo/" + toe_frame_names_[toe_idx] + "_contact_states";
  }

  toe_0_contact_state_sub_ =
      node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
          toe_contact_state_topics[0], 1,
          std::bind(&ContactStatePublisher::onContactToe<0>, this,
                    std::placeholders::_1));
  toe_1_contact_state_sub_ =
      node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
          toe_contact_state_topics[1], 1,
          std::bind(&ContactStatePublisher::onContactToe<1>, this,
                    std::placeholders::_1));
  toe_2_contact_state_sub_ =
      node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
          toe_contact_state_topics[2], 1,
          std::bind(&ContactStatePublisher::onContactToe<2>, this,
                    std::placeholders::_1));
  toe_3_contact_state_sub_ =
      node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
          toe_contact_state_topics[3], 1,
          std::bind(&ContactStatePublisher::onContactToe<3>, this,
                    std::placeholders::_1));

  // RCLCPP_INFO( node_->get_logger(), "Subscription Topic: [%s] ",
  // toe0_contact_state_topic.c_str()); RCLCPP_INFO( node_->get_logger(),
  // "Publisher Topic: [%s] ", grf_topic.c_str());

  // Setup pubs
  grf_pub_ = node_->create_publisher<quad_msgs::msg::GRFArray>(grf_topic, 10);

  // Init messgaes
  grf_array_msg_.vectors.resize(num_feet_);
  grf_array_msg_.points.resize(num_feet_);
  grf_array_msg_.contact_states.resize(num_feet_);

  // Init indicator
  ready_to_publish_ = false;
}

template <int toe_idx>
void ContactStatePublisher::onContactToe(
    const ros_gz_interfaces::msg::Contacts::SharedPtr msg) {
  std::string terrain_name =
      "flat::body::collision";  // Change this to be the world name
  const std::string& toe_string = toe_collision_names_[toe_idx];

  // Toe Transform Names
  std::string ns = node_->get_namespace();
  if (!ns.empty() && ns.front() == '/') {
    ns = ns.substr(1);  // Remove leading slash
  }
  std::array<std::string, 4> toe_transform_names;
  for (int i = 0; i < num_feet_; ++i) {
    toe_transform_names[i] = ns + "_ground_truth/" + toe_frame_names_[i];
  }

  // Initialize outputs
  grf_array_msg_.vectors[toe_idx].x = 0.0;
  grf_array_msg_.vectors[toe_idx].y = 0.0;
  grf_array_msg_.vectors[toe_idx].z = 0.0;

  grf_array_msg_.points[toe_idx].x = 0.0;
  grf_array_msg_.points[toe_idx].y = 0.0;
  grf_array_msg_.points[toe_idx].z = 0.0;

  grf_array_msg_.contact_states[toe_idx] = false;

  for (const auto& contact : msg->contacts) {
    const std::string& str_toe = contact.collision1.name;
    const std::string& str_terrain = contact.collision2.name;
    // RCLCPP_INFO( node_->get_logger(), "Contact detected between: [%s] and
    // [%s]", str_toe.c_str(), str_terrain.c_str());

    std::size_t found_toe = str_toe.find(toe_string);
    std::size_t found_terrain = str_terrain.find(terrain_name);

    if ((found_toe != std::string::npos) &&
        (found_terrain != std::string::npos)) {
      last_contact_time_[toe_idx] = node_->get_clock()->now().seconds();
      // Get total wrench
      if (!contact.wrenches.empty()) {
        double fx = 0.0, fy = 0.0, fz = 0.0;
        for (const auto& wrench : contact.wrenches) {
          fx += wrench.body_1_wrench.force.x;
          fy += wrench.body_1_wrench.force.y;
          fz += wrench.body_1_wrench.force.z;
        }
        grf_array_msg_.vectors[toe_idx].x = fx;
        grf_array_msg_.vectors[toe_idx].y = fy;
        grf_array_msg_.vectors[toe_idx].z = fz;
      }
      // Add up position - there might be multiple contact points for one
      // contaxct pair
      for (const auto& pos : contact.positions) {
        grf_array_msg_.points[toe_idx].x += pos.x;
        grf_array_msg_.points[toe_idx].y += pos.y;
        grf_array_msg_.points[toe_idx].z += pos.z;
      }
      // Compute averaged contact position
      if (!contact.positions.empty()) {
        grf_array_msg_.points[toe_idx].x /= contact.positions.size();
        grf_array_msg_.points[toe_idx].y /= contact.positions.size();
        grf_array_msg_.points[toe_idx].z /= contact.positions.size();
      }
      // Assign contact state
      grf_array_msg_.contact_states[toe_idx] = true;

      // We only want the contact pair with ground
      break;  // Only use the first matching contact
    }
  }
  // Not needed, Automatically published in the World Frame
  // geometry_msgs::msg::TransformStamped transform_stamped;
  // try {
  //   transform_stamped = tf_buffer_.lookupTransform(
  //       "map", toe_transform_names[toe_idx], tf2::TimePointZero);

  // } catch (tf2::TransformException &ex) {
  //   RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
  //   "%s",
  //                        ex.what());
  //   ready_to_publish_ = false;
  //   return;
  // }
  // transform_stamped.transform.translation.x = 0;
  // transform_stamped.transform.translation.y = 0;
  // transform_stamped.transform.translation.z = 0;

  // tf2::doTransform(grf_array_msg_.vectors[toe_idx],
  //                  grf_array_msg_.vectors[toe_idx], transform_stamped);
  ready_to_publish_ = true;
}

// At bottom of contact_state_publisher.cpp
template void ContactStatePublisher::onContactToe<0>(
    ros_gz_interfaces::msg::Contacts::SharedPtr msg);
template void ContactStatePublisher::onContactToe<1>(
    ros_gz_interfaces::msg::Contacts::SharedPtr msg);
template void ContactStatePublisher::onContactToe<2>(
    ros_gz_interfaces::msg::Contacts::SharedPtr msg);
template void ContactStatePublisher::onContactToe<3>(
    ros_gz_interfaces::msg::Contacts::SharedPtr msg);

bool ContactStatePublisher::checkMessageTiming(double current_sim_time,
                                               int toe_idx) {
  double time_since_last_contact =
      current_sim_time - last_contact_time_[toe_idx];

  // If no contact received recently (e.g., in 0.1s), consider contact ended
  if (time_since_last_contact > timeout_threshold_) {
    return true;
  }

  return false;
}

void ContactStatePublisher::resetMessage(int toe_idx) {
  grf_array_msg_.vectors[toe_idx].x = 0.0;
  grf_array_msg_.vectors[toe_idx].y = 0.0;
  grf_array_msg_.vectors[toe_idx].z = 0.0;

  grf_array_msg_.points[toe_idx].x = 0.0;
  grf_array_msg_.points[toe_idx].y = 0.0;
  grf_array_msg_.points[toe_idx].z = 0.0;

  grf_array_msg_.contact_states[toe_idx] = false;

  contact_received_[toe_idx] = false;
  wrench_received_[toe_idx] = false;
}

void ContactStatePublisher::publishContactState() {
  grf_array_msg_.header.stamp = node_->get_clock()->now();
  grf_pub_->publish(grf_array_msg_);
}

void ContactStatePublisher::spin() {
  // rclcpp::Rate r(update_rate_);
  rclcpp::Rate r(update_rate_);
  while (rclcpp::ok()) {
    // Collect new messages on subscriber topics
    rclcpp::spin_some(node_);

    double sim_time_now = node_->get_clock()->now().seconds();
    for (int i = 0; i < 4; ++i) {
      if (checkMessageTiming(sim_time_now, i)) {
        // Leg Contact Has Ended
        resetMessage(i);
      }
    }
    // Publish the contact state
    if (ready_to_publish_) {
      publishContactState();
    }

    ready_to_publish_ = true;

    // Enforce update rate
    r.sleep();
  }
}
