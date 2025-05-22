#include "gazebo_scripts/contact_state_publisher.h"

#include <ignition/msgs/contacts.pb.h>
#include <ignition/msgs/vector3d.pb.h>

#include <ignition/transport/Node.hh>

ContactStatePublisher::ContactStatePublisher(rclcpp::Node::SharedPtr node)
    : node_(node),
      tf_buffer_(node_->get_clock(), tf2::durationFromSec(10.0), node_),
      tf_listener_(tf_buffer_, node_) {
  // Load rosparams from parameter server
  std::string grf_topic, toe0_contact_state_topic, toe1_contact_state_topic,
      toe2_contact_state_topic, toe3_contact_state_topic,
      toe0_wrench_state_topic, toe1_wrench_state_topic, toe2_wrench_state_topic,
      toe3_wrench_state_topic;

  node_->declare_parameter<std::string>("topics.state.grfs");
  node_->declare_parameter<std::string>("namespace");
  node_->declare_parameter<std::string>("world");
  node_->declare_parameter<std::string>("topics.gazebo.toe0_contact_state");
  node_->declare_parameter<std::string>("topics.gazebo.toe1_contact_state");
  node_->declare_parameter<std::string>("topics.gazebo.toe2_contact_state");
  node_->declare_parameter<std::string>("topics.gazebo.toe3_contact_state");
  node_->declare_parameter<std::string>("topics.gazebo.toe0_wrench_state");
  node_->declare_parameter<std::string>("topics.gazebo.toe1_wrench_state");
  node_->declare_parameter<std::string>("topics.gazebo.toe2_wrench_state");
  node_->declare_parameter<std::string>("topics.gazebo.toe3_wrench_state");

  // Load Rosparams from Node Specific yaml File
  quad_utils::loadROSParam(node_, "topics.state.grfs", grf_topic);
  quad_utils::loadROSParam(node_, "topics.gazebo.toe0_contact_state",
                           toe0_contact_state_topic);
  quad_utils::loadROSParam(node_, "topics.gazebo.toe1_contact_state",
                           toe1_contact_state_topic);
  quad_utils::loadROSParam(node_, "topics.gazebo.toe2_contact_state",
                           toe2_contact_state_topic);
  quad_utils::loadROSParam(node_, "topics.gazebo.toe3_contact_state",
                           toe3_contact_state_topic);

  quad_utils::loadROSParam(node_, "topics.gazebo.toe0_wrench_state",
                           toe0_wrench_state_topic);
  quad_utils::loadROSParam(node_, "topics.gazebo.toe1_wrench_state",
                           toe1_wrench_state_topic);
  quad_utils::loadROSParam(node_, "topics.gazebo.toe2_wrench_state",
                           toe2_wrench_state_topic);
  quad_utils::loadROSParam(node_, "topics.gazebo.toe3_wrench_state",
                           toe3_wrench_state_topic);
  quad_utils::loadROSParam(node_, "namespace", ns);
  quad_utils::loadROSParam(node_, "world", world_name);

  // Update Topics with Correct Robot Namespace
  toe0_contact_state_topic = std::regex_replace(
      toe0_contact_state_topic, std::regex("robot_namespace"), ns);
  toe1_contact_state_topic = std::regex_replace(
      toe1_contact_state_topic, std::regex("robot_namespace"), ns);
  toe2_contact_state_topic = std::regex_replace(
      toe2_contact_state_topic, std::regex("robot_namespace"), ns);
  toe3_contact_state_topic = std::regex_replace(
      toe3_contact_state_topic, std::regex("robot_namespace"), ns);
  toe0_wrench_state_topic = std::regex_replace(
      toe0_wrench_state_topic, std::regex("robot_namespace"), ns);
  toe1_wrench_state_topic = std::regex_replace(
      toe1_wrench_state_topic, std::regex("robot_namespace"), ns);
  toe2_wrench_state_topic = std::regex_replace(
      toe2_wrench_state_topic, std::regex("robot_namespace"), ns);
  toe3_wrench_state_topic = std::regex_replace(
      toe3_wrench_state_topic, std::regex("robot_namespace"), ns);

  // Debug Namespace Topic
  ign_node_.Subscribe(toe0_contact_state_topic,
                      &ContactStatePublisher::onContactToe<0>, this);
  ign_node_.Subscribe(toe1_contact_state_topic,
                      &ContactStatePublisher::onContactToe<1>, this);
  ign_node_.Subscribe(toe2_contact_state_topic,
                      &ContactStatePublisher::onContactToe<2>, this);
  ign_node_.Subscribe(toe3_contact_state_topic,
                      &ContactStatePublisher::onContactToe<3>, this);

  ign_node_.Subscribe(toe0_wrench_state_topic,
                      &ContactStatePublisher::onWrenchToe<0>, this);
  ign_node_.Subscribe(toe1_wrench_state_topic,
                      &ContactStatePublisher::onWrenchToe<1>, this);
  ign_node_.Subscribe(toe2_wrench_state_topic,
                      &ContactStatePublisher::onWrenchToe<2>, this);
  ign_node_.Subscribe(toe3_wrench_state_topic,
                      &ContactStatePublisher::onWrenchToe<3>, this);

  contact_received_.fill(false);
  wrench_received_.fill(true);

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
void ContactStatePublisher::onWrenchToe(const ignition::msgs::Wrench &msg) {
  // std::cout << "Updating Wrench" << std::endl;
  last_wrench_msgs_[toe_idx] = msg;
}

template <int toe_idx>
void ContactStatePublisher::onContactToe(const ignition::msgs::Contacts &msg) {
  std::string terrain_name =
      std::regex_replace(world_name, std::regex("\\.sdf$"), "");
  std::string toe_collision_names[4] = {"toe0_collision", "toe1_collision",
                                        "toe2_collision", "toe3_collision"};
  std::string toe_string = toe_collision_names[toe_idx];
  const std::string toe_frame =
      ns + "_ground_truthtoe" + std::to_string(toe_idx);

  // Initialize outputs
  grf_array_msg_.vectors[toe_idx].x = 0.0;
  grf_array_msg_.vectors[toe_idx].y = 0.0;
  grf_array_msg_.vectors[toe_idx].z = 0.0;

  grf_array_msg_.points[toe_idx].x = 0.0;
  grf_array_msg_.points[toe_idx].y = 0.0;
  grf_array_msg_.points[toe_idx].z = 0.0;

  grf_array_msg_.contact_states[toe_idx] = false;
  // std::cout << "BEgin Contact" << std::endl;

  for (int i = 0; i < msg.contact_size(); ++i) {
    const auto &contact = msg.contact(i);
    const std::string &col1 = contact.collision1().name();
    const std::string &col2 = contact.collision2().name();

    bool found_toe = col1.find(toe_string) != std::string::npos ||
                     col2.find(toe_string) != std::string::npos;
    bool found_terrain = col1.find(terrain_name) != std::string::npos ||
                         col2.find(terrain_name) != std::string::npos;

    if (found_toe && found_terrain) {
      last_contact_time_[toe_idx] = node_->get_clock()->now().seconds();
      // Sum all contact forces and positions
      int n_points = contact.position_size();

      for (int j = 0; j < n_points; ++j) {
        const auto &pos = contact.position(j);
        const auto &wrench = last_wrench_msgs_[toe_idx];

        grf_array_msg_.points[toe_idx].x += pos.x();
        grf_array_msg_.points[toe_idx].y += pos.y();
        grf_array_msg_.points[toe_idx].z += pos.z();

        grf_array_msg_.vectors[toe_idx].x = wrench.force().x();
        grf_array_msg_.vectors[toe_idx].y = wrench.force().y();
        grf_array_msg_.vectors[toe_idx].z = wrench.force().z();
      }

      if (n_points > 0) {
        grf_array_msg_.points[toe_idx].x /= n_points;
        grf_array_msg_.points[toe_idx].y /= n_points;
        grf_array_msg_.points[toe_idx].z /= n_points;
      }

      grf_array_msg_.contact_states[toe_idx] = true;
      break;  // Only take the first match
    }
  }
  // RCLCPP_INFO(
  //     node_->get_logger(),
  //     "GRF toe %d: point = (%.3f, %.3f, %.3f), vector = (%.3f, %.3f, %.3f), "
  //     "contact = %s",
  //     toe_idx, grf_array_msg_.points[toe_idx].x,
  //     grf_array_msg_.points[toe_idx].y, grf_array_msg_.points[toe_idx].z,
  //     grf_array_msg_.vectors[toe_idx].x, grf_array_msg_.vectors[toe_idx].y,
  //     grf_array_msg_.vectors[toe_idx].z,
  //     grf_array_msg_.contact_states[toe_idx] ? "true" : "false");

  // std::cout << "End COntact" << std::endl;

  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
        "map", toe_frame, tf2::TimePointZero);  // "0" == latest

    // Zero translation; preserve only rotation
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;

    // Transform GRF vector into map/world frame
    tf2::doTransform(grf_array_msg_.vectors[toe_idx],
                     grf_array_msg_.vectors[toe_idx], tf);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s",
                         ex.what());
    ready_to_publish_ = false;
    return;
  }

  ready_to_publish_ = true;
}

template void ContactStatePublisher::onContactToe<0>(
    const ignition::msgs::Contacts &);
template void ContactStatePublisher::onContactToe<1>(
    const ignition::msgs::Contacts &);
template void ContactStatePublisher::onContactToe<2>(
    const ignition::msgs::Contacts &);
template void ContactStatePublisher::onContactToe<3>(
    const ignition::msgs::Contacts &);

template void ContactStatePublisher::onWrenchToe<0>(
    const ignition::msgs::Wrench &);
template void ContactStatePublisher::onWrenchToe<1>(
    const ignition::msgs::Wrench &);
template void ContactStatePublisher::onWrenchToe<2>(
    const ignition::msgs::Wrench &);
template void ContactStatePublisher::onWrenchToe<3>(
    const ignition::msgs::Wrench &);

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
        // std::cout << "REsetting" << std::endl;
        resetMessage(i);
      }
    }
    // std::cout << "Spinning" << std::endl;
    // Publish the contact state
    if (ready_to_publish_) {
      // std::cout << "Makes it Here" << std::endl;
      publishContactState();
    }

    ready_to_publish_ = true;

    // Enforce update rate
    r.sleep();
  }
}
