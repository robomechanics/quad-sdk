#ifndef CONTACT_STATE_PUBLISHER_H
#define CONTACT_STATE_PUBLISHER_H

#include <quad_utils/ros_utils.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ros_gz_interfaces/msg/contacts.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <eigen3/Eigen/Eigen>
#include <quad_msgs/msg/grf_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <regex>
#define MATH_PI 3.141592

//! Publishes contact states from gz Gazebo
/*!
   This class subscribes to gz Gazebo contact state messages and publishes
   their data under one GRFArray message.
*/

class ContactStatePublisher {
 public:
  /**
   * @brief Constructor for ContactStatePublisher
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type ContactStatePublisher
   */
  ContactStatePublisher(rclcpp::Node::SharedPtr node);
  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

 private:
  /**
   * @brief Processes new contact state data, GRF data
   * @param[in] msg New contact state data
   */
  template <int toe_idx>
  void onContactToe(const ros_gz_interfaces::msg::Contacts::SharedPtr msg);

  bool checkMessageTiming(double current_sim_time, int toe_idx);
  void resetMessage(int toe_idx);

  /**
   * @brief Publishes current contact, force state data
   */
  void publishContactState();

  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr
      toe_0_contact_state_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr
      toe_1_contact_state_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr
      toe_2_contact_state_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr
      toe_3_contact_state_sub_;

  std::array<double, 4> last_contact_time_;
  std::array<std::string, 4> toe_frame_names_;
  std::array<std::string, 4> toe_collision_names_;

  const double SYNC_THRESHOLD = 0.001;  // 1 ms

  /// Update rate for sending and recieving fata:
  double update_rate_ = 500.0;

  // Time before last contact message is invalid (Should be a Mulitple of Sim
  // Update Rate)
  double timeout_threshold_ = 0.005;

  /// Number of feet
  const int num_feet_ = 4;

  /// Most recent local plan
  quad_msgs::msg::GRFArray grf_array_msg_;

  /// Publish ready
  bool ready_to_publish_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<quad_msgs::msg::GRFArray>::SharedPtr grf_pub_;

  std::array<bool, 4> contact_received_;

  std::array<bool, 4> wrench_received_;

  std::string ns, world_name;

  tf2_ros::Buffer tf_buffer_;

  tf2_ros::TransformListener tf_listener_;
};
#endif  // CONTACT_STATE_PUBLISHER_H
