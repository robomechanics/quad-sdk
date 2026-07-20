#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

#include <ros_gz_interfaces/msg/entity_wrench.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>

#include <quad_msgs/msg/robot_state.hpp>
#include <quad_utils/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>
#include <string>
#include <random>
#include <cmath>
#include <chrono>

class ForceApplicator {
 public:
  /**
   * @brief Constructor for ForceApplicator
   * @param[in] node ROS node used for parameters, publishers, and subscribers
   * @return Constructed object of type ForceApplicator
   */

  ForceApplicator(rclcpp::Node::SharedPtr node);

  ~ForceApplicator() {
    // Stop periodic timer (if any)
    if (clear_timer_) {
      clear_timer_->cancel();
    }
    clear_timer_.reset();
  }

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

#ifdef FORCE_APPLICATOR_TESTING
 public:
#else
 private:
#endif
  /**
   * @brief Callback function to handle current robot state
   * @param[in] msg input message contining current robot state
   */
  void robotStateCallback(const quad_msgs::msg::RobotState::SharedPtr msg);

  /**
   * @brief Publishes Service Call and Visualization Marker
   */
  void updateMarker();
  /**
   * @brief Builds Service Request, and Visualization Marker
   */
  void updateForce();
  /**
   * @brief Handles Trigger Condition Logic
   */
  bool shouldTrigger();
  /**
   * @brief Calculates euclidean distance relative to a set point to deploy
   * force
   */
  void applyForce();

  double computeEuclideanDistance(const Eigen::Vector3d& point1,
                                  const Eigen::Vector3d& point2);

  /// Nodehandle to pub to and sub from
  rclcpp::Node::SharedPtr node_;

  /// ros Publisher for Force Visualization Marker
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      force_marker_pub_;

  /// ros Publishers for Force
  rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr
      wrench_persist_pub_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Entity>::SharedPtr
      wrench_clear_pub_;

  /// ros Subscriber for last robot state
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr robot_state_sub_;

  /// Timer that clears the persistent wrench after dt_
  rclcpp::TimerBase::SharedPtr clear_timer_;

  /// Gazebo transport topic names for applying and clearing wrenches
  std::string wrench_topic_persist_, wrench_topic_clear_;

  /// Random generator used by random force mode
  std::mt19937 rng_;

  /// Last time a periodic disturbance was applied
  rclcpp::Time last_fire_time_;

  /// Most recent state estimate
  quad_msgs::msg::RobotState last_robot_state_msg_;

  /// Most Recent Visualization of Applied Force
  visualization_msgs::msg::Marker last_robot_marker_msg_;

  /// Last robot body position used by distance trigger mode
  Eigen::Vector3d last_robot_pose_;

  /// Fixed force components loaded from YAML
  double force_x_, force_y_, force_z_;

  /// Most recently applied force components
  double fx, fy, fz;

  /// Magnitude of the most recently applied force
  double force_magnitude_;

  /// Fixed torque components loaded from YAML
  double torque_x_, torque_y_, torque_z_;

  /// Time to hold each applied wrench
  double dt_;

  /// World, robot type, and robot namespace parameters
  std::string world_name_, robot_type_, robot_ns_;

  /// Trigger mode, force source mode, and target link name
  std::string mode_, force_mode_, link_;

  /// Random force magnitude bounds
  double force_mag_min_, force_mag_max_;

  /// Random torque magnitude bounds
  double torque_mag_min_, torque_mag_max_;

  /// Time between disturbances in periodic mode
  double period_;

  /// Travel distance needed to trigger in distance mode
  double distance_threshold_;

  /// Whether a robot state has been received
  bool have_pose_ = false;

  /// Whether single mode has already fired
  bool single_ = false;

  /// Main loop update rate
  double update_rate_;

  /// Random generator seed
  int seed_;
};

#endif  // APPLY_FORCE_H
