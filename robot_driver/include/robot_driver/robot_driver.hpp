#ifndef ROBOT_DRIVER_H
#define ROBOT_DRIVER_H

#include <tf2_eigen/tf2_eigen.hpp>
#include <quad_msgs/msg/grf_array.hpp>
#include <quad_msgs/msg/leg_command.hpp>
#include <quad_msgs/msg/leg_command_array.hpp>
#include <quad_msgs/msg/motor_command.hpp>
#include <quad_msgs/msg/multi_foot_plan_continuous.hpp>
#include <quad_msgs/msg/robot_plan.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_msgs/msg/body_force_estimate.hpp>
#include <quad_msgs/msg/foot_contact.hpp>
#include <quad_utils/function_timer.hpp>
#include <quad_utils/math_utils.hpp>
#include <quad_utils/quad_kd2.hpp>
#include <quad_utils/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.h>
#include "nav_msgs/msg/path.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "robot_driver/controllers/grf_pid_controller.hpp"
#include "robot_driver/controllers/inverse_dynamics_controller.hpp"
#include "robot_driver/controllers/underbrush_inverse_dynamics.hpp"
#include "robot_driver/controllers/inertia_estimation_controller.hpp"
#include "robot_driver/controllers/joint_controller.hpp"
#include "robot_driver/controllers/leg_controller.hpp"
#ifdef HAS_ONNXRUNTIME
#include "robot_driver/controllers/learned_policy.hpp"
#endif
#include "robot_driver/estimators/comp_filter_estimator.hpp"
#include "robot_driver/estimators/ekf_estimator.hpp"
#include "robot_driver/estimators/state_estimator.hpp"
#include "robot_driver/hardware_interfaces/hardware_interface.hpp"
#include "robot_driver/hardware_interfaces/spirit_interface.hpp"
#include "robot_driver/hardware_interfaces/unitree_interface.hpp"
#include "robot_driver/robot_driver_utils.hpp"

#define MATH_PI 3.141592

//! ROS-based driver to handle computation and interfacing for state and
//! control.
/*!
   RobotDriver implements a class to retrieve state information and generate leg
   commands to be sent to either the robot or a simulator. It may subscribe to
   any number of topics to determine the leg control, but will always publish a
   LegCommandArray message to control the robot's legs.
*/
class RobotDriver {
 public:
  /**
   * @brief Constructor for RobotDriver
   * @param[in] node Shared pointer to rclcpp::Node
   * @return Constructed object of type RobotDriver
   */
  RobotDriver(std::shared_ptr<rclcpp::Node>, int argc, char** argv);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

 private:
  /**
   * @brief Initializes leg controller object
   */
  void initLegController();

  /**
   * @brief Initializes states and controls structures
   */
  void initStateControlStructs();

  /**
   * @brief Initializes state estimator object
   */
  void initStateEstimator();

  /**
   * @brief Verifies and updates new control mode
   * @param[in] msg New control mode
   */
  void controlModeCallback(const std_msgs::msg::UInt8::SharedPtr msg);

  /**
   * @brief Callback function to handle new local plan (states and GRFs)
   * @param[in] msg input message contining the local plan
   */
  void localPlanCallback(const quad_msgs::msg::RobotPlan::SharedPtr msg);

  /**
   * @brief Callback function to handle current robot state
   * @param[in] msg input message contining current robot state
   */
  void robotStateCallback(const quad_msgs::msg::RobotState::SharedPtr msg);

  /**
   * @brief Callback function to handle current robot pose
   * @param[in] msg input message contining current robot pose
   */
  void mocapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Callback function to handle reference trajectory state
   * @param[in] msg input message contining reference trajectory state
   */
  void trajectoryStateCallback(const quad_msgs::msg::RobotState::SharedPtr msg);

  /**
   * @brief Callback to handle new leg override commands
   * @param[in] msg Leg override commands
   */
  void singleJointCommandCallback(
      const geometry_msgs::msg::Vector3::SharedPtr msg);

  /**
   * @brief Callback to handle new body force estimates
   * @param[in] msg body force estimates
   */
  void bodyForceEstimateCallback(
      const quad_msgs::msg::BodyForceEstimate::SharedPtr msg);

  /**
   * @brief Callback to handle control restart flag messages
   */
  void controlRestartFlagCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Callback to handle new remote heartbeat messages
   * @param[in] msg Remote heartbeat message
   */
  void remoteHeartbeatCallback(const std_msgs::msg::Header::SharedPtr msg);

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  /**
   * @brief Check to make sure required messages are fresh
   */
  void checkMessagesForSafety();

  /**
   * @brief Update the most recent state message with the given data
   */
  bool updateState();

  /**
   * @brief Function to compute leg command array message
   */
  bool updateControl();

  /**
   * @brief Publish the most recent state message with the given data
   */
  void publishState();

  /**
   * @brief Function to publish leg command array message
   * @param[in] is_valid Boolean for if the command is valid (only send valid
   * commands to the robot)
   */
  void publishControl(bool is_valid);

  /**
   * @brief Function to publish heartbeat message
   */
  void publishHeartbeat();

  void testDynamics();

  /// Subscriber for control mode
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr control_mode_sub_;

  /// ROS subscriber for local plan
  rclcpp::Subscription<quad_msgs::msg::RobotPlan>::SharedPtr local_plan_sub_;

  /// ROS subscriber for local plan
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_sub_;

  /// ROS subscriber for state estimate
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr robot_state_sub_;

  /// ROS subscriber for body force estimates
  rclcpp::Subscription<quad_msgs::msg::BodyForceEstimate>::SharedPtr
      body_force_estimate_sub_;

  /// ROS subscriber for control restart flag
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      control_restart_flag_sub_;

  /// ROS publisher for ground truth state
  rclcpp::Publisher<quad_msgs::msg::RobotState>::SharedPtr robot_state_pub_;

  // ROS publisher for state estimate
  rclcpp::Publisher<quad_msgs::msg::RobotState>::SharedPtr
      trajectry_robot_state_pub_;

  /// ROS subscriber for remote heartbeat
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr remote_heartbeat_sub_;

  /// ROS subscriber for single joint command
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr
      single_joint_cmd_sub_;

  /// ROS Subscriber for twist velocity commands (for learned policies)
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// ROS publisher for time stamped twist velocity commands
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      cmd_vel_stamped_pub_;

  /// ROS publisher for state estimate messages
  rclcpp::Publisher<quad_msgs::msg::RobotState>::SharedPtr state_estimate_pub_;

  /// ROS publisher for robot heartbeat
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr robot_heartbeat_pub_;

  /// ROS publisher for inverse dynamics
  rclcpp::Publisher<quad_msgs::msg::LegCommandArray>::SharedPtr
      leg_command_array_pub_;

  /// ROS publisher for desired GRF
  rclcpp::Publisher<quad_msgs::msg::GRFArray>::SharedPtr grf_pub_;

  /// ROS publisher for imu data
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  /// ROS publisher for joint data
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  /// ROS publisher for measured foot contact (Unitree foot-force sensor)
  rclcpp::Publisher<quad_msgs::msg::FootContact>::SharedPtr foot_contact_pub_;

  /// Threshold (raw int16 units) above which foot is considered in contact.
  int foot_contact_threshold_;

  /// ROS Wrapper Node
  std::shared_ptr<rclcpp::Node> node_;

  // Robot Namespace
  std::string robot_ns;

  // Robot Description URDF
  std::string robot_description;

  // Robot Name (Type)
  std::string robot_name;

  /// Boolean for whether robot layer is hardware (else sim)
  bool is_hardware_;

  /// Controller type
  std::string controller_id_;

  /// Estimator type
  std::string estimator_id_;

  /// Update rate for computing new controls;
  double update_rate_;

  /// Update rate for publishing data to ROS;
  double publish_rate_;

  /// Number of feet
  const int num_feet_ = 4;

  /// Robot mode
  int control_mode_;

  /// Torque limits
  std::vector<double> torque_limits_;

  /// Define ids for control modes: Sit
  const int SIT = 0;

  /// Define ids for control modes: Stand
  const int READY = 1;

  /// Define ids for control modes: Sit to stand
  const int SIT_TO_READY = 2;

  /// Define ids for control modes: Stand to sit
  const int READY_TO_SIT = 3;

  /// Define ids for control modes: Safety
  const int SAFETY = 4;

  /// Define ids for input types: none
  const int NONE = 0;

  /// Define ids for input types: local plan
  const int LOCAL_PLAN = 1;

  /// Define ids for input types: grf array
  const int GRFS = 2;

  /// Most recent local plan
  quad_msgs::msg::RobotPlan::SharedPtr last_local_plan_msg_;

  /// Most recent state estimate
  quad_msgs::msg::RobotState last_robot_state_msg_;

  quad_msgs::msg::RobotState last_state_estimate_msg_;

  /// Most recent local plan
  quad_msgs::msg::GRFArray::SharedPtr last_grf_array_msg_;

  /// Most recent remote  heartbeat
  std_msgs::msg::Header::SharedPtr last_remote_heartbeat_msg_;

  /// Most recent robot heartbeat
  std_msgs::msg::Header last_robot_heartbeat_msg_;

  // State timeout threshold in seconds
  double last_state_time_;

  // Remote heartbeat timeout threshold in seconds
  double remote_heartbeat_received_time_;

  /// Duration for sit to stand behavior
  const double transition_duration_ = 1.0;

  /// Timeout (in s) for receiving new input reference messages
  double input_timeout_;

  /// Timeout (in s) for receiving new state messages
  double state_timeout_;

  /// Timeout (in s) for receiving new heartbeat messages
  double heartbeat_timeout_;

  /// Latency threshold on robot messages for warnings (s)
  double remote_latency_threshold_warn_;

  /// Latency threshold on robot messages for error (s)
  double remote_latency_threshold_error_;

  /// Message for leg command array
  quad_msgs::msg::LegCommandArray leg_command_array_msg_;

  /// Message for leg command array
  quad_msgs::msg::GRFArray grf_array_msg_;

  /// User transmission data
  Eigen::VectorXd user_tx_data_;

  /// User recieved data
  Eigen::VectorXd user_rx_data_;

  /// Time at which to start transition
  rclcpp::Time transition_timestamp_;

  /// PD gain when in safety mode
  std::vector<double> safety_kp_;
  std::vector<double> safety_kd_;

  /// PD gain when in sit mode
  std::vector<double> sit_kp_;
  std::vector<double> sit_kd_;

  /// PD gain when in standing mode
  std::vector<double> stand_kp_;
  std::vector<double> stand_kd_;

  /// PD gain when foot is in stance
  std::vector<double> stance_kp_;
  std::vector<double> stance_kd_;

  /// PD gain when foot is in swing
  std::vector<double> swing_kp_;
  std::vector<double> swing_kd_;

  /// PD gain when foot is in swing (Cartesian)
  std::vector<double> swing_kp_cart_;
  std::vector<double> swing_kd_cart_;

  /// Define standing joint angles
  std::vector<double> stand_joint_angles_;

  /// Define sitting joint angles
  std::vector<double> sit_joint_angles_;

  /// Define path to Learned Policy ONNX File
  std::string model_path_;

  /// Learned policy inference rate (Hz)
  double policy_inference_rate_;

  /// QuadKD2 (Pinocchio-based kinematics)
  std::shared_ptr<quad_utils::QuadKD2> quadKD2_;

  /// Run state estimator in sim for debugging/testing
  bool debug_estimator_ = false;

  /// Leg Controller template class
  std::shared_ptr<LegController> leg_controller_;

  /// Leg Controller template class
#ifdef HAS_ONNXRUNTIME
  std::shared_ptr<LearnedPolicy> leg_policy_;
#endif

  /// State Estimator template class
  std::shared_ptr<StateEstimator> state_estimator_;

  /// Mblink converter object
  std::shared_ptr<HardwareInterface> hardware_interface_;

  /// Last mocap data
  geometry_msgs::msg::PoseStamped::SharedPtr last_mocap_msg_;

  /// Most recent IMU data
  sensor_msgs::msg::Imu last_imu_msg_;

  /// Most recent joint data
  sensor_msgs::msg::JointState last_joint_state_msg_;

  /// Best estimate of velocity
  Eigen::Vector3d vel_estimate_;

  /// Best estimate of velocity from mocap diff
  Eigen::Vector3d mocap_vel_estimate_;

  /// Best estimate of imu velocity
  Eigen::Vector3d imu_vel_estimate_;

  /// Twist Input
  Eigen::VectorXd cmd_vel_;

  /// Commanded Velocity Filter Constant
  double cmd_vel_filter_const_;

  /// Scale for twist cmd_vel
  double cmd_vel_scale_;

  /// Velocity filter time constant
  double filter_time_constant_;

  /// Velocity filter weight
  double filter_weight_;

  /// Maximum time elapsed between mocap messages before committing to new
  /// message
  double mocap_dropout_threshold_;

  /// Update rate of the motion capture system
  double mocap_rate_;

  /// Last mainboard time
  double last_mainboard_time_;

  /// Last mocap time
  rclcpp::Time last_mocap_time_;

  /// Time of last publishing
  rclcpp::Time t_pub_;

  /// Time of the most recent cmd vel data
  rclcpp::Time last_cmd_vel_msg_time_;

  /// Seed Value for Random Distribution
  double seed_;

  /// Last cmd_vel_msg
  geometry_msgs::msg::Twist last_cmd_vel_msg_;

  /// Required for some hardware interfaces
  int argc_;

  /// Required for some hardware interfaces
  char** argv_;

  /// EKF estimate message (for sim testing, separate from control state)
  quad_msgs::msg::RobotState ekf_estimate_msg_;

  /// Whether the EKF has been initialized from ground truth
  bool ekf_initialized_ = false;

  /// Previous ground truth velocity for finite-difference accel computation
  Eigen::Vector3d ekf_last_vel_ = Eigen::Vector3d::Zero();

  /// ID of the ride-along estimator that runs in parallel for comparison
  /// (e.g. "ekf_filter" while comp_filter drives control). "none" disables.
  std::string debug_estimator_id_;

  /// Parallel estimator running alongside state_estimator_ for logging only.
  /// Its output never reaches the controller.
  std::shared_ptr<StateEstimator> debug_state_estimator_;

  /// Output buffer for the parallel estimator
  quad_msgs::msg::RobotState debug_estimate_msg_;

  /// First-call flag: seed debug_estimate_msg_ from the active estimator
  /// so the ride-along EKF initializes at the same pose as comp_filter.
  bool debug_estimator_seeded_ = false;

  /// Latch: set once the primary EKF first runs (at READY). Before this, an
  /// EKF primary is held dormant (contact-aided filter is meaningless without
  /// foot contact, and initializing in the folded sit pose corrupts it).
  bool ekf_primary_started_ = false;
};

#endif  // ROBOT_DRIVER_H
