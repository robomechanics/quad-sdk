#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quad_estimators/comp_filter_estimator.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_utils/quad_kd2.hpp>
#include <quad_utils/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <memory>
#include <string>

//! Standalone node wrapping the complementary-filter state estimator.
/*!
   Subscribes to joint_states, imu, and mocap; runs the ported
   CompFilterEstimator at a configurable rate; publishes the control state on
   state/ground_truth (what the planner/controllers read, as Gazebo's
   estimator_plugin does in sim) plus a debug copy on state/estimate.
   Mirrors how robot_driver fed the estimator: loadSensorMsg / loadMocapMsg
   then updateOnce.
*/
class CompFilterEstimatorNode : public rclcpp::Node {
 public:
  // automatically_declare_parameters_from_overrides lets a --params-file
  // (e.g. go2.yaml) passed at launch reach QuadKD2: the robot config yaml's
  // /**: block carries body.frame and the leg_X joint/frame params that
  // QuadKD2::initModel requires (otherwise it fatals). Parameters declared
  // explicitly below are also kept for clarity/defaults.
  CompFilterEstimatorNode()
      : rclcpp::Node(
            "comp_filter_estimator_node",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(true)) {
    // robot_description is required by QuadKD2 (pinocchio model)
    if (!this->has_parameter("robot_description"))
      this->declare_parameter<std::string>("robot_description", "");
    // namespace used for robot-specific parameters (joints, body frame, ...)
    if (!this->has_parameter("robot_ns"))
      this->declare_parameter<std::string>("robot_ns", "robot_1");
    // estimator loop rate
    if (!this->has_parameter("update_rate"))
      this->declare_parameter<double>("update_rate", 500.0);
    // mocap dropout gate (must match the mocap frame rate; from robot_driver.yaml)
    if (!this->has_parameter("mocap_rate"))
      this->declare_parameter<double>("mocap_rate", 360.0);
    if (!this->has_parameter("mocap_dropout_threshold"))
      this->declare_parameter<double>("mocap_dropout_threshold", 0.035);

    this->get_parameter("robot_ns", robot_ns_);
    this->get_parameter("update_rate", update_rate_);
    this->get_parameter("mocap_rate", mocap_rate_);
    this->get_parameter("mocap_dropout_threshold", mocap_dropout_threshold_);
  }

  void initialize() {
    auto node = shared_from_this();

    // Build the kinematics/dynamics object from robot_description on this node
    quadKD_ = std::make_shared<quad_utils::QuadKD2>(node, robot_ns_);

    // Construct and initialize the ported estimator
    estimator_ = std::make_shared<quad_estimators::CompFilterEstimator>(
        node, robot_ns_, quadKD_);
    estimator_->init();

    // Subscriptions
    joint_state_sub_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
              last_joint_state_msg_ = msg;
            });
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          last_imu_msg_ = msg;
        });
    mocap_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "mocap", rclcpp::SystemDefaultsQoS(),
            std::bind(&CompFilterEstimatorNode::mocapCallback, this,
                      std::placeholders::_1));

    // Publishers: state/ground_truth is the control state the planner and
    // controllers actually read (same topic Gazebo's estimator_plugin fills in
    // sim); state/estimate is a debug copy. Both mirror robot_driver.
    state_ground_truth_pub_ = this->create_publisher<quad_msgs::msg::RobotState>(
        "state/ground_truth", 1);
    state_estimate_pub_ =
        this->create_publisher<quad_msgs::msg::RobotState>("state/estimate", 1);

    // Driving timer (mirrors robot_driver's fixed-rate updateState)
    auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&CompFilterEstimatorNode::updateLoop, this));

    RCLCPP_INFO(this->get_logger(),
                "CompFilterEstimatorNode initialized (update_rate=%.1f Hz)",
                update_rate_);
  }

 private:
  void mocapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Exact port of robot_driver's mocap dropout gate: only feed the velocity
    // low-pass helper when the inter-message interval is within tolerance of
    // the expected 1/mocap_rate, so a dropped frame doesn't corrupt the
    // mocap-derived velocity estimate.
    Eigen::Vector3d pos;
    quad_utils::pointMsgToEigen(msg->pose.position, pos);

    if (last_mocap_msg_) {
      double t_diff_mocap_msg = (rclcpp::Time(msg->header.stamp) -
                                 rclcpp::Time(last_mocap_msg_->header.stamp))
                                    .seconds();
      if (std::abs(t_diff_mocap_msg - 1.0 / mocap_rate_) <
          mocap_dropout_threshold_) {
        estimator_->mocapCallBackHelper(msg, pos);
      } else {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "Mocap time diff exceeds max dropout threshold: t_diff=%.6f, "
            "expected=%.6f, threshold=%.6f",
            t_diff_mocap_msg, 1.0 / mocap_rate_, mocap_dropout_threshold_);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 100,
          "Mocap time diff exceeds max dropout threshold, hold the last value");
    }
    last_mocap_msg_ = msg;  // always cache latest pose for loadMocapMsg
  }

  void updateLoop() {
    // Need at least imu and joint data to produce an estimate
    if (!last_imu_msg_ || !last_joint_state_msg_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Waiting for imu/joint_states messages");
      return;
    }

    // Feed sensor data to the estimator, mirroring robot_driver
    estimator_->loadSensorMsg(*last_imu_msg_, *last_joint_state_msg_);
    if (last_mocap_msg_) {
      estimator_->loadMocapMsg(last_mocap_msg_);
    }

    if (estimator_->updateOnce(last_robot_state_msg_)) {
      state_ground_truth_pub_->publish(last_robot_state_msg_);
      state_estimate_pub_->publish(last_robot_state_msg_);
    }
  }

  std::string robot_ns_;
  double update_rate_;
  double mocap_rate_;
  double mocap_dropout_threshold_;

  std::shared_ptr<quad_utils::QuadKD2> quadKD_;
  std::shared_ptr<quad_estimators::CompFilterEstimator> estimator_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_sub_;
  rclcpp::Publisher<quad_msgs::msg::RobotState>::SharedPtr
      state_ground_truth_pub_;
  rclcpp::Publisher<quad_msgs::msg::RobotState>::SharedPtr state_estimate_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Imu::SharedPtr last_imu_msg_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_msg_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_mocap_msg_;
  quad_msgs::msg::RobotState last_robot_state_msg_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CompFilterEstimatorNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
