#ifndef LEG_CONTROLLER_INTERFACE_H
#define LEG_CONTROLLER_INTERFACE_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <quad_utils/ros_utils.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/function_timer.h>
#include <quad_msgs/GRFArray.h>
#include <std_msgs/UInt8.h>
#include <quad_msgs/RobotState.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/MotorCommand.h>
#include <quad_msgs/LegCommand.h>
#include <quad_msgs/LegCommandArray.h>
#include <quad_msgs/LegOverride.h>
#include <quad_msgs/MultiFootPlanContinuous.h>
#include <eigen_conversions/eigen_msg.h>
#include "quad_utils/matplotlibcpp.h"
#include "leg_controller/leg_controller_template.h"
#include "leg_controller/inverse_dynamics.h"
#include "leg_controller/grf_pid_controller.h"
#include "leg_controller/joint_controller.h"

#include <cmath>
#define MATH_PI 3.141592


//! ROS Wrapper for a leg controller class
/*!
   LegControllerInterface implements a class to generate leg commands to be sent to either the robot or a simulator.
   It may subscribe to any number of topics to determine the leg control, but will always publish a
   LegCommandArray message to control the robot's legs.
*/
class LegControllerInterface {
  public:
  /**
   * @brief Constructor for LegControllerInterface
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type LegControllerInterface
   */
  LegControllerInterface(ros::NodeHandle nh);
  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();
  
  private:
    /**
     * @brief Verifies and updates new control mode
     * @param[in] msg New control mode
     */ 
    void controlModeCallback(const std_msgs::UInt8::ConstPtr& msg);
    
    /**
     * @brief Callback function to handle new local plan (states and GRFs)
     * @param[in] msg input message contining the local plan
     */
    void localPlanCallback(const quad_msgs::RobotPlan::ConstPtr& msg);
    
    /**
     * @brief Callback function to handle current robot state
     * @param[in] msg input message contining current robot state
     */
    void robotStateCallback(const quad_msgs::RobotState::ConstPtr& msg);
    
    /**
     * @brief Callback function to handle reference trajectory state
     * @param[in] msg input message contining reference trajectory state
     */
    void trajectoryStateCallback(const quad_msgs::RobotState::ConstPtr& msg);
    
    /**
     * @brief Callback to handle new leg override commands
     * @param[in] msg Leg override commands
     */
    void legOverrideCallback(const quad_msgs::LegOverride::ConstPtr& msg);

    /**
     * @brief Callback to handle new leg override commands
     * @param[in] msg Leg override commands
     */
    void singleJointCommandCallback(const geometry_msgs::Vector3::ConstPtr& msg);

    /**
     * @brief Callback to handle new remote heartbeat messages
     * @param[in] msg Remote heartbeat message
     */
    void remoteHeartbeatCallback(const std_msgs::Header::ConstPtr& msg);

    /**
     * @brief Check to make sure required messages are fresh
     */
    void checkMessages();

    /**
     * @brief Function to publish heartbeat message
     */
    void publishHeartbeat();

    /**
     * @brief Function to compute custom leg control.
     */
    void executeCustomController();

    /**
     * @brief Function to compute leg command array message
     */
    bool computeLegCommandArray();
    
      /**
     * @brief Function to publish leg command array message
     */
    void publishLegCommandArray();

    /// Subscriber for control mode
    ros::Subscriber control_mode_sub_;

    /// ROS subscriber for body plan
    ros::Subscriber body_plan_sub_;

      /// ROS subscriber for local plan
    ros::Subscriber local_plan_sub_;

    /// ROS subscriber for state estimate
    ros::Subscriber robot_state_sub_;

    /// ROS subscriber for trajectory
    ros::Subscriber trajectory_state_sub_;

    /// ROS subscriber for leg override commands
    ros::Subscriber leg_override_sub_;

    /// ROS subscriber for remote heartbeat
    ros::Subscriber remote_heartbeat_sub_;

    /// ROS subscriber for single joint command
    ros::Subscriber single_joint_cmd_sub_;

    /// ROS publisher for robot heartbeat
    ros::Publisher robot_heartbeat_pub_;

    /// ROS publisher for inverse dynamics
    ros::Publisher leg_command_array_pub_;

    /// ROS publisher for desired GRF
    ros::Publisher grf_pub_;

    /// Nodehandle to pub to and sub from
    ros::NodeHandle nh_;

    /// Controller type
    std::string controller_id_;

    /// Update rate for sending and receiving data;
    double update_rate_;

    /// Timestep of local plan
    double dt_;

    /// Number of feet
    const int num_feet_ = 4;

    /// Robot mode
    int control_mode_;

    /// Torque limits
    Eigen::Vector3d torque_limits_;

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
    quad_msgs::RobotPlan::ConstPtr last_local_plan_msg_;

    /// Most recent state estimate
    quad_msgs::RobotState::ConstPtr last_robot_state_msg_;

    /// First state estimate
    quad_msgs::RobotState::ConstPtr first_robot_state_msg_;

    /// Most recent local plan
    quad_msgs::GRFArray::ConstPtr last_grf_array_msg_;

    /// Most recent state estimate
    quad_msgs::RobotState::ConstPtr last_trajectory_state_msg_;

    /// Most recent leg override
    quad_msgs::LegOverride last_leg_override_msg_;

    /// Most recent remote 
    std_msgs::Header::ConstPtr last_remote_heartbeat_msg_;

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
    quad_msgs::LegCommandArray leg_command_array_msg_;

    /// Message for leg command array
    quad_msgs::GRFArray grf_array_msg_;

    /// Time at which to start transition
    ros::Time transition_timestamp_;

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

    /// Define standing joint angles
    std::vector<double> stand_joint_angles_;

    /// Define sitting joint angles
    std::vector<double> sit_joint_angles_;

    /// QuadKD class
    std::shared_ptr<quad_utils::QuadKD>quadKD_;

    /// Leg Controller template class
    std::shared_ptr<LegControllerTemplate> leg_controller_;

    /// Trotting duration
    double trotting_duration_;

    /// Trotting count
    double trotting_count_;
};


#endif // LEG_CONTROLLER_H
