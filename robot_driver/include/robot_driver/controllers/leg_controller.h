#ifndef leg_controller_H
#define leg_controller_H

#include <eigen_conversions/eigen_msg.h>
#include <quad_msgs/GRFArray.h>
#include <quad_msgs/LegCommand.h>
#include <quad_msgs/LegCommandArray.h>
#include <quad_msgs/LegOverride.h>
#include <quad_msgs/MotorCommand.h>
#include <quad_msgs/MultiFootPlanContinuous.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>
#include <robot_driver/hardware_interfaces/spirit_interface.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <cmath>
#include <eigen3/Eigen/Eigen>
#define MATH_PI 3.141592

//! Implements an abstract class for leg controllers.
/*!
   LegController provides an abstract leg controller class. It contains
   pure virtual methods for computing motor commands for each leg to be sent to
   the robot.
*/
class LegController {
 public:
  /**
   * @brief Constructor for LegController
   * @return Constructed object of type LegController
   */
  LegController();

  /**
   * @brief Set the desired proportional and derivative gains for all legs
   * @param[in] kp Proportional gains
   * @param[in] kd Derivative gains
   */
  virtual void setGains(double kp, double kd);

  /**
   * @brief Set the desired proportional and derivative gains for each leg
   * @param[in] kp Proportional gains
   * @param[in] kd Derivative gains
   */
  virtual void setGains(std::vector<double> kp, std::vector<double> kd);

  /**
   * @brief Set the desired stance and swing proportional and derivative gains
   * @param[in] stance_kp Stance phase proportional gains
   * @param[in] stance_kd Stance phase derivative gains
   * @param[in] swing_kp Swing phase proportional gains
   * @param[in] swing_kd Swing phase derivative gains
   */
  virtual void setGains(std::vector<double> stance_kp,
                        std::vector<double> stance_kd,
                        std::vector<double> swing_kp,
                        std::vector<double> swing_kd);

  /**
   * @brief Compute the leg command array message for a given current state and
   * reference plan
   * @param[in] local_plan_msg Message of the local referance plan
   */
  void updateLocalPlanMsg(quad_msgs::RobotPlan::ConstPtr msg,
                          const ros::Time &t_msg);

  /**
   * @brief Compute the leg command array message
   */
  virtual bool computeLegCommandArray(
      const quad_msgs::RobotState &robot_state_msg,
      quad_msgs::LegCommandArray &leg_command_array_msg,
      quad_msgs::GRFArray &grf_array_msg) = 0;

  inline bool overrideStateMachine() { return override_state_machine_; }

 protected:
  /// Number of feet
  const int num_feet_ = 4;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD> quadKD_;

  /// PD gain when foot is in stance
  std::vector<double> stance_kp_;
  std::vector<double> stance_kd_;

  /// PD gain when foot is in swing
  std::vector<double> swing_kp_;
  std::vector<double> swing_kd_;

  /// Last local plan message
  quad_msgs::RobotPlan::ConstPtr last_local_plan_msg_;

  /// Time of last local plan message
  ros::Time last_local_plan_time_;

  /// Bool for whether to override the state machine
  bool override_state_machine_;
};

#endif  // MPC_CONTROLLER_H
