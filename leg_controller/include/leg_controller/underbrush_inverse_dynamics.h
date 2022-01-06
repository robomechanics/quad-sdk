#ifndef UNDERBRUSH_INVERSE_DYNAMICS_H
#define UNDERBRUSH_INVERSE_DYNAMICS_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
// #include <eigen3/Eigen/Eigen>
#include <quad_utils/ros_utils.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>
#include <quad_msgs/GRFArray.h>
#include <std_msgs/UInt8.h>
#include <quad_msgs/RobotState.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/MotorCommand.h>
#include <quad_msgs/LegCommand.h>
#include <quad_msgs/LegCommandArray.h>
#include <quad_msgs/LegOverride.h>
#include <quad_msgs/BodyForceEstimate.h>
#include <quad_msgs/MultiFootPlanContinuous.h>
#include <eigen_conversions/eigen_msg.h>


#include <cmath>
#define MATH_PI 3.141592


//! Implements inverse dynamics as a controller within the ROS framework.
/*!
   UnderbrushInverseDynamicsController implements inverse dynamics logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class UnderbrushInverseDynamicsController {
  public:
    /**
     * @brief Constructor for UnderbrushInverseDynamicsController
     * @return Constructed object of type UnderbrushInverseDynamicsController
     */
    UnderbrushInverseDynamicsController();

    /**
     * @brief Compute the leg command array message for a given current state and reference plan
     * @param[in] robot_state_msg Message of the current robot state
     * @param[in] local_plan_msg Message of the local referance plan
     * @param[out] leg_command_array_msg Command message after solving inverse dynamics and including reference setpoints for each joint
     * @param[out] grf_array_msg GRF command message
     */
    void computeLegCommandArrayFromPlan(
      const quad_msgs::RobotState::ConstPtr &robot_state_msg,
      const quad_msgs::RobotPlan::ConstPtr &local_plan_msg,
      quad_msgs::LegCommandArray &leg_command_array_msg,
      quad_msgs::GRFArray &grf_array_msg,
      const quad_msgs::BodyForceEstimate::ConstPtr& body_force_msg);

      /**
       * @brief Set the desired stance and swing proportional and derivative gains
       * @param[in] stance_kp Stance phase proportional gains
       * @param[in] stance_kd Stance phase derivative gains
       * @param[in] swing_kp Swing phase proportional gains
       * @param[in] swing_kd Swing phase derivative gains
       */
      void setGains(std::vector<double> stance_kp, std::vector<double> stance_kd,
        std::vector<double> swing_kp, std::vector<double> swing_kd);

private:

    /// Number of feet
    const int num_feet_ = 4;

    /// QuadKD class
    std::shared_ptr<quad_utils::QuadKD>quadKD_;

    /// PD gain when foot is in stance
    std::vector<double> stance_kp_;
    std::vector<double> stance_kd_;

    /// PD gain when foot is in swing
    std::vector<double> swing_kp_;
    std::vector<double> swing_kd_;
};


#endif // UNDERBRUSH_INVERSE_DYNAMICS_H
