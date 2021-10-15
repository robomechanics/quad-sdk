#ifndef INVERSE_DYNAMICS_H
#define INVERSE_DYNAMICS_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
// #include <eigen3/Eigen/Eigen>
#include <spirit_utils/ros_utils.h>
#include <spirit_utils/math_utils.h>
#include <spirit_utils/ros_utils.h>
#include <spirit_msgs/GRFArray.h>
#include <std_msgs/UInt8.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/RobotPlan.h>
#include <spirit_msgs/RobotPlan.h>
#include <spirit_msgs/MotorCommand.h>
#include <spirit_msgs/LegCommand.h>
#include <spirit_msgs/LegCommandArray.h>
#include <spirit_msgs/LegOverride.h>
#include <spirit_msgs/MultiFootPlanContinuous.h>
#include <eigen_conversions/eigen_msg.h>


#include <cmath>
#define MATH_PI 3.141592


//! Implements inverse dynamics as a controller within the ROS framework.
/*!
   InverseDynamicsController implements inverse dynamics logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class InverseDynamicsController {
  public:
    /**
     * @brief Constructor for InverseDynamicsController
     * @return Constructed object of type InverseDynamicsController
     */
    InverseDynamicsController();

    /**
     * @brief Compute the joint torques for a given state, set of GRFs, and foot accelerations
     * @param[in] state_positions Position coordinates of the robot [body, joints]
     * @param[in] state_velocities Velocity coordinates of the robot [body, joints]
     * @param[in] grf_array Ground reaction forces at each foot
     * @param[in] ref_foot_acceleration Desired acceleration of the feet
     * @param[in] contact_mode Contact mode to determine whether to use stance or swing ID
     * @param[out] tau_array Joint torques from ID solution
     */
    void computeJointTorques(const Eigen::VectorXd &state_positions,
      const Eigen::VectorXd &state_velocities, const Eigen::VectorXd &grf_array,
      const Eigen::VectorXd &ref_foot_acceleration, const std::vector<int> &contact_mode,
      Eigen::VectorXd &tau_array);

    /**
     * @brief Compute the joint torques for a given state and set of GRFs (ignoring swing feet)
     * @param[in] state_positions Position coordinates of the robot [body, joints]
     * @param[in] state_velocities Velocity coordinates of the robot [body, joints]
     * @param[in] grf_array Ground reaction forces at each foot
     * @param[out] tau_array Joint torques from ID solution
     */
    void computeJointTorques(const Eigen::VectorXd &state_positions,
      const Eigen::VectorXd &state_velocities, const Eigen::VectorXd &grf_array,
      Eigen::VectorXd &tau_array);

    /**
     * @brief Compute the leg command array message for a given current state and reference plan
     * @param[in] robot_state_msg Message of the current robot state
     * @param[in] local_plan_msg Message of the local referance plan
     * @param[out] leg_command_array_msg Command message after solving inverse dynamics and including reference setpoints for each joint
     */
    void computeLegCommandArrayFromPlan(
      const spirit_msgs::RobotState::ConstPtr &robot_state_msg,
      const spirit_msgs::RobotPlan::ConstPtr &local_plan_msg,
      spirit_msgs::LegCommandArray &leg_command_array_msg);

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
    std::shared_ptr<spirit_utils::QuadKD>quadKD_;

    /// PD gain when foot is in stance
    std::vector<double> stance_kp_;
    std::vector<double> stance_kd_;

    /// PD gain when foot is in swing
    std::vector<double> swing_kp_;
    std::vector<double> swing_kd_;
};


#endif // MPC_CONTROLLER_H
