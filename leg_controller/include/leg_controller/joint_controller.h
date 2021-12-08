#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <leg_controller/leg_controller_template.h>
#include <geometry_msgs/Vector3.h>

//! Implements inverse dynamics as a controller within the ROS framework.
/*!
   JointController implements inverse dynamics logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class JointController : public LegControllerTemplate {
  public:
    
    /**
     * @brief Constructor for JointController
     * @return Constructed object of type JointController
     */
    JointController();

    void updateSingleJointCommand(const geometry_msgs::Vector3::ConstPtr& msg);

    /**
     * @brief Compute the leg command array message for a given current state and reference plan
     * @param[out] leg_command_array_msg Command message after solving inverse dynamics and including reference setpoints for each joint
     * @param[out] grf_array_msg GRF command message
     */
    bool computeLegCommandArray(
      const quad_msgs::RobotState::ConstPtr &robot_state_msg,
      quad_msgs::LegCommandArray &leg_command_array_msg,
      quad_msgs::GRFArray &grf_array_msg);

  private:
    /// Leg index for controlled joint
    int leg_idx_ = 0;

    /// Joint index for controlled joint
    int joint_idx_ = 0;

    /// Desired torque in Nm
    double joint_torque_ = 0.0;

};


#endif // JOINT_CONTROLLER_H
