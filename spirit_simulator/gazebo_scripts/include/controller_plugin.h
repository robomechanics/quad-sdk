#pragma once

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <spirit_msgs/MotorCommandArray.h>
#include <spirit_msgs/MotorCommand.h>
#include <urdf/model.h>

namespace effort_controllers
{
  /**
 * \brief Forward command controller for spirit
 *
 * This class forwards the commanded efforts down to a set of joints.
 * 
 */
class SpiritController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  typedef std::vector<spirit_msgs::MotorCommand> BufferType;

public:
  SpiritController();
  ~SpiritController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<BufferType> commands_buffer_;
  unsigned int n_joints_;

private:
  ros::Subscriber sub_command_;
  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

  void commandCB(const spirit_msgs::MotorCommandArrayConstPtr& msg);
  void enforceJointLimits(double &command, unsigned int index);

}; // class

} // namespace
