#pragma once

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
<<<<<<< HEAD
=======
#include <quad_msgs/LegCommand.h>
#include <quad_msgs/LegCommandArray.h>
#include <quad_msgs/MotorCommand.h>
#include <quad_utils/ros_utils.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
<<<<<<< HEAD:quad_simulator/gazebo_scripts/include/controller_plugin.h
#include <urdf/model.h>

namespace effort_controllers {
/**
=======
#include <quad_msgs/MotorCommand.h>
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
#include <quad_msgs/LegCommand.h>
#include <quad_msgs/LegCommandArray.h>
#include <quad_msgs/MotorCommand.h>
#include <quad_utils/ros_utils.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <urdf/model.h>

<<<<<<< HEAD
namespace effort_controllers {
/**
=======
namespace effort_controllers
{
  /**
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*:spirit_simulator/gazebo_scripts/include/controller_plugin.h
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
 * \brief Forward command controller for quad
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 */
<<<<<<< HEAD
class QuadController : public controller_interface::Controller<
                           hardware_interface::EffortJointInterface> {
=======
<<<<<<< HEAD:quad_simulator/gazebo_scripts/include/controller_plugin.h
class QuadController : public controller_interface::Controller<
                           hardware_interface::EffortJointInterface> {
=======
class SpiritController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*:spirit_simulator/gazebo_scripts/include/controller_plugin.h
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
  typedef std::vector<quad_msgs::LegCommand> BufferType;

 public:
  QuadController();
  ~QuadController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector<std::string> joint_names_;
  std::vector<hardware_interface::JointHandle> joints_;
  realtime_tools::RealtimeBuffer<BufferType> commands_buffer_;
  unsigned int n_joints_;

 private:
  /// Subscriber for new LegCommandArray messages
  ros::Subscriber sub_command_;

  /// Store reference to gazebo joints
  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

  /// Map gazebo/urdf joint indices to leg/joint pair
  std::map<int, std::pair<int, int>> leg_map_;

  /// Torque limits for each motor
  std::vector<double> torque_lims_;
  std::vector<double> speed_lims_;

  void commandCB(const quad_msgs::LegCommandArrayConstPtr& msg);
<<<<<<< HEAD
  void enforceJointLimits(double& command, unsigned int index);
};  // class
=======
<<<<<<< HEAD:quad_simulator/gazebo_scripts/include/controller_plugin.h
  void enforceJointLimits(double& command, unsigned int index);
};  // class
=======
  void enforceJointLimits(double &command, unsigned int index);

}; // class
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*:spirit_simulator/gazebo_scripts/include/controller_plugin.h
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

}  // namespace effort_controllers
