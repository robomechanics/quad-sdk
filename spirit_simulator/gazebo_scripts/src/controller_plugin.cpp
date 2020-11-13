#include "controller_plugin.h"
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>


namespace effort_controllers
{

/**
 * \brief Forward command controller for a set of effort controlled joints (torque or force).
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 * \section ROS interface
 *
 * \param type Must be "JointGroupEffortController".
 * \param joints List of names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The joint efforts to apply
 */
  SpiritController::SpiritController() {}
  SpiritController::~SpiritController() {sub_command_.shutdown();}

  bool SpiritController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    // Get URDF
    urdf::Model urdf;
    if (!urdf.initParam("robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    for(unsigned int i=0; i<n_joints_; i++)
    {
      const auto& joint_name = joint_names_[i];

      try
      {
        joints_.push_back(hw->getHandle(joint_name));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

      urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
      if (!joint_urdf)
      {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        return false;
      }
      joint_urdfs_.push_back(joint_urdf);
    }

    commands_buffer_.writeFromNonRT(BufferType(n_joints_));

    sub_command_ = n.subscribe<spirit_msgs::MotorCommandArray>("command", 1, &SpiritController::commandCB, this);
    return true;
  }

  void SpiritController::update(const ros::Time& time, const ros::Duration& period)
  {
    BufferType & commands = *commands_buffer_.readFromRT();
    for(unsigned int i=0; i<n_joints_; i++)
    {
        // Collect feedforward torque 
        double torque_ff = commands.at(i).torque_ff;
        
        // Compute position error
        double command_position = commands.at(i).pos_setpoint;
        enforceJointLimits(command_position, i);
        double current_position = joints_.at(i).getPosition();
        double kp = commands.at(i).kp;
        double pos_error;
        angles::shortest_angular_distance_with_large_limits(
          current_position,
          command_position,
          joint_urdfs_[i]->limits->lower,
          joint_urdfs_[i]->limits->upper,
          pos_error);

        // Compute velocity error
        double current_vel = joints_.at(i).getVelocity();
        double command_vel = commands.at(i).vel_setpoint;
        double vel_error = command_vel - current_vel;
        double kd = commands.at(i).kd;

        // Collect feedback 
        double torque_feedback = kp * pos_error + kd * vel_error;

        // Update joint torque
        joints_.at(i).setCommand(torque_feedback + torque_ff);
    }
  }

  void SpiritController::commandCB(const spirit_msgs::MotorCommandArrayConstPtr& msg)
  {
    if(msg->motor_commands.size()!=n_joints_)
    {
      ROS_ERROR_STREAM("Dimension of command (" << msg->motor_commands.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return;
    }
    commands_buffer_.writeFromNonRT(msg->motor_commands);
  }

  void SpiritController::enforceJointLimits(double &command, unsigned int index)
  {
    // Check that this joint has applicable limits
    if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
    {
      if( command > joint_urdfs_[index]->limits->upper ) // above upper limnit
      {
        command = joint_urdfs_[index]->limits->upper;
      }
      else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
      {
        command = joint_urdfs_[index]->limits->lower;
      }
    }
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( effort_controllers::SpiritController, controller_interface::ControllerBase)
