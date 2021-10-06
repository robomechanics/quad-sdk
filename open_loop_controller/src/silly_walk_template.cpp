#include "open_loop_controller/silly_walk_template.h"

SillyWalkTemplate::SillyWalkTemplate(ros::NodeHandle nh) {
  
  // Assign the node handle to the class
  nh_ = nh;

  // Get rosparams from the server
  std::string joint_command_topic,control_mode_topic;
  spirit_utils::loadROSParam(nh_,"topics/control/joint_command",joint_command_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/mode",control_mode_topic);
  spirit_utils::loadROSParam(nh_,"silly_walk_template/update_rate",update_rate_);
  
  // Setup pubs and subs
  joint_control_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(joint_command_topic,1);
  control_mode_sub_ = nh_.subscribe(control_mode_topic,1,&SillyWalkTemplate::controlModeCallback, this);

  // Add any other class initialization goes here
  control_mode_ = SIT;
}

void SillyWalkTemplate::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
  
  // Use this to set any logic for control modes (see inverse_dynamics for more examples)
  if (msg->data == SIT || (msg->data == STANDUP) || (msg->data == STAND) || (msg->data == WALK_1) || (msg->data == WALK_2))
  {  
    control_mode_ = msg->data;
    transition_timestamp_ = ros::Time::now();
  }
}

void SillyWalkTemplate::computeJointControl()
{
  // Put your control code here
  control_msg_.leg_commands.clear();
  control_msg_.leg_commands.resize(num_legs_);

  // Define static position setpoints and gains
  stand_joint_angles_ = {0,0.76,2*0.76};
  sit_joint_angles_ = {0.0,0.0,0.0};
  swing_joint_angles_ = {0,0.3,2*0.3};
  td_joint_angles_ = {0,0.70,2*0.76};

  // The SpiritKinematics class can help do basic kinematic computations (with type Eigen::VectorXd)
  // For example: kinematics_.legIK(leg_index, body_pos, body_rpy, foot_pos_world,joint_state);
  // See inverse_dynamics for more elaborate implementations

  // You can use something like this if you want a state machine
  // (This can be useful to implement basic stop/go functionality)
  if (control_mode_ == SIT) {
    for (int i = 0; i < 4; ++i)
    {
      control_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j)
      {
        control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 0;
        control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 10;
        control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 0.1;
        control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
      }
    }
  } else if (control_mode_ == STAND) {
    for (int i = 0; i < 4; ++i)
      {
        control_msg_.leg_commands.at(i).motor_commands.resize(3);
        for (int j = 0; j < 3; ++j)
        {
          control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 100;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 4;
          control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }
      }
  } else if (control_mode_ == STANDUP) {
    ros::Duration duration = ros::Time::now() - transition_timestamp_;
    double t_interp = duration.toSec()/transition_duration_;

    if (t_interp >= transition_duration_) {
      control_mode_ = STAND;
      return;
    }

    for (int i = 0; i < num_legs_; ++i) {
      control_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          (stand_joint_angles_.at(j) - sit_joint_angles_.at(j))*t_interp + 
          sit_joint_angles_.at(j);
        control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 50;
        control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 2;
        control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
      }
    }
  } else if (control_mode_ == WALK_1) {
    ros::Duration duration = ros::Time::now() - transition_timestamp_;
    double t_interp = duration.toSec()/transition_duration_;

    if (t_interp >= transition_duration_) {
      control_mode_ = WALK_2;
      transition_timestamp_ = ros::Time::now();
      return;
    } else if (t_interp >= 0.5*transition_duration_) {
      for (int i = 0; i < num_legs_; i = ++i) {
        control_msg_.leg_commands.at(i).motor_commands.resize(3);
        for (int j = 0; j < 3; ++j) {
          if (i==0 || i==3) {
            control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
              (swing_joint_angles_.at(j) - stand_joint_angles_.at(j))*(2*(t_interp-0.5*transition_duration_)) + 
              stand_joint_angles_.at(j);
          } else {
            control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          }
          control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 50;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 2;
          control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }
      }
    } else {
      for (int i = 0; i < num_legs_; i = ++i) {
        control_msg_.leg_commands.at(i).motor_commands.resize(3);
        for (int j = 0; j < 3; ++j) {
          if (i==0 || i==3) {
            control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
              (td_joint_angles_.at(j) - swing_joint_angles_.at(j))*2*t_interp + 
              swing_joint_angles_.at(j);
          } else {
            control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          }
          control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 50;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 2;
          control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }
      }
    }
  } else if (control_mode_ == WALK_2) {
    ros::Duration duration = ros::Time::now() - transition_timestamp_;
    double t_interp = duration.toSec()/transition_duration_;

    if (t_interp >= transition_duration_) {
      control_mode_ = WALK_1;
      transition_timestamp_ = ros::Time::now();
      return;
    } else if (t_interp >= 0.5*transition_duration_) {
      for (int i = 0; i < num_legs_; i = ++i) {
        control_msg_.leg_commands.at(i).motor_commands.resize(3);
        for (int j = 0; j < 3; ++j) {
          if (i==1 || i==2) {
            control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
              (swing_joint_angles_.at(j) - stand_joint_angles_.at(j))*(2*(t_interp-0.5*transition_duration_)) + 
              stand_joint_angles_.at(j);
          } else {
            control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          }
          control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 50;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 2;
          control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }
      }
    } else {
      for (int i = 0; i < num_legs_; i = ++i) {
        control_msg_.leg_commands.at(i).motor_commands.resize(3);
        for (int j = 0; j < 3; ++j) {
          if (i==1 || i==2) {
            control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
              (td_joint_angles_.at(j) - swing_joint_angles_.at(j))*2*t_interp + 
              swing_joint_angles_.at(j);
          } else {
            control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          }
          control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 50;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 2;
          control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }
      }
    }
  }
}

void SillyWalkTemplate::publishJointControl()
{
  // Always need to set the timestamp
  control_msg_.header.stamp = ros::Time::now();

  // Publish the message
  joint_control_pub_.publish(control_msg_);
}

void SillyWalkTemplate::spin() {

  // Set update rate and do any other pre-loop stuff
  ros::Rate r(update_rate_);

  // Enter the main loop
  while (ros::ok()) {

    // Compute and publish the control
    // Doesn't need to be structured this way but keep spin() succinct
    this->computeJointControl();
    this->publishJointControl();

    // Always include this to keep the subscribers up to date and the update rate constant
    ros::spinOnce();
    r.sleep();
  }
}