#include "open_loop_controller/silly_walk_template.h"

SillyWalkTemplate::SillyWalkTemplate(ros::NodeHandle nh) {
  
  // Assign the node handle to the class
  nh_ = nh;

  // Get rosparams from the server
  std::string joint_command_topic,control_mode_topic;
  spirit_utils::loadROSParam(nh_,"topics/control/joint_command",joint_command_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/mode",control_mode_topic);
  spirit_utils::loadROSParam(nh_,"silly_walk_template/update_rate",update_rate_);
  spirit_utils::loadROSParam(nh_,"silly_walk_template/stand_angles",stand_joint_angles_);
  
  // Setup pubs and subs
  joint_control_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(joint_command_topic,1);
  control_mode_sub_ = nh_.subscribe(control_mode_topic,1,&SillyWalkTemplate::controlModeCallback, this);

  // Add any other class initialization goes here
  control_mode_ = SIT;
}

void SillyWalkTemplate::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
  
  // Use this to set any logic for control modes (see inverse_dynamics for more examples)
  if (msg->data == SIT || (msg->data == STAND) || msg->data == HOP)
  {  
    control_mode_ = msg->data;
  }
}

void SillyWalkTemplate::computeJointControl(double t)
{
  
  // Put your control code here
  control_msg_.leg_commands.clear();
  control_msg_.leg_commands.resize(num_legs_);

  // The SpiritKinematics class can help do basic kinematic computations (with type Eigen::VectorXd)
  // For example: kinematics_.legIK(leg_index, body_pos, body_rpy, foot_pos_world,joint_state);
  // See inverse_dynamics for more elaborate implementations

  // You can use something like this if you want a state machine
  // (This can be useful to implement basic stop/go functionality)

  std::cout<< "command set at"  << t << std::endl;
  if (control_mode_ == SIT) {
    for (int i = 0; i < 4; ++i)
    {
      control_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j)
      {
        control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 0;
        control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 5;
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
        if (j == 2 && (i == 0 || i == 2)){
          control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 5;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 0.1;
          control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }
        else{
          control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 5;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 0.1;
          control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }

        
      }
    }
  }else if (control_mode_ == HOP) {
    for (int i = 0; i < 4; ++i)
    {
      control_msg_.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j)
      {
        if (j == 2 && (i == 1 || i == 3)){
          double diff = t - floor(t);
          double angleBack = 2.0 +(sin(diff * (8*M_PI))) * 1;
          std::cout << "at " <<diff << " leg angle at back" << angleBack << std::endl;
          control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = angleBack;
          control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 5;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 0.1;
          control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;

        }else if (j == 2 && (i == 0 || i == 2)){
          double diff = t - floor(t);
          double angleFront = 1.6 +(sin(diff * (8*M_PI))) * 0.8;
          std::cout << "at " <<diff << " leg angle at front" << angleFront << std::endl;
          control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = angleFront;
          control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 5;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 0.1;
          control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;


        }
        else{
          control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
          control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 5;
          control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 0.1;
          control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
        }
      }
      // control_msg_.leg_commands.at(i).motor_commands.at(2).pos_setpoint = 1.5+(1 +sin(t))*0.56;

      
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

    double t = ros::Time::now().sec + ros::Time::now().nsec * 0.000000001;

    if (t <= 2){
      control_mode_ = SIT;
    }else if (t > 2 && t <= 5){
      control_mode_ = STAND;
    }else{
      control_mode_ = HOP;
    }

    // std::cout << "current mode is " << control_mode_ << std::endl;

    // Compute and publish the control
    // Doesn't need to be structured this way but keep spin() succinct
    this->computeJointControl(t);
    this->publishJointControl();

    // Always include this to keep the subscribers up to date and the update rate constant
    ros::spinOnce();
    r.sleep();
  }
}