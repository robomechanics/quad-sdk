#include "open_loop_controller/silly_walk_template.h"
#include "spirit_utils/kinematics.h"
#include "spirit_utils/math_utils.h"

using namespace spirit_utils;
SillyWalkTemplate::SillyWalkTemplate(ros::NodeHandle nh) {
  
  // Assign the node handle to the class
  nh_ = nh;

  // Get rosparams from the server
  std::string joint_command_topic,control_mode_topic, joint_state_topic;
  spirit_utils::loadROSParam(nh_,"topics/control/joint_command",joint_command_topic);
  spirit_utils::loadROSParam(nh_,"topics/control/mode",control_mode_topic);
  spirit_utils::loadROSParam(nh_,"silly_walk_template/update_rate",update_rate_);
  spirit_utils::loadROSParam(nh_,"silly_walk_template/stand_angles",stand_joint_angles_);
  spirit_utils::loadROSParam(nh_,"topics/spirit/joint_states",joint_state_topic);
  
  // Setup pubs and subs
  joint_control_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(joint_command_topic,1);
  control_mode_sub_ = nh_.subscribe(control_mode_topic,1,&SillyWalkTemplate::controlModeCallback, this);
  joint_state_sub_ = nh_.subscribe(joint_state_topic,1,&SillyWalkTemplate::jointStateCallback, this);
  // Add any other class initialization goes here
  control_mode_ = SIT;

  // Initialize foot position arrays
  foot_positions_body_ = Eigen::MatrixXd::Zero(num_legs_,3);

  // Convert kinematics
  kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();
}

void SillyWalkTemplate::controlModeCallback(const std_msgs::UInt8::ConstPtr& msg) {
  
  // Use this to set any logic for control modes (see inverse_dynamics for more examples)
  if (msg->data == SIT || (msg->data == STAND))
  {  
    control_mode_ = msg->data;
  }
}

void SillyWalkTemplate::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
  if (msg->position.empty())
    return;
  joint_state_angles_ = msg->position;
  // Calculate the foot position in body frame
  for (int i = 0; i < num_legs_; i++){
    Eigen::Vector3d joint_state_i;
    Eigen::Vector3d foot_positions_body; 
    joint_state_i << joint_state_angles_[3*i], joint_state_angles_[3*i+1], joint_state_angles_[3*i+2];
    kinematics_->bodyToFootFK(i, joint_state_i, foot_positions_body);
    foot_positions_body_.row(i) = foot_positions_body;
  }
}

Eigen::Matrix3d SillyWalkTemplate::setupTrajectory(Eigen::Vector3d init_point_){
  // get the current leg state
  Eigen::Vector3d end_point_ = init_point_;
  end_point_(0) += 0.08;
  Eigen::Vetor3d mid_point_ = (init_point_ + end_point_)/2;
  mid_point_(2) += 0.05;
  std::vector<double> input_vec{0,4,8};
  std::vector<Eigen::Vector3d> output_mat{init_point_, mid_point_, end_point_};
  Eigen::Matrix3d traj = Eigen::Matrax3d::Zeros(9,3)
  for (int i=0; i<=8; i++){
    traj.rows(i) = math_utils::interpVector3d(init_point_, output_mat, i);
  }
  return traj;
}


bool SillyWalkTemplate::computeNextFlight() {
  Eigen::Matrix3d traj = setupTrajectory(foot_positions_body_.row(next_flight));
  publishLocalPlan();
}

bool SillyWalkTemplate::finishFlight(int leg_number_) {
  if(isReached(leg_number_)){
    if(flight_mode!=4){
      next_flight ++;
    }
    else{
      next_flight = 1;
    }
    flight_mode=0;
  }
}

void SillyWalkTemplate::publishLocalPlan() {
  spirit_msgs::RobotPlan local_plan_msg;

  ros::Time timestamp = ros::Time::now();
  local_plan_msg.header.stamp = timestamp;
  
  for (int i = 0; i < N_; i++){
    spirit_msgs::RobotState robot_state_msg;
    robot_state
  }
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


void SillyWalkTemplate::computeJointControl()
{
  // Put your control code here
  control_msg_.leg_commands.clear();
  control_msg_.leg_commands.resize(num_legs_);

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
        control_msg_.leg_commands.at(i).motor_commands.at(j).pos_setpoint = stand_joint_angles_.at(j);
        control_msg_.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0;
        control_msg_.leg_commands.at(i).motor_commands.at(j).kp = 5;
        control_msg_.leg_commands.at(i).motor_commands.at(j).kd = 0.1;
        control_msg_.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
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


