#include "leg_controller/grf_pid_controller.h"

GrfPidController::GrfPidController() {
  pos_error_int_.setZero();
  ang_error_int_.setZero();
  t_old_ = ros::Time::now();
  ang_des_.setZero();
}

void GrfPidController::setDesiredState(const quad_msgs::RobotState::ConstPtr &init_robot_state_msg) {
  Eigen::VectorXd body_state(12);
  body_state = quad_utils::bodyStateMsgToEigen(init_robot_state_msg->body);
  ang_des_ << 0, 0, body_state(5);
  // double x_mean = 0;
  // double y_mean = 0;
  // for (int i = 0; i < num_feet_; i++) {
  //   x_mean += init_robot_state_msg
  // }
  // pos_des_.x() = 0;//init_robot_state_msg->body.pose.position.x;
  // pos_des_.y() = 0;//init_robot_state_msg->body.pose.position.y;
  // pos_des_.z() = 2*0.206*sin(M_PI*0.25);
}

bool GrfPidController::computeLegCommandArray(
  const quad_msgs::RobotState::ConstPtr &robot_state_msg,
  quad_msgs::LegCommandArray &leg_command_array_msg,
  quad_msgs::GRFArray &grf_array_msg)
{
  leg_command_array_msg.leg_commands.resize(num_feet_);

  // Get desired x/y location
  double x_mean = 0;
  double y_mean = 0;
  for (int i = 0; i < num_feet_; i++) {
    x_mean += robot_state_msg->feet.feet[i].position.x/(num_feet_);
    y_mean += robot_state_msg->feet.feet[i].position.y/(num_feet_);
  }
  pos_des_.x() = x_mean;
  pos_des_.y() = y_mean;
  pos_des_.z() = 2*0.206*sin(M_PI*0.25);

  // Define vectors for joint positions and velocities
  Eigen::VectorXd joint_positions(3*num_feet_), joint_velocities(3*num_feet_), body_state(12);
  quad_utils::vectorToEigen(robot_state_msg->joints.position, joint_positions);
  quad_utils::vectorToEigen(robot_state_msg->joints.velocity, joint_velocities);
  body_state = quad_utils::bodyStateMsgToEigen(robot_state_msg->body);

  // Define vectors for state positions and velocities 
  Eigen::VectorXd state_positions(3*num_feet_+6), state_velocities(3*num_feet_+6);
  state_positions << joint_positions, body_state.head(6);
  state_velocities << joint_velocities, body_state.tail(6);

  // Initialize variables for ff and fb
  quad_msgs::RobotState ref_state_msg;
  Eigen::VectorXd tau_array(3*num_feet_), tau_swing_leg_array(3*num_feet_);

  // Declare plan and state data as Eigen vectors
  Eigen::VectorXd grf_array(3 * num_feet_),ref_foot_acceleration(3 * num_feet_);
  grf_array.setZero();
  ref_foot_acceleration.setZero();

  // Load model and desired pos data
  double m = 11.5;
  double g = 9.81;
  double pos_kp = 1e3;
  double ang_kp = 1e2;
  double pos_ki = 0.5*pos_kp;
  double ang_ki = 0.5*ang_kp;
  double pos_kd = 0.1*pos_kp;
  double ang_kd = 0.1*ang_kp;
  Eigen::Vector3d grf_array_ff;
  grf_array_ff << 0, 0, m*g*0.25;

  // Define error terms
  Eigen::Vector3d pos_error = body_state.segment<3>(0) - pos_des_;
  Eigen::Vector3d ang_error = body_state.segment<3>(3) - ang_des_;
  Eigen::Vector3d vel_error = body_state.segment<3>(6);
  Eigen::Vector3d ang_vel_error = body_state.segment<3>(9);


  ros::Time t_now = ros::Time::now();
  pos_error_int_ += pos_error*std::min((t_now - t_old_).toSec(), 0.01);
  ang_error_int_ += ang_error*std::min((t_now - t_old_).toSec(), 0.01);
  if (pos_error_int_.norm() >= 0.5)
    pos_error_int_.setZero();
  if (ang_error_int_.norm() >= 0.5)
    ang_error_int_.setZero();

  t_old_ = t_now;

  for (int i = 0; i < num_feet_; i++) {
    Eigen::Vector3d ang_dir(3);
    ang_dir << ((i <= 1) ? 1 : -1), ((i % 2 == 0) ? -1 : 1), 0;

    grf_array.segment<3>(3*i) = grf_array_ff.array() - pos_kp*pos_error.array() - 
      pos_kd*vel_error.array() - pos_ki*pos_error_int_.array();
    grf_array.segment<3>(3*i).z() += -ang_kp*ang_dir.dot(ang_error) - 
      ang_kd*ang_dir.dot(ang_vel_error) - ang_ki*ang_dir.dot(ang_error_int_);
    double yaw_fb = -ang_kp*(-ang_dir.y())*ang_error.z() - 
      ang_kp*(-ang_dir.y())*ang_vel_error.z()- ang_ki*(-ang_dir.y())*ang_error_int_.z();
    
    grf_array.segment<3>(3*i).x() += -yaw_fb*sin(body_state(5));
    grf_array.segment<3>(3*i).y() += yaw_fb*cos(body_state(5));
  }

  // Load contact mode
  std::vector<int> contact_mode = {1,1,1,1};

  // Compute joint torques
  quadKD_->computeInverseDynamics(state_positions, state_velocities, ref_foot_acceleration,grf_array,
    contact_mode, tau_array);

  for (int i = 0; i < num_feet_; ++i) {
    leg_command_array_msg.leg_commands.at(i).motor_commands.resize(3);
    for (int j = 0; j < 3; ++j) {

      int joint_idx = 3*i+j;

      leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 0.0;
      leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0.0;
      leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).torque_ff =
          tau_array(joint_idx);

      leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp = 0.0;
      leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd = 0.0;

    }
  }

  quad_utils::eigenToGRFArrayMsg(grf_array, robot_state_msg->feet, grf_array_msg);

  return true;
  
}