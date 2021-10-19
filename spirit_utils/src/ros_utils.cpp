#include <spirit_utils/ros_utils.h>

namespace spirit_utils {

  void updateStateHeaders(spirit_msgs::RobotState &msg, ros::Time stamp,
    std::string frame, int traj_index) {

    // Fill in the data across the messages
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.header.seq = traj_index;
    msg.body.header = msg.header;
    msg.feet.header = msg.header;
    for (int i = 0; i < msg.feet.feet.size(); i++) {
      msg.feet.feet[i].header = msg.header;
    }
    msg.joints.header = msg.header;

    msg.traj_index = traj_index;
    msg.feet.traj_index = traj_index;
    for (int i = 0; i < msg.feet.feet.size(); i++) {
      msg.feet.feet[i].traj_index = traj_index;
    }
  }

  void interpHeader(std_msgs::Header header_1,std_msgs::Header header_2,
    double t_interp, std_msgs::Header &interp_header) {
    
    // Copy everything from the first header
    interp_header.frame_id = header_1.frame_id;
    interp_header.seq = header_1.seq;

    // Compute the correct ros::Time corresponding to t_interp
    t_interp = std::max(std::min(t_interp,1.0),0.0);
    ros::Duration state_duration = header_2.stamp - header_1.stamp;
    ros::Duration interp_duration = ros::Duration(t_interp*state_duration.toSec());
    interp_header.stamp = header_1.stamp + ros::Duration(interp_duration);
  }

  void interpOdometry(spirit_msgs::BodyState state_1, spirit_msgs::BodyState state_2, 
      double t_interp, spirit_msgs::BodyState &interp_state) {

    interpHeader(state_1.header, state_2.header,t_interp,interp_state.header);

    // Interp body position
    interp_state.pose.position.x = 
      math_utils::lerp(state_1.pose.position.x, state_2.pose.position.x, t_interp);
    interp_state.pose.position.y = 
      math_utils::lerp(state_1.pose.position.y, state_2.pose.position.y, t_interp);
    interp_state.pose.position.z = 
      math_utils::lerp(state_1.pose.position.z, state_2.pose.position.z, t_interp);
    
    // Interp body orientation with smath_utils::lerp
    tf2::Quaternion q_1, q_2, q_interp;
    tf2::convert(state_1.pose.orientation,q_1);
    tf2::convert(state_2.pose.orientation,q_2);
    q_interp = q_1.slerp(q_2, t_interp);
    interp_state.pose.orientation = tf2::toMsg(q_interp);

    // Interp twist
    interp_state.twist.linear.x = 
      math_utils::lerp(state_1.twist.linear.x, state_2.twist.linear.x, t_interp);
    interp_state.twist.linear.y = 
      math_utils::lerp(state_1.twist.linear.y, state_2.twist.linear.y, t_interp);
    interp_state.twist.linear.z = 
      math_utils::lerp(state_1.twist.linear.z, state_2.twist.linear.z, t_interp);

    interp_state.twist.angular.x = 
      math_utils::lerp(state_1.twist.angular.x, state_2.twist.angular.x, t_interp);
    interp_state.twist.angular.y = 
      math_utils::lerp(state_1.twist.angular.y, state_2.twist.angular.y, t_interp);
    interp_state.twist.angular.z = 
      math_utils::lerp(state_1.twist.angular.z, state_2.twist.angular.z, t_interp);
  }

  void interpJointState(sensor_msgs::JointState state_1,
    sensor_msgs::JointState state_2, double t_interp, sensor_msgs::JointState &interp_state) {

    interpHeader(state_1.header, state_2.header,t_interp,interp_state.header);

    // Interp joints
    interp_state.name.resize(state_1.position.size());
    interp_state.position.resize(state_1.position.size());
    interp_state.velocity.resize(state_1.position.size());
    interp_state.effort.resize(state_1.position.size());
    for (int i = 0; i < state_1.position.size(); i++) {
      interp_state.name[i] = state_1.name[i];
      interp_state.position[i] = math_utils::lerp(state_1.position[i], 
        state_2.position[i], t_interp);
      interp_state.velocity[i] = math_utils::lerp(state_1.velocity[i], 
        state_2.velocity[i], t_interp);
      interp_state.effort[i] = math_utils::lerp(state_1.effort[i], 
        state_2.effort[i], t_interp);
    }
  }

  void interpMultiFootState(spirit_msgs::MultiFootState state_1,
    spirit_msgs::MultiFootState state_2, double t_interp, spirit_msgs::MultiFootState &interp_state) {

    interpHeader(state_1.header, state_2.header,t_interp,interp_state.header);

    // Interp foot state
    interp_state.feet.resize(state_1.feet.size());
    for (int i = 0; i < interp_state.feet.size(); i++) {
      interp_state.feet[i].header = interp_state.header;

      interp_state.feet[i].position.x = math_utils::lerp(state_1.feet[i].position.x, 
        state_2.feet[i].position.x, t_interp);
      interp_state.feet[i].position.y = math_utils::lerp(state_1.feet[i].position.y, 
        state_2.feet[i].position.y, t_interp);
      interp_state.feet[i].position.z = math_utils::lerp(state_1.feet[i].position.z, 
        state_2.feet[i].position.z, t_interp);

      // Interp foot velocity
      interp_state.feet[i].velocity.x = math_utils::lerp(state_1.feet[i].velocity.x, 
        state_2.feet[i].velocity.x, t_interp);
      interp_state.feet[i].velocity.y = math_utils::lerp(state_1.feet[i].velocity.y, 
        state_2.feet[i].velocity.y, t_interp);
      interp_state.feet[i].velocity.z = math_utils::lerp(state_1.feet[i].velocity.z, 
        state_2.feet[i].velocity.z, t_interp);

      // Interp foot acceleration
      interp_state.feet[i].acceleration.x = math_utils::lerp(state_1.feet[i].acceleration.x, 
        state_2.feet[i].acceleration.x, t_interp);
      interp_state.feet[i].acceleration.y = math_utils::lerp(state_1.feet[i].acceleration.y, 
        state_2.feet[i].acceleration.y, t_interp);
      interp_state.feet[i].acceleration.z = math_utils::lerp(state_1.feet[i].acceleration.z, 
        state_2.feet[i].acceleration.z, t_interp);

      // Set contact state to the first state
      interp_state.feet[i].contact = state_1.feet[i].contact;
    }
  }

  void interpGRFArray(spirit_msgs::GRFArray state_1,
    spirit_msgs::GRFArray state_2, double t_interp, spirit_msgs::GRFArray &interp_state) {

    interpHeader(state_1.header, state_2.header,t_interp,interp_state.header);

    // Interp grf state
    interp_state.vectors.resize(state_1.vectors.size());
    interp_state.points.resize(state_1.points.size());
    interp_state.contact_states.resize(state_1.contact_states.size());
    for (int i = 0; i < interp_state.vectors.size(); i++) {

      interp_state.vectors[i].x = math_utils::lerp(state_1.vectors[i].x, 
        state_2.vectors[i].x, t_interp);
      interp_state.vectors[i].y = math_utils::lerp(state_1.vectors[i].y, 
        state_2.vectors[i].y, t_interp);
      interp_state.vectors[i].z = math_utils::lerp(state_1.vectors[i].z, 
        state_2.vectors[i].z, t_interp);

      interp_state.points[i].x = math_utils::lerp(state_1.points[i].x, 
        state_2.points[i].x, t_interp);
      interp_state.points[i].y = math_utils::lerp(state_1.points[i].y, 
        state_2.points[i].y, t_interp);
      interp_state.points[i].z = math_utils::lerp(state_1.points[i].z, 
        state_2.points[i].z, t_interp);

      // Set contact state to the first state
      interp_state.contact_states[i] = state_1.contact_states[i];

    }
  }

  void interpRobotState(spirit_msgs::RobotState state_1,
    spirit_msgs::RobotState state_2, double t_interp, spirit_msgs::RobotState &interp_state) {

    // Interp individual elements
    interpHeader(state_1.header, state_2.header, t_interp, interp_state.header);
    interpOdometry(state_1.body, state_2.body, t_interp, interp_state.body);
    interpJointState(state_1.joints, state_2.joints, t_interp, interp_state.joints);
    interpMultiFootState(state_1.feet, state_2.feet, t_interp, interp_state.feet);
  }

  void interpRobotPlan(spirit_msgs::RobotPlan msg, double t,
    spirit_msgs::RobotState &interp_state, int &interp_primitive_id,
    spirit_msgs::GRFArray &interp_grf) {    

    // Define some useful timing parameters
    ros::Time t0_ros = msg.states.front().header.stamp;
    ros::Time t_ros = t0_ros + ros::Duration(t);

    // Declare variables for interpolating between, both for input and output data
    spirit_msgs::RobotState state_1, state_2;
    int primitive_id_1, primitive_id_2;
    spirit_msgs::GRFArray grf_1, grf_2;

    // Find the correct index for interp (return the first index if t < 0)
    int index = 0;
    if (t>=0) {
      for(int i=0;i<msg.states.size()-1;i++)
      {
        index = i;
        if( msg.states[i].header.stamp<=t_ros && t_ros<msg.states[i+1].header.stamp)
        {
          break;
        }
      }
    }

    // Extract correct states
    state_1 = msg.states[index];
    state_2 = msg.states[index+1];
    primitive_id_1 = msg.primitive_ids[index];
    grf_1 = msg.grfs[index];
    grf_2 = msg.grfs[index+1];

    // Compute t_interp = [0,1]
    double t1, t2;
    ros::Duration t1_ros = state_1.header.stamp - t0_ros;
    t1 = t1_ros.toSec();
    ros::Duration t2_ros = state_2.header.stamp - t0_ros;
    t2 = t2_ros.toSec();
    double t_interp = (t - t1)/(t2-t1);

    // Compute interpolation
    interpRobotState(state_1, state_2, t_interp, interp_state);
    interp_primitive_id = primitive_id_1;
    interpGRFArray(grf_1, grf_2, t_interp, interp_grf);
  }

  spirit_msgs::MultiFootState interpMultiFootPlanContinuous(
    spirit_msgs::MultiFootPlanContinuous msg, double t) {

    // Define some useful timing parameters
    ros::Time t0_ros = msg.states.front().header.stamp;
    ros::Time t_ros = t0_ros + ros::Duration(t);

    // Declare variables for interpolating between, both for input and output data
    spirit_msgs::MultiFootState state_1, state_2, interp_state;

    // Find the correct index for interp (return the first index if t < 0)
    int index = 0;
    if (t>=0) {
      for(int i=0;i<msg.states.size()-1;i++)
      {
        index = i;
        if( msg.states[i].header.stamp<=t_ros && t_ros<msg.states[i+1].header.stamp)
        {
          break;
        }
      }
    }

    // Extract correct states
    state_1 = msg.states[index];
    state_2 = msg.states[index+1];

    // Compute t_interp = [0,1]
    double t1, t2;
    ros::Duration t1_ros = state_1.header.stamp - t0_ros;
    t1 = t1_ros.toSec();
    ros::Duration t2_ros = state_2.header.stamp - t0_ros;
    t2 = t2_ros.toSec();
    double t_interp = (t - t1)/(t2-t1);

    // Compute interpolation
    interpMultiFootState(state_1, state_2, t_interp, interp_state);
    
    return interp_state;
  }

  spirit_msgs::RobotState interpRobotStateTraj(spirit_msgs::RobotStateTrajectory
    msg, double t) {    

    // Define some useful timing parameters
    ros::Time t0_ros = msg.states.front().header.stamp;
    ros::Time tf_ros = msg.states.back().header.stamp;
    ros::Duration traj_duration = tf_ros - t0_ros;
    
    t = std::max(std::min(t,traj_duration.toSec()),0.0);
    ros::Time t_ros = t0_ros + ros::Duration(t);

    // Declare variables for interpolating between, both for input and output data
    spirit_msgs::RobotState state_1, state_2, interp_state;

    // Find the correct index for interp (return the first index if t < 0)
    int index = 0;
    if (t>=0) {
      for(int i=0;i<msg.states.size()-1;i++)
      {
        index = i;
        if( msg.states[i].header.stamp<=t_ros && t_ros<msg.states[i+1].header.stamp)
        {
          break;
        }
      }
    }

    // Extract correct states
    state_1 = msg.states[index];
    state_2 = msg.states[index+1];

    // Compute t_interp = [0,1]
    double t1, t2;
    ros::Duration t1_ros = state_1.header.stamp - t0_ros;
    t1 = t1_ros.toSec();
    ros::Duration t2_ros = state_2.header.stamp - t0_ros;
    t2 = t2_ros.toSec();
    double t_interp = (t - t1)/(t2-t1);

    // Compute interpolation
    interpRobotState(state_1, state_2, t_interp, interp_state);

    return interp_state;

  }

  void ikRobotState(const spirit_utils::QuadKD &kinematics,
    spirit_msgs::BodyState body_state, spirit_msgs::MultiFootState multi_foot_state,
    sensor_msgs::JointState &joint_state) {

    joint_state.header = multi_foot_state.header;
    // If this message is empty set the joint names
    if (joint_state.name.empty()) {
      joint_state.name = {"8", "0", "1", "9","2", "3", "10", "4","5", "11", "6", "7"};
    }
    joint_state.position.clear();
    joint_state.velocity.clear();
    joint_state.effort.clear();

    for (int i=0; i < multi_foot_state.feet.size(); i++) {

      // Get foot position data
      Eigen::Vector3d foot_pos;
      foot_pos[0] = multi_foot_state.feet[i].position.x;
      foot_pos[1] = multi_foot_state.feet[i].position.y;
      foot_pos[2] = multi_foot_state.feet[i].position.z;      

      // Get corresponding body plan data
      Eigen::Vector3d body_pos = {body_state.pose.position.x,
        body_state.pose.position.y,body_state.pose.position.z};

      tf2::Quaternion q;
      tf2::convert(body_state.pose.orientation,q);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      Eigen::Vector3d body_rpy = {roll,pitch,yaw};

      // Compute IK to get joint data
      Eigen::Vector3d leg_joint_state;
      kinematics.worldToFootIKWorldFrame(i,body_pos,body_rpy,foot_pos,leg_joint_state);

      // Add to the joint state vector
      joint_state.position.push_back(leg_joint_state[0]);
      joint_state.position.push_back(leg_joint_state[1]);
      joint_state.position.push_back(leg_joint_state[2]);

      joint_state.effort.push_back(0.0);
      joint_state.effort.push_back(0.0);
      joint_state.effort.push_back(0.0);
    }

    // Declare state data as Eigen vectors
    Eigen::VectorXd ref_body_state(12), ref_foot_positions(12), ref_foot_velocities(12);

    // Load state data
    ref_body_state = spirit_utils::bodyStateMsgToEigen(body_state);
    spirit_utils::multiFootStateMsgToEigen(
      multi_foot_state, ref_foot_positions, ref_foot_velocities);

    // Define vectors for joint positions and velocities
    Eigen::VectorXd joint_positions(12), joint_velocities(12);

    // Load joint positions
    spirit_utils::vectorToEigen(joint_state.position, joint_positions);

    // Define vectors for state positions
    Eigen::VectorXd state_positions(18);

    // Load state positions
    state_positions << joint_positions, ref_body_state.head(6);

    // Compute jacobian
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);
    kinematics.getJacobianBodyAngVel(state_positions, jacobian);

    // Compute joint velocities
    joint_velocities = jacobian.leftCols(12).colPivHouseholderQr().solve(
      ref_foot_velocities - jacobian.rightCols(6)*ref_body_state.tail(6));
    
    // Populate joint velocities message
    for (int i = 0; i < 12; ++i)
    {
      joint_state.velocity.push_back(joint_velocities(i));
    }
  }

  void ikRobotState(const spirit_utils::QuadKD &kinematics,
    spirit_msgs::RobotState &state) {
      
    ikRobotState(kinematics, state.body, state.feet, state.joints);
  }

  void fkRobotState(const spirit_utils::QuadKD &kinematics,
    spirit_msgs::BodyState body_state, sensor_msgs::JointState joint_state,
    spirit_msgs::MultiFootState &multi_foot_state) {

    multi_foot_state.header = joint_state.header;
    // If this message is empty set the joint names

    int num_feet = 4;
    multi_foot_state.feet.resize(num_feet);

    int joint_index = -1;
    for (int i=0; i < multi_foot_state.feet.size(); i++) {

      // Get joint data for indexed leg leg
      Eigen::Vector3d leg_joint_state;
      
      for (int j=0; j < 3; j++) {
        joint_index++;
        leg_joint_state[j] = joint_state.position.at(joint_index);
      }    

      // Get corresponding body plan data
      Eigen::Vector3d body_pos = {body_state.pose.position.x,
        body_state.pose.position.y,body_state.pose.position.z};

      tf2::Quaternion q;
      tf2::convert(body_state.pose.orientation,q);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      Eigen::Vector3d body_rpy = {roll,pitch,yaw};

      // Compute IK to get joint data
      Eigen::Vector3d foot_pos;
      kinematics.worldToFootFKWorldFrame(i,body_pos,body_rpy,leg_joint_state,foot_pos);

      // Add to the foot position vector
      multi_foot_state.feet[i].position.x = foot_pos[0];
      multi_foot_state.feet[i].position.y = foot_pos[1];
      multi_foot_state.feet[i].position.z = foot_pos[2];

      multi_foot_state.feet[i].header = multi_foot_state.header;
    }

    // Declare state data as Eigen vectors
    Eigen::VectorXd ref_body_state(12), foot_velocities(12);

    // Load state data
    ref_body_state = spirit_utils::bodyStateMsgToEigen(body_state);

    // Define vectors for joint positions and velocities
    Eigen::VectorXd joint_positions(12), joint_velocities(12);

    // Load joint positions
    spirit_utils::vectorToEigen(joint_state.position, joint_positions);

    // Load joint velocities
    spirit_utils::vectorToEigen(joint_state.velocity, joint_velocities);

    // Define vectors for state positions
    Eigen::VectorXd state_positions(18), state_velocities(18);

    // Load state positions
    state_positions << joint_positions, ref_body_state.head(6);

    // Load state velocities
    state_velocities << joint_velocities, ref_body_state.tail(6);

    // Compute jacobian
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);
    kinematics.getJacobianBodyAngVel(state_positions, jacobian);

    // Compute foot velocities
    foot_velocities = jacobian * state_velocities;

    // Populate foot velocities message
    for (int i = 0; i < multi_foot_state.feet.size(); ++i)
    {
      multi_foot_state.feet[i].velocity.x = foot_velocities(i * 3 + 0);
      multi_foot_state.feet[i].velocity.y = foot_velocities(i * 3 + 1);
      multi_foot_state.feet[i].velocity.z = foot_velocities(i * 3 + 2);
    }
  }

  void fkRobotState(const spirit_utils::QuadKD &kinematics,
    spirit_msgs::RobotState &state) {
    
    fkRobotState(kinematics, state.body, state.joints, state.feet);
  }

  spirit_msgs::BodyState eigenToBodyStateMsg(const Eigen::VectorXd &state) {
    
    spirit_msgs::BodyState state_msg;

    // Transform from RPY to quat msg
    tf2::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;
    quat_tf.setRPY(state[3],state[4],state[5]);
    quat_msg = tf2::toMsg(quat_tf);

    // Load the data into the message
    state_msg.pose.position.x = state[0];
    state_msg.pose.position.y = state[1];
    state_msg.pose.position.z = state[2];
    state_msg.pose.orientation = quat_msg;

    state_msg.twist.linear.x = state[6];
    state_msg.twist.linear.y = state[7];
    state_msg.twist.linear.z = state[8];
    state_msg.twist.angular.x = state[9];
    state_msg.twist.angular.y = state[10];
    state_msg.twist.angular.z = state[11];

    return state_msg;
  }

  Eigen::VectorXd bodyStateMsgToEigen(const spirit_msgs::BodyState &body) {

    Eigen::VectorXd state = Eigen::VectorXd::Zero(12);

    // Position
    state(0) = body.pose.position.x;
    state(1) = body.pose.position.y;
    state(2) = body.pose.position.z;

    // Orientation
    tf2::Quaternion quat;
    tf2::convert(body.pose.orientation, quat);
    double r,p,y;
    tf2::Matrix3x3 m(quat);
    m.getRPY(r,p,y);
    state(3) = r;
    state(4) = p;
    state(5) = y;

    // Linear Velocity
    state(6) = body.twist.linear.x;
    state(7) = body.twist.linear.y;
    state(8) = body.twist.linear.z;

    // Angular Velocity
    state(9) = body.twist.angular.x;
    state(10) = body.twist.angular.y;
    state(11) = body.twist.angular.z;

    return state;
  }

  void eigenToGRFArrayMsg(Eigen::VectorXd grf_array, spirit_msgs::MultiFootState multi_foot_state_msg,
    spirit_msgs::GRFArray &grf_msg) {

    grf_msg.vectors.clear();
    grf_msg.points.clear();
    grf_msg.contact_states.clear();
  
    for (int i = 0; i < multi_foot_state_msg.feet.size(); i++) {

      Eigen::Vector3d grf = grf_array.segment<3>(3*i);

      geometry_msgs::Vector3 vector_msg;
      vector_msg.x = grf[0];
      vector_msg.y = grf[1];
      vector_msg.z = grf[2];
      geometry_msgs::Point point_msg;
      point_msg.x = multi_foot_state_msg.feet[i].position.x;
      point_msg.y = multi_foot_state_msg.feet[i].position.y;
      point_msg.z = multi_foot_state_msg.feet[i].position.z;

      grf_msg.vectors.push_back(vector_msg);
      grf_msg.points.push_back(point_msg);

      bool contact_state = (grf.norm() >= 1e-6);
      grf_msg.contact_states.push_back(contact_state);
    }

  }

  Eigen::VectorXd grfArrayMsgToEigen(const spirit_msgs::GRFArray &grf_array_msg_) {
  
    Eigen::VectorXd grf_array(3*grf_array_msg_.vectors.size());

    for (int i = 0; i < grf_array_msg_.vectors.size(); i++) {

      grf_array(3*i) = grf_array_msg_.vectors[i].x;
      grf_array(3*i+1) = grf_array_msg_.vectors[i].y;
      grf_array(3*i+2) = grf_array_msg_.vectors[i].z;
    }

    return grf_array;

  }

  void footStateMsgToEigen(const spirit_msgs::FootState &foot_state_msg, 
    Eigen::Vector3d &foot_position) {
  
    foot_position[0] = foot_state_msg.position.x;
    foot_position[1] = foot_state_msg.position.y;
    foot_position[2] = foot_state_msg.position.z;

  }

  void multiFootStateMsgToEigen(const spirit_msgs::MultiFootState &multi_foot_state_msg, 
    Eigen::VectorXd &foot_positions) {
  
    for (int i = 0; i < multi_foot_state_msg.feet.size(); i++) {

      foot_positions[3*i] = multi_foot_state_msg.feet[i].position.x;
      foot_positions[3*i+1] = multi_foot_state_msg.feet[i].position.y;
      foot_positions[3*i+2] = multi_foot_state_msg.feet[i].position.z;

    }
  }

  void multiFootStateMsgToEigen(const spirit_msgs::MultiFootState &multi_foot_state_msg, Eigen::VectorXd &foot_positions,
                                Eigen::VectorXd &foot_velocities, Eigen::VectorXd &foot_acceleration)
  {
    multiFootStateMsgToEigen(multi_foot_state_msg, foot_positions, foot_velocities);

    for (int i = 0; i < multi_foot_state_msg.feet.size(); i++)
    {
      foot_acceleration[3 * i] = multi_foot_state_msg.feet[i].acceleration.x;
      foot_acceleration[3 * i + 1] = multi_foot_state_msg.feet[i].acceleration.y;
      foot_acceleration[3 * i + 2] = multi_foot_state_msg.feet[i].acceleration.z;
    }
  }

  void multiFootStateMsgToEigen(const spirit_msgs::MultiFootState &multi_foot_state_msg, 
    Eigen::VectorXd &foot_positions, Eigen::VectorXd &foot_velocities) {
  
    for (int i = 0; i < multi_foot_state_msg.feet.size(); i++) {

      foot_positions[3*i] = multi_foot_state_msg.feet[i].position.x;
      foot_positions[3*i+1] = multi_foot_state_msg.feet[i].position.y;
      foot_positions[3*i+2] = multi_foot_state_msg.feet[i].position.z;

      foot_velocities[3*i] = multi_foot_state_msg.feet[i].velocity.x;
      foot_velocities[3*i+1] = multi_foot_state_msg.feet[i].velocity.y;
      foot_velocities[3*i+2] = multi_foot_state_msg.feet[i].velocity.z;

    }
  }

  void eigenToFootStateMsg(Eigen::VectorXd foot_positions, Eigen::VectorXd foot_velocities,
                           Eigen::VectorXd foot_acceleration, spirit_msgs::FootState &foot_state_msg)
  {
    eigenToFootStateMsg(foot_positions, foot_velocities, foot_state_msg);

    foot_state_msg.acceleration.x = foot_acceleration[0];
    foot_state_msg.acceleration.y = foot_acceleration[1];
    foot_state_msg.acceleration.z = foot_acceleration[2];
  }

  void eigenToFootStateMsg(Eigen::VectorXd foot_positions, 
    Eigen::VectorXd foot_velocities, spirit_msgs::FootState &foot_state_msg) {

      foot_state_msg.position.x = foot_positions[0];
      foot_state_msg.position.y = foot_positions[1];
      foot_state_msg.position.z = foot_positions[2];

      foot_state_msg.velocity.x = foot_velocities[0];
      foot_state_msg.velocity.y = foot_velocities[1];
      foot_state_msg.velocity.z = foot_velocities[2];

  }

  void eigenToVector(const Eigen::VectorXd &eigen_vec, std::vector<double> &vec) {
    vec.resize(eigen_vec.size());
    for (int i = 0; i < eigen_vec.size(); i++) {
      vec[i] = eigen_vec(i);
    }
  }

  void vectorToEigen(const std::vector<double> &vec, Eigen::VectorXd &eigen_vec) {
    eigen_vec.resize(vec.size());
    for (int i = 0; i < vec.size(); i++) {
      eigen_vec(i) = vec[i];
    }
  }
}