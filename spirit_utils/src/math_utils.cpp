#include <spirit_utils/math_utils.h>

inline double math_utils::lerp(double a, double b, double t) {
  return (a+t*(b-a));
}

std::vector<double> math_utils::interpMat(std::vector<double> input_vec, 
  std::vector<std::vector<double>> input_mat, double query_point) {

  // Check bounds, throw an error if invalid since this shouldn't ever happen
  if ((query_point < input_vec.front()) || (query_point > input_vec.back())){
    throw std::runtime_error("Tried to interp out of bounds");
  }

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  std::vector<double> y1, y2, interp_data;

  // Find the correct values to interp between
  for(int i=0;i<input_vec.size();i++)
  {
    if(input_vec[i]<=query_point && query_point<input_vec[i+1])
    {
      t1 = input_vec[i];
      t2 = input_vec[i+1];
      y1 = input_mat[i];
      y2 = input_mat[i+1]; 
      break;
    }
  }

  // Apply linear interpolation for each element in the vector
  for (int i = 0; i<input_mat.front().size(); i++) {
    double result = y1[i] + (y2[i]-y1[i])/(t2-t1)*(query_point-t1);
    interp_data.push_back(result);
  }
  
  return interp_data;
}

Eigen::Vector3d math_utils::interpVector3d(std::vector<double> input_vec, std::vector<Eigen::Vector3d> input_mat, double query_point) {

  // Check bounds, throw an error if invalid since this shouldn't ever happen
  if ((query_point < input_vec.front()) || (query_point > input_vec.back())){
    throw std::runtime_error("Tried to interp out of bounds");
  }

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  Eigen::Vector3d y1, y2, interp_data;

  // Find the correct values to interp between
  for(int i=0;i<input_vec.size();i++)
  {
    if(input_vec[i]<=query_point && query_point<input_vec[i+1])
    {
      t1 = input_vec[i];
      t2 = input_vec[i+1];
      y1 = input_mat[i];
      y2 = input_mat[i+1]; 
      break;
    }
  }

  // Apply linear interpolation for each element in the vector
  interp_data = y1.array() + (y2.array()-y1.array())/(t2-t1)*(query_point-t1);
  
  return interp_data;
}

std::vector<Eigen::Vector3d> math_utils::interpMatVector3d(std::vector<double> input_vec, 
  std::vector<std::vector<Eigen::Vector3d>> output_mat, double query_point) {

  // Check bounds, throw an error if invalid since this shouldn't ever happen
  if ((query_point < input_vec.front()) || (query_point > input_vec.back())){
    throw std::runtime_error("Tried to interp out of bounds");
  }

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  std::vector<Eigen::Vector3d> y1, y2, interp_data;

  // Find the correct values to interp between
  for(int i=0;i<input_vec.size();i++)
  {
    if(input_vec[i]<=query_point && query_point<input_vec[i+1])
    {
      t1 = input_vec[i];
      t2 = input_vec[i+1];
      y1 = output_mat[i];
      y2 = output_mat[i+1]; 
      break;
    }
  }

  // Apply linear interpolation for each element in the vector
  for (int i = 0; i<output_mat.front().size(); i++) {
    Eigen::Vector3d interp_eigen_vec;
    interp_eigen_vec = y1[i].array() + (y2[i].array()-y1[i].array())/(t2-t1)*(query_point-t1);
    interp_data.push_back(interp_eigen_vec);
  }
  
  return interp_data;
}

void math_utils::interpHeader(std_msgs::Header header_1,std_msgs::Header header_2,
  double t_interp, std_msgs::Header &interp_header) {
  
  // Copy everything from the first header
  interp_header.frame_id = header_1.frame_id;
  interp_header.seq = header_1.seq;

  // Compute the correct ros::Time corresponding to t_interp
  t_interp = std::min(std::max(t_interp,1.0),0.0);
  ros::Duration state_duration = header_2.stamp - header_1.stamp;
  ros::Duration interp_duration = ros::Duration(t_interp*state_duration.toSec());
  interp_header.stamp = header_1.stamp + ros::Duration(interp_duration);
}

void math_utils::interpOdometry(nav_msgs::Odometry state_1, nav_msgs::Odometry state_2, 
    double t_interp, nav_msgs::Odometry &interp_state) {

  interpHeader(state_1.header, state_2.header,t_interp,interp_state.header);

  // Interp body position
  interp_state.pose.pose.position.x = 
    lerp(state_1.pose.pose.position.x, state_2.pose.pose.position.x, t_interp);
  interp_state.pose.pose.position.y = 
    lerp(state_1.pose.pose.position.y, state_2.pose.pose.position.y, t_interp);
  interp_state.pose.pose.position.z = 
    lerp(state_1.pose.pose.position.z, state_2.pose.pose.position.z, t_interp);
  
  // Interp body orientation with slerp
  tf2::Quaternion q_1, q_2, q_interp;
  tf2::convert(state_1.pose.pose.orientation,q_1);
  tf2::convert(state_2.pose.pose.orientation,q_2);
  q_interp = q_1.slerp(q_2, t_interp);
  interp_state.pose.pose.orientation = tf2::toMsg(q_interp);

  // Interp twist
  interp_state.twist.twist.linear.x = 
    lerp(state_1.twist.twist.linear.x, state_2.twist.twist.linear.x, t_interp);
  interp_state.twist.twist.linear.y = 
    lerp(state_1.twist.twist.linear.y, state_2.twist.twist.linear.y, t_interp);
  interp_state.twist.twist.linear.z = 
    lerp(state_1.twist.twist.linear.z, state_2.twist.twist.linear.z, t_interp);

  interp_state.twist.twist.angular.x = 
    lerp(state_1.twist.twist.angular.x, state_2.twist.twist.angular.x, t_interp);
  interp_state.twist.twist.angular.y = 
    lerp(state_1.twist.twist.angular.y, state_2.twist.twist.angular.y, t_interp);
  interp_state.twist.twist.angular.z = 
    lerp(state_1.twist.twist.angular.z, state_2.twist.twist.angular.z, t_interp);
}

void math_utils::interpJointState(sensor_msgs::JointState state_1,
  sensor_msgs::JointState state_2, double t_interp, sensor_msgs::JointState &interp_state) {

  interpHeader(state_1.header, state_2.header,t_interp,interp_state.header);

  // Interp joints
  interp_state.name.resize(state_1.position.size());
  interp_state.position.resize(state_1.position.size());
  interp_state.velocity.resize(state_1.position.size());
  interp_state.effort.resize(state_1.position.size());
  for (int i = 0; i < state_1.position.size(); i++) {
    interp_state.name[i] = state_1.name[i];
    interp_state.position[i] = lerp(state_1.position[i], 
      state_2.position[i], t_interp);
    interp_state.velocity[i] = lerp(state_1.velocity[i], 
      state_2.velocity[i], t_interp);
    interp_state.effort[i] = lerp(state_1.effort[i], 
      state_2.effort[i], t_interp);
  }
}

void math_utils::interpMultiFootState(spirit_msgs::MultiFootState state_1,
  spirit_msgs::MultiFootState state_2, double t_interp, spirit_msgs::MultiFootState &interp_state) {

  interpHeader(state_1.header, state_2.header,t_interp,interp_state.header);

  // Interp foot state
  interp_state.feet.resize(state_1.feet.size());
  for (int i = 0; i < interp_state.feet.size(); i++) {
    interp_state.feet[i].header = interp_state.header;

    interp_state.feet[i].position.x = lerp(state_1.feet[i].position.x, 
      state_2.feet[i].position.x, t_interp);
    interp_state.feet[i].position.y = lerp(state_1.feet[i].position.y, 
      state_2.feet[i].position.y, t_interp);
    interp_state.feet[i].position.z = lerp(state_1.feet[i].position.z, 
      state_2.feet[i].position.z, t_interp);

    // Interp foot velocity
    interp_state.feet[i].velocity.x = lerp(state_1.feet[i].velocity.x, 
      state_2.feet[i].velocity.x, t_interp);
    interp_state.feet[i].velocity.y = lerp(state_1.feet[i].velocity.y, 
      state_2.feet[i].velocity.y, t_interp);
    interp_state.feet[i].velocity.z = lerp(state_1.feet[i].velocity.z, 
      state_2.feet[i].velocity.z, t_interp);

    // Set contact state to the first state
    interp_state.feet[i].contact = state_1.feet[i].contact;
  }
}

void math_utils::interpRobotState(spirit_msgs::RobotState state_1,
  spirit_msgs::RobotState state_2, double t_interp, spirit_msgs::RobotState &interp_state) {

  // Interp individual elements
  interpHeader(state_1.header, state_2.header, t_interp, interp_state.header);
  interpOdometry(state_1.body, state_2.body, t_interp, interp_state.body);
  interpJointState(state_1.joints, state_2.joints, t_interp, interp_state.joints);
  interpMultiFootState(state_1.feet, state_2.feet, t_interp, interp_state.feet);
}

nav_msgs::Odometry math_utils::interpBodyPlan(spirit_msgs::BodyPlan msg, double t) {    

  // Define some useful timing parameters
  ros::Time t0_ros = msg.states.front().header.stamp;
  ros::Time t_ros = t0_ros + ros::Duration(t);

  // Declare variables for interpolating between, both for input and output data
  nav_msgs::Odometry state_1, state_2, interp_state;
  int primitive_id_1, primitive_id_2, interp_primitive_id;
  geometry_msgs::Vector3 grf_1, grf_2, interp_grf;

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
  interpOdometry(state_1, state_2, t_interp, interp_state);
  interp_primitive_id = primitive_id_1;
  interp_grf.x = lerp(grf_1.x, grf_2.x, t_interp);
  interp_grf.y = lerp(grf_1.y, grf_2.y, t_interp);
  interp_grf.z = lerp(grf_1.z, grf_2.z, t_interp);

  return interp_state;

}

spirit_msgs::MultiFootState math_utils::interpMultiFootPlanContinuous(
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

spirit_msgs::RobotState math_utils::interpRobotStateTraj(spirit_msgs::RobotStateTrajectory
  msg, double t) {    

  // Define some useful timing parameters
  ros::Time t0_ros = msg.states.front().header.stamp;
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

void math_utils::convertBodyAndFootToJoint(nav_msgs::Odometry body_state,
  spirit_msgs::MultiFootState multi_foot_state, sensor_msgs::JointState &joint_state) {

  joint_state.header = multi_foot_state.header;
  // If this message is empty set the joint names
  if (joint_state.name.empty()) {
    joint_state.name = {"8", "0", "1", "9","2", "3", "10", "4","5", "11", "6", "7"};
  }
  joint_state.position.clear();
  joint_state.velocity.clear();
  joint_state.effort.clear();

  spirit_utils::SpiritKinematics spirit;

  for (int i=0; i < multi_foot_state.feet.size(); i++) {

    // Get foot position data
    Eigen::Vector3d foot_pos;
    foot_pos[0] = multi_foot_state.feet[i].position.x;
    foot_pos[1] = multi_foot_state.feet[i].position.y;
    foot_pos[2] = multi_foot_state.feet[i].position.z;      

    // Get corresponding body plan data
    Eigen::Vector3d body_pos = {body_state.pose.pose.position.x,
      body_state.pose.pose.position.y,body_state.pose.pose.position.z};

    tf2::Quaternion q;
    tf2::convert(body_state.pose.pose.orientation,q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    Eigen::Vector3d body_rpy = {roll,pitch,yaw};

    // Compute IK to get joint data
    Eigen::Vector3d leg_joint_state;
    spirit.legIK(i,body_pos,body_rpy,foot_pos,leg_joint_state);

    // Add to the joint state vector
    joint_state.position.push_back(leg_joint_state[0]);
    joint_state.position.push_back(leg_joint_state[1]);
    joint_state.position.push_back(leg_joint_state[2]);

    // Fill in the other elements with zeros for now
    joint_state.velocity.push_back(0.0);
    joint_state.velocity.push_back(0.0);
    joint_state.velocity.push_back(0.0);

    joint_state.effort.push_back(0.0);
    joint_state.effort.push_back(0.0);
    joint_state.effort.push_back(0.0);
  }

}


int math_utils::interpInt(std::vector<double> input_vec,
  std::vector<int> output_vec, double query_point) {

  // Check bounds, throw an error if invalid since this shouldn't ever happen
  if ((query_point < input_vec.front()) || (query_point > input_vec.back())){
    throw std::runtime_error("Tried to interp out of bounds");
  }

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  Eigen::Vector3d y1, y2, interp_data;

  // Find the correct values to interp between
  int idx=0;
  for(int i=0;i<input_vec.size();i++)
  {
      if(input_vec[i]<=query_point && query_point<input_vec[i+1])
      {
        return output_vec[i];
      }
  }

  throw std::runtime_error("Didn't find the query point, something happened");
}

std::vector<double> math_utils::movingAverageFilter(std::vector<double> data,
  int window_size) {

  std::vector<double> filtered_data;
  int N = data.size();

  // Check to ensure window size is an odd integer, if not add one to make it so
  if ((window_size % 2) == 0) {
    window_size += 1;
    ROS_WARN("Filter window size is even, adding one to maintain symmetry");
  }

  // Make sure that the window size is acceptable
  if (window_size>=N) {
    ROS_ERROR("Filter window size is bigger than data");
  }

  // Loop through the data
  for (int i = 0; i < N; i++) {

    // Initialize sum and count of data samples
    double sum = 0;
    double count = 0;

    // Shrink the window size if it would result in out of bounds data
    int current_window_size = std::min(window_size, 2*i+1);
    // int current_window_size = window_size;
    
    // Loop through the window, adding to the sum and averaging
    for (int j = 0; j < current_window_size; j++) {
      double index = i + (j - (current_window_size-1)/2);

      // Make sure data is in bounds
      if (index>=0 && index<N) {
        sum += data[index];
        count += 1;
      }
    }

    filtered_data.push_back((float)sum/count);
  }

  return filtered_data;
}

std::vector<double> math_utils::centralDiff(std::vector<double> data, double dt) {
  std::vector<double> data_diff;

  for (int i = 0; i < data.size(); i++) {

    // Compute lower and upper indices, with forward/backward difference at the ends
    int lower_index = std::max(i-1,0);
    int upper_index = std::min(i+1,(int)data.size()-1);

    double estimate = (data[upper_index] - data[lower_index])/(dt*(upper_index - lower_index));
    data_diff.push_back(estimate);
  }

  return data_diff;
}