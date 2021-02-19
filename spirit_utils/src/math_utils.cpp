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

std_msgs::Header math_utils::interpHeader(std_msgs::Header header_1,std_msgs::Header header_2,
  double t_interp) {
  
  // Copy everything from the first header
  std_msgs::Header interp_header = header_1;

  // Compute the correct ros::Time corresponding to t_interp
  t_interp = std::min(std::max(t_interp,1.0),0.0);
  ros::Duration state_duration = header_2.stamp - header_1.stamp;
  ros::Duration interp_duration = ros::Duration(t_interp*state_duration.toSec());
  interp_header.stamp = header_1.stamp + ros::Duration(interp_duration);
}

spirit_msgs::FootState math_utils::interpFootState(spirit_msgs::FootState state_1,
  spirit_msgs::FootState state_2, double t_interp) {

  spirit_msgs::FootState interp_state;// = state_1;
  interp_state.header = interpHeader(state_1.header, state_2.header,t_interp);

  // Interp foot position
  interp_state.position.x = lerp(state_1.position.x, state_2.position.x, t_interp);
  interp_state.position.y = lerp(state_1.position.y, state_2.position.y, t_interp);
  interp_state.position.z = lerp(state_1.position.z, state_2.position.z, t_interp);

  // Interp foot velocity
  interp_state.velocity.x = lerp(state_1.velocity.x, state_2.velocity.x, t_interp);
  interp_state.velocity.y = lerp(state_1.velocity.y, state_2.velocity.y, t_interp);
  interp_state.velocity.z = lerp(state_1.velocity.z, state_2.velocity.z, t_interp);

  // Set contact state to the first state
  interp_state.contact = state_1.contact;

  return interp_state;
}

spirit_msgs::RobotState math_utils::interpRobotState(spirit_msgs::RobotState state_1,
  spirit_msgs::RobotState state_2, double t_interp) {

  // Set interp_state to copy state_1 (won't change map frame or contact mode)
  spirit_msgs::RobotState interp_state = state_1;

  // Interp header
  interp_state.header = interpHeader(state_1.header, state_2.header,t_interp);

  // Interp body position
  interp_state.body.pose.pose.position.x = 
    lerp(state_1.body.pose.pose.position.x, state_2.body.pose.pose.position.x, t_interp);
  interp_state.body.pose.pose.position.y = 
    lerp(state_1.body.pose.pose.position.y, state_2.body.pose.pose.position.y, t_interp);
  interp_state.body.pose.pose.position.z = 
    lerp(state_1.body.pose.pose.position.z, state_2.body.pose.pose.position.z, t_interp);
  
  // Interp body orientation with slerp
  tf2::Quaternion q_1, q_2, q_interp;
  tf2::convert(state_1.body.pose.pose.orientation,q_1);
  tf2::convert(state_2.body.pose.pose.orientation,q_2);
  q_interp = q_1.slerp(q_2, t_interp);
  interp_state.body.pose.pose.orientation = tf2::toMsg(q_interp);

  // Interp twist
  interp_state.body.twist.twist.linear.x = 
    lerp(state_1.body.twist.twist.linear.x, state_2.body.twist.twist.linear.x, t_interp);
  interp_state.body.twist.twist.linear.y = 
    lerp(state_1.body.twist.twist.linear.y, state_2.body.twist.twist.linear.y, t_interp);
  interp_state.body.twist.twist.linear.z = 
    lerp(state_1.body.twist.twist.linear.z, state_2.body.twist.twist.linear.z, t_interp);

  interp_state.body.twist.twist.angular.x = 
    lerp(state_1.body.twist.twist.angular.x, state_2.body.twist.twist.angular.x, t_interp);
  interp_state.body.twist.twist.angular.y = 
    lerp(state_1.body.twist.twist.angular.y, state_2.body.twist.twist.angular.y, t_interp);
  interp_state.body.twist.twist.angular.z = 
    lerp(state_1.body.twist.twist.angular.z, state_2.body.twist.twist.angular.z, t_interp);

  // Interp joints
  // interp_state.joints.name.resize(state_1.joints.position.size());
  // interp_state.joints.position.resize(state_1.joints.position.size());
  // interp_state.joints.velocity.resize(state_1.joints.position.size());
  // interp_state.joints.effort.resize(state_1.joints.position.size());
  for (int i = 0; i < state_1.joints.position.size(); i++) {
    interp_state.joints.position[i] = lerp(state_1.joints.position[i], 
      state_2.joints.position[i], t_interp);
    interp_state.joints.velocity[i] = lerp(state_1.joints.velocity[i], 
      state_2.joints.velocity[i], t_interp);
    interp_state.joints.effort[i] = lerp(state_1.joints.effort[i], 
      state_2.joints.effort[i], t_interp);
  }

  // // Interp feet
  // for (int i = 0; i < 4; i++) {
  //   printf("Iteration: %d \n", i);
  //   spirit_msgs::FootState interp_foot_state = interpFootState(state_1.feet[i], state_2.feet[i], t_interp) ;
  //   printf("interpFootStateDone \n");
  //   interp_state.feet[i] = interp_foot_state;
  //   printf("FootState added to vector \n");
  // }
  
  // Interp feet
  // for (int i = 0; i < state_1.feet.size(); i++) {
  //   printf("iteration = %d", i);

  //   spirit_msgs::FootState interp_foot_state = state_1.feet[i];

  //   // interp_foot_state.header.stamp = interp_state.header.stamp;

  //   // interp_foot_state.position.x = lerp(state_1.feet[i].position.x, 
  //   //   state_2.feet[i].position.x, t_interp);
  //   // interp_foot_state.position.y = lerp(state_1.feet[i].position.y, 
  //   //   state_2.feet[i].position.y, t_interp);
  //   // interp_foot_state.position.z = lerp(state_1.feet[i].position.z, 
  //   //   state_2.feet[i].position.z, t_interp);

  //   // interp_foot_state.velocity.x = lerp(state_1.feet[i].velocity.x, 
  //   //   state_2.feet[i].velocity.x, t_interp);
  //   // interp_foot_state.velocity.y = lerp(state_1.feet[i].velocity.y, 
  //   //   state_2.feet[i].velocity.y, t_interp);
  //   // interp_foot_state.velocity.z = lerp(state_1.feet[i].velocity.z, 
  //   //   state_2.feet[i].velocity.z, t_interp);

  //     interp_state.feet[i] = interp_foot_state;
  // }
  
  // Interp feet
  printf("interp_state.feet.size() = %d", interp_state.feet.size());
  interp_state.feet.clear();
  interp_state.feet.resize(4);
  printf("interp_state.feet.size() = %d", interp_state.feet.size());
  for (int i = 0; i < state_1.feet.size(); i++) {
    printf("iteration = %d", i);

    interp_state.feet[i].header.stamp = interp_state.header.stamp;

    interp_state.feet[i].position.x = lerp(state_1.feet[i].position.x, 
      state_2.feet[i].position.x, t_interp);
    interp_state.feet[i].position.y = lerp(state_1.feet[i].position.y, 
      state_2.feet[i].position.y, t_interp);
    interp_state.feet[i].position.z = lerp(state_1.feet[i].position.z, 
      state_2.feet[i].position.z, t_interp);

    interp_state.feet[i].velocity.x = lerp(state_1.feet[i].velocity.x, 
      state_2.feet[i].velocity.x, t_interp);
    interp_state.feet[i].velocity.y = lerp(state_1.feet[i].velocity.y, 
      state_2.feet[i].velocity.y, t_interp);
    interp_state.feet[i].velocity.z = lerp(state_1.feet[i].velocity.z, 
      state_2.feet[i].velocity.z, t_interp);

    // spirit_msgs::FootState temp_state = interp_state.feet[i];
    //std::cout << interp_state.feet[i].header.stamp << std::endl;
    //std::cout << temp_state.header.stamp << std::endl;

    // interp_state.feet[i] = temp_state;
  }

  return interp_state;
}

spirit_msgs::RobotState math_utils::interpRobotStateTraj(spirit_msgs::RobotStateTrajectory
  msg, double t) {    

  double t0 = 0;
  ros::Time t0_ros = msg.states.front().header.stamp;
  ros::Duration traj_duration = msg.states.back().header.stamp - t0_ros;
  double tf = traj_duration.toSec();

  // Check bounds, return boundary state if outside
  if (t <= t0) {
    return msg.states.front();
  } else if (t >= tf) {
    return msg.states.back();
  }

  int num_states = msg.states.size();

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  spirit_msgs::RobotState state_1, state_2, interp_state;

  // Find the correct values to interp between
  for(int i=0;i<num_states-1;i++)
  {
    ros::Duration t1_ros = msg.states[i].header.stamp - t0_ros;
    t1 = t1_ros.toSec();

    ros::Duration t2_ros = msg.states[i+1].header.stamp - t0_ros;
    t2 = t2_ros.toSec();

    if(t1<=t && t<t2)
    {
      state_1 = msg.states[i];
      state_2 = msg.states[i+1]; 
      break;
    }
  }

  double t_interp = (t - t1)/(t2-t1);
  interp_state = interpRobotState(state_1, state_2, t_interp);

  return interp_state;
}