#ifndef SPIRIT_ROS_UTILS_H
#define SPIRIT_ROS_UTILS_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <quad_utils/math_utils.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

namespace quad_utils {
  /**
   * @brief Gets the relative age of a timestamped header
   * @param[in] header ROS Header that we wish to compute the age of
   * @param[in] t_compare ROS time we wish to compare to
   * @return Age in ms (compared to t_compare)
   */
  inline double getROSMessageAgeInMs(std_msgs::Header header, ros::Time t_compare)
  {
    return (t_compare - header.stamp).toSec()*1000.0;
  }

  /**
   * @brief Gets the relative age of a timestamped header
   * @param[in] header ROS Header that we wish to compute the age of
   * @return Age in ms (compared to ros::Time::now())
   */
  inline double getROSMessageAgeInMs(std_msgs::Header header)
  {
    ros::Time t_compare = ros::Time::now();
    return quad_utils::getROSMessageAgeInMs(header,t_compare);
  }

  /**
   * @brief Gets the relative time (in s) since the beginning of the plan
   * @param[in] plan_start ROS Time to to compare to
   * @return Time in plan (compared to ros::Time::now())
   */
  inline double getDurationSinceTime(ros::Time plan_start)
  {
    return (ros::Time::now() - plan_start).toSec();
  }

  /**
   * @brief Gets the index associated with a given time
   * @param[out] index Index in plan (compared to ros::Time::now())
   * @param[out] time_ahead Time duration to next index in plan (compared to ros::Time::now())
   * @param[in] plan_start ROS Time to to compare to
   * @param[in] dt Timestep used to discretize the plan
   */
  inline void getPlanIndex(ros::Time plan_start, double dt, int &index, double &time_ahead)
  {
    double duration = getDurationSinceTime(plan_start);
    index = std::floor(duration / dt);
    time_ahead = (index + 1) * dt - duration;
  }

  /**
   * @brief Load ros parameter into class variable
   * @param[in] nh ROS nodehandle
   * @param[in] paramName string storing key of param in rosparam server
   * @param[in] varName address of variable to store loaded param
   * @return boolean success
   */
  template <class ParamType>
  inline bool loadROSParam(ros::NodeHandle nh, std::string paramName, ParamType &varName)
  {
    if(!nh.getParam(paramName, varName))
    {
      ROS_ERROR("Can't find param %s from parameter server", paramName.c_str());
      return false;
    }
    return true;
  }

  /**
   * @brief Load ros parameter into class variable
   * @param[in] nh ROS nodehandle
   * @param[in] paramName string storing key of param in rosparam server
   * @param[in] varName address of variable to store loaded param
   * @param[in] defaultVal default value to use if rosparam server doesn't contain key
   * @return boolean (true if found rosparam, false if loaded default)
   */
  template <class ParamType>
  inline bool loadROSParamDefault(ros::NodeHandle nh, std::string paramName, ParamType &varName, ParamType defaultVal)
  {
    if(!nh.getParam(paramName, varName))
    {
      varName = defaultVal;
      ROS_INFO("Can't find param %s on rosparam server, loading default value.",paramName.c_str());
      return false;
    }
    return true;
  }

  // /**
  //  * @brief Interpolate two headers
  //  * @param[out] msg State message to popluate
  //  * @param[in] stamp Timestamp for the state message
  //  * @param[in] frame Frame_id for the state message
  //  */
  // void updateStateHeaders(quad_msgs::RobotState &msg, ros::Time stamp, std::string frame);

  /**
   * @brief Interpolate two headers
   * @param[out] msg State message to popluate
   * @param[in] stamp Timestamp for the state message
   * @param[in] frame Frame_id for the state message
   * @param[in] traj_index Trajectory index of this state message
   */
  void updateStateHeaders(quad_msgs::RobotState &msg, ros::Time stamp, std::string frame,
    int traj_index);

  /**
   * @brief Interpolate two headers
   * @param[in] header_1 First header message
   * @param[in] header_2 Second header message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @param[out] interp_state Interpolated header
   */
  void interpHeader(std_msgs::Header header_1,std_msgs::Header header_2, double t_interp,
    std_msgs::Header &interp_header);

  /**
   * @brief Interpolate data between two Odometry messages.
   * @param[in] state_1 First Odometry message
   * @param[in] state_2 Second Odometry message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @param[out] interp_state Interpolated Odometry message
   */
  void interpOdometry(quad_msgs::BodyState state_1, quad_msgs::BodyState state_2, 
    double t_interp, quad_msgs::BodyState &interp_state);

  /**
   * @brief Interpolate data between two JointState messages.
   * @param[in] state_1 First JointState message
   * @param[in] state_2 Second JointState message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @param[out] interp_state Interpolated JointState message
   */
  void interpJointState(sensor_msgs::JointState state_1, sensor_msgs::JointState state_2, 
    double t_interp, sensor_msgs::JointState &interp_state);

  /**
   * @brief Interpolate data between two FootState messages.
   * @param[in] state_1 First FootState message
   * @param[in] state_2 Second FootState message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @param[out] interp_state Interpolated FootState message
   */
  void interpMultiFootState(quad_msgs::MultiFootState state_1,quad_msgs::MultiFootState state_2,
    double t_interp, quad_msgs::MultiFootState &interp_state);

  /**
   * @brief Interpolate data between two GRFArray messages.
   * @param[in] state_1 First GRFArray message
   * @param[in] state_2 Second GRFArray message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @param[out] interp_state Interpolated GRFArray message
   */
  void interpGRFArray(quad_msgs::GRFArray state_1,quad_msgs::GRFArray state_2,
    double t_interp, quad_msgs::GRFArray &interp_state);

  /**
   * @brief Interpolate data between two RobotState messages.
   * @param[in] state_1 First RobotState message
   * @param[in] state_2 Second RobotState message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @param[out] interp_state Interpolated RobotState message
   */
  void interpRobotState(quad_msgs::RobotState state_1, quad_msgs::RobotState state_2, 
    double t_interp, quad_msgs::RobotState &interp_state) ;

  /**
   * @brief Interpolate data from a BodyPlan message.
   * @param[in] msg BodyPlan message
   * @param[in] t Time since beginning of trajectory (will return last state if too large)
   * @param[out] interp_state Interpolated Odometry message
   * @param[out] interp_primitive_id Interpolated primitive id
   * @param[out] interp_grf Interpolated GRF array
   */
void interpRobotPlan(quad_msgs::RobotPlan msg, double t,
  quad_msgs::RobotState &interp_state, int &interp_primitive_id,
  quad_msgs::GRFArray &interp_grf);

  /**
   * @brief Interpolate data from a MultiFootPlanContinuous message.
   * @param[in] msg MultiFootPlanContinuous message
   * @param[in] t Time since beginning of trajectory (will return last state if too large)
   * @return MultiFootState message
   */
  quad_msgs::MultiFootState interpMultiFootPlanContinuous(
    quad_msgs::MultiFootPlanContinuous msg, double t);

  /**
   * @brief Interpolate data from a robot state trajectory message.
   * @param[in] msg robot state trajectory message
   * @param[in] t Time since beginning of trajectory (will return last state if too large)
   * @return Robot state message
   */
  quad_msgs::RobotState interpRobotStateTraj(quad_msgs::RobotStateTrajectory msg, double t);

  /**
   * @brief Perform IK to compute a joint state message corresponding to body and foot messages
   * @param[in] kinematics Pointer to kinematics object
   * @param[in] body_state message of body state
   * @param[in] multi_foot_state message of state of each foot
   * @param[out] joint_state message of the corresponding joint state
   */
  void ikRobotState(const quad_utils::QuadKD &kinematics,
    quad_msgs::BodyState body_state, quad_msgs::MultiFootState multi_foot_state,
    sensor_msgs::JointState &joint_state);

  /**
   * @brief Perform IK and save to the state.joint field
   * @param[in] kinematics Pointer to kinematics object
   * @param[out] state RobotState message to which to add joint data
   */
  void ikRobotState(const quad_utils::QuadKD &kinematics,
    quad_msgs::RobotState &state);

  /**
   * @brief Perform FK to compute a foot state message corresponding to body and joint messages
   * @param[in] kinematics Pointer to kinematics object
   * @param[in] body_state message of body state
   * @param[in] joint_state message of the corresponding joint state
   * @param[out] multi_foot_state message of state of each foot
   */
  void fkRobotState(const quad_utils::QuadKD &kinematics,
    quad_msgs::BodyState body_state, sensor_msgs::JointState joint_state,
    quad_msgs::MultiFootState &multi_foot_state);

  /**
   * @brief Perform FK and save to the state.feet field
   * @param[in] kinematics Pointer to kinematics object
   * @param[out] state RobotState message to which to add joint data
   */
  void fkRobotState(const quad_utils::QuadKD &kinematics,
    quad_msgs::RobotState &state);

  /**
   * @brief Convert robot state message to Eigen
   * @param[in] state Eigen vector with body state data
   * @return Odometry msg with body state data
   */
  quad_msgs::BodyState eigenToBodyStateMsg(const Eigen::VectorXd &state);

  /**
   * @brief Convert robot state message to Eigen
   * @param[in] body Odometry msg with body state data
   * @return Eigen vector with body state data
   */
  Eigen::VectorXd bodyStateMsgToEigen(const quad_msgs::BodyState &body);

  /**
   * @brief Convert robot state message to tail state in Eigen
   * @param[in] body Odometry msg with body state data
   * @return Eigen vector with tail state data
   */
  Eigen::VectorXd odomMsgToEigenForTail(const quad_msgs::RobotState &msg);

  /**
   * @brief Convert Eigen vector of GRFs to GRFArray msg
   * @param[in] grf_array Eigen vector with grf data in leg order
   * @param[in] multi_foot_state_msg MultiFootState msg containing foot position information
   * @param[out] grf_msg GRFArray msg containing GRF data
   */
  void eigenToGRFArrayMsg(Eigen::VectorXd grf_array, quad_msgs::MultiFootState multi_foot_state_msg,
    quad_msgs::GRFArray &grf_msg);

  /**
   * @brief Convert GRFArray msg to Eigen vector of GRFs
   * @param[in] grf_array_msg_ GRFArray msg with grf data
   * @return grf_array Eigen vector with grf data in leg order
   */
  Eigen::VectorXd grfArrayMsgToEigen(const quad_msgs::GRFArray &grf_array_msg_);

  /**
   * @brief Convert robot foot state message to Eigen
   * @param[in] foot_state_msg MultiFootState msg containing foot position information
   * @param[out] foot_position Eigen vector with foot position
   */
  void footStateMsgToEigen(const quad_msgs::FootState &foot_state_msg, 
    Eigen::Vector3d &foot_position);

  /**
   * @brief Convert robot multi foot state message to Eigen
   * @param[in] multi_foot_state_msg MultiFootState msg containing foot position information
   * @param[out] foot_positions Eigen vector with foot state data
   */
  void multiFootStateMsgToEigen(const quad_msgs::MultiFootState &multi_foot_state_msg, 
    Eigen::VectorXd &foot_positions);

  /**
   * @brief Convert robot multi foot state message to Eigen
   * @param[in] multi_foot_state_msg MultiFootState msg containing foot position information
   * @param[out] foot_positions Eigen vector with foot position data
   * @param[out] foot_velocities Eigen vector with foot velocity data
   */
  void multiFootStateMsgToEigen(const quad_msgs::MultiFootState &multi_foot_state_msg, 
    Eigen::VectorXd &foot_positions, Eigen::VectorXd &foot_velocities);

  /**
   * @brief Convert robot multi foot state message to Eigen
   * @param[in] multi_foot_state_msg MultiFootState msg containing foot position information
   * @param[out] foot_positions Eigen vector with foot position data
   * @param[out] foot_velocities Eigen vector with foot velocity data
   * @param[out] foot_acceleration Eigen vector with foot acceleration data
   */
  void multiFootStateMsgToEigen(const quad_msgs::MultiFootState &multi_foot_state_msg, 
    Eigen::VectorXd &foot_positions, Eigen::VectorXd &foot_velocities, Eigen::VectorXd &foot_acceleration);

  /**
   * @brief Convert eigen vectors to foot state messages
   * @param[in] foot_position Eigen vector with foot position data
   * @param[in] foot_velocity Eigen vector with foot velocity data
   * @param[out] foot_state_msg FootState msg containing foot position and velocity data
   */
  void eigenToFootStateMsg(Eigen::VectorXd foot_position, 
    Eigen::VectorXd foot_velocity, quad_msgs::FootState &foot_state_msg);

  /**
   * @brief Convert eigen vectors to foot state messages
   * @param[in] foot_position Eigen vector with foot position data
   * @param[in] foot_velocity Eigen vector with foot velocity data
   * @param[in] foot_acceleration Eigen vector with foot acceleration data
   * @param[out] foot_state_msg FootState msg containing foot position and velocity data
   */
  void eigenToFootStateMsg(Eigen::VectorXd foot_position, 
    Eigen::VectorXd foot_velocity, Eigen::VectorXd foot_acceleration, quad_msgs::FootState &foot_state_msg);
  
  /**
   * @brief Convert eigen vector to stl vector
   * @param[in] eigen_vec Eigen vector with data
   * @param[out] vec stl vector
   */
  void eigenToVector(const Eigen::VectorXd &eigen_vec, std::vector<double> &vec);

  /**
   * @brief Convert eigen vector to stl vector
   * @param[in] vec stl vector
   * @param[out] eigen_vec Eigen vector with data
   */
  void vectorToEigen(const std::vector<double> &vec, Eigen::VectorXd &eigen_vec);

  /**
   * @brief Convert eigen vector to geometry_msgs::Vector3
   * @param[in] vec Eigen vector
   * @param[out] eigen_vec msg vector
   */
  void Eigen3ToVector3Msg(const Eigen::Vector3d &eigen_vec, geometry_msgs::Vector3 &vec);

  /**
   * @brief Convert geometry_msgs::Vector3 vector to eigen vector
   * @param[in] vec msg vector
   * @param[out] eigen_vec Eigen vector
   */
  void vector3MsgToEigen(const geometry_msgs::Vector3 &vec, Eigen::Vector3d &eigen_vec);

  /**
   * @brief Convert eigen vector to geometry_msgs::Point
   * @param[in] vec Eigen vector
   * @param[out] eigen_vec msg point
   */
  void Eigen3ToPointMsg(const Eigen::Vector3d &eigen_vec, geometry_msgs::Point &vec);

  /**
   * @brief Convert geometry_msgs::Point vector to eigen vector
   * @param[in] vec msg point
   * @param[out] eigen_vec Eigen vector
   */
  void pointMsgToEigen(const geometry_msgs::Point &vec, Eigen::Vector3d &eigen_vec);
}

#endif