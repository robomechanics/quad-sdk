#ifndef SPIRIT_ROS_UTILS_H
#define SPIRIT_ROS_UTILS_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <spirit_utils/math_utils.h>

namespace spirit_utils {
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
    return spirit_utils::getROSMessageAgeInMs(header,t_compare);
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
  void interpOdometry(nav_msgs::Odometry state_1, nav_msgs::Odometry state_2, 
    double t_interp, nav_msgs::Odometry &interp_state);

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
  void interpMultiFootState(spirit_msgs::MultiFootState state_1,spirit_msgs::MultiFootState state_2,
    double t_interp, spirit_msgs::MultiFootState &interp_state);

  /**
   * @brief Interpolate data between two GRFArray messages.
   * @param[in] state_1 First GRFArray message
   * @param[in] state_2 Second GRFArray message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @param[out] interp_state Interpolated GRFArray message
   */
  void interpGRFArray(spirit_msgs::GRFArray state_1,spirit_msgs::GRFArray state_2,
    double t_interp, spirit_msgs::GRFArray &interp_state);

  /**
   * @brief Interpolate data between two RobotState messages.
   * @param[in] state_1 First RobotState message
   * @param[in] state_2 Second RobotState message
   * @param[in] t_interp Fraction of time between the messages [0,1]
   * @param[out] interp_state Interpolated RobotState message
   */
  void interpRobotState(spirit_msgs::RobotState state_1, spirit_msgs::RobotState state_2, 
    double t_interp, spirit_msgs::RobotState &interp_state) ;

  /**
   * @brief Interpolate data from a BodyPlan message.
   * @param[in] msg BodyPlan message
   * @param[in] t Time since beginning of trajectory (will return last state if too large)
   * @param[out] interp_state Interpolated Odometry message
   * @param[out] interp_primitive_id Interpolated primitive id
   * @param[out] interp_grf Interpolated GRF array
   */
void interpBodyPlan(spirit_msgs::BodyPlan msg, double t,
  nav_msgs::Odometry &interp_state, int &interp_primitive_id, spirit_msgs::GRFArray &interp_grf);

  /**
   * @brief Interpolate data from a MultiFootPlanContinuous message.
   * @param[in] msg MultiFootPlanContinuous message
   * @param[in] t Time since beginning of trajectory (will return last state if too large)
   * @return MultiFootState message
   */
  spirit_msgs::MultiFootState interpMultiFootPlanContinuous(
    spirit_msgs::MultiFootPlanContinuous msg, double t);

  /**
   * @brief Interpolate data from a robot state trajectory message.
   * @param[in] msg robot state trajectory message
   * @param[in] t Time since beginning of trajectory (will return last state if too large)
   * @return Robot state message
   */
  spirit_msgs::RobotState interpRobotStateTraj(spirit_msgs::RobotStateTrajectory msg, double t);

  /**
   * @brief Perform IK to compute a joint state message corresponding to body and foot messages
   * @param[in] kinematics Pointer to kinematics object
   * @param[in] body_state message of body state
   * @param[in] multi_foot_state message of state of each foot
   * @param[out] joint_state message of the corresponding joint state
   */
  void ikRobotState(const spirit_utils::SpiritKinematics &kinematics,
    nav_msgs::Odometry body_state, spirit_msgs::MultiFootState multi_foot_state,
    sensor_msgs::JointState &joint_state);

  /**
   * @brief Perform IK and save to the state.joint field
   * @param[in] kinematics Pointer to kinematics object
   * @param[out] state RobotState message to which to add joint data
   */
  void ikRobotState(const spirit_utils::SpiritKinematics &kinematics,
    spirit_msgs::RobotState &state);

  /**
   * @brief Perform FK to compute a foot state message corresponding to body and joint messages
   * @param[in] kinematics Pointer to kinematics object
   * @param[in] body_state message of body state
   * @param[in] joint_state message of the corresponding joint state
   * @param[out] multi_foot_state message of state of each foot
   */
  void fkRobotState(const spirit_utils::SpiritKinematics &kinematics,
    nav_msgs::Odometry body_state, sensor_msgs::JointState joint_state,
    spirit_msgs::MultiFootState &multi_foot_state);

  /**
   * @brief Perform FK and save to the state.feet field
   * @param[in] kinematics Pointer to kinematics object
   * @param[out] state RobotState message to which to add joint data
   */
  void fkRobotState(const spirit_utils::SpiritKinematics &kinematics,
    spirit_msgs::RobotState &state);

  /**
   * @brief Convert robot state message to Eigen
   * @param[in] state Eigen vector with body state data
   * @return Odometry msg with body state data
   */
  nav_msgs::Odometry eigenToOdomMsg(const Eigen::VectorXd &state);

  /**
   * @brief Convert robot state message to Eigen
   * @param[in] body Odometry msg with body state data
   * @return Eigen vector with body state data
   */
  Eigen::VectorXd odomMsgToEigen(const nav_msgs::Odometry &body);

  /**
   * @brief Convert eigen vector of GRFs to GRFArray msg
   * @param[in] grf_array Eigen vector with grf data in leg order
   * @param[in] multi_foot_state_msg MultiFootState msg containing foot position information
   * @return GRFArray msg with grf data
   */
  spirit_msgs::GRFArray eigenToGRFArrayMsg(Eigen::VectorXd grf_array,
    spirit_msgs::MultiFootState multi_foot_state_msg);

}

#endif