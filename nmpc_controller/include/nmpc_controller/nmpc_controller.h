#ifndef NMPC_CONTROLLER_H
#define NMPC_CONTROLLER_H

#include <ros/ros.h>
#include <math.h>
// #include <eigen3/Eigen/Eigen>
#include <spirit_msgs/RobotPlan.h>
#include <spirit_msgs/MultiFootPlanDiscrete.h>
#include <spirit_msgs/GRFArray.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/RobotStateTrajectory.h>
// #include <local_planner/quadruped_mpc.h>
#include <spirit_utils/ros_utils.h>
#include <spirit_utils/kinematics.h>
#include "spirit_utils/matplotlibcpp.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <spirit_msgs/LegCommand.h>

#include "nmpc_controller/spirit_nlp.h"
#include "IpIpoptApplication.hpp"
#include <Eigen/Dense>

//! NMPC controller ROS node
/*!
   Wrapper around Quadrupedal MPC that interfaces with our ROS architecture
*/
class NMPCController
{
public:
  /**
	 * @brief Constructor for MPCController
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type MPCController
	 */
  NMPCController();

  NMPCController(int type);

  /**
   * @brief Update the contact and dynamic matrices, solve, and return the output
   * @param[in] initial_state Vector with initial state
   * @param[in] ref_traj Matrix holding desired reference trajectory
   * @param[in] foot_positions Matrix holding foot positions
   * @param[in] contact_schedule Matrix holding the contact schedule
   * @param[out] state_traj Optimized state trajectory output
   * @param[out] control_traj Optimized control trajectory output
   * @return good_solve
   */
  bool computePlan(const Eigen::VectorXd &initial_state,
                   const Eigen::MatrixXd &ref_traj,
                   const Eigen::MatrixXd &foot_positions,
                   const std::vector<std::vector<bool>> &contact_schedule,
                   Eigen::MatrixXd &state_traj,
                   Eigen::MatrixXd &control_traj);

  bool computeLegPlan(const Eigen::VectorXd &initial_state,
                      const Eigen::MatrixXd &ref_traj,
                      const Eigen::MatrixXd &foot_positions,
                      const std::vector<std::vector<bool>> &contact_schedule,
                      Eigen::MatrixXd &state_traj,
                      Eigen::MatrixXd &control_traj);

  bool computeCentralizedTailPlan(const Eigen::VectorXd &initial_state,
                                  const Eigen::MatrixXd &ref_traj,
                                  const Eigen::MatrixXd &foot_positions,
                                  const std::vector<std::vector<bool>> &contact_schedule,
                                  const Eigen::VectorXd &tail_initial_state,
                                  const Eigen::MatrixXd &tail_ref_traj,
                                  Eigen::MatrixXd &state_traj,
                                  Eigen::MatrixXd &control_traj,
                                  Eigen::MatrixXd &tail_state_traj,
                                  Eigen::MatrixXd &tail_control_traj);

  bool computeDistributedTailPlan(const Eigen::VectorXd &initial_state,
                                  const Eigen::MatrixXd &ref_traj,
                                  const Eigen::MatrixXd &foot_positions,
                                  const std::vector<std::vector<bool>> &contact_schedule,
                                  const Eigen::VectorXd &tail_initial_state,
                                  const Eigen::MatrixXd &tail_ref_traj,
                                  const Eigen::MatrixXd &state_traj,
                                  const Eigen::MatrixXd &control_traj,
                                  Eigen::MatrixXd &tail_state_traj,
                                  Eigen::MatrixXd &tail_control_traj);

private:
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// Spirit Kinematics class
  std::shared_ptr<spirit_utils::SpiritKinematics> kinematics_;

  SmartPtr<spiritNLP> mynlp_;

  SmartPtr<IpoptApplication> app_;

  Eigen::MatrixXd last_state_traj_;

  Eigen::MatrixXd last_control_traj_;

  int N_;

  int n_;

  int m_;

  double dt_;

  int type_;

  std::string param_ns_;
};

#endif // MPC_CONTROLLER_H
