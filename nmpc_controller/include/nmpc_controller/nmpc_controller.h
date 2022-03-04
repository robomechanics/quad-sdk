#ifndef NMPC_CONTROLLER_H
#define NMPC_CONTROLLER_H

#include <math.h>
#include <quad_msgs/GRFArray.h>
#include <quad_msgs/LegCommand.h>
#include <quad_msgs/MultiFootPlanDiscrete.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <quad_msgs/RobotStateTrajectory.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

#include "IpIpoptApplication.hpp"
#include "nmpc_controller/quad_nlp.h"

//! NMPC controller ROS node
/*!
   Wrapper around Quadrupedal MPC that interfaces with our ROS architecture
*/
class NMPCController {
 public:
  /**
   * @brief Constructor for MPCController
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type MPCController
   */
  NMPCController();

  NMPCController(int type);

  /**
   * @brief Update the contact and dynamic matrices, solve, and return the
   * output
   * @param[in] initial_state Vector with initial state
   * @param[in] ref_traj Matrix holding desired reference trajectory
   * @param[in] foot_positions Matrix holding foot positions
   * @param[in] contact_schedule Matrix holding the contact schedule
   * @param[in] first_element_duration Time duration to the next plan index
   * @param[in] same_plan_index If the current solving is duplicated in the same
   * index
   * @param[out] state_traj Optimized state trajectory output
   * @param[out] control_traj Optimized control trajectory output
   * @return good_solve
   */
  bool computePlan(const Eigen::VectorXd &initial_state,
                   const Eigen::MatrixXd &ref_traj,
                   const Eigen::MatrixXd &foot_positions,
                   const std::vector<std::vector<bool>> &contact_schedule,
                   Eigen::MatrixXd &state_traj, Eigen::MatrixXd &control_traj);

  bool computeLegPlan(const Eigen::VectorXd &initial_state,
                      const Eigen::MatrixXd &ref_traj,
                      const Eigen::MatrixXd &foot_positions,
                      const Eigen::MatrixXd &foot_velocities,
                      const std::vector<std::vector<bool>> &contact_schedule,
                      const Eigen::VectorXd &ref_ground_height,
                      const double &first_element_duration,
                      const bool &same_plan_index,
                      const Eigen::VectorXi &ref_primitive_id,
                      const Eigen::VectorXi &complexity_schedule,
                      Eigen::MatrixXd &state_traj,
                      Eigen::MatrixXd &control_traj);

  bool computeCentralizedTailPlan(
      const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
      const Eigen::MatrixXd &foot_positions,
      const std::vector<std::vector<bool>> &contact_schedule,
      const Eigen::VectorXd &tail_initial_state,
      const Eigen::MatrixXd &tail_ref_traj,
      const Eigen::VectorXd &ref_ground_height, Eigen::MatrixXd &state_traj,
      Eigen::MatrixXd &control_traj, Eigen::MatrixXd &tail_state_traj,
      Eigen::MatrixXd &tail_control_traj);

  bool computeDistributedTailPlan(
      const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &ref_traj,
      const Eigen::MatrixXd &foot_positions,
      const std::vector<std::vector<bool>> &contact_schedule,
      const Eigen::VectorXd &tail_initial_state,
      const Eigen::MatrixXd &tail_ref_traj, const Eigen::MatrixXd &state_traj,
      const Eigen::MatrixXd &control_traj,
      const Eigen::VectorXd &ref_ground_height,
      const double &first_element_duration, const bool &same_plan_index,
      Eigen::MatrixXd &tail_state_traj, Eigen::MatrixXd &tail_control_traj);

  /** Method to return the constraint residual for requested data */
  Eigen::VectorXd eval_g_single_fe(int sys_id, double dt,
                                   const Eigen::VectorXd &x0,
                                   const Eigen::VectorXd &u,
                                   const Eigen::VectorXd &x1,
                                   const Eigen::VectorXd &params);

 private:
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  SmartPtr<quadNLP> mynlp_;

  SmartPtr<IpoptApplication> app_;

  int N_;

  int n_;

  int n_null_;

  int m_;

  double dt_;

  int type_;

  /// Weight for takeoff state
  double takeoff_state_weight_factor_;

  std::string param_ns_;

  bool require_init_;
};

#endif  // MPC_CONTROLLER_H
