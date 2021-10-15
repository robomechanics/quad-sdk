#ifndef NMPC_CONTROLLER_H
#define NMPC_CONTROLLER_H

#include <math.h>
// #include <eigen3/Eigen/Eigen>
#include <spirit_msgs/RobotPlan.h>
#include <spirit_msgs/MultiFootPlanDiscrete.h>
#include <spirit_msgs/GRFArray.h>
#include <spirit_msgs/RobotState.h>
#include <spirit_msgs/RobotStateTrajectory.h>
// #include <local_planner/quadruped_mpc.h>
#include <spirit_utils/ros_utils.h>
#include <spirit_utils/quad_kd.h>
#include "spirit_utils/matplotlibcpp.h"
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
  NMPCController(ros::NodeHandle &nh, int type);

  /**
   * @brief Update the contact and dynamic matrices, solve, and return the
   * output
   * @param[in] initial_state Vector with initial state
   * @param[in] ref_traj Matrix holding desired reference trajectory
   * @param[in] contact_schedule Matrix holding the contact schedule
   * @param[in] foot_positions Matrix holding foot positions
   * @param[in] foot_velocities Matrix holding foot velocities
   * @param[in] first_element_duration Time duration to the next plan index
   * @param[in] plan_index_diff If the current solving is duplicated in the same
   * index
   * @param[out] state_traj Optimized state trajectory output
   * @param[out] control_traj Optimized control trajectory output
   * @return good_solve
   */
  bool computePlan(const Eigen::VectorXd &initial_state,
                   const Eigen::MatrixXd &ref_traj,
                   const std::vector<std::vector<bool>> &contact_schedule,
                   Eigen::MatrixXd &foot_positions,
                   Eigen::MatrixXd &foot_velocities,
                   Eigen::MatrixXd &state_traj, Eigen::MatrixXd &control_traj);
  /**
   * @brief Interface to Local Planner to update the contact and dynamic
   * matrices, solve, and return the output
   * @param initial_state Vector with initial state
   * @param ref_traj Matrix holding desired reference trajectory
   * @param foot_positions_body Matrix holding foot positions in body frame
   * @param foot_positions_world Matrix holding foot positions in world frame
   * @param foot_velocities Matrix holding foot velocities
   * @param contact_schedule A vector of sequence of contact
   * @param ref_ground_height  Matrix holding ground height at reference
   * footholds
   * @param first_element_duration Time duration to the next plan index
   * @param plan_index_diff Plan index difference of the last solve and the
   * current solve
   * @param terrain Gridmap holding terrain height
   * @param state_traj Matrix holding output state trajectory
   * @param control_traj Matrix holding output control trajectory
   * @return whether solve successfully
   */
  bool computeLegPlan(const Eigen::VectorXd &initial_state,
                      const Eigen::MatrixXd &ref_traj,
                      const Eigen::MatrixXd &foot_positions_body,
                      Eigen::MatrixXd &foot_positions_world,
                      Eigen::MatrixXd &foot_velocities,
                      const std::vector<std::vector<bool>> &contact_schedule,
                      const Eigen::VectorXd &ref_ground_height,
                      const double &first_element_duration, int plan_index_diff,
                      const grid_map::GridMap &terrain,
                      Eigen::MatrixXd &state_traj,
                      Eigen::MatrixXd &control_traj);

  /** Method to return the constraint residual for requested data */
  Eigen::VectorXi updateAdaptiveComplexitySchedule(
      const Eigen::MatrixXd &state_traj_heuristic,
      const Eigen::MatrixXd &control_traj_heuristic,
      const Eigen::MatrixXd &state_traj_lifted,
      const Eigen::MatrixXd &control_traj_lifted);

  /** Method to update the prediction horizon length */
  void updateHorizonLength();

  /**
   * @brief Return the NLP diagnostics
   * @return NLP diagnostics with most recent meta-data
   */
  inline NLPDiagnostics getNLPDiagnostics() const { return diagnostics_; }

 private:
  /// ROS node handler
  ros::NodeHandle nh_;

  int robot_id_;

  /// Robot type: A1 or Spirit
  std::string robot_ns_;

  /// Update rate for sending and receiving data;
  double update_rate_;

<<<<<<< HEAD
  /// Pointer to nlp formulation
  SmartPtr<quadNLP> mynlp_;
=======
  /// QuadKD class
  std::shared_ptr<spirit_utils::QuadKD>quadKD_;

  SmartPtr<spiritNLP> mynlp_;
>>>>>>> Switch SpiritKinematics to QuadKD, switch inverse dynamics function to QuadKD

  /// Pointer to IPOPT solver
  SmartPtr<IpoptApplication> app_;

  /// Pointer to robot kinematics
  std::shared_ptr<quad_utils::QuadKD> quadKD_;

  /// Whether enable variable horizon
  bool enable_variable_horizon_;

  /// Whether enable mixed complexity
  bool enable_mixed_complexity_ = false;

  /// Whether enable adaptive complexity
  bool enable_adaptive_complexity_ = false;

  bool allow_new_interior_complexity_;

  bool is_adaptive_complexity_sparse_;

  /// Horizon length, maximal horizon length, minimal horizon length
  int N_, N_max_, N_min_;

  /// Number of states in different components
  const int n_body_ = 12, n_foot_ = 24, n_joints_ = 24, m_body_ = 12,
            m_foot_ = 24;

  /// Time resolution
  double dt_;

  /// Whether warm start is needed
  bool require_init_;

  /// Adaptive complexity schedule
  Eigen::VectorXi adaptive_complexity_schedule_;

  /// Diagnostics struct for gathering metadata
  NLPDiagnostics diagnostics_;

  /// Config struct for storing meta parameters
  NLPConfig config_;
};

#endif  // MPC_CONTROLLER_H
