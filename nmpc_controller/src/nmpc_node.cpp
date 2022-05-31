#include <nmpc_controller/nmpc_controller.h>
#include <ros/ros.h>

#include <chrono>
#include <iostream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "nmpc_node");
  ros::NodeHandle nh;

  double dt_;
  ros::param::get("/nmpc_controller/leg/step_length", dt_);

  // Test w/o warmstart
  // Set statistics
  Eigen::MatrixXi statistics(25 * 12, 5);

  // Test different horizon length
  for (int N_ = 12; N_ < 37; N_++) {
    // Test different start step
    for (int start_step = 0; start_step < 12; start_step++) {
      // Set statistics
      statistics((N_ - 12) * 12 + start_step, 0) = N_;
      statistics((N_ - 12) * 12 + start_step, 1) = start_step;

      // Set param
      ros::param::set("/nmpc_controller/leg/horizon_length", N_);
      ros::param::set("/nmpc_controller/centralized_tail/horizon_length", N_);
      ros::param::set("/nmpc_controller/distributed_tail/horizon_length", N_);

      // Init MPC
      NMPCController leg_planner_(0);
      NMPCController centralized_tail_planner_(1);
      NMPCController distributed_tail_planner_(2);

      // Set current states
      Eigen::VectorXd current_state_(12);
      current_state_.fill(0);
      current_state_(2) = 0.27;
      // current_state_(3) = -0.2;
      current_state_(5) = 1.57079632679;
      current_state_(6) = -1.;
      Eigen::VectorXd tail_current_state_(4);
      tail_current_state_.fill(0);

      // Set reference
      Eigen::MatrixXd ref_body_plan_(N_ + 1, 12);
      ref_body_plan_.fill(0);
      ref_body_plan_.row(0) = current_state_.transpose();
      ref_body_plan_.col(2).fill(0.27);
      ref_body_plan_.col(5).fill(1.57079632679);
      ref_body_plan_.col(6).fill(-1.);
      for (size_t i = 1; i < N_ + 1; i++) {
        ref_body_plan_(i, 0) =
            ref_body_plan_(i - 1, 0) + dt_ * ref_body_plan_(i - 1, 6);
      }
      Eigen::MatrixXd ref_tail_plan_(N_ + 1, 4);
      ref_tail_plan_.fill(0);

      // Set ground
      Eigen::VectorXd ref_ground_height(N_ + 1);
      ref_ground_height.fill(0);

      // Set parameter
      double first_element_duration = dt_;
      bool same_plan_index = false;

      // Set contact schedule from ith step
      std::vector<std::vector<bool>> adpative_contact_schedule_;
      adpative_contact_schedule_.resize(N_);
      for (size_t i = 0; i < N_; i++) {
        adpative_contact_schedule_.at(i).resize(4);
        if ((i + start_step) % 12 < 6) {
          adpative_contact_schedule_.at(i) = {true, false, false, true};
        } else {
          adpative_contact_schedule_.at(i) = {false, true, true, false};
        }
      }

      // Set foot position from ith step
      Eigen::MatrixXd foot_positions_body_(N_, 12);
      for (size_t i = 0; i < N_; i++) {
        foot_positions_body_.row(i) << 0.2263, 0.098, -0.27, -0.2263, 0.098,
            -0.27, 0.2263, -0.098, -0.27, -0.2263, -0.098, -0.27;
        for (size_t j = 0; j < 4; j++) {
          Eigen::Vector3d gait_shift;
          gait_shift << 0, 1, 0;
          double gait_idx = -(fmod(i + start_step, 6)) + 2.5;
          foot_positions_body_.block(i, j * 3, 1, 3) =
              foot_positions_body_.block(i, j * 3, 1, 3) +
              gait_shift.transpose() * dt_ * gait_idx;
        }
      }

      // Set output
      Eigen::MatrixXd body_plan_(N_ + 1, 12);
      Eigen::MatrixXd grf_plan_(N_, 12);
      Eigen::MatrixXd tail_plan_(N_ + 1, 16);
      Eigen::MatrixXd tail_torque_plan_(N_, 14);

      leg_planner_.computeLegPlan(
          current_state_, ref_body_plan_, foot_positions_body_,
          adpative_contact_schedule_, ref_ground_height, first_element_duration,
          same_plan_index, body_plan_, grf_plan_);
      distributed_tail_planner_.computeDistributedTailPlan(
          current_state_, ref_body_plan_, foot_positions_body_,
          adpative_contact_schedule_, tail_current_state_, ref_tail_plan_,
          body_plan_, grf_plan_, ref_ground_height, first_element_duration,
          same_plan_index, tail_plan_, tail_torque_plan_);
      centralized_tail_planner_.computeCentralizedTailPlan(
          current_state_, ref_body_plan_, foot_positions_body_,
          adpative_contact_schedule_, tail_current_state_, ref_tail_plan_,
          body_plan_, grf_plan_, ref_ground_height, first_element_duration,
          same_plan_index, tail_plan_, tail_torque_plan_);
      // current_state_(3) = -0.2;
      same_plan_index = true;

      for (size_t i = 0; i < 6; i++) {
        if (adpative_contact_schedule_.at(i).at(0)) {
          adpative_contact_schedule_.at(i) = {false, false, false, true};
        } else {
          adpative_contact_schedule_.at(i) = {false, false, true, false};
        }
      }

      // Solve distributed MPC
      std::chrono::steady_clock::time_point tic, toc;
      tic = std::chrono::steady_clock::now();

      leg_planner_.computeLegPlan(
          current_state_, ref_body_plan_, foot_positions_body_,
          adpative_contact_schedule_, ref_ground_height, first_element_duration,
          same_plan_index, body_plan_, grf_plan_);

      toc = std::chrono::steady_clock::now();
      statistics((N_ - 12) * 12 + start_step, 2) =
          std::chrono::duration_cast<std::chrono::microseconds>(toc - tic)
              .count();
      // std::cout << "Leg time difference = "
      //           << std::chrono::duration_cast<std::chrono::microseconds>(toc
      //           -
      //                                                                    tic)
      //                  .count()
      //           << "[µs]" << std::endl;
      // std::cout << grf_plan_ <<std::endl;

      tic = std::chrono::steady_clock::now();

      distributed_tail_planner_.computeDistributedTailPlan(
          current_state_, ref_body_plan_, foot_positions_body_,
          adpative_contact_schedule_, tail_current_state_, ref_tail_plan_,
          body_plan_, grf_plan_, ref_ground_height, first_element_duration,
          same_plan_index, tail_plan_, tail_torque_plan_);

      toc = std::chrono::steady_clock::now();
      statistics((N_ - 12) * 12 + start_step, 3) =
          std::chrono::duration_cast<std::chrono::microseconds>(toc - tic)
              .count();
      // std::cout << "Distributed tail time difference = "
      //           << std::chrono::duration_cast<std::chrono::microseconds>(toc
      //           -
      //                                                                    tic)
      //                  .count()
      //           << "[µs]" << std::endl;
      // std::cout << tail_torque_plan_ <<std::endl;

      tic = std::chrono::steady_clock::now();

      centralized_tail_planner_.computeCentralizedTailPlan(
          current_state_, ref_body_plan_, foot_positions_body_,
          adpative_contact_schedule_, tail_current_state_, ref_tail_plan_,
          body_plan_, grf_plan_, ref_ground_height, first_element_duration,
          same_plan_index, tail_plan_, tail_torque_plan_);

      toc = std::chrono::steady_clock::now();
      statistics((N_ - 12) * 12 + start_step, 4) =
          std::chrono::duration_cast<std::chrono::microseconds>(toc - tic)
              .count();
      // std::cout << "Centralized tail time difference = "
      //           << std::chrono::duration_cast<std::chrono::microseconds>(toc
      //           -
      //                                                                    tic)
      //                  .count()
      //           << "[µs]" << std::endl;
      // std::cout << tail_torque_plan_ <<std::endl;
    }
  }

  std::cout << statistics << std::endl;

  return 0;
}
