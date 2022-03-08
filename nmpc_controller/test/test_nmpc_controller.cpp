#include <gtest/gtest.h>
#include <ros/ros.h>

#include <chrono>

#include "nmpc_controller/nmpc_controller.h"

TEST(NMPCTest, testTailMPC) {
  // int N_, N_tail_;
  // double dt_;
  // ros::param::get("/local_planner/horizon_length", N_);
  // ros::param::get("/local_planner/timestep", dt_);
  // ros::param::get("/nmpc_controller/distributed_tail/horizon_length",
  // N_tail_);

  // std::shared_ptr<NMPCController> leg_planner_ =
  //     std::make_shared<NMPCController>(0);
  // std::shared_ptr<NMPCController> distributed_tail_planner_ =
  //     std::make_shared<NMPCController>(2);

  // Eigen::VectorXd current_state_(12);
  // current_state_.fill(0);
  // current_state_(2) = 0.3;
  // current_state_(9) = 0;

  // Eigen::MatrixXd ref_body_plan_(N_ + 1, 12);
  // ref_body_plan_.fill(0);
  // ref_body_plan_.col(2).fill(0.3);

  // Eigen::MatrixXd foot_positions_body_(N_, 12);
  // Eigen::MatrixXd foot_positions_world_(N_, 12);
  // Eigen::MatrixXd foot_velocities_world_(N_, 12);
  // foot_velocities_world_.setZero();
  // for (size_t i = 0; i < N_; i++) {
  //   foot_positions_body_.row(i) << 0.2263, 0.098, -0.3, 0.2263, -0.098, -0.3,
  //       -0.2263, 0.098, -0.3, -0.2263, -0.098, -0.3;
  //   foot_positions_world_.row(i) << 0.2263, 0.098, 0, 0.2263, -0.098, 0,
  //       -0.2263, 0.098, 0, -0.2263, -0.098, 0;
  // }

  // std::vector<std::vector<bool>> adpative_contact_schedule_;
  // adpative_contact_schedule_.resize(N_);
  // for (size_t i = 0; i < N_; i++) {
  //   adpative_contact_schedule_.at(i).resize(4);
  //   if (i % 12 < 6) {
  //     adpative_contact_schedule_.at(i) = {true, false, false, true};
  //   } else {
  //     adpative_contact_schedule_.at(i) = {false, true, true, false};
  //   }
  // }

  // Eigen::VectorXd ref_ground_height(N_ + 1);
  // ref_ground_height.fill(0);

  // Eigen::MatrixXd body_plan_(N_ + 1, 12);
  // body_plan_.col(2).fill(0.3);

  // Eigen::MatrixXd grf_plan_(N_, 12);
  // grf_plan_.fill(0);
  // grf_plan_.col(2).fill(13.3 * 9.81 / 2);
  // grf_plan_.col(5).fill(13.3 * 9.81 / 2);
  // grf_plan_.col(8).fill(13.3 * 9.81 / 2);
  // grf_plan_.col(11).fill(13.3 * 9.81 / 2);

  // Eigen::VectorXd tail_current_state_(4);
  // tail_current_state_.fill(0);
  // tail_current_state_(0) = 0.76;

  // Eigen::MatrixXd ref_tail_plan_(N_tail_ + 1, 4);
  // ref_tail_plan_.fill(0);
  // ref_tail_plan_.col(0).fill(0.76);

  // Eigen::MatrixXd tail_plan_(N_tail_ + 1, 4);
  // tail_plan_.fill(0);

  // Eigen::MatrixXd tail_torque_plan_(N_tail_, 2);
  // tail_torque_plan_.fill(0);

  // double first_element_duration = dt_;

  // bool same_plan_index = false;

  // Eigen::VectorXi complexity_schedule(N_ + 1), ref_primitive_id(N_ + 1);
  // complexity_schedule.setZero();
  // ref_primitive_id.setZero();

  // std::chrono::steady_clock::time_point tic, toc;
  // tic = std::chrono::steady_clock::now();

  // for (size_t i = 0; i < 10; i++) {
  //   tic = std::chrono::steady_clock::now();

  //   leg_planner_->computeLegPlan(
  //       current_state_, ref_body_plan_, foot_positions_world_,
  //       foot_velocities_world_, adpative_contact_schedule_,
  //       ref_ground_height, first_element_duration, same_plan_index,
  //       ref_primitive_id, complexity_schedule, body_plan_, grf_plan_);
  //   toc = std::chrono::steady_clock::now();
  //   std::cout << "Time difference = "
  //             << std::chrono::duration_cast<std::chrono::microseconds>(toc -
  //                                                                      tic)
  //                    .count()
  //             << "[µs]" << std::endl;

  //   tic = std::chrono::steady_clock::now();
  //   distributed_tail_planner_->computeDistributedTailPlan(
  //       current_state_, ref_body_plan_.topRows(N_tail_ + 1),
  //       foot_positions_body_.topRows(N_tail_),
  //       std::vector<std::vector<bool>>(
  //           adpative_contact_schedule_.begin(),
  //           adpative_contact_schedule_.begin() + N_tail_),
  //       tail_current_state_, ref_tail_plan_, body_plan_.topRows(N_tail_ + 1),
  //       grf_plan_.topRows(N_tail_), ref_ground_height.topRows(N_tail_ + 1),
  //       first_element_duration, same_plan_index, tail_plan_,
  //       tail_torque_plan_);

  //   toc = std::chrono::steady_clock::now();
  //   std::cout << "Time difference = "
  //             << std::chrono::duration_cast<std::chrono::microseconds>(toc -
  //                                                                      tic)
  //                    .count()
  //             << "[µs]" << std::endl;

  //   current_state_ = body_plan_.row(1).transpose();
  //   tail_current_state_ = tail_plan_.row(1).transpose();
  //   std::rotate(adpative_contact_schedule_.begin(),
  //               adpative_contact_schedule_.begin() + 1,
  //               adpative_contact_schedule_.end());
  // }

  EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "nmpc_controller_tester");

  return RUN_ALL_TESTS();
}
