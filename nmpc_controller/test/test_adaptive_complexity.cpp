#include <gtest/gtest.h>
#include <nmpc_controller/nmpc_controller.h>
#include <ros/ros.h>

#include <chrono>

TEST(NMPCTest, testAdaptiveComplexity) {
  int N_;
  ros::param::get("/nmpc_controller/leg/horizon_length", N_);

  std::shared_ptr<NMPCController> leg_planner_ =
      std::make_shared<NMPCController>(0);

  Eigen::VectorXd current_state_(12);
  current_state_.fill(0);
  current_state_(2) = 0.3;
  current_state_(9) = 0;

  Eigen::MatrixXd ref_body_plan_(N_ + 1, 12);
  ref_body_plan_.fill(0);
  ref_body_plan_.col(2).fill(0.3);

  Eigen::MatrixXd foot_positions_body_(N_, 12);
  for (size_t i = 0; i < N_; i++) {
    foot_positions_body_.row(i) << 0.2263, 0.098, -0.3, 0.2263, -0.098, -0.3,
        -0.2263, 0.098, -0.3, -0.2263, -0.098, -0.3;
  }

  std::vector<std::vector<bool>> adpative_contact_schedule_;
  adpative_contact_schedule_.resize(N_);
  for (size_t i = 0; i < N_; i++) {
    adpative_contact_schedule_.at(i).resize(4);
    if (i % 12 < 6) {
      adpative_contact_schedule_.at(i) = {true, false, false, true};
    } else {
      adpative_contact_schedule_.at(i) = {false, true, true, false};
    }
  }

  Eigen::VectorXd ref_ground_height(N_);
  ref_ground_height.fill(0);

  Eigen::MatrixXd body_plan_(N_, 12);
  body_plan_.col(2).fill(0.3);

  Eigen::MatrixXd grf_plan_(N_, 12);
  grf_plan_.fill(0);
  grf_plan_.col(2).fill(11.51 * 9.81 / 2);
  grf_plan_.col(5).fill(11.51 * 9.81 / 2);
  grf_plan_.col(8).fill(11.51 * 9.81 / 2);
  grf_plan_.col(11).fill(11.51 * 9.81 / 2);

  double first_element_duration = 0;
  bool same_plan_index = false;

  Eigen::VectorXi complexity_schedule(N_);
  complexity_schedule.setZero();

  std::chrono::steady_clock::time_point tic, toc;
  tic = std::chrono::steady_clock::now();

  for (size_t i = 0; i < 10; i++) {
    tic = std::chrono::steady_clock::now();

    leg_planner_->computeLegPlan(
        current_state_, ref_body_plan_, foot_positions_body_,
        adpative_contact_schedule_, ref_ground_height, first_element_duration,
        same_plan_index, complexity_schedule, body_plan_, grf_plan_);

    toc = std::chrono::steady_clock::now();
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::microseconds>(toc -
                                                                       tic)
                     .count()
              << "[µs]" << std::endl;

    current_state_ = body_plan_.row(1).transpose();
    std::rotate(adpative_contact_schedule_.begin(),
                adpative_contact_schedule_.begin() + 1,
                adpative_contact_schedule_.end());
  }

  EXPECT_TRUE(true);
}
