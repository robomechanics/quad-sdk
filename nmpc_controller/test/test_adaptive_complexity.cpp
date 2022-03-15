#include <gtest/gtest.h>
#include <nmpc_controller/nmpc_controller.h>
#include <ros/ros.h>

#include <chrono>

TEST(NMPCTest, testAdaptiveComplexity) {
  int N_;
  double dt_;
  ros::param::get("/local_planner/horizon_length", N_);
  ros::param::get("/local_planner/timestep", dt_);

  std::shared_ptr<NMPCController> leg_planner_ =
      std::make_shared<NMPCController>(0);

  Eigen::MatrixXd ref_body_plan_(N_, 12);
  ref_body_plan_.fill(0);
  ref_body_plan_.col(2).fill(0.3);

  Eigen::MatrixXd foot_positions_body_(N_, 12);
  Eigen::MatrixXd foot_positions_world_(N_, 12);
  Eigen::MatrixXd foot_velocities_world_(N_, 12);
  foot_velocities_world_.setZero();
  for (size_t i = 0; i < N_; i++) {
    foot_positions_body_.row(i) << 0.2263, 0.098, -0.3, 0.2263, -0.098, -0.3,
        -0.2263, 0.098, -0.3, -0.2263, -0.098, -0.3;
    foot_positions_world_.row(i) << 0.2263, 0.098, 0, -0.2263, 0.098, 0, 0.2263,
        -0.098, 0, -0.2263, -0.098, 0;
  }

  // Load the current state
  Eigen::VectorXd current_state_(36);
  current_state_.fill(0);
  current_state_(2) = 0.3;
  current_state_(9) = 0;

  quad_utils::QuadKD quad_kd;
  Eigen::VectorXd x_null_nom_(24);
  x_null_nom_.setZero();
  for (int i = 0; i < 4; i++) {
    Eigen::Vector3d joint_state;
    quad_kd.worldToFootIKWorldFrame(
        i, current_state_.segment(0, 3), current_state_.segment(3, 3),
        foot_positions_world_.row(0).segment(3 * i, 3), joint_state);
    x_null_nom_.segment<3>(3 * i) = joint_state;
  }

  // double abad_nom = 0;
  // double hip_nom = 1.57 * 0.5;
  // double knee_nom = 1.57;
  // x_null_nom_ << abad_nom, hip_nom, knee_nom, abad_nom, hip_nom, knee_nom,
  //     abad_nom, hip_nom, knee_nom, abad_nom, hip_nom, knee_nom, 0, 0, 0, 0,
  //     0, 0, 0, 0, 0, 0, 0, 0;

  current_state_.segment(12, 24) = x_null_nom_;

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

  Eigen::MatrixXd grf_plan_(N_ - 1, 12);
  grf_plan_.fill(0);
  grf_plan_.col(2).fill(13.3 * 9.81 / 2);
  grf_plan_.col(5).fill(13.3 * 9.81 / 2);
  grf_plan_.col(8).fill(13.3 * 9.81 / 2);
  grf_plan_.col(11).fill(13.3 * 9.81 / 2);

  double first_element_duration = dt_;

  bool same_plan_index = false;

  Eigen::VectorXi complexity_schedule(N_), ref_primitive_id(N_);
  complexity_schedule.setZero();
  ref_primitive_id.setZero();

  std::chrono::steady_clock::time_point tic, toc;
  tic = std::chrono::steady_clock::now();

  Eigen::VectorXd joint_positions(12), joint_velocities(12), torques(12);

  for (int i = 0; i < 10; i++) {
    tic = std::chrono::steady_clock::now();
    // complexity_schedule.tail(2 * i + 1).fill(1);
    // complexity_schedule.head(2 * i + 1).fill(1);

    // std::cout << "current state = " << current_state_.transpose() <<
    // std::endl;

    leg_planner_->computeLegPlan(
        current_state_, ref_body_plan_, foot_positions_world_,
        foot_velocities_world_, adpative_contact_schedule_, ref_ground_height,
        first_element_duration, same_plan_index, ref_primitive_id,
        complexity_schedule, body_plan_, grf_plan_);
    toc = std::chrono::steady_clock::now();
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::microseconds>(toc -
                                                                       tic)
                     .count()
              << "[µs]" << std::endl;

    current_state_.head(12) = body_plan_.row(1).transpose();

    quad_kd.convertCentroidalToFullBody(
        current_state_.head(12), foot_positions_world_.row(i),
        foot_velocities_world_.row(i), grf_plan_.row(1), joint_positions,
        joint_velocities, torques);
    current_state_.segment(12, 12) = joint_positions;
    current_state_.segment(24, 12) = joint_velocities;

    std::rotate(adpative_contact_schedule_.begin(),
                adpative_contact_schedule_.begin() + 1,
                adpative_contact_schedule_.end());
  }

  EXPECT_TRUE(true);
}
