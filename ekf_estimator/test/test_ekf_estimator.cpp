#include <gtest/gtest.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>

#include "ekf_estimator/ekf_estimator.h"

TEST(EKFEstimator, basicTestCase) {
  ros::NodeHandle nh_;

  EKFEstimator ekf_estimator(nh_);

  //  void predict(const double& dt, const Eigen::VectorXd& fk,
  //              const Eigen::VectorXd& wk, const Eigen::Quaterniond& qk);
  // Declare dt, fk, wk, qk
  ekf_estimator.predict();

  ekf_estimator.update();

  // state-wise difference between gt and estimate < epsilon (user-selected
  // parameter)
  //  delta_error
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ekf_estimator_tester");

  return RUN_ALL_TESTS();
}
