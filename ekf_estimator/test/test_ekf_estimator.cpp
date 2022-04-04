#include <gtest/gtest.h>
#include <quad_msgs/ContactMode.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/quad_kd.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Eigen>
#include <vector>

#include "ekf_estimator/ekf_estimator.h"

TEST(EKFEstimator, basicTestCase) {
  ros::NodeHandle nh_;

  EKFEstimator ekf_estimator(nh_);

  std::shared_ptr<quad_utils::QuadKD> quadKD_ =
      std::make_shared<quad_utils::QuadKD>();
  double dt = 0.025;
  // IMU reading linear acceleration
  Eigen::VectorXd fk = Eigen::VectorXd::Zero(3);
  // IMU reading angular acceleration
  Eigen::VectorXd wk = Eigen::VectorXd::Zero(3);
  // IMU orientation (w, x, y, z)
  Eigen::Quaterniond qk(1, 0, 0, 0);

  Eigen::VectorXd X0 = Eigen::VectorXd::Zero(28);
  X0 << -1.457778, 1.004244, 0.308681, 0, 0, 0, 0.998927, 0.004160, -0.003017,
      -0.046032, -1.251841, 1.185387, 0.012734, -1.695057, 1.148678, 0.007092,
      -1.236598, 0.861900, 0.016119, -1.678741, 0.831065, 0.020651, 0, 0, 0, 0,
      0, 0;

  ekf_estimator.setX(X0);
  ekf_estimator.setlast_X(X0);

  Eigen::MatrixXd P0 = 0.01 * Eigen::MatrixXd::Identity(27, 27);
  ekf_estimator.setP(P0);
  // std::cout << "this is X" << ekf_estimator.getX() << std::endl;
  ekf_estimator.setNoise();

  ekf_estimator.predict(dt, fk, wk, qk);

  Eigen::VectorXd Xtemp = ekf_estimator.getX_pre();

  EXPECT_NEAR(Xtemp[0], X0[0], 0.001);
  EXPECT_NEAR(Xtemp[1], X0[1], 0.001);
  EXPECT_NEAR(Xtemp[2], X0[2], 0.001);

  Eigen::VectorXd jk = Eigen::VectorXd::Zero(12);

  ekf_estimator.update(jk);

  Eigen::VectorXd X_updated = ekf_estimator.getX();

  EXPECT_NEAR(Xtemp[0], X0[0], 0.001);
  EXPECT_NEAR(Xtemp[1], X0[1], 0.001);
  EXPECT_NEAR(Xtemp[2], X0[2], 0.001);

  // std::cout << "x" << Xtemp[0] << std::endl;
  // std::cout << "y" << Xtemp[1] << std::endl;
  // std::cout << "z" << Xtemp[2] << std::endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ekf_estimator_tester");

  return RUN_ALL_TESTS();
}
