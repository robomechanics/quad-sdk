#ifndef EKF_FILTER_H
#define EKF_FILTER_H

#include <robot_driver/estimators/state_estimator.h>

//! Implements complementary filter as an estimator within the ROS framework.
class EKFFilterEstimator : public StateEstimator {
 public:
  /**
   * @brief Constructor for EKFFilterEstimator
   * @return Constructed object of type EKFFilterEstimator
   */
  EKFFilterEstimator();

  void init();

  quad_msgs::RobotState updateState();
};

#endif  // EKF_FILTER_H
