#ifndef EKF_FILTER_H
#define EKF_FILTER_H

#include <robot_driver/estimators/state_estimator.h>

//! Implements Extended Kalman Filter as an estimator within the ROS framework.
class EKFFilterEstimator : public StateEstimator {
 public:
  /**
   * @brief Constructor for EKFFilterEstimator
   * @return Constructed object of type EKFFilterEstimator
   */
  EKFFilterEstimator();

  void init();

  bool updateState();
};

#endif  // EKF_FILTER_H
