#ifndef COMP_FILTER_H
#define COMP_FILTER_H

#include <robot_driver/estimators/state_estimator.h>

//! Implements complementary filter as an estimator within the ROS framework.
class CompFilterEstimator : public StateEstimator {
 public:
  /**
   * @brief Constructor for CompFilterEstimator
   * @return Constructed object of type CompFilterEstimator
   */
  CompFilterEstimator();

  void init();

  quad_msgs::RobotState updateState();


};

#endif  // COMP_FILTER_H
