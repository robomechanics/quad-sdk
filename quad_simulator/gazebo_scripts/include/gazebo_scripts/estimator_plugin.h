#ifndef GAZEBO_SPIRIT_ESTIMATOR_PLUGIN
#define GAZEBO_SPIRIT_ESTIMATOR_PLUGIN
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/WorldPose.hh>
#include <memory>
#include <quad_msgs/msg/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>


namespace gazebo_plugins{
    class GroundTruthEstimator
}

#endif