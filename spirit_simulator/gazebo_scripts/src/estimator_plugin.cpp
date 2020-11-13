#include "estimator_plugin.h"
#include <iostream>
namespace gazebo{

  SpiritEstimatorGroundTruth::SpiritEstimatorGroundTruth()
  {

  }
  SpiritEstimatorGroundTruth::~SpiritEstimatorGroundTruth(){}

  void SpiritEstimatorGroundTruth::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    model_ = _parent;

    last_time_ = model_->GetWorld()->SimTime();
    // Load update rate from SDF
    if (_sdf->HasElement("updateRateHZ"))
    {
      update_rate_ =  _sdf->Get<double>("updateRateHZ");
      ROS_INFO_STREAM("Ground Truth State Estimator: <updateRateHZ> set to: " << update_rate_);
     }
     else
     {
       update_rate_ = 100.0;
       ROS_WARN_STREAM("Ground Truth State Estimator: missing <updateRateHZ>, set to default: " << update_rate_);
     }

    // Setup state estimate publisher
    ros::NodeHandle nh;
    state_estimate_pub_ = nh.advertise<spirit_msgs::StateEstimate>("/gazebo/ground_truth_state",1);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
    updateConnection_= event::Events::ConnectWorldUpdateBegin(
        std::bind(&SpiritEstimatorGroundTruth::OnUpdate, this));
  }
  void SpiritEstimatorGroundTruth::OnUpdate()
  {
    common::Time current_time = model_->GetWorld()->SimTime();
    
    if(update_rate_>0 && (current_time-last_time_).Double() < 1.0/update_rate_) //update rate check
      return;

    // Extract all relevant information from simulator
    physics::LinkPtr body_link = model_->GetChildLink("body");
    if (!body_link)
    {
      ROS_ERROR("Can't find body link in sdf. Make sure the name in the plugin matches the sdf.");
      return;
    }
    ignition::math::Pose3d pose = body_link->WorldPose();
    ignition::math::Vector3d lin_pos = pose.Pos();
    ignition::math::Quaternion<double> ang_pos = pose.Rot();
    ignition::math::Vector3d lin_vel = body_link->WorldLinearVel();
    ignition::math::Vector3d ang_vel = body_link->WorldAngularVel();

    // Update and publish state estimate message
    spirit_msgs::StateEstimate state;
    state.body.pose.pose.position.x = lin_pos.X();
    state.body.pose.pose.position.y = lin_pos.Y();
    state.body.pose.pose.position.z = lin_pos.Z();
    state.body.pose.pose.orientation.w = ang_pos.W();
    state.body.pose.pose.orientation.x = ang_pos.X();
    state.body.pose.pose.orientation.y = ang_pos.Y();
    state.body.pose.pose.orientation.z = ang_pos.Z();
    state.body.twist.twist.linear.x = lin_vel.X();
    state.body.twist.twist.linear.y = lin_vel.Y();
    state.body.twist.twist.linear.z = lin_vel.Z();
    state.body.twist.twist.angular.x = ang_vel.X();
    state.body.twist.twist.angular.y = ang_vel.Y();
    state.body.twist.twist.angular.z = ang_vel.Z();
    state.header.stamp = ros::Time::now();
    state_estimate_pub_.publish(state);

    last_time_ = current_time;
  }
}