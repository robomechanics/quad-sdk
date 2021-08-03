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
    ground_truth_state_pub_ = nh.advertise<spirit_msgs::RobotState>("/state/ground_truth",1);

    std::string param_name = "/spirit/joint_estimator/joints";
    if(!nh.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nh.getNamespace() << ").");
      return;
    }

    nh.param<int>("/tail_controller/tail_type", tail_type_, 0);

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

    ignition::math::Vector3d toe_offset(0.206, 0, 0);

    physics::LinkPtr lower0 = model_->GetChildLink("lower0");
    physics::LinkPtr lower1 = model_->GetChildLink("lower1");
    physics::LinkPtr lower2 = model_->GetChildLink("lower2");
    physics::LinkPtr lower3 = model_->GetChildLink("lower3");

    physics::LinkPtr toe0 = model_->GetChildLink("toe0");
    physics::LinkPtr toe1 = model_->GetChildLink("toe1");
    physics::LinkPtr toe2 = model_->GetChildLink("toe2");
    physics::LinkPtr toe3 = model_->GetChildLink("toe3");

    if (!body_link)
    {
      ROS_ERROR("Can't find body link in sdf. Make sure the name in the plugin matches the sdf.");
      return;
    }

    ignition::math::Pose3d pose = body_link->WorldPose();
    ignition::math::Vector3d lin_pos = pose.Pos();
    ignition::math::Quaternion<double> ang_pos = pose.Rot();
    ignition::math::Vector3d lin_vel = body_link->WorldLinearVel();
    // ignition::math::Vector3d ang_vel = body_link->WorldAngularVel();
    ignition::math::Vector3d ang_vel = body_link->RelativeAngularVel();

    ignition::math::Pose3d toe0_pose = toe0->WorldPose();
    ignition::math::Vector3d toe0_pos = toe0_pose.Pos();
    ignition::math::Vector3d toe0_vel = toe0->WorldLinearVel();

    ignition::math::Pose3d toe1_pose = toe1->WorldPose();
    ignition::math::Vector3d toe1_pos = toe1_pose.Pos();
    ignition::math::Vector3d toe1_vel = toe1->WorldLinearVel();

    ignition::math::Pose3d toe2_pose = toe2->WorldPose();
    ignition::math::Vector3d toe2_pos = toe2_pose.Pos();
    ignition::math::Vector3d toe2_vel = toe2->WorldLinearVel();

    ignition::math::Pose3d toe3_pose = toe3->WorldPose();
    ignition::math::Vector3d toe3_pos = toe3_pose.Pos();
    ignition::math::Vector3d toe3_vel = toe3->WorldLinearVel();

    // Update and publish state estimate message
    spirit_msgs::RobotState state;
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

    physics::Joint_V joint_vec = model_->GetJoints();
    std::vector<std::string> model_joint_name;

    // Record all joint names
    for (int i = 0; i<joint_vec.size(); i++) {
      physics::JointPtr joint = joint_vec[i];
      model_joint_name.push_back(joint->GetName());
    }

    int num_joints = joint_names_.size();

    if (tail_type_ != NONE)
    {
      // With tail
      for (size_t i = 12; i < 14; i++)
      {
        // Get expected joint index by name
        int idx;
        auto it = find(model_joint_name.begin(), model_joint_name.end(), joint_names_[i]);
        if (it != model_joint_name.end())
        {
          idx = it - model_joint_name.begin();
        }
        else
        {
          ROS_ERROR_STREAM("Cannot find joint '" << joint_names_[i] << "'.");
        }

        physics::JointPtr joint = joint_vec[idx];
        physics::JointWrench wrench = joint->GetForceTorque(0);
        double torque = wrench.body1Torque.Z(); // Note that this doesn't seem to work but at least will populate with zeros

        state.tail_joints.name.push_back(joint_names_.at(i));
        state.tail_joints.position.push_back(joint->Position());
        state.tail_joints.velocity.push_back(joint->GetVelocity(0));
        state.tail_joints.effort.push_back(torque);
      }
    }

    for (int i = 0; i<12; i++) {
      // std::cout << joint->GetName() << std::endl;
      // std::cout << joint->Position() << std::endl;
      // std::cout << joint->GetVelocity(0) << std::endl;

      // Get expected joint index by name
      int idx;
      auto it = find(model_joint_name.begin(), model_joint_name.end(), joint_names_[i]);
      if (it != model_joint_name.end()) {
          idx = it - model_joint_name.begin();
      }
      else {
          ROS_ERROR_STREAM("Cannot find joint '" << joint_names_[i] << "'.");
      }

      physics::JointPtr joint = joint_vec[idx];
      physics::JointWrench wrench = joint->GetForceTorque(0);
      double torque = wrench.body1Torque.Z(); // Note that this doesn't seem to work but at least will populate with zeros

      state.joints.name.push_back(joint_names_[i]);
      state.joints.position.push_back(joint->Position());
      state.joints.velocity.push_back(joint->GetVelocity(0));
      state.joints.effort.push_back(torque);
    }

    int num_feet = 4;
    state.feet.feet.resize(num_feet);

    spirit_utils::SpiritKinematics kinematics;
    spirit_utils::fkRobotState(kinematics,state);

    for (int i = 0; i<num_feet; i++) {
      switch (i) {
        case 0:
          state.feet.feet[i].position.x = toe0_pos.X();
          state.feet.feet[i].position.y = toe0_pos.Y();
          state.feet.feet[i].position.z = toe0_pos.Z();

          state.feet.feet[i].velocity.x = toe0_vel.X();
          state.feet.feet[i].velocity.y = toe0_vel.Y();
          state.feet.feet[i].velocity.z = toe0_vel.Z();
          break;

        case 1:
          state.feet.feet[i].position.x = toe1_pos.X();
          state.feet.feet[i].position.y = toe1_pos.Y();
          state.feet.feet[i].position.z = toe1_pos.Z();

          state.feet.feet[i].velocity.x = toe1_vel.X();
          state.feet.feet[i].velocity.y = toe1_vel.Y();
          state.feet.feet[i].velocity.z = toe1_vel.Z();
          break;

        case 2:
          state.feet.feet[i].position.x = toe2_pos.X();
          state.feet.feet[i].position.y = toe2_pos.Y();
          state.feet.feet[i].position.z = toe2_pos.Z();

          state.feet.feet[i].velocity.x = toe2_vel.X();
          state.feet.feet[i].velocity.y = toe2_vel.Y();
          state.feet.feet[i].velocity.z = toe2_vel.Z();
          break;

        case 3:
          state.feet.feet[i].position.x = toe3_pos.X();
          state.feet.feet[i].position.y = toe3_pos.Y();
          state.feet.feet[i].position.z = toe3_pos.Z();

          state.feet.feet[i].velocity.x = toe3_vel.X();
          state.feet.feet[i].velocity.y = toe3_vel.Y();
          state.feet.feet[i].velocity.z = toe3_vel.Z();
          break;
      }
    }

    state.header.stamp = ros::Time::now();
    ground_truth_state_pub_.publish(state);

    last_time_ = current_time;
  }
}