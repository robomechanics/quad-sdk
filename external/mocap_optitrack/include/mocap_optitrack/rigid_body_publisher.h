/* 
 * Copyright (c) 2018, Houston Mechatronics Inc., JD Yamokoski
 * Copyright (c) 2012, Clearpath Robotics, Inc., Alex Bencz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __MOCAP_OPTITRACK_RIGID_BODY_PUBLISHER_H__
#define __MOCAP_OPTITRACK_RIGID_BODY_PUBLISHER_H__

#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mocap_optitrack/version.h>
#include <mocap_optitrack/data_model.h>
#include <mocap_optitrack/mocap_config.h>

namespace mocap_optitrack
{

/// \brief Encapsulation of a RigidBody data publisher.
class RigidBodyPublisher
{
public:
  RigidBodyPublisher(rclcpp::Node::SharedPtr &node, 
    Version const& natNetVersion, 
    PublisherConfiguration const& config);
  ~RigidBodyPublisher();
  void publish(rclcpp::Time const& time, RigidBody const&, rclcpp::Logger);

private:
  PublisherConfiguration config;

  Version coordinatesVersion;

  double timeDifference;	//For syncing optitrack clock to ROS clock

  tf2_ros::TransformBroadcaster tfPublisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2dPublisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
};

/// \brief Dispatches RigidBody data to the correct publisher.
class RigidBodyPublishDispatcher
{
    typedef std::shared_ptr<RigidBodyPublisher> RigidBodyPublisherPtr;
    typedef std::map<int,RigidBodyPublisherPtr> RigidBodyPublisherMap;
    RigidBodyPublisherMap rigidBodyPublisherMap;

public:
    RigidBodyPublishDispatcher(rclcpp::Node::SharedPtr &node, 
        Version const& natNetVersion, 
        PublisherConfigurations const& configs);
    void publish(rclcpp::Time const& time, std::vector<RigidBody> const& rigidBodies, rclcpp::Logger logger);
};

} // namespace

#endif
