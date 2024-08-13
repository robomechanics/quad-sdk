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
#ifndef __MOCAP_OPTITRACK_MOCAP_CONFIG_H__
#define __MOCAP_OPTITRACK_MOCAP_CONFIG_H__

#include <vector>
#include <string>

#include <rclcpp/node.hpp>

namespace mocap_optitrack
{
// Param keys
namespace rosparam
{
  namespace keys
  {
    const std::string MulticastIpAddress = "optitrack_config.multicast_address";
    const std::string CommandPort = "optitrack_config.command_port";
    const std::string DataPort = "optitrack_config.data_port";
    const std::string EnableOptitrack = "optitrack_config.enable_optitrack";
    const std::string Version = "optitrack_config.version";
    const std::string RigidBodies = "rigid_bodies";
    const std::string PoseTopicName = "pose";
    const std::string Pose2dTopicName = "pose2d";
    const std::string OdomTopicName = "odom";
    const std::string EnableTfPublisher = "tf";
    const std::string ChildFrameId = "child_frame_id";
    const std::string ParentFrameId = "parent_frame_id";
  }
}

/// \brief Server communication info
struct ServerDescription
{
  struct Default
  {
    static const int CommandPort;
    static const int DataPort;
    static const std::string MulticastIpAddress;
    static const bool EnableOptitrack;
  };

  ServerDescription();
  int commandPort;
  int dataPort;
  std::string multicastIpAddress;
  bool enableOptitrack;
  std::vector<int64_t> version;
};

/// \brief ROS publisher configuration
struct PublisherConfiguration
{
  int rigidBodyId;
  std::string poseTopicName;
  std::string pose2dTopicName;
  std::string odomTopicName;
  std::string enableTfPublisher;
  std::string childFrameId;
  std::string parentFrameId;

  bool publishPose;
  bool publishPose2d;
  bool publishOdom;
  bool publishTf;
};

typedef std::vector<PublisherConfiguration> PublisherConfigurations;

/// \brief Handles loading node configuration from different sources
struct NodeConfiguration
{
  static void fromRosParam(rclcpp::Node::SharedPtr& nh, 
    ServerDescription& serverDescription, 
    PublisherConfigurations& pubConfigs);
};

} // namespace

#endif  // __MOCAP_OPTITRACK_MOCAP_CONFIG_H__
