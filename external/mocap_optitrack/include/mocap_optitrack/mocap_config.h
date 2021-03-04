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

#include <ros/ros.h>

namespace mocap_optitrack
{

/// \brief Server communication info
struct ServerDescription
{
  struct Default
  {
    static const int CommandPort;
    static const int DataPort;
    static const std::string MulticastIpAddress;
  };

  ServerDescription();
  int commandPort;
  int dataPort;
  std::string multicastIpAddress;
  std::vector<int> version;
};

/// \brief ROS publisher configuration
struct PublisherConfiguration
{
  int rigidBodyId;
  std::string poseTopicName;
  std::string pose2dTopicName;
  std::string childFrameId;
  std::string parentFrameId;

  bool publishPose;
  bool publishPose2d;
  bool publishTf;
};

typedef std::vector<PublisherConfiguration> PublisherConfigurations;

/// \brief Handles loading node configuration from different sources
struct NodeConfiguration
{
  static void fromRosParam(ros::NodeHandle& nh, 
    ServerDescription& serverDescription, 
    PublisherConfigurations& pubConfigs);
};

} // namespace

#endif  // __MOCAP_OPTITRACK_MOCAP_CONFIG_H__
