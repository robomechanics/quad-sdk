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
// Local includes
#include <mocap_optitrack/socket.h>
#include <mocap_optitrack/data_model.h>
#include <mocap_optitrack/mocap_config.h>
#include <mocap_optitrack/rigid_body_publisher.h>
#include "natnet/natnet_messages.h"

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

namespace mocap_optitrack
{

  class OptiTrackRosBridge
  {
  public:
    OptiTrackRosBridge(
      rclcpp::Node::SharedPtr &node,
      ServerDescription const& serverDescr, 
      PublisherConfigurations const& pubConfigs) :
        node(node),
        clock(node->get_clock()),
        serverDescription(serverDescr),
        publisherConfigurations(pubConfigs)
    {
      m_param_callback = node->add_on_set_parameters_callback(
          std::bind(&OptiTrackRosBridge::parametersCallback, this, std::placeholders::_1));
    }

    void initialize()
    {
      if (serverDescription.enableOptitrack)
      {
        // Create socket
        multicastClientSocketPtr.reset(
            new UdpMulticastSocket(node, serverDescription.dataPort, serverDescription.multicastIpAddress));

        if (!serverDescription.version.empty())
        {
          dataModel.setVersions(&serverDescription.version[0], &serverDescription.version[0]);
        }

        // Need verion information from the server to properly decode any of their packets.
        // If we have not recieved that yet, send another request.
        while (rclcpp::ok() && !dataModel.hasServerInfo())
        {
          natnet::ConnectionRequestMessage connectionRequestMsg;
          natnet::MessageBuffer connectionRequestMsgBuffer;
          connectionRequestMsg.serialize(connectionRequestMsgBuffer, NULL);
          int ret = multicastClientSocketPtr->send(&connectionRequestMsgBuffer[0], connectionRequestMsgBuffer.size(),
                                                   serverDescription.commandPort);
          if (updateDataModelFromServer()) usleep(10);
          else sleep(1);

          spin_some(node);
        }
        // Once we have the server info, create publishers
        publishDispatcherPtr.reset(
            new RigidBodyPublishDispatcher(node, dataModel.getNatNetVersion(), publisherConfigurations));
        RCLCPP_INFO(node->get_logger(), "Initialization complete");
        initialized = true;
      }
      else
      {
        RCLCPP_INFO(node->get_logger(), "Initialization incomplete");
        initialized = false;
      }
    };

    void run()
    {
      while (rclcpp::ok())
      {
        if (initialized && serverDescription.enableOptitrack)
        {
          if (updateDataModelFromServer())
          {
            // Maybe we got some data? If we did it would be in the form of one or more
            // rigid bodies in the data model
            const rclcpp::Time time = clock->now();
            publishDispatcherPtr->publish(time, dataModel.dataFrame.rigidBodies, node->get_logger());

            // Clear out the model to prepare for the next frame of data
            dataModel.clear();
          }
            // If we processed some data, take a short break
            usleep(100);
        }
        else
        {
            if (serverDescription.enableOptitrack)
            {
              initialize();
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        // whether receive or nor, give a short break to relieft the CPU load due to while()
        spin_some(node);
      }
    }

  private:
    bool updateDataModelFromServer()
    {
      // Get data from mocap server
      int numBytesReceived = multicastClientSocketPtr->recv();
      if( numBytesReceived > 0 )
      {
        // Grab latest message buffer
        const char* pMsgBuffer = multicastClientSocketPtr->getBuffer();

        // Copy char* buffer into MessageBuffer and dispatch to be deserialized
        natnet::MessageBuffer msgBuffer(pMsgBuffer, pMsgBuffer + numBytesReceived);
        natnet::MessageDispatcher::dispatch(node->get_logger(), msgBuffer, &dataModel);

        return true;
      }

      return false;
    };

    rcl_interfaces::msg::SetParametersResult
    parametersCallback(std::vector<rclcpp::Parameter> parameters)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful         = true;
      result.reason             = "";

      for (const auto& param : parameters)
      {
        std::stringstream ss;
        ss << "{" << param.get_name() << ", " << param.value_to_string() << "}";
        RCLCPP_INFO(node->get_logger(), "Got parameter: '%s'", ss.str().c_str());

        std::string rigid_bodies_prefix = "rigid_bodies.";

        if (!param.get_name().compare(rosparam::keys::CommandPort))
        {
          serverDescription.commandPort = param.as_int();
        }
        else if (!param.get_name().compare(rosparam::keys::DataPort))
        {
          serverDescription.dataPort = param.as_int();
        }
        else if (!param.get_name().compare(rosparam::keys::MulticastIpAddress))
        {
          serverDescription.multicastIpAddress = param.as_string();
        }
        else if (!param.get_name().compare(rosparam::keys::EnableOptitrack))
        {
          serverDescription.enableOptitrack = param.as_bool();
        }
        else
        {
          result.successful = false;
          result.reason     = "Parameter is not dynamic reconfigurable";
          RCLCPP_WARN(node->get_logger(),
                      "Parameter %s not dynamically reconfigurable",
                      param.get_name().c_str());
        }
      }
      return result;
    }

    rclcpp::Node::SharedPtr node;
    rclcpp::Clock::SharedPtr clock;
    ServerDescription serverDescription;
    PublisherConfigurations publisherConfigurations;
    DataModel dataModel;
    std::unique_ptr<UdpMulticastSocket> multicastClientSocketPtr;
    std::unique_ptr<RigidBodyPublishDispatcher> publishDispatcherPtr;
    bool initialized = false;

    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_param_callback;

  };

} // namespace


////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr rclcpp_node = std::make_shared<rclcpp::Node>("mocap_node", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));
  
  // Grab node configuration from rosparam
  mocap_optitrack::ServerDescription serverDescription;
  mocap_optitrack::PublisherConfigurations publisherConfigurations;
  mocap_optitrack::NodeConfiguration::fromRosParam(rclcpp_node, serverDescription, publisherConfigurations);

  // Create node object, initialize and run
  mocap_optitrack::OptiTrackRosBridge node(rclcpp_node, serverDescription, publisherConfigurations);
  node.initialize();
  node.run();

  return 0;
}
