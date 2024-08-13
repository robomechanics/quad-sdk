/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "rcutils/logging_macros.h"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "teleop_twist_joy/teleop_twist_joy.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string & which_map);
  void fillCmdVelMsg(
    const sensor_msgs::msg::Joy::SharedPtr, const std::string & which_map,
    geometry_msgs::msg::Twist * cmd_vel_msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub;
  rclcpp::Clock::SharedPtr clock;

  bool publish_stamped_twist;
  std::string frame_id;
  bool require_enable_button;
  int64_t enable_button;
  int64_t enable_turbo_button;

  bool inverted_reverse;

  std::map<std::string, int64_t> axis_linear_map;
  std::map<std::string, std::map<std::string, double>> scale_linear_map;

  std::map<std::string, int64_t> axis_angular_map;
  std::map<std::string, std::map<std::string, double>> scale_angular_map;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 */
TeleopTwistJoy::TeleopTwistJoy(const rclcpp::NodeOptions & options)
: rclcpp::Node("teleop_twist_joy_node", options)
{
  pimpl_ = new Impl;

  pimpl_->clock = this->get_clock();

  pimpl_->publish_stamped_twist = this->declare_parameter("publish_stamped_twist", false);
  pimpl_->frame_id = this->declare_parameter("frame", "teleop_twist_joy");

  if (pimpl_->publish_stamped_twist) {
    pimpl_->cmd_vel_stamped_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10);
  } else {
    pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::QoS(10),
    std::bind(&TeleopTwistJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  pimpl_->require_enable_button = this->declare_parameter("require_enable_button", true);

  pimpl_->enable_button = this->declare_parameter("enable_button", 5);

  pimpl_->enable_turbo_button = this->declare_parameter("enable_turbo_button", -1);

  pimpl_->inverted_reverse = this->declare_parameter("inverted_reverse", false);

  std::map<std::string, int64_t> default_linear_map{
    {"x", 5L},
    {"y", -1L},
    {"z", -1L},
  };
  this->declare_parameters("axis_linear", default_linear_map);
  this->get_parameters("axis_linear", pimpl_->axis_linear_map);

  std::map<std::string, int64_t> default_angular_map{
    {"yaw", 2L},
    {"pitch", -1L},
    {"roll", -1L},
  };
  this->declare_parameters("axis_angular", default_angular_map);
  this->get_parameters("axis_angular", pimpl_->axis_angular_map);

  std::map<std::string, double> default_scale_linear_normal_map{
    {"x", 0.5},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear", default_scale_linear_normal_map);
  this->get_parameters("scale_linear", pimpl_->scale_linear_map["normal"]);

  std::map<std::string, double> default_scale_linear_turbo_map{
    {"x", 1.0},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear_turbo", default_scale_linear_turbo_map);
  this->get_parameters("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);

  std::map<std::string, double> default_scale_angular_normal_map{
    {"yaw", 0.5},
    {"pitch", 0.0},
    {"roll", 0.0},
  };
  this->declare_parameters("scale_angular", default_scale_angular_normal_map);
  this->get_parameters("scale_angular", pimpl_->scale_angular_map["normal"]);

  std::map<std::string, double> default_scale_angular_turbo_map{
    {"yaw", 1.0},
    {"pitch", 0.0},
    {"roll", 0.0},
  };
  this->declare_parameters("scale_angular_turbo", default_scale_angular_turbo_map);
  this->get_parameters("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);

  ROS_INFO_COND_NAMED(
    pimpl_->require_enable_button, "TeleopTwistJoy",
    "Teleop enable button %" PRId64 ".", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(
    pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
    "Turbo on button %" PRId64 ".", pimpl_->enable_turbo_button);
  ROS_INFO_COND_NAMED(
    pimpl_->inverted_reverse, "TeleopTwistJoy", "%s", "Teleop enable inverted reverse.");

  for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_linear_map.begin();
    it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_COND_NAMED(
      it->second != -1L, "TeleopTwistJoy", "Linear axis %s on %" PRId64 " at scale %f.",
      it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(
      pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
      "Turbo for linear axis %s is scale %f.", it->first.c_str(),
      pimpl_->scale_linear_map["turbo"][it->first]);
  }

  for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_angular_map.begin();
    it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_COND_NAMED(
      it->second != -1L, "TeleopTwistJoy", "Angular axis %s on %" PRId64 " at scale %f.",
      it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(
      pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
      "Turbo for angular axis %s is scale %f.", it->first.c_str(),
      pimpl_->scale_angular_map["turbo"][it->first]);
  }

  pimpl_->sent_disable_msg = false;

  auto param_callback =
    [this](std::vector<rclcpp::Parameter> parameters)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      // Loop to assign changed parameters to the member variables
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "require_enable_button") {
          this->pimpl_->require_enable_button = parameter.get_value<rclcpp::PARAMETER_BOOL>();
        } else if (parameter.get_name() == "inverted_reverse") {
          this->pimpl_->inverted_reverse = parameter.get_value<rclcpp::PARAMETER_BOOL>();
        } else if (parameter.get_name() == "enable_button") {
          this->pimpl_->enable_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        } else if (parameter.get_name() == "enable_turbo_button") {
          this->pimpl_->enable_turbo_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        } else if (parameter.get_name() == "axis_linear.x") {
          this->pimpl_->axis_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        } else if (parameter.get_name() == "axis_linear.y") {
          this->pimpl_->axis_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        } else if (parameter.get_name() == "axis_linear.z") {
          this->pimpl_->axis_linear_map["z"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        } else if (parameter.get_name() == "axis_angular.yaw") {
          this->pimpl_->axis_angular_map["yaw"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        } else if (parameter.get_name() == "axis_angular.pitch") {
          this->pimpl_->axis_angular_map["pitch"] =
            parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        } else if (parameter.get_name() == "axis_angular.roll") {
          this->pimpl_->axis_angular_map["roll"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        } else if (parameter.get_name() == "scale_linear_turbo.x") {
          this->pimpl_->scale_linear_map["turbo"]["x"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        } else if (parameter.get_name() == "scale_linear_turbo.y") {
          this->pimpl_->scale_linear_map["turbo"]["y"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        } else if (parameter.get_name() == "scale_linear_turbo.z") {
          this->pimpl_->scale_linear_map["turbo"]["z"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        } else if (parameter.get_name() == "scale_linear.x") {
          this->pimpl_->scale_linear_map["normal"]["x"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        } else if (parameter.get_name() == "scale_linear.y") {
          this->pimpl_->scale_linear_map["normal"]["y"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        } else if (parameter.get_name() == "scale_linear.z") {
          this->pimpl_->scale_linear_map["normal"]["z"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        } else if (parameter.get_name() == "scale_angular_turbo.yaw") {
          this->pimpl_->scale_angular_map["turbo"]["yaw"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        } else if (parameter.get_name() == "scale_angular_turbo.pitch") {
          this->pimpl_->scale_angular_map["turbo"]["pitch"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        } else if (parameter.get_name() == "scale_angular_turbo.roll") {
          this->pimpl_->scale_angular_map["turbo"]["roll"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        } else if (parameter.get_name() == "scale_angular.yaw") {
          this->pimpl_->scale_angular_map["normal"]["yaw"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        } else if (parameter.get_name() == "scale_angular.pitch") {
          this->pimpl_->scale_angular_map["normal"]["pitch"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        } else if (parameter.get_name() == "scale_angular.roll") {
          this->pimpl_->scale_angular_map["normal"]["roll"] =
            parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
      }
      return result;
    };

  callback_handle = this->add_on_set_parameters_callback(param_callback);
}

TeleopTwistJoy::~TeleopTwistJoy()
{
  delete pimpl_;
}

double getVal(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t> & axis_map,
  const std::map<std::string, double> & scale_map, const std::string & fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
    axis_map.at(fieldname) == -1L ||
    scale_map.find(fieldname) == scale_map.end() ||
    static_cast<int>(joy_msg->axes.size()) <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg,
  const std::string & which_map)
{
  if (publish_stamped_twist) {
    auto cmd_vel_stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    cmd_vel_stamped_msg->header.stamp = clock->now();
    cmd_vel_stamped_msg->header.frame_id = frame_id;
    fillCmdVelMsg(joy_msg, which_map, &cmd_vel_stamped_msg->twist);
    cmd_vel_stamped_pub->publish(std::move(cmd_vel_stamped_msg));
  } else {
    auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
    fillCmdVelMsg(joy_msg, which_map, cmd_vel_msg.get());
    cmd_vel_pub->publish(std::move(cmd_vel_msg));
  }
  sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::fillCmdVelMsg(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg,
  const std::string & which_map,
  geometry_msgs::msg::Twist * cmd_vel_msg)
{
  double lin_x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
  double ang_z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");

  cmd_vel_msg->linear.x = lin_x;
  cmd_vel_msg->linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
  cmd_vel_msg->linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
  cmd_vel_msg->angular.z = (lin_x < 0.0 && inverted_reverse) ? -ang_z : ang_z;
  cmd_vel_msg->angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
  cmd_vel_msg->angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  if (enable_turbo_button >= 0 &&
    static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button &&
    joy_msg->buttons[enable_turbo_button])
  {
    sendCmdVelMsg(joy_msg, "turbo");
  } else if (!require_enable_button ||  // NOLINT
    (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
    joy_msg->buttons[enable_button]))
  {
    sendCmdVelMsg(joy_msg, "normal");
  } else {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg) {
      // Initializes with zeros by default.
      if (publish_stamped_twist) {
        auto cmd_vel_stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        cmd_vel_stamped_msg->header.stamp = clock->now();
        cmd_vel_stamped_msg->header.frame_id = frame_id;
        cmd_vel_stamped_pub->publish(std::move(cmd_vel_stamped_msg));
      } else {
        auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_vel_pub->publish(std::move(cmd_vel_msg));
      }
      sent_disable_msg = true;
    }
  }
}

}  // namespace teleop_twist_joy

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_twist_joy::TeleopTwistJoy)
