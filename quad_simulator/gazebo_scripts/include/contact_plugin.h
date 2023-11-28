/*
MIT License (modified)

Copyright (c) 2018 The Trustees of the University of Pennsylvania
Authors:
Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this **file** (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <geometry_msgs/Vector3.h>
#include <quad_msgs/ContactMode.h>
<<<<<<< HEAD:quad_simulator/gazebo_scripts/include/contact_plugin.h
#include <ros/ros.h>
=======
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*:spirit_simulator/gazebo_scripts/include/contact_plugin.h

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <string>

namespace gazebo {
/// \brief An example plugin for a contact sensor.
class ContactPlugin : public SensorPlugin {
  /// \brief Constructor.
 public:
  ContactPlugin();

  // \brief ROS related members
 private:
  std::unique_ptr<ros::NodeHandle> rosNode;

 private:
  ros::Publisher contact_publisher;

  /// \brief Destructor.
 public:
  virtual ~ContactPlugin();

  /// \brief Load the sensor plugin.
  /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
 public:
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

  /// \brief Callback that receives the contact sensor's update signal.
 private:
  virtual void OnUpdate();

  /// \brief Pointer to the contact sensor
 private:
  sensors::ContactSensorPtr parentSensor;

  /// \brief Connection that maintains a link between the contact sensor's
  /// updated signal and the OnUpdate callback.
 private:
  event::ConnectionPtr updateConnection;
};
}  // namespace gazebo
#endif
