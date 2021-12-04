/**
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

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <rosgraph_msgs/Clock.h>
#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


// ROS node handle
std::unique_ptr<ros::NodeHandle> rosNode;

// Declare callbacks
void modelCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);

// Declare ROS subscribers and publishers
ros::Subscriber modelSub;
ros::Publisher odomPub;

// Declare ROS parameters
std::string robot_name;
std::string child_frame;
std::string odom_frame;
double child_frame_x;
double child_frame_y;
double child_frame_z;
double child_frame_roll;
double child_frame_pitch;
double child_frame_yaw;

// Declare global variables
bool flag_first = true;
tf::Vector3 position_first;
tf::Vector3 odom_to_global_pos;
tf::Quaternion odom_to_global_rot;