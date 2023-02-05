/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/
// modified for ylo2 robot.

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>


void callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0,0.0,0.0));
  tf::Quaternion q;
  q.setW(imu_msg->orientation.w);
  q.setX(imu_msg->orientation.x);
  q.setY(imu_msg->orientation.y);
  q.setZ(imu_msg->orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "trunk", "world"));
}


int main(int argc, char** argv)
{
  std::string ns = "ylo2_imu_node";

  ros::init(argc, argv, ns);

  ros::NodeHandle n(ns); // load the relative namespace

  ros::Subscriber imu_subscriber = n.subscribe("/imu/data", 20, callback);

  ros::spin();

  return 0;
}
