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

#include <map>
#include <string>
#include "odom_publisher.h"


// Model callback
void modelCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
	int des_index = 0;

	// Determine the index of the robot from all the models included
	if (robot_name == "minitaur") {
		for (int j=0 ; j<msg->name.size() ; j++) {
			if (msg->name[j] == "minitaur_constrained::base_chassis_link_dummy") {
				des_index = j;
			}
		}
	} else if (robot_name == "vision60") {
		for (int j=0 ; j<msg->name.size() ; j++) {
			if (msg->name[j] == "vision60::body") {
				des_index = j;
			}
		}
	}

    // Find rotation of robot in the global frame
	tf::Quaternion robot_to_global = tf::Quaternion(msg->pose[des_index].orientation.x,
						  				  	        msg->pose[des_index].orientation.y,
						  				  	        msg->pose[des_index].orientation.z,
						  				  	        msg->pose[des_index].orientation.w);

    // Find rotation of child in the robot frame
    tf::Matrix3x3 m;
    m.setRPY(child_frame_roll, child_frame_pitch, child_frame_yaw);
    tf::Quaternion child_to_robot;
	m.getRotation(child_to_robot);

    // Store the first pose of the odom frame in the global Gazebo frame - Runs only once
    if (flag_first) {
        odom_to_global_pos = tf::Vector3(msg->pose[des_index].position.x + child_frame_x, msg->pose[des_index].position.y + child_frame_y, msg->pose[des_index].position.z + child_frame_z);
        odom_to_global_rot = robot_to_global*child_to_robot;
        flag_first = false;
    }

    // Find orientation of child frame in odom frame
    tf::Quaternion child_to_odom = odom_to_global_rot.inverse()*robot_to_global*child_to_robot;

    // Initialize Odometry message
    nav_msgs::Odometry msg_odometry;

    // Populate odometry message header
    msg_odometry.header.stamp = ros::Time::now();
    msg_odometry.header.frame_id = odom_frame;

    // Find position of child in the odom frame
    tf::Vector3 vector_position(msg->pose[des_index].position.x, msg->pose[des_index].position.y, msg->pose[des_index].position.z);
    tf::Vector3 rotated_position = tf::quatRotate(odom_to_global_rot.inverse(), vector_position);
    tf::Vector3 rotated_p = tf::quatRotate(odom_to_global_rot.inverse(), odom_to_global_pos);
    tf::Vector3 child_to_odom_pos = rotated_position - rotated_p;

    // Populate odometry message
    msg_odometry.child_frame_id = child_frame;
    msg_odometry.pose.pose.position.x = child_to_odom_pos.getX();
    msg_odometry.pose.pose.position.y = child_to_odom_pos.getY();
    msg_odometry.pose.pose.position.z = child_to_odom_pos.getZ();
    msg_odometry.pose.pose.orientation.x = child_to_odom.getX();
    msg_odometry.pose.pose.orientation.y = child_to_odom.getY();
    msg_odometry.pose.pose.orientation.z = child_to_odom.getZ();
    msg_odometry.pose.pose.orientation.w = child_to_odom.getW();

	// Get angular twist
	tf::Vector3 vector_angular(msg->twist[des_index].angular.x, msg->twist[des_index].angular.y, msg->twist[des_index].angular.z);
	tf::Vector3 rotated_vector_angular = tf::quatRotate((robot_to_global*child_to_robot).inverse(), vector_angular);
	msg_odometry.twist.twist.angular.x = rotated_vector_angular.getX();
	msg_odometry.twist.twist.angular.y = rotated_vector_angular.getY();
	msg_odometry.twist.twist.angular.z = rotated_vector_angular.getZ();

	// Get linear twist
	tf::Vector3 vector_linear(msg->twist[des_index].linear.x, msg->twist[des_index].linear.y, msg->twist[des_index].linear.z);
	tf::Vector3 rotated_vector_linear = tf::quatRotate((robot_to_global*child_to_robot).inverse(), vector_linear);
	msg_odometry.twist.twist.linear.x = rotated_vector_linear.getX();
	msg_odometry.twist.twist.linear.y = rotated_vector_linear.getY();
	msg_odometry.twist.twist.linear.z = rotated_vector_linear.getZ();

    // Publish odometry
    odomPub.publish(msg_odometry);

    // Publish tf transform
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(child_to_odom_pos.getX(), child_to_odom_pos.getY(), child_to_odom_pos.getZ()));
    transform.setRotation(child_to_odom);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame, child_frame));

	return;
}

int main(int argc, char **argv) {
    // Initialize ROS if necessary
  	if (!ros::isInitialized()) {
    	ros::init(argc, argv, "odom_publisher", ros::init_options::NoSigintHandler);
  	}

	// Initialize node
  	rosNode.reset(new ros::NodeHandle("odom_publisher"));
    
    // Get parameters
    ros::param::get("~robot_name", robot_name);
    ros::param::get("~child_frame", child_frame);
    ros::param::get("~odom_frame", odom_frame);
    ros::param::get("~child_frame_x", child_frame_x);
    ros::param::get("~child_frame_y", child_frame_y);
    ros::param::get("~child_frame_z", child_frame_z);
    ros::param::get("~child_frame_roll", child_frame_roll);
    ros::param::get("~child_frame_pitch", child_frame_pitch);
    ros::param::get("~child_frame_yaw", child_frame_yaw);

    // Initialize subscriber
    modelSub = rosNode->subscribe("/gazebo/link_states", 1, &modelCallback);

    // Initialize publisher
    odomPub = rosNode->advertise<nav_msgs::Odometry>("/robot_odom", 1);

	while (ros::ok()) {
		// Set ROS rate
		ros::Rate r(60);

		// Spin once
		ros::spinOnce();

		r.sleep();
	}

	return 0;
}
