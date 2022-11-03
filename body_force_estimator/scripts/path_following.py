#!/usr/bin/env python

# Command the robot to walk along a straight line parallel to the x-axis with
# zero yaw

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from quad_msgs.msg import RobotState
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Initialization
last_state_msg_ = RobotState() # Robot state message

def callback(data):
    global last_state_msg_
    last_state_msg_ = data

def path_following():
    global last_state_msg_

    pub = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/robot_1/state/ground_truth', RobotState, callback)
    rospy.init_node('path_following')

    # Parameters
    rate = 100
    ros_rate = rospy.Rate(rate) # Rate
    speed = 0.4 # Forward velocity
    y_pt = 0.7 # Lateral shift
    y_gain = 2.0 # Lateral P gain in m/s per m (or Hz)
    yaw_gain = 1.0 # Yaw P gain in rad/s per rad (or Hz)
    speed_i = 0.1 # Integration rate

    speed_integral = 0.0 # Initialize integrator

    while last_state_msg_.header.seq == 0:
        print('path_following: no state message')
        ros_rate.sleep()

    while not rospy.is_shutdown():
        cmd = Twist() # Initialize twist command
        q = last_state_msg_.body.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])

        speed_error = last_state_msg_.body.twist.linear.x - speed
        if speed_error < 0:
            speed_integral = speed_integral - speed_i/rate*speed_error
        else:
            speed_integral = speed_integral - speed_i/rate*speed_error*10

        cmd.linear.x = speed + speed_integral
        cmd.linear.y = -y_gain*(last_state_msg_.body.pose.position.y - y_pt)
        cmd.angular.z = -yaw_gain*euler[2]

        pub.publish(cmd)
        print(cmd)
        ros_rate.sleep()

if __name__ == '__main__':
    try:
        path_following()
    except rospy.ROSInterruptException:
        pass
