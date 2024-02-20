#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from quad_msgs.msg import RobotState

last_state_msg_ = RobotState()

def callback(data):
    last_state_msg_ = data

def leg_flail():
    pub = rospy.Publisher('/robot_1/control/single_joint_command', Vector3, queue_size=10)
    #rospy.Subscriber('/robot_1/state/ground_truth', RobotState, callback)
    rospy.init_node('leg_flail')
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        cmd = Vector3()
        #print(last_state_msg_)
        for i in range(0,4):
            for j in range(0,3):
                cmd.x = i
                cmd.y = j
                cmd.z = 10

                pub.publish(cmd)
                rate.sleep()

if __name__ == '__main__':
    try:
        leg_flail()
    except rospy.ROSInterruptException:
        pass
