import time
import numpy as np
import rospy
# from rospkg import RosPack

import pybullet as pb
# import rostime
import pybullet_data

from quad_msgs.msg import RobotState
from quad_pybullet.estimate import Robot_sensors
from quad_pybullet.actuate import Robot_pydriver
from rosgraph_msgs.msg import Clock


class pybullet_estimation_node:

#  This is a very simple implementation that runs only one quadruped robot, with potentially extra joints.

    def __init__(self,node_name,robot_id, physicsClient_id, step_rate = 100,pub_name = None):
        self.node_name = node_name
        if pub_name == None:
            self.pub_name = 'pybullet_pub'
        else:
            self.pub_name = pub_name


        self.robot_id = robot_id
        self.pcid = physicsClient_id
        self.sensor = None # placeholder
        self.clock_topic = '/double_clock'
        self.clock_sub = None
        self.step_rate = step_rate

    def run(self):
        # rospy.init_node(self.node_name)
        print(self.robot_id,'robot\n')
        print(self.pcid,'client\n')

        # single robot:
        self.sensor = Robot_sensors(self.robot_id,physicsClientId=self.pcid)
        quad_pb_states = rospy.Publisher(self.pub_name,RobotState,queue_size=5)
        rate = rospy.Rate(self.step_rate)
        while not rospy.is_shutdown():
            new_msg = self.sensor.write_RobotState_msg()
            quad_pb_states.publish(new_msg)
            rate.sleep()
