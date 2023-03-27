import time
import numpy as np
import rospy
# from rospkg import RosPack

import pybullet as pb
# import rostime
import pybullet_data

from quad_msgs.msg import LegCommandArray
from quad_pybullet.estimate import Robot_sensors
from quad_pybullet.actuate import Robot_pydriver
from rosgraph_msgs.msg import Clock


class pybullet_actuation_node:

#  This is a very simple implementation that runs only one quadruped robot, with potentially extra joints.

    def __init__(self,node_name,robot_id,pybullet_client_id,sub_name = None,step_rate = 500,clock_freq = 1000):
        self.node_name = node_name

        if sub_name == None:
            self.sub_name = 'pybullet_sub'
        else:
            self.sub_name = sub_name

        self.robot_id = robot_id
        self.pcid = pybullet_client_id

        self.driver = Robot_pydriver(self.robot_id,self.pcid,torque_control=True)
        self.subscriber_callback = self.drive_robot # setup callback function when running
        # self.step_time = 1.0/step_rate

        # self.counter = 0
        # self.counter_reset = clock_freq/step_rate # Assume 1000Hz clock

    # def update_counter(self):
    #     self.counter +=1

    def drive_robot(self,robot_msg):
        #print(robot_msg)
        # self.driver.drive_all_old(robot_msg)
        # self.driver.drive_all_pos(robot_msg)
        # self.driver.drive_all_torque(robot_msg)
        
        # if self.counter >= self.counter_reset:
            self.driver.drive_all_torque(robot_msg)
        #     self.counter = 0
        # else:
        #     pass
        # time.sleep(self.step_time)

    def robot_stand(self):
        self.driver.stand()

    def run(self):
        # rospy.init_node(self.node_name)
        # self.ros_rate = rospy.Rate(self.step_rate)
        rospy.Subscriber(self.sub_name,LegCommandArray,self.subscriber_callback,queue_size=5)
        # rospy.spin()
