import time
import numpy as np
import rospy
# from rospkg import RosPack

import pybullet as pb
# import rostime
import pybullet_data

from quad_msgs.msg import RobotState,GRFArray
from quad_pybullet.estimate import Robot_sensors
from quad_pybullet.actuate import Robot_pydriver
from rosgraph_msgs.msg import Clock


class pybullet_estimation_node:

#  This is a very simple implementation that runs only one quadruped robot, with potentially extra joints.

    def __init__(self,node_name,robot_id, physicsClient_id, step_rate = 500,state_topic_name = None,grf_topic_name = None,clock_topic = None):
        self.node_name = node_name
        if state_topic_name == None:
            self.state_topic_name = 'pybullet_state_pub'
        else:
            self.state_topic_name = state_topic_name

        if grf_topic_name == None:
            self.grf_topic_name = 'pybullet_grf_pub'
        else:
            self.grf_topic_name = grf_topic_name

        if clock_topic == None:
            self.clock_name = '/clock'

        self.robot_id = robot_id
        self.pcid = physicsClient_id
        self.sensor = None # placeholder
        # self.clock_sub = None
        self.rate = None
        self.step_rate = step_rate
        self.quad_pb_states = None
        self.quad_pb_grfs = None
        self.internal_counter = 0
        self.internal_counter_reset = 4

    def publish_state(self,clocktime):
            # print(clocktime.tsecs)
            
            if self.internal_counter == self.internal_counter_reset:
                new_state_msg = self.sensor.write_RobotState_msg()
                new_grf_msg = self.sensor.write_contact_msg()
                self.quad_pb_states.publish(new_state_msg)
                self.quad_pb_grfs.publish(new_grf_msg)
                self.internal_counter =0
                self.internal_counter +=1
                return
            else:
                self.internal_counter +=1
                return
            

    def run(self):
        # rospy.init_node(self.node_name)
        print(self.robot_id,'robot\n')
        print(self.pcid,'client\n')

        # single robot:
        self.sensor = Robot_sensors(self.robot_id,physicsClientId=self.pcid)
        self.quad_pb_states = rospy.Publisher(self.state_topic_name,RobotState,queue_size=5)
        self.quad_pb_grfs = rospy.Publisher(self.grf_topic_name,GRFArray,queue_size=5)

        self.rate = rospy.Rate(self.step_rate)
        
        # while not rospy.is_shutdown():
        #     new_state_msg = self.sensor.write_RobotState_msg()
        #     new_grf_msg = self.sensor.write_contact_msg()
        #     quad_pb_states.publish(new_state_msg)
        #     quad_pb_grfs.publish(new_grf_msg)
        #     # rate.sleep()

        rospy.Subscriber(self.clock_name,Clock,self.publish_state,queue_size=5)
