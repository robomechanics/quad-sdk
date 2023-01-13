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
        self.clock_topic = '/clock'
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

        # rospy.spin()

# def main():
#     rospy.init_node('pybullet_simnode')
    
#     quad_pb_states = rospy.Publisher('test_pybullet',RobotState)
#     physicsClient = pb.connect(pb.GUI)#or pb.DIRECT for non-graphical version
#     pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
#     pb.setGravity(0,0,-10)
#     planeId = pb.loadURDF("plane.urdf")
#     startPos = [1.3,0.8,0.4]
#     startOrientation = pb.getQuaternionFromEuler([0,0,1.1])
#     # boxId = pb.loadURDF("/home/haoluo/catkin_ws/src/quad-sdk/quad_simulator/spinebot_description/quad_spine_sdf/testSpinebot.urdf",startPos, startOrientation)
#     boxId = pb.loadURDF("/home/haoluo/catkin_ws/src/quad-sdk/quad_simulator/spirit_description/urdf/spirit.urdf",startPos, startOrientation)  


#     p_gain = 0.12
#     leg_phase = [0,2,2,0]
#     test_sensor = Robot_sensors(boxId)
#     all_legs = test_sensor.leg_joint_ids

#     Stand_angs = [0.0,0.7,1.5]
#     rate = rospy.Rate(100)
#     while not rospy.is_shutdown():
#         pb.stepSimulation()
#         # pb.setJointMotorControl2(boxId, jointIndex=spine,controlMode=pb.POSITION_CONTROL,targetPosition=0.0,positionGain=p_gain) 
#         for k,j in zip(all_legs,leg_phase):
#             pb.setJointMotorControl2(boxId, jointIndex=k[0],controlMode=pb.POSITION_CONTROL,targetPosition=Stand_angs[0],positionGain=p_gain)
#             pb.setJointMotorControl2(boxId, jointIndex=k[1],controlMode=pb.POSITION_CONTROL,targetPosition=Stand_angs[1],positionGain=p_gain)
#             pb.setJointMotorControl2(boxId, jointIndex=k[2],controlMode=pb.POSITION_CONTROL,targetPosition=Stand_angs[2],positionGain=p_gain)
#         new_msg = test_sensor.write_RobotState_msg()
#         quad_pb_states.publish(new_msg)
#         rate.sleep()


    
#     rospy.spin()