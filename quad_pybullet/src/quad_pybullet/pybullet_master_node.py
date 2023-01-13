import time
import numpy as np
import rospy
import threading
# from rospkg import RosPack

import pybullet as pb
# import rostime
import pybullet_data

from quad_msgs.msg import RobotState,LegCommandArray

from quad_pybullet.estimation_node import pybullet_estimation_node
from quad_pybullet.actuation_node import pybullet_actuation_node
from rosgraph_msgs.msg import Clock


class pybullet_sim_node:

#  This is a very simple implementation that runs only one quadruped robot, with potentially extra joints.

    def __init__(self,node_name,step_rate,robot_urdf,world_urdf = None,sub_name = None,pub_name = None):
        self.node_name = node_name
        self.step_rate = step_rate
        if pub_name == None:
            self.pub_name = 'pybullet_pub'
        else:
            self.pub_name = pub_name
        if sub_name == None:
            self.sub_name = 'pybullet_sub'
        else:
            self.sub_name = sub_name

        self.robot_urdf = []
        self.add_robot(robot_urdf)
        if world_urdf == None:
            self.world = "plane.urdf"
            
        else:
            self.world = world_urdf

        self.robot_id = []
        self.sensor_node = None
        self.driver_node = None
        self.clock_topic = 'clock'
        self.clock_pub = None
        self.step_rate = step_rate
        self.ros_sleep_rate = None
        rospy.set_param('use_sim_time',False)

    def sim_routine(self):
        while not rospy.is_shutdown():
        # while True:
            pb.stepSimulation()
            now = rospy.get_rostime()
            self.clock_pub.publish(now)
            self.ros_sleep_rate.sleep()
        # rospy.spin()


    def actuate_robot(self,Leg_cmd_msgs):
        # Assuming cmds as a dict, with motor names as keys, and contain target pos and vel info
        Legcmds = Leg_cmd_msgs
        self.driver.drive_all_legs(Legcmds)
        return

    def add_robot(self,robot_urdf):
        self.robot_urdf.append(robot_urdf)

    def run(self):
        rospy.init_node(self.node_name)
        self.clock_pub = rospy.Publisher('clock',Clock,queue_size=5)
        print('\n')
        print(rospy.get_rostime(),'\n')
        driver_node_name = 'testdriver'
        # driver_node_name = 'testdriver'
        # cmd_topic_name = 'testdrive'
        cmd_topic_name = self.sub_name
        sensor_node_name = 'testsenser'
        # state_topic_name = 'testsense'
        state_topic_name = self.pub_name

        physicsClient = pb.connect(pb.GUI)#or pb.DIRECT for non-graphical version
        
        pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        pb.setGravity(0,0,-10)
        # planeId = pb.loadURDF(self.world)
        planeId = pb.loadSDF(self.world)
        startPos = [1.3,0.8,0.4]
        startOrientation = pb.getQuaternionFromEuler([0,0,1.1])
        # If single robot:
        self.robot_id = pb.loadURDF(self.robot_urdf[0],startPos, startOrientation)  

    
        pb.setRealTimeSimulation(1,physicsClient)
        # all_legs = test_sensor.leg_joint_ids
        
        self.ros_sleep_rate = rospy.Rate(self.step_rate)

        Stand_angs = [0.0,0.7,1.5]
        

        
        self.sensor_node = pybullet_estimation_node(sensor_node_name,self.robot_id,physicsClient,self.step_rate,state_topic_name)
        self.driver_node = pybullet_actuation_node(driver_node_name,self.robot_id,physicsClient,cmd_topic_name)
        
        
        clockthread = threading.Thread(target=self.sim_routine)
        sensor_thread = threading.Thread(target=self.sensor_node.run)
        driver_thread = threading.Thread(target=self.driver_node.run)
        # self.drivers.run()
        clockthread.start()
        sensor_thread.start()
        driver_thread.start()
        # clockthread.join()




        rospy.spin()

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