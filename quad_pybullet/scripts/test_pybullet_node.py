#! /usr/bin/env python

print('hello!')
print('yeahyeah')

import time
import numpy as np
import rospy
# from rospkg import RosPack

import pybullet as pb
import time
import pybullet_data

# ---------------------------------------------------------------------------- Quick and dirty test -----------------------------------
from quad_msgs.msg import RobotState
from quad_pybullet.estimate import Robot_sensors

def main():
    rospy.init_node('pybullet_sensor')
    
    testPub = rospy.Publisher('test_pybullet',RobotState)
    physicsClient = pb.connect(pb.GUI)#or pb.DIRECT for non-graphical version
    pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    pb.setGravity(0,0,-10)
    planeId = pb.loadURDF("plane.urdf")
    startPos = [1.3,0.8,0.4]
    startOrientation = pb.getQuaternionFromEuler([0,0,1.1])
    # boxId = pb.loadURDF("/home/haoluo/catkin_ws/src/quad-sdk/quad_simulator/spinebot_description/quad_spine_sdf/testSpinebot.urdf",startPos, startOrientation)
    boxId = pb.loadURDF("/home/haoluo/catkin_ws/src/quad-sdk/quad_simulator/spirit_description/urdf/spirit.urdf",startPos, startOrientation)  


    p_gain = 0.12
    leg_phase = [0,2,2,0]
    test_sensor = Robot_sensors(boxId)
    all_legs = test_sensor.leg_joint_ids

    Stand_angs = [0.0,0.7,1.5]
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        pb.stepSimulation()
        # pb.setJointMotorControl2(boxId, jointIndex=spine,controlMode=pb.POSITION_CONTROL,targetPosition=0.0,positionGain=p_gain) 
        for k,j in zip(all_legs,leg_phase):
            pb.setJointMotorControl2(boxId, jointIndex=k[0],controlMode=pb.POSITION_CONTROL,targetPosition=Stand_angs[0],positionGain=p_gain)
            pb.setJointMotorControl2(boxId, jointIndex=k[1],controlMode=pb.POSITION_CONTROL,targetPosition=Stand_angs[1],positionGain=p_gain)
            pb.setJointMotorControl2(boxId, jointIndex=k[2],controlMode=pb.POSITION_CONTROL,targetPosition=Stand_angs[2],positionGain=p_gain)
        new_msg = test_sensor.write_RobotState_msg()
        testPub.publish(new_msg)
        rate.sleep()


    
    rospy.spin()

if __name__ == '__main__':
    main()