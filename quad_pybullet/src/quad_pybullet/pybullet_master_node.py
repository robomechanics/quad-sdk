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

    def __init__(self,node_name,step_rate,robot_file,world_urdf = None,sub_name = None,state_topic_name = None,grf_topic_name = None,clock_topic = None,\
        load_sdf = False):

        self.node_name = node_name
        self.step_rate = step_rate
        if state_topic_name == None:
            self.state_topic_name = 'pybullet_state_pub'
        else:
            self.state_topic_name = state_topic_name


        if grf_topic_name == None:
            self.grf_topic_name = 'pybullet_grf_pub'
        else:
            self.grf_topic_name = grf_topic_name

        if sub_name == None:
            self.sub_name = 'pybullet_sub'
        else:
            self.sub_name = sub_name

        self.robot_file = []
        self.add_robot(robot_file)
        if world_urdf == None:
            self.world = "plane.urdf"
            
        else:
            self.world = world_urdf

        self.use_sdf = load_sdf

        self.robot_id = []
        self.sensor_node = None
        self.driver_node = None
        self.clock_topic = clock_topic
        self.clock_pub = None
        self.clock_sub = None
        # self.step_rate = step_rate
        self.ros_sleep_rate = None
        rospy.set_param('use_sim_time',False)
        # rospy.set_param('use_sim_time',True)

    def sim_routine(self):
        while not rospy.is_shutdown():
        # while True:
            pb.stepSimulation()
            # self.ros_sleep_rate.sleep()
            # time.sleep(1/self.step_rate)
        # rospy.spin()

    def sim_routine_step(self,clock_msg):
        # while True:
        pb.stepSimulation()
        # time.sleep(1/self.step_rate)

    def sim_routine_init(self):
        # while True:
        self.clock_sub = rospy.Subscriber(self.clock_topic,Clock,self.sim_routine_step,queue_size=1)
        # self.ros_sleep_rate.sleep()
            # time.sleep(1/self.step_rate)
        # rospy.spin()

    def actuate_robot(self,Leg_cmd_msgs):
        # Assuming cmds as a dict, with motor names as keys, and contain target pos and vel info
        Legcmds = Leg_cmd_msgs
        self.driver.drive_all_legs(Legcmds)
        return

    def add_robot(self,robot_file):
        self.robot_file.append(robot_file)

    def robot_stand(self):



        return


    def run(self):
        rospy.init_node(self.node_name)
        self.control_mode = rospy.Publisher(self.clock_topic,Clock,queue_size=5)

        print('\n')
        # print(rospy.get_rostime(),'\n')
        driver_node_name = 'testdriver'
        # driver_node_name = 'testdriver'
        # cmd_topic_name = 'testdrive'
        cmd_topic_name = self.sub_name
        sensor_node_name = 'testsenser'
        # state_topic_name = 'testsense'
        state_topic_name = self.state_topic_name

        physicsClient = pb.connect(pb.GUI)#or pb.DIRECT for non-graphical version
        pb.setTimeStep(0.001,physicsClientId=physicsClient)

        pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        planeId = pb.loadURDF(self.world)
        pb.setGravity(0,0,-9.81)
        
        # planeId = pb.loadSDF(self.world)
        startPos = [0,0,0.4]
        startOrientation = pb.getQuaternionFromEuler([0.0,0,0],physicsClientId = physicsClient)
        # If single robot:
        if self.use_sdf:
            self.robot_id = pb.loadSDF(self.robot_file[0])  
        else:
            self.robot_id = pb.loadURDF(self.robot_file[0],startPos, startOrientation)
        
        self.sensor_node = pybullet_estimation_node(sensor_node_name,self.robot_id,physicsClient,self.step_rate,\
        state_topic_name= self.state_topic_name,grf_topic_name=self.grf_topic_name)
        self.driver_node = pybullet_actuation_node(driver_node_name,self.robot_id,physicsClient,cmd_topic_name)
        self.driver_node.robot_stand()
        # self.driver_node.robot_stand()
        simthread = threading.Thread(target=self.sim_routine_init)
        sensor_thread = threading.Thread(target=self.sensor_node.run)
        driver_thread = threading.Thread(target=self.driver_node.run)
        simthread.start()
        sensor_thread.start()
        driver_thread.start()
        
        rospy.spin()

