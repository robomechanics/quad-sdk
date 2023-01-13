import pybullet as pb
import time
import pybullet_data
import numpy as np
import rospy


from quad_msgs.msg import RobotState, MultiFootState, FootState, BodyState
from sensor_msgs.msg import JointState


# ------------------------------------------------------------------------------------

# Written specifically for 12 Dof quadrupedal robots, with potentially extra joints
# Could replace contact_state_publisher? Publish directly into state/grfs?



# Topics to use
#  * /robot_1/state/ground_truth [quad_msgs/RobotState]
#  * /robot_1/state/ground_truth_body_frame [quad_msgs/RobotState]


# -----------------------------------------------------------------------------

# Some utility functions using pybullet

# def pos_quat_to_SE3(pos,quat):
#     SO3 = (np.array(pb.getMatrixFromQuaternion(quat))).reshape([3,3]) 
#     P = np.array(pos).reshape([3,1])
#     SE3 = np.r_[np.c_[SO3,P],np.array([[0,0,0,1]])] # Join cols and rows together
#     return SE3

# def SE3_transform_pos(pos_vec,pos,quat):
#     homo_pos_vec = np.append(np.array(pos_vec),1).reshape([4,1]) # turn vector into homogeneous coordinate form
#     SO3 = (np.array(pb.getMatrixFromQuaternion(quat))).reshape([3,3])
#     P = np.array(pos).reshape([3,1])
#     SE3 = np.r_[np.c_[SO3,P],np.array([[0,0,0,1]])] # construct SE3 matrix
#     transformed_vec = SE3@homo_pos_vec
#     return transformed_vec[:3] # Return 3x1, same dimension as 3x1 input pos_vec


#--------------------------------------------------------------------------------
# Only need to publish to /robot_1/state/ground_truth [quad_msgs/RobotState]?


class Robot_pydriver:

    def __init__(self,robot,physicsClientId,drive_msg_class = None):


        self.robot = robot # Get bodyID from higher level node
        self.pcid = physicsClientId
        # self.drive_msg_class = drive_msg_class

        nJoints =pb.getNumJoints(self.robot,physicsClientId = self.pcid)
        jointNameToId ={}
        for i in range (nJoints):
            jointInfo =pb.getJointInfo(self.robot, i,physicsClientId = self.pcid)
            jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0] 
        # spine=jointNameToId["spine"]

        # Leg0 = FL
        abdfl=jointNameToId["8"]
        hipfl=jointNameToId["0"]
        kneefl=jointNameToId["1"]

        # Leg1 = BL
        abdbl=jointNameToId["9"]
        hipbl=jointNameToId["2"]
        kneebl=jointNameToId["3"]

        # Leg2 = FR
        abdfr=jointNameToId["10"]
        hipfr=jointNameToId["4"]
        kneefr=jointNameToId["5"]

        # Leg3 = BR
        abdbr=jointNameToId["11"]
        hipbr=jointNameToId["6"]
        kneebr=jointNameToId["7"]

        legFR = [abdfr,hipfr,kneefr]
        legFL = [abdfl,hipfl,kneefl]
        legBR = [abdbr,hipbr,kneebr]
        legBL = [abdbl,hipbl,kneebl]

        jtoes = [jointNameToId['jtoe%d'%i] for i in range(4)]
        self.toeIdx = jtoes
        self.leg_joint_ids = [legFL,legBL,legFR,legBR]
        self.basic_joint_ids = legFL+legBL+legFR+legBR
        self.basic_joint_names = \
            ["8","0","1","9","2","3",\
            "10","4","5","11","6","7"]

        self.all_joint_names = list(jointNameToId.keys())
        self._nameToId = jointNameToId

    def unpack_quadrobot_cmds(self,ros_robotcmds):
        # Turn rosmsg into commands
        # Assume quad_msgs.LegCommandArray for now
        # Assume leg and joint sequence as [FL,BL,FR,BR], and [abd,hip,knee] for one leg
        blank_msg = [[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]] for i in range(4)]
        robot_cmds = []
        if not len(ros_robotcmds.leg_commands) == 4:
            return blank_msg
        else:
            [FL_cmd,BL_cmd,FR_cmd,BR_cmd] = ros_robotcmds.leg_commands
            for leg_i_cmd in [FL_cmd,BL_cmd,FR_cmd,BR_cmd]:
                abd_cmd,hip_cmd,knee_cmd = leg_i_cmd.motor_commands
                legpos = []
                legvels = []
                legkps = []
                legkds = []
                torqueff = []
                for motor_j_cmd in [abd_cmd,hip_cmd,knee_cmd]:
                    legpos.append(motor_j_cmd.pos_setpoint)
                    legvels.append(motor_j_cmd.vel_setpoint)
                    legkps.append(motor_j_cmd.kp)
                    legkds.append(motor_j_cmd.kd)
                    # torqueff.append(motor_j_cmd.torque_ff)
                    torqueff.append(10)
                robot_cmds.append([legpos,legvels,legkps,legkds,torqueff])
            # print(robot_cmds)
            return robot_cmds


    def drive_one_motor(self,motor_idx,target_pos,target_vel = None,kp = 0.0,kd = 0.0,torque_ff = 1e6):
        # torque_ff is the max actuator effort, set to large number by default
        if target_vel == None:
            pb.setJointMotorControl2(self.robot, jointIndex=motor_idx,controlMode=pb.POSITION_CONTROL,targetPosition=target_pos,\
            positionGain=kp, velocityGain=kd, \
                physicsClientId = self.pcid) # client id optional
        else:
            pb.setJointMotorControl2(self.robot, jointIndex=motor_idx,controlMode=pb.POSITION_CONTROL,targetPosition=target_pos,\
            targetVelocity=target_vel,force = torque_ff, positionGain=kp, velocityGain=kd,\
                physicsClientId = self.pcid) # client id optional

    def drive_one_leg(self,leg_idx,leg_cmd):
        # leg_cmd as nested list: 
        # [[motor1pos,motor2pos...],[motor1vel,motor2vel ...],[kp1,kp2 ...],[kd1,kd2 ...],[torqueff1,torqueff2...]]
        # sequence is (abduction, hip and knee) by default
        robot_id = self.robot
        clientid = self.pcid
        mode = pb.POSITION_CONTROL # pos control by default
        leg_motor_ids = self.leg_joint_ids[leg_idx]
        pos,vel,kp,kd,toruqeff = leg_cmd
        print(pos,vel,kp,kd,toruqeff)
        pb.setJointMotorControlArray(robot_id,leg_motor_ids,mode,pos,vel,toruqeff,kp,kd,clientid)

    # def drive_one_leg2(self,leg_idx,leg_cmd):
    #     # leg_cmd as flattened list: [motor1pos,motor1vel,kp1,kd1,torqueff1,motor2pos,motor2vel, etc...]]
    #     # sequence is (abduction, hip and knee) by default
    #     # This implemetation iterate thru motos, slow. Replacing!
    #     leg_motor_ids = self.leg_joint_ids[leg_idx]
    #     for i,j in zip(leg_motor_ids,leg_cmd): # iterate over abd, hip and knee joint motors
    #         target_pos, target_vel,kp,kd,torqueff = j
    #         self.drive_one_motor(target_pos, target_vel,kp,kd,torqueff)

    def drive_all_legs(self,ros_robot_cmd):
        # robot_cmd as nested list:
        # [[FLleg_cmds],[BLleg_cmds]] ,[[BLmotor1],[BLmotor2]], etc...]
        #  change leg sequencing in class init() function
        robot_cmd = self.unpack_quadrobot_cmds(ros_robot_cmd)
        print(len(robot_cmd))
        for leg_idx,leg_cmd in zip((0,1,2,3),robot_cmd):
            self.drive_one_leg(leg_idx,leg_cmd)

