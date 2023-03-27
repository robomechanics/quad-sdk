import pybullet as pb
import time
import pybullet_data
import numpy as np
import rospy


from quad_msgs.msg import RobotState, MultiFootState, FootState, LegCommandArray
from sensor_msgs.msg import JointState

# ------------------------------------------------------------------------------------

# Written specifically for 12 Dof quadrupedal robots, with potentially extra joints
# Could replace contact_state_publisher? Publish directly into state/grfs?



# Topics to use
#  * /robot_1/state/ground_truth [quad_msgs/RobotState]
#  * /robot_1/state/ground_truth_body_frame [quad_msgs/RobotState]

#--------------------------------------------------------------------------------
# Only need to publish to /robot_1/state/ground_truth [quad_msgs/RobotState]?


class Robot_pydriver:

    def __init__(self,robot,physicsClientId,drive_msg_class = None,torque_control = True):


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

        self.abds = [legFL[0],legBL[0],legFR[0],legBR[0]]
        self.hips = [legFL[1],legBL[1],legFR[1],legBR[1]]
        self.knees = [legFL[2],legBL[2],legFR[2],legBR[2]]

        self.all_joint_names = list(jointNameToId.keys())
        self._nameToId = jointNameToId

        self.torque_ub = 40.0
        self.torque_lb = -40.0

        if torque_control:
            self.enable_torque_control()

    def enable_torque_control(self):
        for i in self.basic_joint_ids:
            pb.setJointMotorControl2(bodyUniqueId=self.robot, jointIndex=i, controlMode=pb.VELOCITY_CONTROL,force = 0)

    def stand(self):
        for i in self.abds:
            pb.setJointMotorControl2(targetPosition = 0, bodyUniqueId=self.robot, jointIndex=i, controlMode=pb.POSITION_CONTROL)
        for i in self.hips:
            pb.setJointMotorControl2(targetPosition = 0.8, bodyUniqueId=self.robot, jointIndex=i, controlMode=pb.POSITION_CONTROL)
        for i in self.knees:
            pb.setJointMotorControl2(targetPosition = 1.6, bodyUniqueId=self.robot, jointIndex=i, controlMode=pb.POSITION_CONTROL)


    def unpack_quadrobot_cmds_list(self,ros_robotcmds):
        # Turn rosmsg into commands
        # Assume quad_msgs.LegCommandArray for now
        
        # By default, the output is a nested list:
        #  [[all motor pos], [all motor vels], [all motor kp], [all motor kd], [all motor torque_ff]]
        robot_cmd_list = [[],[],[],[],[]]
        blank_msg = [[0 for i in range(len(self.basic_joint_ids))] for j in range(5)]\
        # Decompose cmds as lists of pos,vel,kp,kd,torque_ff
        if not len(ros_robotcmds.leg_commands) == 4:
            robot_cmd_list = [[],[],[],[],[]]
            return blank_msg
        else:
            robot_cmd_list = [[],[],[],[],[]]
            [FL_cmd,BL_cmd,FR_cmd,BR_cmd] = ros_robotcmds.leg_commands
            for leg_i_cmd,leg_i_ids in zip([FL_cmd,BL_cmd,FR_cmd,BR_cmd],self.leg_joint_ids):
                abd_cmd,hip_cmd,knee_cmd = leg_i_cmd.motor_commands
                # abd_motor,hip_motor,knee_motor = leg_i_ids
                for motor_j_cmd,motor_j_id in zip([abd_cmd,hip_cmd,knee_cmd],leg_i_ids):

                    robot_cmd_list[0].append(motor_j_cmd.pos_setpoint) # list of pos, ordered as in self.basic_joint__names
                    robot_cmd_list[1].append(motor_j_cmd.vel_setpoint) # list of vel, ordered as in self.basic_joint__names
                    # robot_cmd_list[2].append(self.forced_kp) # list of motor kp
                    # robot_cmd_list[3].append(self.forced_kd) # list of motor kd
                    # robot_cmd_list[4].append(0.0)
                    robot_cmd_list[2].append(motor_j_cmd.kp)
                    robot_cmd_list[3].append(motor_j_cmd.kd)
                    # robot_cmd_list[4].append(motor_j_cmd.torque_ff)
                    robot_cmd_list[4].append(motor_j_cmd.effort)
                    # robot_cmd_list[4].append(0.0)
            return robot_cmd_list


    def drive_all_pos(self,ros_robot_cmd):
        # robot_cmd as nested list:
        # [[all motor pos],[all motor vels],[all motor kp],[all motor kd],[all motor torque_ff]]]
        #  change leg sequencing in class init() function
        # robot_cmd = self.unpack_quadrobot_cmds(ros_robot_cmd)
        # [pos_cmds,vel_cmds,kp_cmds,kd_cmds] = self.unpack_quadrobot_cmds_list(ros_robot_cmd)

        # This function use pybullet built-in position control
        [pos_cmds,vel_cmds,kp_cmds,kd_cmds,torque_ff_cmds] = self.unpack_quadrobot_cmds_list(ros_robot_cmd)
        robot_id = self.robot
        clientid = self.pcid
        joint_ids = self.basic_joint_ids
        mode = pb.POSITION_CONTROL
        pb.setJointMotorControlArray(robot_id,joint_ids,mode,pos_cmds,vel_cmds,positionGains = kp_cmds,\
            velocityGains = kd_cmds, physicsClientId = clientid) # Don't use torque yet


    def joints_PD(self,joint_ids,all_target_pos,all_target_vel,kps,kds,efforts):
        joint_kps = np.array(kps)
        joint_kds = np.array(kds)
        curr_pos = [] # Current positions
        curr_vel = []
        curr_rfs = []
        # efforts = []
        # efforts = [0.0*len(self.basic_joint_ids)]
        all_joint_states = pb.getJointStates(self.robot,joint_ids,physicsClientId = self.pcid)
        for i in all_joint_states:
            curr_pos.append(i[0])    
            curr_vel.append(i[1])   
        pos_err = np.array(all_target_pos)-np.array(curr_pos)
        vel_err = np.array(all_target_vel)-np.array(curr_vel)
        torque_out = 1.2*pos_err*joint_kps+1.0*vel_err*joint_kds
        # torque_out = np.array(efforts)*1.0
        for i in torque_out:
            if i>self.torque_ub:
                i = self.torque_ub
            elif i < self.torque_lb:
                i = self.torque_lb
            else:
                continue
     
        # torque_out = (pos_err*joint_kps*0.5-curr_vel*joint_kds*0.5)

        # outstring_pos_err = "pos err: "
        # outstring_vel_err = "vel err: "
        # outstring_kp = "kps: "
        # outstring_kd = "kds: "
        outstring_torque = "torque: "
        for i in torque_out:
        #     outstring_pos_err +="%f,"%(pos_err[i])             
        #     outstring_vel_err +="%f,"%(vel_err[i])
        #     outstring_kp +="%f,"%(joint_kps[i])
            # outstring_kd +="%f,"%i
            outstring_torque +="%f,"%i
        rospy.loginfo(outstring_torque)
        return pos_err.tolist(),vel_err.tolist(),torque_out.tolist()

    def drive_all_torque(self,ros_robot_cmd):
        # robot_cmd as nested list:
        # [[all motor pos],[all motor vels],[all motor kp],[all motor kd],[all motor torque_ff]]]
        #  change leg sequencing in class init() function
        # robot_cmd = self.unpack_quadrobot_cmds(ros_robot_cmd)
        # [pos_cmds,vel_cmds,kp_cmds,kd_cmds] = self.unpack_quadrobot_cmds_list(ros_robot_cmd)
        [pos_cmds,vel_cmds,kp_cmds,kd_cmds,torque_ff_cmds] = self.unpack_quadrobot_cmds_list(ros_robot_cmd)

        robot_id = self.robot
        clientid = self.pcid
        # mode = pb.POSITION_CONTROL
        mode = pb.TORQUE_CONTROL
        
        pos_err,vel_err,PD_torque_cmds = self.joints_PD(self.basic_joint_ids,pos_cmds,vel_cmds,kp_cmds,kd_cmds,torque_ff_cmds)
        # pos_err,vel_err,PD_torque_cmds = self.joints_PD(self.basic_joint_ids,pos_cmds,vel_cmds,kp_cmds,kd_cmds,torque_ff_cmds)


        # torque_cmd = torque_ff_cmds
        torque_cmd = (np.array(PD_torque_cmds)).tolist()

        # PD_torque_cmds = [20.0 for i in self.basic_joint_ids]
        pb.setJointMotorControlArray(robot_id,self.basic_joint_ids,mode,forces = torque_cmd, physicsClientId = clientid)
        # pb.setJointMotorControlArray(robot_id,dummy_id,mode,forces = dummy_cmd, physicsClientId = clientid)     # Use torque cmd



            
# ############################################ Some extra utility functions or old functions #######################


    def single_joint_err(self,joint_id,target_pos,target_vel):
        curr_pos,curr_vel,curr_rf,curr_effort = \
            pb.getJointState(self.robot,joint_id,physicsClientId = self.pcid)
        pos_err = curr_pos-target_pos
        vel_err = curr_vel-target_vel
        return pos_err,vel_err,curr_vel

    def joints_err(self,joint_ids,all_target_pos,all_target_vel):
        curr_pos = [] # Current positions
        curr_vel = []
        curr_rfs = []
        # efforts = []
        efforts = [0.0*len(self.basic_joint_ids)]
        all_joint_states = pb.getJointStates(self.robot,joint_ids,physicsClientId = self.pcid)
        for i in all_joint_states:
            curr_pos.append(i[0])    
            curr_vel.append(i[1])   
        pos_err = np.array(curr_pos)-np.array(all_target_pos)
        vel_err = np.array(curr_vel)-np.array(all_target_vel)
        return pos_err.tolist(),vel_err.tolist()



    def drive_all_old(self,ros_robot_cmd):
        # robot_cmd as nested list:
        # [[all motor pos],[all motor vels],[all motor kp],[all motor kd],[all motor torque_ff]]]
        #  change leg sequencing in class init() function
        # robot_cmd = self.unpack_quadrobot_cmds(ros_robot_cmd)
        # [pos_cmds,vel_cmds,kp_cmds,kd_cmds] = self.unpack_quadrobot_cmds_list(ros_robot_cmd)
        [pos_cmds,vel_cmds,kp_cmds,kd_cmds,torque_cmds] = self.unpack_quadrobot_cmds_list(ros_robot_cmd)
        if len(pos_cmds)>0:
            robot_id = self.robot
            clientid = self.pcid
            mode = pb.POSITION_CONTROL
            abds = [0,3,6,9]
            hips = [1,4,7,10]
            knees = [2,5,8,11]
            # all_seq = [abds,hips,knees]
            all_seq = [knees,hips,abds]
            # for j,k in zip(all_seq,[self.abds,self.hips,self.knees]):
            for j,k in zip(all_seq,[self.knees,self.hips,self.abds]):
                pb.setJointMotorControlArray(robot_id,k,mode,[pos_cmds[i] for i in j],[vel_cmds[i] for i in j],\
                positionGains = [kp_cmds[i]*0.5 for i in j],\
                    velocityGains = [kd_cmds[i]*0.5 for i in j], physicsClientId = clientid,forces=[10.0 for i in j])
                # Don't use torque yet    

                # pb.setJointMotorControlArray(robot_id,k,mode,[pos_cmds[i] for i in j],[vel_cmds[i] for i in j],\
                # positionGains = [kp_cmds[i] for i in j],\
                #     velocityGains = [kd_cmds[i] for i in j], 
                #     forces = [torque_cmds[i] for i in j], physicsClientId = clientid)    # Use torque cmd
            else:
                pass

    
    def drive_one_motor(self,motor_idx,target_pos,target_vel = None,kp = 0.0,kd = 0.0,torque_ff = 1e6):
        # torque_ff is the max actuator effort, set to large number by default
        if target_vel == None:
            pb.setJointMotorControl2(self.robot, jointIndex=motor_idx,controlMode=pb.POSITION_CONTROL,targetPosition=target_pos,\
            positionGain=kp, velocityGain=kd, \
                physicsClientId = self.pcid) # client id optional
        else:
            pb.setJointMotorControl2(self.robot, jointIndex=motor_idx,controlMode=pb.POSITION_CONTROL,targetPosition=target_pos,\
            targetVelocity=target_vel, positionGain=kp, velocityGain=kd,\
                # force = torque_ff,\
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
        pb.setJointMotorControlArray(robot_id,leg_motor_ids,mode,pos,vel,positionGains = kp,\
            velocityGains = kd,physicsClientId = clientid)