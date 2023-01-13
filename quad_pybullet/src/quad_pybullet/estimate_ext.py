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

# -----------------------------------------------------------------------------

# Some utility functions using pybullet

def pos_quat_to_SE3(pos,quat):
    SO3 = (np.array(p.getMatrixFromQuaternion(quat))).reshape([3,3]) 
    P = np.array(pos).reshape([3,1])
    SE3 = np.r_[np.c_[SO3,P],np.array([[0,0,0,1]])] # Join cols and rows together
    return SE3

def SE3_transform_pos(pos_vec,pos,quat):
    homo_pos_vec = np.append(np.array(pos_vec),1).reshape([4,1]) # turn vector into homogeneous coordinate form
    SO3 = (np.array(pb.getMatrixFromQuaternion(quat))).reshape([3,3])
    P = np.array(pos).reshape([3,1])
    SE3 = np.r_[np.c_[SO3,P],np.array([[0,0,0,1]])] # construct SE3 matrix
    transformed_vec = SE3@homo_pos_vec
    return transformed_vec[:3] # Return 3x1, same dimension as 3x1 input pos_vec


#--------------------------------------------------------------------------------
# Only need to publish to /robot_1/state/ground_truth [quad_msgs/RobotState]?


class Robot_sensors_ext:

    def __init__(self,robot,joint_topic = None,ground_truth_topics = None,contact_topics= None):


        # self.jointPub = rospy.Publisher(joint_topic,JointState)
        # self.ground_truth_Pub = rospy.Publisher(ground_truth_topics[0],RobotState) 
        # self.ground_truth_body_Pub = rospy.Publisher(ground_truth_topics[1],RobotState) 
        # self.contactPubs = [None for j in range(len(contact_topics))]
        # for i in range(len(contact_topics)):
        #     self.contactPubs[i] = rospy.Publisher(contact_topics[i],ContactsState)

        self.robot = robot # Get bodyID from higher level node

        nJoints = pb.getNumJoints(self.robot)
        jointNameToId ={}
        for i in range (nJoints):
            jointInfo = pb.getJointInfo(self.robot, i)
            jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0] 
        # spine=jointNameToId["spine"]
        abdfl=jointNameToId["8"]
        hipfl=jointNameToId["0"]
        kneefl=jointNameToId["1"]
        abdfr=jointNameToId["9"]
        hipfr=jointNameToId["2"]
        kneefr=jointNameToId["3"]
        abdbl=jointNameToId["10"]
        hipbl=jointNameToId["4"]
        kneebl=jointNameToId["5"]
        abdbr=jointNameToId["11"]
        hipbr=jointNameToId["6"]
        kneebr=jointNameToId["7"]

        legFR = [abdfr,hipfr,kneefr]
        legFL = [abdfl,hipfl,kneefl]
        legBR = [abdbr,hipbr,kneebr]
        legBL = [abdbl,hipbl,kneebl]


        jtoes = [jointNameToId['jtoe%d'%i] for i in range(4)]
        print(jtoes)
        self.toeIdx = jtoes
        self.leg_joint_ids = [legFL,legBL,legFR,legBR]
        self.basic_joint_ids = legFL+legBL+legFR+legBR
        self.basic_joint_names = \
            ["8","0","1","9","2","3",\
            "10","4","5","11","6","7"]

        self.all_joint_names = list(jointNameToId.keys())
        self._nameToId = jointNameToId

    def get_basic_joints(self):
        positions = []
        vels = []
        efforts = [0.0*len(self.basic_joint_ids)]
        for i in self.basic_joint_ids:
            pos_i,vel_i,rf_i,eff_i =pb.getJointState(self.robot,i)
            positions.append(pos_i)
            vels.append(vel_i)
        return [self.basic_joint_names,positions,vels,efforts]


    def get_single_toe(self,idx):
        toe = self.toeIdx[idx]
        toe_state =pb.getLinkState(self.robot,toe, computeLinkVelocity = 1)
        location = toe_state[0] # Only need world frame pos
        vel = toe_state[6] # world frame linear vel
        contacting = self.get_single_contact_state(idx) # bool
        acc = (0.0,0.0,0.0) # hardcode 0

        return [location,vel,acc,contacting]

    def get_single_contact_state(self,idx):
        toe = self.toeIdx[idx]
        contact =pb.getContactPoints(bodyB = self.robot,linkIndexB = toe)
        if len(contact) >0:
            contact_state = True
        else:
            contact_state = False
        return contact_state


    def get_all_toes(self):
        toe_states = []
        for toe in self.toeIdx:
            toestate = self.get_single_toe(toe)
            toe_states.append(toestate)
        return toe_states

    def get_body_state(self):
        body_state = pb.getLinkState(self.robot,0, computeLinkVelocity = 1)
        location = body_state[0] # Only need world frame pos
        orientation = body_state[1]
        linear_vel = body_state[6] # world frame linear vel
        angular_vel = body_state[7] # world frame ang vel, cartesian, not twist!
        return [location,orientation,linear_vel,angular_vel]
        # return [location,orientation,linear_vel,angular_vel,body_state]



# ------------------------- More advanced features, might use in the future? --------------------

    def get_single_contact_pt(self,idx):

        #  Preferably use get_all_contacts for all toes instead of looping this function.
        body_pos_ori = pb.getBasePositionAndOrientation(self.robot)
        body_quat = body_pos_ori[1] # from calling getBasePositionAndOrientation externally
        body_pos = body_pos_ori[0]
        toe = self.toeIdx[idx]
        contact = pb.getContactPoints(bodyB = self.robot,linkIndexB = toe)
        if len(contact) >0:
            location_world = np.array(contact[0][5]) # Assume first body id is ground plane
            location_body = np.linalg.inv(pos_quat_to_SE3(body_pos,body_quat))@(np.append(location_world,1)).reshape(4,1)
            location = location_body[:3]
            grf = self.get_contact_grf(contact[0])
        else:
            location = np.zeros(3)
            grf = np.zeros(3)
        return [location,grf]

    def get_all_contacts_pts(self):
        # Return contact point location info in world frame
        ground_contact = pb.getContactPoints(bodyA = -1)
        locations = np.zeros([len(self.toeIdx),3]) # initialize location, grf arrays
        # grfs = np.zeros([len(self.toeIdx),3])
        if len(ground_contact) >0:
            for i in ground_contact:
                location_world = np.array(i[5]) # Assume first body id is ground plane
                # grf_i = self.get_contact_grf(i)
                locations += np.array([location_world*int(i[4]== m) for m in self.toeIdx])
                # grfs += np.array([grf_i*int(i[4]== m) for m in self.toeIdx])
            return locations
        else:
            return locations

    def get_all_contacts_pts_body(self):
        # Return contact points relative location to body in body frame
        
        body_pos_ori = pb.getBasePositionAndOrientation(self.robot)
        body_quat = body_pos_ori[1] # from calling getBasePositionAndOrientation externally
        body_pos = body_pos_ori[0]
        SE3_wb = (pos_quat_to_SE3(body_pos,body_quat))
        ground_contact = pb.getContactPoints(bodyA = -1)
        locations = np.zeros([len(self.toeIdx),3]) # initialize location, grf arrays
        grfs = np.zeros([len(self.toeIdx),3])
        if len(ground_contact) >0:
            for i in ground_contact:
                location_world = np.array(i[5]) # Assume first body id is ground plane
                grf_i = self.get_contact_grf(i)
                locations += np.array([location_world*int(i[4]== m) for m in self.toeIdx])
                grfs += np.array([grf_i*int(i[4]== m) for m in self.toeIdx])

            locations_body = np.linalg.inv(SE3_wb)@(np.r_[locations.T,np.ones([1,4])])
            locations_body = locations_body[:3,:]
            return [locations_body.T,grfs]
        else:
            return [locations,grfs]

    def get_contact_grf(self,contact):
        # Calculate contact forces in xyz, 
        # from "contact" object returned by getContactPoints 
        grf = np.zeros([1,3])
        f_norm = contact[9]
        e0_fnorm = np.array(contact[7]) # unit vector for normal force
        f_f1 = contact[10]
        e0_f1 = np.array(contact[11]) # unit vector for fricition direction
        f_f2 = contact[12]
        e0_f2 = np.array(contact[13]) # unit vector for fricition direction
        grf = f_norm*e0_fnorm+f_f1*e0_f1+f_f2*e0_f2
        return grf


    def world_vec_to_body(self,world_vec):
        # Rotate world frame vectors to body frame
        world_vec = np.array(world_vec)
        body_orient = pb.getBasePositionAndOrientation(self.robot)[1]
        SO3_flat = pb.getMatrixFromQuaternion(body_orient)
        SO3_wb = np.array([np.array(SO3_flat[0:3]),np.array(SO3_flat[3:6]),np.array(SO3_flat[6:9])])
        body_vec = SO3_wb@(world_vec.reshape(3,1))
        return body_vec

    def body_vec_to_world(self,body_vec):
        # Rotate world frame vectors to body frame
        body_vec = np.array(body_vec)
        body_orient = pb.getBasePositionAndOrientation(self.robot)[1]
        SO3_flat = pb.getMatrixFromQuaternion(body_orient)
        SO3_wb = np.array([np.array(SO3_flat[0:3]),np.array(SO3_flat[3:6]),np.array(SO3_flat[6:9])])
        world_vec = SO3_wb.T@(body_vec.reshape(3,1))
        return world_vec
