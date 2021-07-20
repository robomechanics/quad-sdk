#!/usr/bin/python3
# 
# RBDL - Rigid Body Dynamics Library
# Copyright (c) 2011-2015 Martin Felis <martin@fysx.org>
# 
# Licensed under the zlib license. See LICENSE for more details.

import unittest

import math
import numpy as np
from numpy.testing import *
import rbdl
from typing import List

class JointTests (unittest.TestCase):
    def test_JointConstructorAxesSimple(self):

        axis = np.asarray([[1., 0., 0., 0., 0., 0.]])
        joint_rot_x = rbdl.Joint.fromJointAxes (axis)
        joint_rot_x_type = rbdl.Joint.fromJointType ("JointTypeRevoluteX")
        
        assert_equal (joint_rot_x.getJointAxis(0), axis[0])
        assert_equal (joint_rot_x_type.getJointAxis(0), axis[0])
        assert_equal (joint_rot_x.mDoFCount, 1)
        assert_equal (joint_rot_x_type.mDoFCount, 1)

    def test_JointConstructorAxes6DoF(self):

        axis = np.asarray([
            [1., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0.],
            [0., 0., 1., 0., 0., 0.],
            [0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 1., 0.],
            [0., 0., 0., 0., 0., 1.],
            ])
            
        joint = rbdl.Joint.fromJointAxes (axis)
        joint2 = rbdl.Joint.fromJointType ("JointTypeFloatingBase")

        
        for i in range (axis.shape[0]):
            assert_equal (joint.getJointAxis(i), axis[i])

class SampleModel3R (unittest.TestCase):
    """ Example planar triple pendulum 
   
    - All joints along the positive Z axis in rest position
    - All joints revolute y joints
    - all "links" are 1 unit length
    """
    def setUp(self):
      
        self.model = rbdl.Model()
        joint_rot_y = rbdl.Joint.fromJointType ("JointTypeRevoluteY")
        self.body = rbdl.Body.fromMassComInertia (1., np.array([0., 0.0, 0.5]), 
            np.eye(3) * 0.05)
        self.xtrans = rbdl.SpatialTransform()
        self.xtrans.r = np.array([0., 0., 1.])
        
        self.body_1 = self.model.AppendBody (rbdl.SpatialTransform(), 
            joint_rot_y, self.body)
        self.body_2 = self.model.AppendBody (self.xtrans, joint_rot_y, 
            self.body)
        self.body_3 = self.model.AppendBody (self.xtrans, joint_rot_y, 
            self.body)

        self.q = np.zeros (self.model.q_size)
        self.qdot = np.zeros (self.model.qdot_size)
        self.qddot = np.zeros (self.model.qdot_size)
        self.tau = np.zeros (self.model.qdot_size)

    def test_AccessToModelParameters (self):
        """
        Checks whether vital model parameters can be accessed that are 
        stored in the "Model" class.
        """    
        rbdl.UpdateKinematics (self.model, self.q, self.qdot, self.qddot)
        
        assert_equal (self.model.mBodies[2].mMass, self.body.mMass)
        assert_equal (self.model.mBodies[2].mCenterOfMass, 
                                        self.body.mCenterOfMass)
        assert_equal (self.model.mBodies[2].mInertia, 
                                        self.body.mInertia )
                                        
        assert_equal (self.model.X_T[1].E, rbdl.SpatialTransform().E )
        assert_equal (self.model.X_T[1].r, rbdl.SpatialTransform().r )
        assert_equal (self.model.X_T[2].E, self.xtrans.E )
        assert_equal (self.model.X_T[2].r, self.xtrans.r )
        
        assert_almost_equal (self.model.X_base[0].E, np.identity(3) )
        assert_almost_equal (self.model.X_base[3].r, 
                                        self.xtrans.r + self.xtrans.r )
        
        assert_equal (self.model.mJoints[2].mJointType, 
                                                "JointTypeRevoluteY")

    def test_CoordinateTransformBodyBase (self):
        """
        Checks whether CalcBodyToBaseCoordinates and CalcBaseToBodyCoordinates
        give the right results.
        """
        q = np.random.rand(self.model.q_size)
        point_local = np.array ([1., 2., 3.])
        point_base = rbdl.CalcBodyToBaseCoordinates (
                self.model,
                q,
                self.body_3,
                point_local)
        point_local_2 = rbdl.CalcBaseToBodyCoordinates (
                self.model,
                q,
                self.body_3,
                point_base)
      
        assert_almost_equal (point_local, point_local_2)

    def test_CalcPointVelocity (self):
        """
        Checks whether CalcBodyToBaseCoordinates and CalcBaseToBodyCoordinates
        give the right results.
        """
        q = np.zeros(self.model.q_size)
        qdot = np.zeros(self.model.q_size)
        qdot[0] = 1.
        point_local = np.array ([0., 0., 0.])
        point_vel = rbdl.CalcPointVelocity (
                self.model,
                q,
                qdot,
                self.body_3,
                point_local
                )

        assert_almost_equal (np.array([2., 0., 0.]), point_vel)

    def test_CalcCenterOfMass (self):
        """ Tests calculation of center of mass
        TODO: add checks for angular momentum
        """
        com = np.array ([-1., -1., -1.])
        com_vel = np.array([-2., -2., -2.])
        ang_mom = np.array([-3., -3., -3.])
        self.qdot[0] = 1.

        mass = rbdl.CalcCenterOfMass (
                self.model,
                self.q,
                self.qdot,
                com
                )
                
        self.assertEqual (3, mass)
        assert_almost_equal (np.array([0., 0., 1.5]), com)
        assert_almost_equal (np.array([0., 0., 1.5]), com)

        mass = rbdl.CalcCenterOfMass (
                self.model,
                self.q,
                self.qdot,
                com,
                None,
                com_vel
                )
        self.assertEqual (3, mass)
        assert_almost_equal (np.array([0., 0., 1.5]), com)
        assert_almost_equal (np.array([1.5, 0., 0.0]), com_vel)

        mass = rbdl.CalcCenterOfMass (
                self.model,
                self.q,
                self.qdot,
                com,
                None,
                com_vel,
                None,
                ang_mom,
                None
                )
        self.assertEqual (3, mass)
        assert_almost_equal (np.array([0., 0., 1.5]), com)

    def test_DynamicsConsistency (self):
        """ Checks whether forward and inverse dynamics are consistent """
        q = np.random.rand (self.model.q_size)
        qdot = np.random.rand (self.model.q_size)
        qddot = np.random.rand (self.model.q_size)

        tau = np.random.rand (self.model.q_size)

        rbdl.ForwardDynamics (
                self.model,
                q,
                qdot,
                tau,
                qddot
                )

        tau_id = np.zeros ((self.model.q_size))
        rbdl.InverseDynamics (
                self.model,
                q,
                qdot, 
                qddot,
                tau_id
                )

        assert_almost_equal (tau, tau_id)
        
        
    def test_Dynamics_fextConsistency (self):
        """ Checks whether forward and inverse dynamics are consistent """
        q = np.random.rand (self.model.q_size)
        qdot = np.random.rand (self.model.q_size)
        qddot = np.random.rand (self.model.q_size)
        qddot_fext = qddot

        tau = np.random.rand (self.model.q_size)
        
        forceA = np.zeros (6)
        forceB = np.zeros (6)
        forceC = np.zeros (6)
        forceD = np.zeros (6)
        
        fext = np.array([forceA, forceB, forceC, forceD])

        rbdl.ForwardDynamics (
                self.model,
                q,
                qdot,
                tau,
                qddot,
                )
        rbdl.ForwardDynamics (
                self.model,
                q,
                qdot,
                tau,
                qddot_fext,
                fext
                )

        tau_id = np.zeros ((self.model.q_size))
        tau_id_fext = tau_id
        
        rbdl.InverseDynamics (
                self.model,
                q,
                qdot, 
                qddot,
                tau_id,
                )
        rbdl.InverseDynamics (
                self.model,
                q,
                qdot, 
                qddot,
                tau_id_fext,
                fext
                )
        
        assert_almost_equal (qddot, qddot_fext)
        assert_almost_equal (tau_id, tau_id_fext)
        
    def test_DynamicsConsistency_with_fext (self):
        """ Checks whether forward and inverse dynamics are consistent """
        q = np.random.rand (self.model.q_size)
        qdot = np.random.rand (self.model.q_size)
        qddot = np.random.rand (self.model.q_size)

        tau = np.random.rand (self.model.q_size)
        
        forceA = np.zeros (6)
        forceB = np.random.rand (6)
        forceC = np.random.rand (6)
        forceD = np.random.rand (6)
        
        fext = np.array([forceA, forceB, forceC, forceD])

        rbdl.ForwardDynamics (
                self.model,
                q,
                qdot,
                tau,
                qddot,
                fext
                )

        tau_id = np.zeros ((self.model.q_size))
        rbdl.InverseDynamics (
                self.model,
                q,
                qdot, 
                qddot,
                tau_id,
                fext
                )
                

        assert_almost_equal (tau, tau_id)
        
    def test_NonlinearEffectsConsistency (self):
        """ Checks whether NonlinearEffects is consistent with InverseDynamics """
        q = np.random.rand (self.model.q_size)
        qdot = np.random.rand (self.model.q_size)

        nle_id = np.random.rand (self.model.q_size)

        rbdl.InverseDynamics(
                self.model,
                q,
                qdot,
                np.zeros (self.model.qdot_size),
                nle_id)

        nle = np.zeros ((self.model.q_size))
        rbdl.NonlinearEffects (
                self.model,
                q,
                qdot, 
                nle
                )

        assert_almost_equal (nle_id, nle)

    def test_CalcPointJacobian (self):
        """ Computes point Jacobian and checks whether G * qdot is consistent
        with CalcPointVelocity. """
        q = np.zeros (self.model.q_size)
        G = np.zeros ([3, self.model.q_size])
        point_coords = np.array ([0., 0., 1.])

        rbdl.CalcPointJacobian (
                self.model,
                q,
                self.body_3,
                point_coords,
                G
                )

        qdot = np.ones(self.model.qdot_size)
        point_vel = rbdl.CalcPointVelocity (
                self.model,
                q,
                qdot,
                self.body_3,
                point_coords
                )

        jac_point_vel = np.dot (G, qdot)
        assert_almost_equal (jac_point_vel, point_vel)

    def test_CalcPointJacobianNonSquare (self):
        """ Computes point Jacobian and checks whether G * qdot is consistent
        with CalcPointVelocity. """

        self.model = rbdl.Model()
        joint_trans_xyz = rbdl.Joint.fromJointType ("JointTypeTranslationXYZ")

        self.body_1 = self.model.AppendBody (rbdl.SpatialTransform(),
                joint_trans_xyz, self.body)

        self.body_4 = self.model.AppendBody (rbdl.SpatialTransform(),
                joint_trans_xyz, self.body)

        point_coords = np.array ([0., 0., 1.])
        q = np.zeros (self.model.q_size)
        G = np.zeros ([3, self.model.q_size])

        rbdl.CalcPointJacobian (
                self.model,
                q,
                self.body_4,
                point_coords,
                G
                )

        qdot = np.ones(self.model.qdot_size)
        jac_point_vel = np.dot (G, qdot)

        point_vel = rbdl.CalcPointVelocity (
                self.model,
                q,
                qdot,
                self.body_4,
                point_coords
                )
       
        assert_almost_equal (jac_point_vel, point_vel)
    
    def test_InverseKinematics (self):
        """ Checks whether inverse kinematics methods are consistent """
        
        q = np.random.rand (self.model.q_size)
        q_res = np.random.rand (self.model.q_size)
        qCS_res = np.random.rand (self.model.q_size)

        target_body_ids = np.array([self.body_3]) 
        body_points = np.array([np.zeros(3)])
        body_points[0][2] = 1.0
        target_positions = np.array([np.zeros(3)])
        target_positions[0][2] = 1.0
        target_positions[0][0] = 2.0
        ori_matrix = np.array([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]])
        
        CS = rbdl.InverseKinematicsConstraintSet()
        CS_size = CS.AddPointConstraint (target_body_ids[0],
                body_points[0],
                target_positions[0]
                )        
        CS.AddOrientationConstraint (self.body_1, ori_matrix)
                
        CS.dlambda = 0.9   
        CS.max_steps = 2000  
        CS.step_tol = 1.0e-8
        
               
        rbdl.InverseKinematics ( self.model, q, target_body_ids, 
                    body_points, target_positions, q_res, 
                    CS.step_tol, CS.dlambda, CS.max_steps 
                    )
        rbdl.InverseKinematicsCS ( self.model, q, CS, qCS_res )
        
        res_point = rbdl.CalcBodyToBaseCoordinates(self.model, 
                                    q_res, self.body_3, body_points[0])
        res_point2 = rbdl.CalcBodyToBaseCoordinates(self.model, 
                                    qCS_res, self.body_3, body_points[0])
        res_ori = rbdl.CalcBodyWorldOrientation (self.model, 
                                    qCS_res, self.body_1)
        
        assert_almost_equal (target_positions[0], res_point)
        assert_almost_equal (target_positions[0], res_point2)
        assert_almost_equal (ori_matrix, res_ori)


class FloatingBaseModel (unittest.TestCase):
    """ Model with a floating base
    """
    def setUp(self):
      
        self.model = rbdl.Model()
        joint_rot_y = rbdl.Joint.fromJointType ("JointTypeFloatingBase")
        self.body = rbdl.Body.fromMassComInertia (1., np.array([0., 0.0, 0.5]), 
            np.eye(3) *
                0.05)
        self.xtrans = rbdl.SpatialTransform()
        self.xtrans.r = np.array([0., 0., 0.])
        
        self.body_1 = self.model.AppendBody (rbdl.SpatialTransform(), 
            joint_rot_y, self.body)

        self.q = np.zeros (self.model.q_size)
        self.qdot = np.zeros (self.model.qdot_size)
        self.qddot = np.zeros (self.model.qdot_size)
        self.tau = np.zeros (self.model.qdot_size)

    def test_Dimensions (self):
        """
        Checks whether the dimensions of q and qdot are correct
        """

        q = np.random.rand(self.model.q_size)
        self.assertEqual (7, self.model.q_size)
        self.assertEqual (6, self.model.qdot_size)

    def test_SetQuaternion (self):
        mat = np.asarray ([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        rbdl_quat = rbdl.Quaternion.fromPythonMatrix (mat)
        ref_q = self.q.copy()

        self.model.SetQuaternion (2, rbdl_quat.toNumpy(), self.q)

        ref_q[3:6] = rbdl_quat[0:3]
        ref_q[-1] = rbdl_quat[3]

        assert_array_equal (ref_q, self.q)

    def test_GetQuaternion (self):
        mat = np.asarray ([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        rbdl_quat = rbdl.Quaternion.fromPythonMatrix (mat)

        self.assertEqual (4, len(rbdl_quat))
        self.q[5] = math.sqrt(2.) * 0.5
        self.q[6] = math.sqrt(2.) * 0.5

        ref_quat = [0., 0., math.sqrt(2.) * 0.5, math.sqrt(2.) * 0.5]
        quat = self.model.GetQuaternion (2, self.q)

        assert_array_equal (np.asarray(ref_quat), quat)
    


class FloatingBaseModel2 (unittest.TestCase):
    
    """ Model with a floating base
    """
    def setUp(self):
      
        axis = np.asarray([
            [0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 1., 0.],
            [0., 0., 0., 0., 0., 1.],
            [0., 0., 1., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0.],
            [1., 0., 0., 0., 0., 0.],
            ])
     
      
        self.model = rbdl.Model()
        self.model.gravity = np.array ([0., -9.81, 0.])
        joint_rot_y = rbdl.Joint.fromJointAxes (axis)
        self.body = rbdl.Body.fromMassComInertia (1., np.array([0., 0.0, 0.0]), 
            np.eye(3))
        self.xtrans = rbdl.SpatialTransform()
        self.xtrans.r = np.array([0., 0., 0.])
        
        self.body_1 = self.model.AppendBody (rbdl.SpatialTransform(), 
            joint_rot_y, self.body)

        self.q = np.zeros (self.model.q_size)
        self.qdot = np.zeros (self.model.qdot_size)
        self.qddot = np.zeros (self.model.qdot_size)
        self.tau = np.zeros (self.model.qdot_size)
    
    def test_UpdateKinematicsConsistency (self):
      
        contact_body_id = self.body_1
        contact_point = np.array( [0., -1., 0.]);
        
        self.q[0] = 0.1
        self.q[1] = 0.2
        self.q[2] = 0.3
        self.q[3] = 0.4
        self.q[4] = 0.5
        self.q[5] = 0.6
        
        rbdl.UpdateKinematics (self.model, self.q, self.qdot, self.qddot)
        point1 = rbdl.CalcBodyToBaseCoordinates(self.model, 
                                      self.q, contact_body_id, contact_point)
        rbdl.UpdateKinematicsCustom (self.model, self.q)
        point2 = rbdl.CalcBodyToBaseCoordinates(self.model, 
                                      self.q, contact_body_id, contact_point)
        
        self.qdot[0] = 1.1
        self.qdot[1] = 1.2
        self.qdot[2] = 1.3
        self.qdot[3] = -1.4
        self.qdot[4] = -1.5
        self.qdot[5] = -1.6
        
        rbdl.UpdateKinematics (self.model, self.q, self.qdot, self.qddot)
        point_velocity1 = rbdl.CalcPointVelocity (self.model, self.q, 
                                self.qdot, contact_body_id, contact_point);
        rbdl.UpdateKinematicsCustom (self.model, self.q, self.qdot)
        point_velocity2 = rbdl.CalcPointVelocity (self.model, self.q, 
                                self.qdot, contact_body_id, contact_point);
        
        
        self.qdot[0] = 10.1
        self.qdot[1] = 10.2
        self.qdot[2] = 10.3
        self.qdot[3] = -10.4
        self.qdot[4] = -10.5
        self.qdot[5] = -10.6
        
        rbdl.UpdateKinematics (self.model, self.q, self.qdot, self.qddot)
        point_acceleration1 = rbdl.CalcPointAcceleration (self.model, 
            self.q, self.qdot, self.qddot, contact_body_id, contact_point);
        rbdl.UpdateKinematicsCustom (self.model, self.q, self.qdot, self.qddot)
        point_acceleration2 = rbdl.CalcPointAcceleration (self.model, 
            self.q, self.qdot, self.qddot, contact_body_id, contact_point);
            
            
        assert_almost_equal (point1, point2)
        assert_almost_equal (point_velocity1, point_velocity2)
        assert_almost_equal (point_acceleration1, point_acceleration2)

        
    
    # ForwardDynamicsConstraintsDirect 
    def test_ForwardDynamicsConstraintsDirectSimple (self):
      
        self.q[1] = 1.
        self.qdot[0] = 1.
        self.qdot[3] = -1.

        contact_body_id = self.body_1
        contact_point = np.array( [0., -1., 0.]);

        constraint_set = rbdl.ConstraintSet()


        #Since each of these constraints have different user-defined-id values
        #the do not get grouped together.
        i0 = constraint_set.AddContactConstraint (contact_body_id, 
            contact_point, np.array ([1., 0., 0.]), "ground_x",3);
        i1 = constraint_set.AddContactConstraint (contact_body_id, 
            contact_point, np.array ([0., 1., 0.]), "ground_y",4);
        i2 = constraint_set.AddContactConstraint (contact_body_id, 
            contact_point, np.array ([0., 0., 1.]), "ground_z",5);

        constraint_set.Bind (self.model);

        rbdl.ForwardDynamicsConstraintsDirect (self.model, self.q, self.qdot, 
            self.tau, constraint_set, self.qddot);
        point_acceleration = rbdl.CalcPointAcceleration (self.model, self.q, 
            self.qdot, self.qddot, contact_body_id, contact_point);

        assert_almost_equal( np.array([0., 0., 0.]), point_acceleration)

        #Test the functions to access the group index        
        gId = constraint_set.getGroupIndexByName("ground_x")
        assert_equal(0,gId)
        gId = constraint_set.getGroupIndexByName("ground_y")
        assert_equal(1,gId)
        gId = constraint_set.getGroupIndexByName("ground_z")
        assert_equal(2,gId)

        gId = constraint_set.getGroupIndexById(3)
        assert_equal(0,gId)
        gId = constraint_set.getGroupIndexById(4)
        assert_equal(1,gId)
        gId = constraint_set.getGroupIndexById(5)
        assert_equal(2,gId)

        gId = constraint_set.getGroupIndexByAssignedId(i0)
        assert_equal(0,gId)
        gId = constraint_set.getGroupIndexByAssignedId(i1)
        assert_equal(1,gId)
        gId = constraint_set.getGroupIndexByAssignedId(i2)
        assert_equal(2,gId)

    def test_ForwardDynamicsConstraintsDirectMoving (self):
      
      
        self.q[0] = 0.1
        self.q[1] = 0.2
        self.q[2] = 0.3
        self.q[3] = 0.4
        self.q[4] = 0.5
        self.q[5] = 0.6
        
        self.qdot[0] = 1.1
        self.qdot[1] = 1.2
        self.qdot[2] = 1.3
        self.qdot[3] = -1.4
        self.qdot[4] = -1.5
        self.qdot[5] = -1.6

        contact_body_id = self.body_1
        contact_point = np.array( [0., -1., 0.])

        constraint_set = rbdl.ConstraintSet()
        constraint_set.AddContactConstraint (contact_body_id, contact_point, 
            np.array ([1., 0., 0.]), "ground_x")
        constraint_set.AddContactConstraint (contact_body_id, contact_point, 
            np.array ([0., 1., 0.]), "ground_y")
        constraint_set.AddContactConstraint (contact_body_id, contact_point, 
            np.array ([0., 0., 1.]), "ground_z")

        constraint_set.Bind (self.model)
        
        rbdl.ForwardDynamicsConstraintsDirect (self.model, self.q, self.qdot, 
            self.tau, constraint_set, self.qddot)

        point_acceleration = rbdl.CalcPointAcceleration (self.model, self.q, 
            self.qdot, self.qddot, contact_body_id, contact_point);
        
        assert_almost_equal( np.array([0., 0., 0.]), point_acceleration)
        
    
    def test_CompositeRigidBodyAlgorithm_NonlinearEffects (self):
      
      
        self.q = np.random.rand (self.model.q_size)
        self.qdot = np.random.rand (self.model.q_size)
        self.qddot = np.random.rand (self.model.q_size)
        
        tau_nonlin = np.zeros(self.model.q_size)
        H = np.zeros( (self.model.q_size, self.model.q_size) )
        
        rbdl.InverseDynamics (self.model, self.q, self.qdot, self.qddot, 
            self.tau)
        
        rbdl.CompositeRigidBodyAlgorithm (self.model, self.q, H)        
        rbdl.NonlinearEffects (self.model, self.q, self.qdot, tau_nonlin)
        tau_comp = H.dot(self.qddot) + tau_nonlin

        
        assert_almost_equal( tau_comp, self.tau)


class ConstraintSetTests (unittest.TestCase):
    def setUp(self):

        # Make a triple-perpendicular-pendulum in absolute coordinates using
        # 5 loop constraints to implement the first 2 pin joints 
        #
        #    The perpendicular pendulum pictured with joint angles of 0,0.
        #    The first joint rotates about the x axis, while the second
        #    joint rotates about the local y axis of link 1
        #
        #             y
        #             |
        #             |___ x
        #         z / |
        #           | |
        #         / | | link1
        #           | |
        #       /   | |
        #   axis1:z0| |__________
        #          (_____________) link 2
        #           | |
        #            |
        #   
        #            |
        #   
        #            | axis2:y1        
        #   
        self.model = rbdl.Model()
        self.model.gravity = np.array([0.,-9.81,0.])

        l1=1.
        l2=1.
        m1=1.
        m2=1.
        self.l1=l1
        self.l2=l2
        self.m1=m1
        self.m2=m2

        c1 = np.array([0.,-l1*0.5,0.])
        J1 = np.array([[m1*l1*l1/3.,           0.,          0.],
                       [         0., m1*l1*l1/30.,          0.],
                       [         0.,           0., m1*l1*l1/3.]]) 

        c2 = np.array([l2*0.5,0.,0.])        
        J2 = np.array([ [m2*l2*l2/30.,          0., 0.],
                        [          0., m2*l2*l2/3., 0.],
                        [          0.,          0., m2*l2*l2/3.]]) 

        Xp1   = rbdl.SpatialTransform()
        Xp1.r = np.array([0.,0.,0.])
        Xp1.E = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
        
        Xs1   = rbdl.SpatialTransform()
        Xs1.r = np.array([0.,0.,0.])
        Xs1.E = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

        Xp2   = rbdl.SpatialTransform()
        Xp2.r = np.array([0.,-l1,0.])
        Xp2.E = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

        Xs2   = rbdl.SpatialTransform()
        Xs2.r = np.array([0.,0.,0.])
        Xs2.E = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

        axis = np.asarray([
            [0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 1., 0.],
            [0., 0., 0., 0., 0., 1.],
            [0., 0., 1., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0.],
            [1., 0., 0., 0., 0., 0.],
            ])

        joint6Dof = rbdl.Joint.fromJointAxes (axis)

        self.link1 = rbdl.Body.fromMassComInertia(m1,c1,J1)
        self.link2 = rbdl.Body.fromMassComInertia(m2,c2,J2)

        self.iLink1= self.model.AppendBody(rbdl.SpatialTransform(),joint6Dof, 
                                            self.link1)
        self.iLink2= self.model.AppendBody(rbdl.SpatialTransform(),joint6Dof, 
                                            self.link2)

        #point_coords = np.array ([0., 0., 1.])
        self.q   = np.zeros (self.model.q_size)
        self.qd  = np.zeros (self.model.qdot_size)
        self.qdd = np.zeros (self.model.qdot_size)
        self.tau = np.zeros (self.model.qdot_size)        

        assert(self.iLink1==6)
        assert(self.iLink2==12)

        self.cs = rbdl.ConstraintSet()

        self.cg0AssignedId = self.cs.AddLoopConstraint(0,self.iLink1,Xp1,Xs1, 
            rbdl.SpatialVector(0,[0,0,0,1,0,0]),False,0.1,"LoopGroundLink1",7)
        self.cs.AddLoopConstraint(0,self.iLink1,Xp1,Xs1,
            rbdl.SpatialVector(0,[0,0,0,0,1,0]),False,0.1,"LoopGroundLink1",7)
        self.cs.AddLoopConstraint(0,self.iLink1,Xp1,Xs1,
            rbdl.SpatialVector(0,[0,0,0,0,0,1]),False,0.1,"LoopGroundLink1",7)
        self.cs.AddLoopConstraint(0,self.iLink1,Xp1,Xs1,
            rbdl.SpatialVector(0,[1,0,0,0,0,0]),False,0.1,"LoopGroundLink1",7)
        self.cs.AddLoopConstraint(0,self.iLink1,Xp1,Xs1,
            rbdl.SpatialVector(0,[0,1,0,0,0,0]),False,0.1,"LoopGroundLink1",7)

        self.cg1AssignedId = self.cs.AddLoopConstraint( self.iLink1,self.iLink2, Xp2, Xs2, 
            rbdl.SpatialVector(0,[0,0,0,1,0,0]),False,0.1,"LoopLink1Link2",11)
        self.cs.AddLoopConstraint( self.iLink1, self.iLink2, Xp2, Xs2, 
            rbdl.SpatialVector(0,[0,0,0,0,1,0]),False,0.1,"LoopLink1Link2",11)
        self.cs.AddLoopConstraint( self.iLink1, self.iLink2, Xp2, Xs2, 
            rbdl.SpatialVector(0,[0,0,0,0,0,1]),False,0.1,"LoopLink1Link2",11)
        self.cs.AddLoopConstraint( self.iLink1, self.iLink2, Xp2, Xs2, 
            rbdl.SpatialVector(0,[1,0,0,0,0,0]),False,0.1,"LoopLink1Link2",11)
        self.cs.AddLoopConstraint( self.iLink1, self.iLink2, Xp2, Xs2, 
            rbdl.SpatialVector(0,[0,0,1,0,0,0]),False,0.1,"LoopLink1Link2",11)

        self.cs.Bind(self.model)


    def test_GroupIndexAccess (self):


        #Test that the groupIndex accessors work
        gId = self.cs.getGroupIndexByName("LoopGroundLink1")
        assert_equal(0,gId)

        #Note: the Id is the user-defined-id (the last argument in 
        #Add ... Constraints) functions        
        gId = self.cs.getGroupIndexById(7)
        assert_equal(0,gId)
        gId = self.cs.getGroupIndexByAssignedId(self.cg0AssignedId)
        assert_equal(0,gId)
        gId = self.cs.getGroupIndexByName("LoopLink1Link2")
        assert_equal(1,gId)
        gId = self.cs.getGroupIndexById(11)
        assert_equal(1,gId)
        gId = self.cs.getGroupIndexByAssignedId(self.cg1AssignedId)
        assert_equal(1,gId)

        #Note: the assigned-id is the integer that is returned by the 
        #      Add ... Constraint() function
        testName = self.cs.getGroupName(0)
        assert_equal(testName,"LoopGroundLink1")

        #Note: the group type corresponds to the enum that appears in
        #Constraints.h line 18 of include/Constraint.h which at the time of
        #writing is:
        #
        #enum ConstraintType {
        #      ConstraintTypeContact=0,
        #      ConstraintTypeLoop,
        #      ConstraintTypeCustom,
        #      ConstraintTypeLast,
        #};
        #
        #
        gType = self.cs.getGroupType(0)
        assert_equal(gType,1)


    def test_calcForces (self):

        self.q.fill(0.)
        self.qd.fill(0.)
        self.qdd.fill(0.)
        self.tau.fill(0.)

        self.q[7] = -self.l1
        self.tau[3] = -self.l2*0.5*self.m2*self.model.gravity[1]

        #print("Q")
        #print(self.q)
        #print("QDot")
        #print(self.qd)
        #print("Tau")
        #print(self.tau)

        rbdl.ForwardDynamicsConstraintsDirect(self.model,self.q,self.qd,
            self.tau,self.cs,self.qdd)

        #print("QDDot")
        #print(self.qdd)

        for i in range(0,self.model.qdot_size):
            assert_almost_equal(self.qdd[i],0.)
                
        gId0 = 0
        gId1 = 1

        csListBodyIds = np.ndarray([2], dtype=np.uintc )
        csListX = np.ndarray([6,6,2],dtype=float)
        csListF = np.ndarray([6,2],dtype=float)

        csListXTest = np.ndarray([6,6,2],dtype=float)
        csListFTest = np.ndarray([6,2],dtype=float)

        csListXTest.fill(0.)
        csListFTest.fill(0.)
        csListFTest[4,0] =  (self.m1+self.m2)*self.model.gravity[1]
        csListFTest[4,1] = -(self.m1+self.m2)*self.model.gravity[1]

        for i in range(0,6):
            csListXTest[i,i,0]=1. 
            csListXTest[i,i,1]=1. 

        self.cs.calcForces( gId0,self.model,self.q,self.qd,
                            csListBodyIds,csListX,csListF,False,False)

        #print("bodyId, Transform, Forces")    
        #for i in range(0,csListBodyIds.shape[0]):
        #    print(csListBodyIds[i])
        #    print(csListX[:,:,i])
        #    print(csListF[:,i])

        assert_equal(csListBodyIds[0],0)
        assert_equal(csListBodyIds[1],self.iLink1)

        assert_almost_equal(csListX, csListXTest)
        assert_almost_equal(csListF, csListFTest) 

        csListBodyIds = np.ndarray([2], dtype=np.uintc )
        csListX = np.ndarray([6,6,2],dtype=float)
        csListF = np.ndarray([6,2],dtype=float)

        self.cs.calcForces( gId1,self.model,self.q,self.qd,
                            csListBodyIds,csListX,csListF,False,False)

        #print("bodyId, Transform, Forces")    
        #for i in range(0,csListBodyIds.shape[0]):
        #    print(csListBodyIds[i])
        #    print(csListX[:,:,i])
        #    print(csListF[:,i])

        assert_equal(csListBodyIds[0],self.iLink1)
        assert_equal(csListBodyIds[1],self.iLink2)

        csListXTest.fill(0.)
        csListFTest.fill(0.)
        csListFTest[2,0] =  (self.m2*self.l2*0.5)*self.model.gravity[1]
        csListFTest[2,1] = -csListFTest[2,0]         
        csListFTest[4,0] =  (self.m2)*self.model.gravity[1]
        csListFTest[4,1] = -csListFTest[4,0]

        for i in range(0,6):
            csListXTest[i,i,0]=1. 
            csListXTest[i,i,1]=1. 

        csListXTest[3,2,0] =  self.l1  #Spatial transform of r=(0,-l1,0), E=eye
        csListXTest[5,0,0] = -self.l1    

        assert_almost_equal(csListX, csListXTest)
        assert_almost_equal(csListF, csListFTest) 

    def test_calcPositionError (self):
        self.q.fill(0.)
        self.qd.fill(0.)
        self.qdd.fill(0.)
        self.tau.fill(0.)

        self.q[7] = -self.l1
        self.tau[3] = -self.l2*0.5*self.m2*self.model.gravity[1]
        self.qd[1] = -1.0
        self.qd[7] = -1.0

        self.q[0] = 1.0

        posErr = np.ndarray([5],dtype=float)
        posErrTest=np.ndarray([5],dtype=float)

        self.cs.calcPositionError(0,self.model,self.q,posErr,True)

        posErrTest.fill(0.)
        posErrTest[0] = 1.0

        assert_almost_equal(posErr,posErrTest)

    def test_calcVelocityError (self):
        self.q.fill(0.)
        self.qd.fill(0.)
        self.qdd.fill(0.)
        self.tau.fill(0.)

        self.q[7] = -self.l1
        self.tau[3] = -self.l2*0.5*self.m2*self.model.gravity[1]
        self.qd[0] = -1.0
        self.qd[6] = -1.0

        velErr = np.ndarray([5],dtype=float)
        velErrTest=np.ndarray([5],dtype=float)

        self.cs.calcVelocityError(0,self.model,self.q,self.qd,velErr,True)

        velErrTest.fill(0.)
        velErrTest[0] = -1.0

        assert_almost_equal(velErr,velErrTest)

    def test_Baumgarte(self):

        assert_equal(self.cs.isBaumgarteStabilizationEnabled(0), False)
        self.cs.enableBaumgarteStabilization(0)
        assert_equal(self.cs.isBaumgarteStabilizationEnabled(0), True)
        self.cs.disableBaumgarteStabilization(0)
        assert_equal(self.cs.isBaumgarteStabilizationEnabled(0), False)

        bgCoeff=np.ndarray([2],dtype=float)
        self.cs.getBaumgarteStabilizationCoefficients(0, bgCoeff)

        bgCoeffTest=np.ndarray([2],dtype=float)
        bgCoeffTest[0]=10.
        bgCoeffTest[1]=10.
        assert_almost_equal(bgCoeff,bgCoeffTest)

        self.q.fill(0.)
        self.qd.fill(0.)
        self.qdd.fill(0.)
        self.tau.fill(0.)

        self.q[7] = -self.l1

        self.q[0] = 1.
        self.qd[1] = 2.
        
        bgStabForceTest=np.ndarray([5],dtype=float)
        bgStabForce=np.ndarray([5],dtype=float)

        posErr = np.ndarray([5],dtype=float)
        velErr = np.ndarray([5],dtype=float)
        self.cs.calcPositionError(0,self.model,self.q,posErr,True)
        self.cs.calcVelocityError(0,self.model,self.q,self.qd,velErr,True)
        self.cs.calcBaumgarteStabilizationForces(0,self.model,posErr,velErr,
            bgStabForce)

        for i in range(0,5):
            bgStabForceTest[i] = -2.*bgCoeff[0]*velErr[i] - bgCoeff[1]*bgCoeff[1]*posErr[i]

        assert_almost_equal(bgStabForce, bgStabForceTest)

if __name__ == '__main__':
    unittest.main()
