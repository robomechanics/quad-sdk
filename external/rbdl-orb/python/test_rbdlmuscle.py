#!/usr/bin/python3
import unittest

import math
import numpy as np
from numpy.testing import *
import rbdl
import rbdlmuscle


class SubjectInformationTests (unittest.TestCase):
  
    def test_SubjectInformation(self):

        si = rbdlmuscle.SubjectInformation()
        si.gender = rbdlmuscle.GenderSet.Male
        si.ageGroup = rbdlmuscle.AgeGroupSet.Middle55To65
        si.heightInMeters = 1.8
        si.massInKg = 90
        
        assert_equal (si.gender, rbdlmuscle.GenderSet.Male)
        assert_equal (si.ageGroup, rbdlmuscle.AgeGroupSet.Middle55To65)
        assert_equal (si.heightInMeters, 1.8)
        assert_equal (si.massInKg, 90)


class Millard2016TorqueMuscleTests (unittest.TestCase):
  
    def test_constructor(self):
  
        si = rbdlmuscle.SubjectInformation()
        si.gender = rbdlmuscle.GenderSet.Male
        si.ageGroup = rbdlmuscle.AgeGroupSet.Young18To25
        si.heightInMeters = 1.8
        si.massInKg = 90
            
        mz = rbdlmuscle.Millard2016TorqueMuscle(rbdlmuscle.DataSet.Gymnast, si, 
                          rbdlmuscle.JointTorqueSet.HipExtension , 0, -1, -1)
        
        maxtorque = mz.getMaximumActiveIsometricTorque()
        mz.setMaximumActiveIsometricTorque(maxtorque-50)
        maxvelocity = mz.getMaximumConcentricJointAngularVelocity()
        mz.setMaximumConcentricJointAngularVelocity(maxvelocity-10)
           
        assert_equal ( mz.getMaximumActiveIsometricTorque() , maxtorque-50)
        assert_equal ( mz.getJointAngleOffset(), 0 )
        assert_equal ( mz.getJointAngleSign(), -1 )
        assert_equal ( mz.getJointTorqueSign(), -1 )
        
        assert_equal ( mz.getMaximumConcentricJointAngularVelocity(), maxvelocity-10 )
        assert_equal ( mz.getJointAngleAtOneNormalizedPassiveIsometricTorque(), -1.5708 )
        assert_equal ( mz.getPassiveTorqueScale(), 1)

    
    
      
        

if __name__ == '__main__':
    unittest.main()
