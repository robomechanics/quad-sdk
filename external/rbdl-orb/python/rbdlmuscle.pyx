#cython: c_string_type=unicode, c_string_encoding=default, boundscheck=False, embedsignature=True

import numpy as np
cimport numpy as np
from enum import IntEnum
from enum import Enum
from libc.stdint cimport uintptr_t
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool


#cimport cpython.ref as cpy_ref

from crbdl cimport VectorNd  
from crbdl cimport Vector3d 
from crbdl cimport MatrixNd 

cimport crbdlmuscle


from rbdl cimport NumpyToVector3d as NumpyToVector3d
from rbdl cimport Vector3dToNumpy as Vector3dToNumpy
from rbdl cimport NumpyToVectorNd as NumpyToVectorNd
from rbdl cimport VectorNdToNumpy as VectorNdToNumpy
from rbdl cimport NumpyToMatrixNd as NumpyToMatrixNd
from rbdl cimport MatrixNdToNumpy as MatrixNdToNumpy





##############################
#
# SmoothSegmentedFunction.h
#
##############################




cdef class SmoothSegmentedFunction:
  
    cdef crbdlmuscle.SmoothSegmentedFunction *thisptr
    cdef free_on_dealloc
    
    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __cinit__(self,
          np.ndarray[double, ndim=2, mode="c"] mX = None, 
          np.ndarray[double, ndim=2, mode="c"] mY = None,
          double x0 = 0, double x1 = 0, double y0 = 0, double y1 =0,
          double dydx0 = 0, 
          double dydx1 = 0, 
          string name = b"", 
          uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            if mY is None:
                self.thisptr = new crbdlmuscle.SmoothSegmentedFunction()
            else:
                self.thisptr = new crbdlmuscle.SmoothSegmentedFunction(
                        NumpyToMatrixNd(mX), NumpyToMatrixNd(mY), 
                        x0, x1, y0, y1, dydx0, dydx1, name)
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdlmuscle.SmoothSegmentedFunction*>address

    def updSmoothSegmentedFunction( self,
             np.ndarray[double, ndim=2, mode="c"] mX, 
             np.ndarray[double, ndim=2, mode="c"] mY, 
             double x0, double x1,
             double y0, double y1,
             double dydx0, double dydx1,
             string name):
        self.thisptr.updSmoothSegmentedFunction(  
             NumpyToMatrixNd(mX), 
             NumpyToMatrixNd(mY), 
             x0, x1, y0, y1, dydx0, dydx1, name)
             
    def shift(self, double xShift, double yShift):
        self.thisptr.shift(xShift, yShift)
        
    def scale(self, double xScale, double yScale):
        self.thisptr.scale(xScale, yScale)
    
    def calcValue(self, double x):
        return self.thisptr.calcValue(x)
    
    def calcDerivative(self, double x, int order):
        return self.thisptr.calcDerivative(x, order)
    
    def getName(self):
        return self.thisptr.getName()
        
    def setName(self, string name):
        self.thisptr.setName(name)
    
    def getCurveDomain(self):
        return VectorNdToNumpy( self.thisptr.getCurveDomain())
    
    def printCurveToCSVFile(self, string path,
          string fileNameWithoutExtension,
            double domainMin,
            double domainMax):
          self.thisptr.printCurveToCSVFile(path,
                       fileNameWithoutExtension,
                       domainMin,
                       domainMax)
                      
    def calcSampledCurve(self, int maxOrder,
                double domainMin,
                double domainMax):
        return MatrixNdToNumpy( self.thisptr.calcSampledCurve(maxOrder,
                domainMin,
                domainMax) )
                
    def getXControlPoints(self, np.ndarray[double, ndim=2, mode="c"] mat):
        cdef MatrixNd cmat = MatrixNd()
        self.thisptr.getXControlPoints( cmat )
        
        np.resize(mat,(cmat.rows(),cmat.cols()))
        for i in range(cmat.rows()):
          for j in range (cmat.cols()):
            mat[i,j] = cmat.coeff(i,j)
        
    
    def getYControlPoints(self, np.ndarray[double, ndim=2, mode="c"] mat):
        cdef MatrixNd cmat = MatrixNd()
        self.thisptr.getYControlPoints( cmat )
        
        np.resize(mat,(cmat.rows(),cmat.cols()))
        for i in range(cmat.rows()):
          for j in range (cmat.cols()):
            mat[i,j] = cmat.coeff(i,j)




##############################
#
# Millard2016TorqueMuscle.h
#
##############################


class DataSet(IntEnum):
    Anderson2007 = 0
    Gymnast = 1
    Last = 2


class GenderSet(IntEnum):
    Male = 0
    Female = 1
    Last = 2
    
    
class AgeGroupSet(IntEnum):
    Young18To25 = 0
    Middle55To65 = 1
    SeniorOver65 = 2
    Last = 3
    
class JointTorqueSet(IntEnum):
    HipExtension                  = 0
    HipFlexion                    = 1
    KneeExtension                 = 2
    KneeFlexion                   = 3
    AnkleExtension                = 4
    AnkleFlexion                  = 5
    ElbowExtension                = 6
    ElbowFlexion                  = 7
    ShoulderExtension             = 8
    ShoulderFlexion               = 9
    WristExtension                = 10
    WristFlexion                  = 11
    ShoulderHorizontalAdduction   = 12
    ShoulderHorizontalAbduction   = 13
    ShoulderInternalRotation      = 14
    ShoulderExternalRotation      = 15
    WristUlnarDeviation           = 16
    WristRadialDeviation          = 17
    WristPronation                = 18
    WristSupination               = 19
    LumbarExtension               = 20
    LumbarFlexion                 = 21
    Last                          = 22
       
            

cdef class SubjectInformation:
  
    cdef crbdlmuscle.SubjectInformation *thisptr
    cdef free_on_dealloc
    
    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr
                        
    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdlmuscle.SubjectInformation()
            
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdlmuscle.SubjectInformation*>address
    
    property gender:
      def __get__(self):
        return GenderSet(self.thisptr.gender)
      def __set__ (self, value):
         if value == GenderSet.Male:
            self.thisptr.gender = crbdlmuscle.GenderSet_Male
         elif value == GenderSet.Female:
            self.thisptr.gender = crbdlmuscle.GenderSet_Female
         else:
             raise TypeError('gender must be an instance of GenderSet Enum or 0/1')
            
                
    property ageGroup:
      def __get__(self):
        return AgeGroupSet(self.thisptr.ageGroup)
      def __set__ (self, value):
         if value == AgeGroupSet.Young18To25:
             self.thisptr.ageGroup = crbdlmuscle.AgeGroupSet_Young18To25
         elif value == AgeGroupSet.Middle55To65:
             self.thisptr.ageGroup = crbdlmuscle.AgeGroupSet_Middle55To65
         elif value == AgeGroupSet.SeniorOver65:
             self.thisptr.ageGroup = crbdlmuscle.AgeGroupSet_SeniorOver65
         else:
             raise TypeError('gender must be an instance of GenderSet Enum or 0/1/2')
    
    property heightInMeters:
      def __get__ (self):
        return self.thisptr.heightInMeters
      def __set__ (self, value):
        self.thisptr.heightInMeters = value
    
    property massInKg:
      def __get__ (self):
        return self.thisptr.massInKg
      def __set__ (self, value):
        self.thisptr.massInKg = value




cdef class TorqueMuscleSummary:
  
    cdef crbdlmuscle.TorqueMuscleSummary *thisptr
    cdef free_on_dealloc
    
    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr
            
    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdlmuscle.TorqueMuscleSummary()
            
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdlmuscle.TorqueMuscleSummary*>address
        
    property fiberAngle:
        def __get__ (self):
            return self.thisptr.fiberAngle
        def __set__ (self, value):
            self.thisptr.fiberAngle = value
    
    property fiberAngularVelocity:
        def __get__ (self):
            return self.thisptr.fiberAngularVelocity
        def __set__ (self, value):
            self.thisptr.fiberAngularVelocity = value
    
    property activation:
        def __get__ (self):
            return self.thisptr.activation
        def __set__ (self, value):
            self.thisptr.activation = value
    
    property fiberPassiveTorqueAngleMultiplier:
        def __get__ (self):
            return self.thisptr.fiberPassiveTorqueAngleMultiplier
        def __set__ (self, value):
            self.thisptr.fiberPassiveTorqueAngleMultiplier = value
    
    property fiberActiveTorqueAngleMultiplier:
        def __get__ (self):
            return self.thisptr.fiberActiveTorqueAngleMultiplier
        def __set__ (self, value):
            self.thisptr.fiberActiveTorqueAngleMultiplier = value
    
    property fiberTorqueAngularVelocityMultiplier:
        def __get__ (self):
            return self.thisptr.fiberTorqueAngularVelocityMultiplier
        def __set__ (self, value):
            self.thisptr.fiberTorqueAngularVelocityMultiplier = value
    
    property fiberNormalizedDampingTorque:
        def __get__ (self):
            return self.thisptr.fiberNormalizedDampingTorque
        def __set__ (self, value):
            self.thisptr.fiberNormalizedDampingTorque = value
    
    property fiberTorque:
        def __get__ (self):
            return self.thisptr.fiberTorque
        def __set__ (self, value):
            self.thisptr.fiberTorque = value
    
    property jointTorque:
        def __get__ (self):
            return self.thisptr.jointTorque
        def __set__ (self, value):
            self.thisptr.jointTorque = value



    

cdef class TorqueMuscleInfo:
    cdef crbdlmuscle.TorqueMuscleInfo *thisptr
    cdef free_on_dealloc
    
    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr
    
    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdlmuscle.TorqueMuscleInfo()
            
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdlmuscle.TorqueMuscleInfo*>address
    
    
    property jointAngle:
        def __get__ (self):
            return self.thisptr.jointAngle
        def __set__ (self, value):
            self.thisptr.jointAngle = value
    
    property jointAngularVelocity:
        def __get__ (self):
            return self.thisptr.jointAngularVelocity
        def __set__ (self, value):
            self.thisptr.jointAngularVelocity = value
    
    property fiberAngle:
        def __get__ (self):
            return self.thisptr.fiberAngle
        def __set__ (self, value):
            self.thisptr.fiberAngle = value
            
    property fiberAngularVelocity:
        def __get__ (self):
            return self.thisptr.fiberAngularVelocity
        def __set__ (self, value):
            self.thisptr.fiberAngularVelocity = value
            
    property fiberPassiveTorqueAngleMultiplier:
        def __get__ (self):
            return self.thisptr.fiberPassiveTorqueAngleMultiplier
        def __set__ (self, value):
            self.thisptr.fiberPassiveTorqueAngleMultiplier = value
    
    property DfiberPassiveTorqueAngleMultiplier_DblendingVariable:
        def __get__ (self):
            return self.thisptr.DfiberPassiveTorqueAngleMultiplier_DblendingVariable
        def __set__ (self, value):
            self.thisptr.DfiberPassiveTorqueAngleMultiplier_DblendingVariable = value
    
    property DfiberPassiveTorqueAngleMultiplier_DangleOffset:
        def __get__ (self):
            return self.thisptr.DfiberPassiveTorqueAngleMultiplier_DangleOffset
        def __set__ (self, value):
            self.thisptr.DfiberPassiveTorqueAngleMultiplier_DangleOffset = value
            
    property fiberActiveTorqueAngleMultiplier:
        def __get__ (self):
            return self.thisptr.fiberActiveTorqueAngleMultiplier
        def __set__ (self, value):
            self.thisptr.fiberActiveTorqueAngleMultiplier = value
    
    property DfiberActiveTorqueAngleMultiplier_DblendingVariable:
        def __get__ (self):
            return self.thisptr.DfiberActiveTorqueAngleMultiplier_DblendingVariable
        def __set__ (self, value):
            self.thisptr.DfiberActiveTorqueAngleMultiplier_DblendingVariable = value
            
    property fiberTorqueAngularVelocityMultiplier:
        def __get__ (self):
            return self.thisptr.fiberTorqueAngularVelocityMultiplier
        def __set__ (self, value):
            self.thisptr.fiberTorqueAngularVelocityMultiplier = value
    
    property DfiberTorqueAngularVelocityMultiplier_DblendingVariable:
        def __get__ (self):
            return self.thisptr.DfiberTorqueAngularVelocityMultiplier_DblendingVariable
        def __set__ (self, value):
            self.thisptr.DfiberTorqueAngularVelocityMultiplier_DblendingVariable = value
            
    property activation:
        def __get__ (self):
            return self.thisptr.activation
        def __set__ (self, value):
            self.thisptr.activation = value
            
            
    property fiberActiveTorque:
        def __get__ (self):
            return self.thisptr.fiberActiveTorque
        def __set__ (self, value):
            self.thisptr.fiberActiveTorque = value
    
    property fiberPassiveTorque:
        def __get__ (self):
            return self.thisptr.fiberPassiveTorque
        def __set__ (self, value):
            self.thisptr.fiberPassiveTorque = value
    
    property fiberPassiveElasticTorque:
        def __get__ (self):
            return self.thisptr.fiberPassiveElasticTorque
        def __set__ (self, value):
            self.thisptr.fiberPassiveElasticTorque = value
    
    property fiberDampingTorque:
        def __get__ (self):
            return self.thisptr.fiberDampingTorque
        def __set__ (self, value):
            self.thisptr.fiberDampingTorque = value
    
    property fiberNormDampingTorque:
        def __get__ (self):
            return self.thisptr.fiberNormDampingTorque
        def __set__ (self, value):
            self.thisptr.fiberNormDampingTorque = value
            
    property fiberTorque:
        def __get__ (self):
            return self.thisptr.fiberTorque
        def __set__ (self, value):
            self.thisptr.fiberTorque = value
            
    property jointTorque:
        def __get__ (self):
            return self.thisptr.jointTorque
        def __set__ (self, value):
            self.thisptr.jointTorque = value
            
    property fiberStiffness:
        def __get__ (self):
            return self.thisptr.fiberStiffness
        def __set__ (self, value):
            self.thisptr.fiberStiffness = value
            
    property jointStiffness:
        def __get__ (self):
            return self.thisptr.jointStiffness
        def __set__ (self, value):
            self.thisptr.jointStiffness = value
            
            
    property fiberActivePower:
        def __get__ (self):
            return self.thisptr.fiberActivePower
        def __set__ (self, value):
            self.thisptr.fiberActivePower = value
            
    property fiberPassivePower:
        def __get__ (self):
            return self.thisptr.fiberPassivePower
        def __set__ (self, value):
            self.thisptr.fiberPassivePower = value
            
    property fiberPower:
        def __get__ (self):
            return self.thisptr.fiberPower
        def __set__ (self, value):
            self.thisptr.fiberPower = value
    
    property jointPower:
        def __get__ (self):
            return self.thisptr.jointPower
        def __set__ (self, value):
            self.thisptr.jointPower = value
            
    property DjointTorque_Dactivation:
        def __get__ (self):
            return self.thisptr.DjointTorque_Dactivation
        def __set__ (self, value):
            self.thisptr.DjointTorque_Dactivation = value
    
    property DjointTorque_DjointAngle:
        def __get__ (self):
            return self.thisptr.DjointTorque_DjointAngle
        def __set__ (self, value):
            self.thisptr.DjointTorque_DjointAngle = value
            
    property DjointTorque_DjointAngularVelocity:
        def __get__ (self):
            return self.thisptr.DjointTorque_DjointAngularVelocity
        def __set__ (self, value):
            self.thisptr.DjointTorque_DjointAngularVelocity = value

       
    property DjointTorque_DactiveTorqueAngleBlendingVariable:
        def __get__ (self):
            return self.thisptr.DjointTorque_DactiveTorqueAngleBlendingVariable
        def __set__ (self, value):
            self.thisptr.DjointTorque_DactiveTorqueAngleBlendingVariable = value
    
    property DjointTorque_DpassiveTorqueAngleBlendingVariable:
        def __get__ (self):
            return self.thisptr.DjointTorque_DpassiveTorqueAngleBlendingVariable
        def __set__ (self, value):
            self.thisptr.DjointTorque_DpassiveTorqueAngleBlendingVariable = value
            
    property DjointTorque_DtorqueAngularVelocityBlendingVariable:
        def __get__ (self):
            return self.thisptr.DjointTorque_DtorqueAngularVelocityBlendingVariable
        def __set__ (self, value):
            self.thisptr.DjointTorque_DtorqueAngularVelocityBlendingVariable = value
            
    property DjointTorque_DmaximumIsometricTorque:
        def __get__ (self):
            return self.thisptr.DjointTorque_DmaximumIsometricTorque
        def __set__ (self, value):
            self.thisptr.DjointTorque_DmaximumIsometricTorque = value
            
    property DjointTorque_DpassiveTorqueAngleCurveAngleOffset:
        def __get__ (self):
            return self.thisptr.DjointTorque_DpassiveTorqueAngleCurveAngleOffset
        def __set__ (self, value):
            self.thisptr.DjointTorque_DpassiveTorqueAngleCurveAngleOffset = value
            
    property DjointTorque_DactiveTorqueAngleAngleScaling:
        def __get__ (self):
            return self.thisptr.DjointTorque_DactiveTorqueAngleAngleScaling
        def __set__ (self, value):
            self.thisptr.DjointTorque_DactiveTorqueAngleAngleScaling = value
    
    property DjointTorque_DmaximumAngularVelocity:
        def __get__ (self):
            return self.thisptr.DjointTorque_DmaximumAngularVelocity
        def __set__ (self, value):
            self.thisptr.DjointTorque_DmaximumAngularVelocity = value
            
    property fittingInfo:
        def __get__ (self):
           return VectorNdToNumpy(self.thisptr.fittingInfo)

        def __set__ (self, value):
           self.thisptr.fittingInfo.resize(value.shape[0])
           for i in range (value.shape[0]):
                    (&(self.thisptr.fittingInfo[i]))[0] = value[i]   
            
   

cdef class Millard2016TorqueMuscle:
    cdef crbdlmuscle.Millard2016TorqueMuscle *thisptr
    cdef free_on_dealloc
    
    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr
    

    def __cinit__(self, dataSet = None, 
          SubjectInformation subjectInfo = None, 
          int jointTorque = 1, 
          double jointAngleOffsetRelativeToDoxygenFigures = 1., 
          double signOfJointAngleRelativeToDoxygenFigures = 1., 
          double signOfJointTorqueToDoxygenFigures = 1., 
          string name = b"empty", 
          uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            if dataSet is None:
                self.thisptr = new crbdlmuscle.Millard2016TorqueMuscle()
            else:
                # Type checking
                #if not isinstance(dataSet, DataSet):
                #     raise TypeError('dataSet must be an instance of DataSet Enum')
                if subjectInfo is None:
                    print ("Warning: dataset, but no subject info specified, using default constructor")
                    self.thisptr = new crbdlmuscle.Millard2016TorqueMuscle()
                else:
                    if dataSet == DataSet.Anderson2007:
                      
                        self.thisptr = new crbdlmuscle.Millard2016TorqueMuscle(
                        crbdlmuscle.DataSet_Anderson2007, subjectInfo.thisptr[0], jointTorque, 
                        jointAngleOffsetRelativeToDoxygenFigures, 
                        signOfJointAngleRelativeToDoxygenFigures, 
                        signOfJointTorqueToDoxygenFigures, name)
                        
                    elif dataSet == DataSet.Gymnast:
                      
                        self.thisptr = new crbdlmuscle.Millard2016TorqueMuscle(
                        crbdlmuscle.DataSet_Gymnast, subjectInfo.thisptr[0], jointTorque, 
                        jointAngleOffsetRelativeToDoxygenFigures, 
                        signOfJointAngleRelativeToDoxygenFigures, 
                        signOfJointTorqueToDoxygenFigures, name)
                    else:
                        raise TypeError('dataSet must be an instance of DataSet Enum or 0/1')
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdlmuscle.Millard2016TorqueMuscle*>address
    
    def calcJointTorque( self, double jointAngle, double jointAngularVelocity, 
                         double activation):
        return self.thisptr.calcJointTorque( jointAngle, jointAngularVelocity, activation)
        
    def calcActivation(self, double jointAngle,
                                    double jointAngularVelocity,
                                    double jointTorque,
                                    TorqueMuscleSummary tms):
        self.thisptr.calcActivation( jointAngle,
                                     jointAngularVelocity,
                                     jointTorque,
                                     tms.thisptr[0])
                                                                                                                                               

    def calcMaimumActiveIsometricTorqueScalingFactor(self, double jointAngle,
                          double jointAngularVelocity,
                          double activation,
                          double jointTorque):
        return self.thisptr.calcMaximumActiveIsometricTorqueScalingFactor(
                          jointAngle,
                          jointAngularVelocity,
                          activation,
                          jointTorque)
                          
    def calcTorqueMuscleInfo(self, double jointAngle, 
                          double jointAngularVelocity,
                          double activation,
                          TorqueMuscleInfo torqueMuscleInfoStruct):
        self.thisptr.calcTorqueMuscleInfo(
                        jointAngle,
                        jointAngularVelocity,
                        activation,
                        torqueMuscleInfoStruct.thisptr[0])
                      
    def getJointTorqueSign(self):
        return self.thisptr.getJointTorqueSign() 
        
    def getJointAngleSign(self):
        return self.thisptr.getJointAngleSign() 
        
    def getJointAngleOffset(self):
        return self.thisptr.getJointAngleOffset()
    
    def getMaximumActiveIsometricTorque(self):
        return self.thisptr.getMaximumActiveIsometricTorque()
        
    def getJointAngleAtMaximumActiveIsometricTorque(self):
        return self.thisptr.getJointAngleAtMaximumActiveIsometricTorque()
    
    def getActiveTorqueAngleCurveWidth(self):
        return self.thisptr.getActiveTorqueAngleCurveWidth()
    
    def getJointAngleAtOneNormalizedPassiveIsometricTorque(self):
        return self.thisptr.getJointAngleAtOneNormalizedPassiveIsometricTorque()
    
    def getMaximumConcentricJointAngularVelocity(self):
        return self.thisptr.getMaximumConcentricJointAngularVelocity() 
    
    def getPassiveTorqueScale(self):
        return self.thisptr.getPassiveTorqueScale() 
        
    def getPassiveCurveAngleOffset(self):    
        return self.thisptr.getPassiveCurveAngleOffset() 
        
    def getNormalizedDampingCoefficient(self):
        return self.thisptr.getNormalizedDampingCoefficient() 
        
    def setNormalizedDampingCoefficient(self, double beta):    
        self.thisptr.setNormalizedDampingCoefficient(beta)
        
        
    def setPassiveTorqueScale(self, double passiveTorqueScale):    
        self.thisptr.setPassiveTorqueScale(passiveTorqueScale)
        
    def setPassiveCurveAngleOffset( self,
                  double passiveCurveAngleOffsetVal):
        self.thisptr.setPassiveCurveAngleOffset(
                  passiveCurveAngleOffsetVal)
                  
    def fitPassiveTorqueScale(self, double jointAngle,
                                           double passiveTorque):          
        self.thisptr.fitPassiveTorqueScale(jointAngle,
                                           passiveTorque)
                                           
    def fitPassiveCurveAngleOffset(self, double jointAngle,
                                           double passiveTorque):                                       
        self.thisptr.fitPassiveCurveAngleOffset(jointAngle,
                                           passiveTorque)
                                           
    def setMaximumActiveIsometricTorque(self, double maxIsometricTorque):                                       
        self.thisptr.setMaximumActiveIsometricTorque(maxIsometricTorque)
        
    def setMaximumConcentricJointAngularVelocity(self, double maxAngularVelocity):    
        self.thisptr.setMaximumConcentricJointAngularVelocity(maxAngularVelocity)

    def getActiveTorqueAngleCurve(self):
      cdef crbdlmuscle.SmoothSegmentedFunction ssf = crbdlmuscle.SmoothSegmentedFunction()
      cdef VectorNd x_ = VectorNd()
      cdef MatrixNd Mx = MatrixNd()
      cdef MatrixNd My = MatrixNd()
      cdef string name_
      sff = self.thisptr.getActiveTorqueAngleCurve()
      
      x_ = sff.getCurveDomain()
      sff.getXControlPoints(Mx)
      sff.getYControlPoints(My)
      
      return SmoothSegmentedFunction( MatrixNdToNumpy(Mx), 
                                      MatrixNdToNumpy(My),
                                      x_[0], x_[1], 
                                      sff.calcValue(x_[0]),
                                      sff.calcValue(x_[1]),
                                      sff.calcDerivative(x_[0], 1),
                                      sff.calcDerivative(x_[1], 1),
                                      sff.getName())
                                      
    def getPassiveTorqueAngleCurve(self):
      cdef crbdlmuscle.SmoothSegmentedFunction ssf = crbdlmuscle.SmoothSegmentedFunction()
      cdef VectorNd x_ = VectorNd()
      cdef MatrixNd Mx = MatrixNd()
      cdef MatrixNd My = MatrixNd()
      cdef string name_
      sff = self.thisptr.getPassiveTorqueAngleCurve()
      
      x_ = sff.getCurveDomain()
      sff.getXControlPoints(Mx)
      sff.getYControlPoints(My)
      
      return SmoothSegmentedFunction( MatrixNdToNumpy(Mx), 
                                      MatrixNdToNumpy(My),
                                      x_[0], x_[1], 
                                      sff.calcValue(x_[0]),
                                      sff.calcValue(x_[1]),
                                      sff.calcDerivative(x_[0], 1),
                                      sff.calcDerivative(x_[1], 1),
                                      sff.getName())
                                
    def getTorqueAngularVelocityCurve(self):
      cdef crbdlmuscle.SmoothSegmentedFunction ssf = crbdlmuscle.SmoothSegmentedFunction()
      cdef VectorNd x_ = VectorNd()
      cdef MatrixNd Mx = MatrixNd()
      cdef MatrixNd My = MatrixNd()

      sff = self.thisptr.getTorqueAngularVelocityCurve()
      
      x_ = sff.getCurveDomain()
      sff.getXControlPoints(Mx)
      sff.getYControlPoints(My)
      
      return SmoothSegmentedFunction( MatrixNdToNumpy(Mx), 
                                      MatrixNdToNumpy(My),
                                      x_[0], x_[1], 
                                      sff.calcValue(x_[0]),
                                      sff.calcValue(x_[1]),
                                      sff.calcDerivative(x_[0], 1),
                                      sff.calcDerivative(x_[1], 1),
                                      sff.getName())
      

    def printJointTorqueProfileToFile(self, const string path,
                        const string fileNameWithoutExtension,
                        int numberOfSamplePoints):
                          
        self.thisptr.printJointTorqueProfileToFile(
                        path,
                        fileNameWithoutExtension,
                        numberOfSamplePoints)
                        
    def getName(self):
        return self.thisptr.getName()
        
    def setName(self,  string name):   
        self.thisptr.setName(name)
    
    def setActiveTorqueAngleCurveBlendingVariable (self, blendingVariable):   
        self.thisptr.setActiveTorqueAngleCurveBlendingVariable ( blendingVariable)
        
    def getActiveTorqueAngleCurveBlendingVariable(self):    
        return self.thisptr.getActiveTorqueAngleCurveBlendingVariable ()
        
    def setPassiveTorqueAngleCurveBlendingVariable (self, blendingVariable):
        self.thisptr.setPassiveTorqueAngleCurveBlendingVariable (blendingVariable)
        
    def getPassiveTorqueAngleCurveBlendingVariable(self):
        return self.thisptr.getPassiveTorqueAngleCurveBlendingVariable ()
        
    def setTorqueAngularVelocityCurveBlendingVariable (self, blendingVariable):
        self.thisptr.setTorqueAngularVelocityCurveBlendingVariable (blendingVariable)
 
    def getTorqueAngularVelocityCurveBlendingVariable(self):
        return self.thisptr.getTorqueAngularVelocityCurveBlendingVariable ()
    
    def getActiveTorqueAngleCurveAngleScaling(self):
        return self.thisptr.getActiveTorqueAngleCurveAngleScaling ()

    def setActiveTorqueAngleCurveAngleScaling(self, angleScaling):
        self.thisptr.setActiveTorqueAngleCurveAngleScaling(angleScaling)
    
    def setFittedParameters (self, TorqueMuscleParameterFittingData fittedParameters):
        self.thisptr.setFittedParameters (fittedParameters.thisptr[0])




cdef class TorqueMuscleParameterFittingData:
    
    
    cdef crbdlmuscle.TorqueMuscleParameterFittingData *thisptr
    cdef free_on_dealloc

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr


    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdlmuscle.TorqueMuscleParameterFittingData()
            
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdlmuscle.TorqueMuscleParameterFittingData*>address
    
    property indexAtMaximumActivation:
        def __get__ (self):
            return self.thisptr.indexAtMaximumActivation
        def __set__ (self, value):
            self.thisptr.indexAtMaximumActivation = value
    
    property indexAtMinimumActivation:
        def __get__ (self):
            return self.thisptr.indexAtMinimumActivation
        def __set__ (self, value):
            self.thisptr.indexAtMinimumActivation = value
    
    property indexAtMaxPassiveTorqueAngleMultiplier:
        def __get__ (self):
            return self.thisptr.indexAtMaxPassiveTorqueAngleMultiplier
        def __set__ (self, value):
            self.thisptr.indexAtMaxPassiveTorqueAngleMultiplier = value
    
    property isTorqueMuscleActive:
        def __get__ (self):
            return self.thisptr.isTorqueMuscleActive
        def __set__ (self, value):
            self.thisptr.isTorqueMuscleActive = value
    
    property activeTorqueAngleBlendingVariable:
        def __get__ (self):
            return self.thisptr.activeTorqueAngleBlendingVariable
        def __set__ (self, value):
            self.thisptr.activeTorqueAngleBlendingVariable = value
    
    property passiveTorqueAngleBlendingVariable:
        def __get__ (self):
            return self.thisptr.passiveTorqueAngleBlendingVariable
        def __set__ (self, value):
            self.thisptr.passiveTorqueAngleBlendingVariable = value
    
    property torqueVelocityBlendingVariable:
        def __get__ (self):
            return self.thisptr.torqueVelocityBlendingVariable
        def __set__ (self, value):
            self.thisptr.torqueVelocityBlendingVariable = value
    
    property passiveTorqueAngleCurveOffset:
        def __get__ (self):
            return self.thisptr.passiveTorqueAngleCurveOffset
        def __set__ (self, value):
            self.thisptr.passiveTorqueAngleCurveOffset = value
    
    property maximumActiveIsometricTorque:
        def __get__ (self):
            return self.thisptr.maximumActiveIsometricTorque
        def __set__ (self, value):
            self.thisptr.maximumActiveIsometricTorque = value
    
    property activeTorqueAngleAngleScaling:
        def __get__ (self):
            return self.thisptr.activeTorqueAngleAngleScaling
        def __set__ (self, value):
            self.thisptr.activeTorqueAngleAngleScaling = value

    property maximumAngularVelocity:
        def __get__ (self):
            return self.thisptr.maximumAngularVelocity
        def __set__ (self, value):
            self.thisptr.maximumAngularVelocity = value

    property fittingConverged:
        def __get__ (self):
            return self.thisptr.fittingConverged
        def __set__ (self, value):
            self.thisptr.fittingConverged = value


    property summaryDataAtMinimumActivation:
        def __get__ (self):
            MS = TorqueMuscleSummary()
            MS.fiberAngle = self.thisptr.summaryDataAtMinimumActivation.fiberAngle
            MS.fiberAngularVelocity = self.thisptr.summaryDataAtMinimumActivation.fiberAngularVelocity
            MS.activation = self.thisptr.summaryDataAtMinimumActivation.activation
            MS.fiberPassiveTorqueAngleMultiplier = self.thisptr.summaryDataAtMinimumActivation.fiberPassiveTorqueAngleMultiplier
            MS.fiberActiveTorqueAngleMultiplier = self.thisptr.summaryDataAtMinimumActivation.fiberActiveTorqueAngleMultiplier
            MS.fiberTorqueAngularVelocityMultiplier = self.thisptr.summaryDataAtMinimumActivation.fiberTorqueAngularVelocityMultiplier
            MS.fiberNormalizedDampingTorque = self.thisptr.summaryDataAtMinimumActivation.fiberNormalizedDampingTorque
            MS.fiberTorque = self.thisptr.summaryDataAtMinimumActivation.fiberTorque
            MS.jointTorque = self.thisptr.summaryDataAtMinimumActivation.jointTorque          
              
            return MS
                
        
    
    property summaryDataAtMaximumActivation:
        def __get__ (self):
            MS = TorqueMuscleSummary()
            MS.fiberAngle = self.thisptr.summaryDataAtMaximumActivation.fiberAngle
            MS.fiberAngularVelocity = self.thisptr.summaryDataAtMaximumActivation.fiberAngularVelocity
            MS.activation = self.thisptr.summaryDataAtMaximumActivation.activation
            MS.fiberPassiveTorqueAngleMultiplier = self.thisptr.summaryDataAtMaximumActivation.fiberPassiveTorqueAngleMultiplier
            MS.fiberActiveTorqueAngleMultiplier = self.thisptr.summaryDataAtMaximumActivation.fiberActiveTorqueAngleMultiplier
            MS.fiberTorqueAngularVelocityMultiplier = self.thisptr.summaryDataAtMaximumActivation.fiberTorqueAngularVelocityMultiplier
            MS.fiberNormalizedDampingTorque = self.thisptr.summaryDataAtMaximumActivation.fiberNormalizedDampingTorque
            MS.fiberTorque = self.thisptr.summaryDataAtMaximumActivation.fiberTorque
            MS.jointTorque = self.thisptr.summaryDataAtMaximumActivation.jointTorque          
              
            return MS

    property summaryDataAtMaximumPassiveTorqueAngleMultiplier:
        def __get__ (self):
            MS = TorqueMuscleSummary()
            MS.fiberAngle = self.thisptr.summaryDataAtMaximumPassiveTorqueAngleMultiplier.fiberAngle
            MS.fiberAngularVelocity = self.thisptr.summaryDataAtMaximumPassiveTorqueAngleMultiplier.fiberAngularVelocity
            MS.activation = self.thisptr.summaryDataAtMaximumPassiveTorqueAngleMultiplier.activation
            MS.fiberPassiveTorqueAngleMultiplier = self.thisptr.summaryDataAtMaximumPassiveTorqueAngleMultiplier.fiberPassiveTorqueAngleMultiplier
            MS.fiberActiveTorqueAngleMultiplier = self.thisptr.summaryDataAtMaximumPassiveTorqueAngleMultiplier.fiberActiveTorqueAngleMultiplier
            MS.fiberTorqueAngularVelocityMultiplier = self.thisptr.summaryDataAtMaximumPassiveTorqueAngleMultiplier.fiberTorqueAngularVelocityMultiplier
            MS.fiberNormalizedDampingTorque = self.thisptr.summaryDataAtMaximumPassiveTorqueAngleMultiplier.fiberNormalizedDampingTorque
            MS.fiberTorque = self.thisptr.summaryDataAtMaximumPassiveTorqueAngleMultiplier.fiberTorque
            MS.jointTorque = self.thisptr.summaryDataAtMaximumPassiveTorqueAngleMultiplier.jointTorque          
              
            return MS


IF "@RBDL_BUILD_ADDON_MUSCLE_FITTING@" == "ON":
    cdef class TorqueMuscleFittingToolkit:
        
        def fitTorqueMuscleParameters( self,
                        Millard2016TorqueMuscle tqMcl,
                        np.ndarray[double, ndim=1, mode="c"] jointAngle,
                        np.ndarray[double, ndim=1, mode="c"] jointAngularVelocity,
                        np.ndarray[double, ndim=1, mode="c"] jointTorque,
                        activationUpperBound,
                        passiveTorqueAngleMultiplierUpperBound,
                        TorqueMuscleParameterFittingData parametersOfBestFit,
                        verbose=False):
            crbdlmuscle.fitTorqueMuscleParameters(
                        tqMcl.thisptr[0],
                        NumpyToVectorNd(jointAngle),
                        NumpyToVectorNd(jointAngularVelocity),
                        NumpyToVectorNd(jointTorque),
                        activationUpperBound,
                        passiveTorqueAngleMultiplierUpperBound,
                        parametersOfBestFit.thisptr[0],
                        verbose)

        
    
    
    
