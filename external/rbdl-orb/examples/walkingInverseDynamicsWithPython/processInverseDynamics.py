import numpy as np 
from scipy import signal
import rbdl
import csv
from numpy import genfromtxt


import matplotlib.pyplot as plt

plotInputData     = True
plotOutput        = True

#Preprocessing flags
resampleFrequency = 100
# All of the input data is resampled to the above rate. This step is included
# so that data coming from different equipment, at different sample rates, 
# is properly handled.

filterFreq        = 7.5;
# The inverse-kinematics data is filtered using a 2nd order Butterworth filter
# in the forwards and backwards direction (so there is no phase introduced).
# This is the 3db frequency of


#Read in the model
# The model name convention follows the one in OpenSim:
# gait: model intended for walking simulations
#    9: DoF
#   12: Number of muscles. In this case torque muscles
model = rbdl.loadModel("gait912.lua", kwargs={"floating_base":True,"verbose":True})
print("DoF: ", model.q_size)
q_size    = model.q_size
qdot_size = model.qdot_size

#Read in the experimental data
qVIn  = genfromtxt("qIK.csv",delimiter=",") 
#Column  0: is time
#Columns 1 ,..., N: correspond to q0, ..., qN

fVIn  = genfromtxt("grf.ff", delimiter=",") #0th column is time
#Column          0: is time
#Columns  1, 2, 3: Right foot: CoP 
#Columns  4, 5, 6: Right foot: Ground Forces 
#Columns  7, 8, 9: Right foot: Ground torques (zero)
#Columns 10,11,12: Left foot : CoP 
#Columns 13,14,15: Left foot : Ground Forces 
#Columns 16,17,18: Left foot : Ground torques (zero)


fVInShape=fVIn.shape
print(fVInShape)
timeSpan  = np.max(qVIn[:,0])-np.min(qVIn[:,0])
n         = int(np.round(timeSpan*resampleFrequency))

#Take numerical derivatives of q to get estimates qDot (generalized velocities)
#and qDDot (generalized accelerations)
fV     = np.zeros(shape=(n,fVInShape[1]-1),dtype=float)
timeV  = np.zeros(shape=(n,1),dtype=float)
qV     = np.zeros(shape=(n,q_size),dtype=float)
qDotV  = np.zeros(shape=(n,q_size),dtype=float)
qDDotV = np.zeros(shape=(n,q_size),dtype=float)
tauV   = np.zeros(shape=(n,q_size),dtype=float)

#3a. The force and mocap data are interpolated to have a common sense of time.
for i in range(0,n):
  timeV[i,0]  = (i/(n-1))*timeSpan
  for j in range(0,model.q_size):
    qV[i,j] = np.interp(timeV[i,0],qVIn[:,0],qVIn[:,j+1])
  for j in range(0,fVInShape[1]-1):
    fV[i,j] = np.interp(timeV[i,0],fVIn[:,0],fVIn[:,j+1])


#3b. The q's are filtered using a low-pass 2nd order Butterworth filter 
#    (signal.butter) applied in the forwards and then backwards directions 
#    (filtfilt) so that no phase error is introduced

freq = resampleFrequency
wn   = freq*0.5;

b,a=signal.butter(2, filterFreq/wn, btype='low',analog=False,output='ba')


for i in range(0,model.q_size):
  qV[:,i]=signal.filtfilt(b,a,qV[:,i])
  #scipy complains about this form of indexing. I'm not sure how
  #to fix this.

#3e. The values for qdot and qddot are formed using numerical derivatives
for i in range(0,model.q_size):
  qDotV[:,i] = np.gradient(qV[:,i],timeV[:,0])

for i in range(0,model.q_size):
  qDDotV[:,i] = np.gradient(qDotV[:,i],timeV[:,0])


#Now we're ready to perform the inverse-dynamics analysis

#Working variables used to transform the recorded center-of-pressure (cop)
#and ground force recordings (grf) into a wrench resolved in the ROOT frame
copR      = np.zeros(shape=(3),dtype=float)
grfR      = np.zeros(shape=(3),dtype=float)
tqR       = np.zeros(shape=(3),dtype=float)
copL      = np.zeros(shape=(3),dtype=float)
grfL      = np.zeros(shape=(3),dtype=float)
tqL       = np.zeros(shape=(3),dtype=float)
fextR     = np.zeros(shape=(6),dtype=float)
fextL     = np.zeros(shape=(6),dtype=float)

q   = np.zeros(shape=(q_size),   dtype=float)
qd  = np.zeros(shape=(qdot_size),dtype=float)
qdd = np.zeros(shape=(qdot_size),dtype=float)
tau = np.zeros(shape=(qdot_size),dtype=float)
#for i in range(0,n):
idRightFoot = model.GetBodyId("Foot_R")
idLeftFoot  = model.GetBodyId("Foot_L")
nBodies     = len(model.mBodies)
fext = np.zeros(shape=(nBodies,6),dtype=float)



for i in range(0,n):
  #3f. The ground forces are resolved into wrenches in the ROOT frame  
  #Right foot: resolve cop & grf into a wrench in the ROOT frame
  for j in range(0,3):  
    copR[j]=fV[i,j] 
    grfR[j]=fV[i,j+3]
  tqR=np.cross(np.transpose(copR),np.transpose(grfR))
  for j in range(0,3):
    fextR[j  ] = tqR[j]
    fextR[j+3] = grfR[j]
  for j in range(0,3):
    copL[j]=fV[i,j+9]
    grfL[j]=fV[i,j+12]
  tqL=np.cross(np.transpose(copL),np.transpose(grfL))
  for j in range(0,3):
    fextL[j  ]= tqL[j]
    fextL[j+3]= grfL[j]
  #3g. The foot wrenches are applied to the appropriate id for each foot
  #    see above for the command to get idRightFoot and idLeftFoot
  for j in range(0,6):
    fext[idRightFoot,j] = fextR[j]
    fext[idLeftFoot,j]  = fextL[j]
  for j in range(0,q_size):
    q[j]=qV[i,j]
  for j in range(0,qdot_size):
    qd[j]=qDotV[i,j]
    qdd[j]=qDDotV[i,j]
  #3h. The inverse dynamics function in RBDL is called
  rbdl.InverseDynamics(model, q,qd,qdd,tau,fext)
  #3i. The generalized force vector tau is copied to a matrix
  for j in range(0,qdot_size):
    tauV[i,j]=tau[j]


#3j. Plots are generated of the input and output data 
if (plotInputData == True):
  i=0;
  m=np.ceil(np.sqrt(model.q_size))
  plt.rc('font',family='serif')
  plt.figure(figsize=(8,8))
  for i in range(0,model.q_size):
    plt.subplot(m,m,i+1)
    plt.plot(qVIn[:,0],qVIn[:,i+1],'m')
    plt.plot(timeV,qV[:,i],'k')  
    plt.xlabel('Time (s)')
    plt.title('Q'+str(i))
    plt.tight_layout()
    plt.grid(True)
    plt.box(False)
  plt.figure(figsize=(8,8))
  for i in range(0,model.q_size):
    plt.subplot(m,m,i+1)
    plt.plot(timeV,qDotV[:,i],'b')
    plt.xlabel('Time (s)')
    plt.title('QDot'+str(i))
    plt.tight_layout()
    plt.grid(True)  
    plt.box(False)
  plt.figure(figsize=(8,8))
  for i in range(0,model.q_size):
    plt.subplot(m,m,i+1)
    plt.plot(timeV,qDDotV[:,i],'r')  
    plt.xlabel('Time (s)')
    plt.title('QDDot'+str(i))
    plt.tight_layout()
    plt.grid(True)  
    plt.box(False)
  #Plot the ground forces
  plt.figure(figsize=(8,8))
  plt.subplot(2,1,1)
  plt.plot(timeV,fV[:,3],'b')
  plt.plot(timeV,fV[:,5],'c')    
  plt.xlabel('Time (s)')
  plt.title('Right Foot Ground Forces')
  plt.tight_layout()
  plt.grid(True)  
  plt.box(False)
  plt.subplot(2,1,2)
  plt.plot(timeV,fV[:,12],'b')
  plt.plot(timeV,fV[:,14],'c')    
  plt.xlabel('Time (s)')
  plt.title('Left Foot Ground Forces')
  plt.tight_layout()
  plt.grid(True)  
  plt.box(False)
  

#Plot the generalized forces
if(plotOutput==True):
  #Plot the computed generalized forces
  i=0;
  m=np.ceil(np.sqrt(model.q_size))
  plt.rc('font',family='serif')
  plt.figure(figsize=(8,8))
  for i in range(0,model.q_size):
    plt.subplot(m,m,i+1)
    plt.plot(timeV,tauV[:,i],'g')
    plt.xlabel('Time (s)')
    plt.title('Tau'+str(i))
    plt.tight_layout()
    plt.grid(True)
    plt.box(False)

if(plotOutput==True or plotInputData == True):
  plt.show()