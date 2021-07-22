import numpy as np
import rbdl

# Create a new model
model = rbdl.Model()

# Create a joint from joint type
joint_rot_y = rbdl.Joint.fromJointType ("JointTypeRevoluteY")

# Create a body for given mass, center of mass, and inertia at
# the CoM
body = rbdl.Body.fromMassComInertia (
    1., 
    np.array([0., 0.5, 0.]),
    np.eye(3) * 0.05)
xtrans= rbdl.SpatialTransform()
xtrans.r = np.array([0., 1., 0.])

# You can print all types
print (joint_rot_y)
print (model)
print (body)
print (body.mInertia)
print (xtrans)

# Construct the model
body_1 = model.AppendBody (rbdl.SpatialTransform(), joint_rot_y, body)
body_2 = model.AppendBody (xtrans, joint_rot_y, body)
body_3 = model.AppendBody (xtrans, joint_rot_y, body)

# Create numpy arrays for the state
q = np.zeros (model.q_size)
qdot = np.zeros (model.qdot_size)
qddot = np.zeros (model.qdot_size)
tau = np.zeros (model.qdot_size)

# Modify the state
q[0] = 1.3
q[1] = -0.5
q[2] = 3.2

# Transform coordinates from local to global coordinates
point_local = np.array([1., 2., 3.])
point_base = rbdl.CalcBodyToBaseCoordinates (model, q, body_3, point_local)
point_local_2 = rbdl.CalcBaseToBodyCoordinates (model, q, body_3, point_base)

# Perform forward dynamics and print the result
rbdl.ForwardDynamics (model, q, qdot, tau, qddot)
print ("qddot = " + str(qddot.transpose()))

# Compute and print the jacobian (note: the output parameter
# of the Jacobian G must have proper size!)
G = np.zeros([3,model.qdot_size])
rbdl.CalcPointJacobian (model, q, body_3, point_local, G)
print ("G = \n" + str(G))
