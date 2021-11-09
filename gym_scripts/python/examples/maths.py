"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Gym API Math
------------
Examples of math operations available in the Gym API
Demonstrates conversion to numpy data types
"""


import numpy as np
import math
from isaacgym import gymapi
a = gymapi.Vec3(2, 3, 4)
b = gymapi.Vec3(0.5, 0.25, 0.125)
print("(%f, %f, %f)" % (a.x, a.y, a.z))

c = a + b
d = a - b
e = a * 2
f = a / 2.0
print("(%f, %f, %f)" % (c.x, c.y, c.z))
print("(%f, %f, %f)" % (d.x, d.y, d.z))
print("(%f, %f, %f)" % (e.x, e.y, e.z))
print("(%f, %f, %f)" % (f.x, f.y, f.z))
print()

# Quaternions from axis-angle
qx1 = gymapi.Quat.from_axis_angle(gymapi.Vec3(1, 0, 0), 0.5 * math.pi)
qy1 = gymapi.Quat.from_axis_angle(gymapi.Vec3(0, 1, 0), 0.5 * math.pi)
qz1 = gymapi.Quat.from_axis_angle(gymapi.Vec3(0, 0, 1), 0.5 * math.pi)
print("(%f, %f, %f, %f)" % (qx1.x, qx1.y, qx1.z, qx1.w))
print("(%f, %f, %f, %f)" % (qy1.x, qy1.y, qy1.z, qy1.w))
print("(%f, %f, %f, %f)" % (qz1.x, qz1.y, qz1.z, qz1.w))
print()

# Quaternions from roll, pitch, yaw
qx2 = gymapi.Quat.from_euler_zyx(0.5 * math.pi, 0, 0)
qy2 = gymapi.Quat.from_euler_zyx(0, 0.5 * math.pi, 0)
qz2 = gymapi.Quat.from_euler_zyx(0, 0, 0.5 * math.pi)
print("(%f, %f, %f, %f)" % (qx2.x, qx2.y, qx2.z, qx2.w))
print("(%f, %f, %f, %f)" % (qy2.x, qy2.y, qy2.z, qy2.w))
print("(%f, %f, %f, %f)" % (qz2.x, qz2.y, qz2.z, qz2.w))
print()

# Roll, pitch, yaw from quaternions
r1, p1, y1 = qx2.to_euler_zyx()
r2, p2, y2 = qy2.to_euler_zyx()
r3, p3, y3 = qz2.to_euler_zyx()
print("%f, %f, %f" % (r1, p1, y1))
print("%f, %f, %f" % (r2, p2, y2))
print("%f, %f, %f" % (r3, p3, y3))
print()

# Quaternion multiplication and normalization
qxy = (qx1 * qy2).normalize()

c = qx1.rotate(a)
d = qy1.rotate(a)
e = qxy.rotate(a)
print("(%f, %f, %f)" % (a.x, a.y, a.z))
print("(%f, %f, %f)" % (c.x, c.y, c.z))
print("(%f, %f, %f)" % (d.x, d.y, d.z))
print("(%f, %f, %f)" % (e.x, e.y, e.z))
print()

# Quaternion inverse
qx1_inv = qx1.inverse()
qx1_inv_inv = qx1_inv.inverse()
print(qx1)
print(qx1_inv)
print(qx1_inv_inv)
print()

# Vector magnitude and direction
length = a.length()
d = a.normalize()
print("%f" % length)
print("(%f, %f, %f)" % (d.x, d.y, d.z))
print()

# Dot and cross products
dp = a.dot(b)
cp = a.cross(b)
print("%f" % dp)
print("(%f, %f, %f)" % (cp.x, cp.y, cp.z))
print()

# Transforms
tx = gymapi.Transform(b, qx1)
c = tx.transform_point(a)
e = tx.transform_vector(d)
print("(%f, %f, %f)" % (c.x, c.y, c.z))
print("(%f, %f, %f)" % (e.x, e.y, e.z))
print()

# Inverse transforms
tx_inv = tx.inverse()
tx_inv_inv = tx_inv.inverse()
print(tx.p, tx.r)
print(tx_inv.p, tx_inv.r)
print(tx_inv_inv.p, tx_inv_inv.r)
print()

# Access gym data types as structured arrays
print(gymapi.Vec3.dtype)
print(gymapi.Quat.dtype)
print(gymapi.Transform.dtype)
print()

# Convert a gym data type to a numpy array
arr = np.zeros(2, dtype=gymapi.Transform.dtype)
print(arr)
print(arr['p'])
print(arr['r'])
print()

# Accesing components of translation and rotation from transform and writing to structured arrays
arr[0] = ((tx.p.x, tx.p.y, tx.p.z), (tx.r.x, tx.r.y, tx.r.z, tx.r.w))
arr['p'][1] = (1, 2, 3)
arr['r'][1] = (0, 0, 0, 1)
print(arr)
print(arr[0])
print(arr[1])
print()

# Reading from structured arrays
pose1 = gymapi.Transform.from_buffer(arr[0])
pose2 = gymapi.Transform.from_buffer(arr[1])
print("(%f, %f, %f) (%f, %f, %f, %f)" % (pose1.p.x, pose1.p.y, pose1.p.z,
                                         pose1.r.x, pose1.r.y, pose1.r.z, pose1.r.w))
print("(%f, %f, %f) (%f, %f, %f, %f)" % (pose2.p.x, pose2.p.y, pose2.p.z,
                                         pose2.r.x, pose2.r.y, pose2.r.z, pose2.r.w))
print()
