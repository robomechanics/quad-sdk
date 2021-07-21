#cython: boundscheck=False

from libcpp.string cimport string
from libcpp cimport bool
from libcpp.vector cimport vector

cimport numpy as np

cimport cpython.ref as cpy_ref

cimport crbdl

cdef crbdl.Vector2d NumpyToVector2d (np.ndarray[double, ndim=1, mode="c"] x)
cdef np.ndarray Vector2dToNumpy (crbdl.Vector2d cx)

cdef crbdl.Vector3d NumpyToVector3d (np.ndarray[double, ndim=1, mode="c"] x)
cdef np.ndarray Vector3dToNumpy (crbdl.Vector3d cx)

cdef crbdl.Matrix3d NumpyToMatrix3d (np.ndarray[double, ndim=2, mode="c"] M)
cdef np.ndarray Matrix3dToNumpy (crbdl.Matrix3d cM)

cdef crbdl.VectorNd NumpyToVectorNd (np.ndarray[double, ndim=1, mode="c"] x)
cdef np.ndarray VectorNdToNumpy (crbdl.VectorNd cx)

cdef crbdl.MatrixNd NumpyToMatrixNd (np.ndarray[double, ndim=2, mode="c"] M)
cdef np.ndarray MatrixNdToNumpy (crbdl.MatrixNd cM)

cdef crbdl.SpatialVector NumpyToSpatialVector (np.ndarray[double, ndim=1, mode="c"] cx)
cdef np.ndarray SpatialVectorToNumpy (crbdl.SpatialVector cx)

cdef crbdl.Quaternion NumpyToQuaternion (np.ndarray[double, ndim=1, mode="c"] x)
cdef np.ndarray QuaternionToNumpy (crbdl.Quaternion cx)





