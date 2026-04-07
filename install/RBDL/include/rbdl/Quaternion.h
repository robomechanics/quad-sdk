/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_QUATERNION_H
#define RBDL_QUATERNION_H

#include <cmath>
#include <assert.h>

namespace RigidBodyDynamics {

namespace Math {

/** \brief Quaternion that are used for \ref joint_singularities "singularity free" joints.
 *
 * order: x,y,z,w
 */
class Quaternion : public Vector4d {
  public:
    Quaternion () :
      Vector4d (0., 0., 0., 1.)
  {}
    Quaternion (const Vector4d &vec4) :
      Vector4d (vec4)
  {}
    Quaternion (Scalar x, Scalar y, Scalar z, Scalar w):
      Vector4d (x, y, z, w)
  {}
    Quaternion operator* (const double &s) const {
      return Quaternion (
          (*this)[0] * s,
          (*this)[1] * s,
          (*this)[2] * s,
          (*this)[3] * s
          );
    }
    /** This function is equivalent to multiplicate their corresponding rotation matrices */
    Quaternion operator* (const Quaternion &q) const {
      return Quaternion (
          (*this)[3] * q[0] + (*this)[0] * q[3] + (*this)[1] * q[2] - (*this)[2] * q[1],
          (*this)[3] * q[1] + (*this)[1] * q[3] + (*this)[2] * q[0] - (*this)[0] * q[2],
          (*this)[3] * q[2] + (*this)[2] * q[3] + (*this)[0] * q[1] - (*this)[1] * q[0],
          (*this)[3] * q[3] - (*this)[0] * q[0] - (*this)[1] * q[1] - (*this)[2] * q[2]
          );
    }
    Quaternion& operator*=(const Quaternion &q) {
      set (
          (*this)[3] * q[0] + (*this)[0] * q[3] + (*this)[1] * q[2] - (*this)[2] * q[1],
          (*this)[3] * q[1] + (*this)[1] * q[3] + (*this)[2] * q[0] - (*this)[0] * q[2],
          (*this)[3] * q[2] + (*this)[2] * q[3] + (*this)[0] * q[1] - (*this)[1] * q[0],
          (*this)[3] * q[3] - (*this)[0] * q[0] - (*this)[1] * q[1] - (*this)[2] * q[2]
          );
      return *this;
    }

    static Quaternion fromGLRotate (double angle, double x, double y, double z) {
      double st = std::sin (angle * M_PI / 360.);
      return Quaternion (
          st * x,
          st * y,
          st * z,
          std::cos (angle * M_PI / 360.)
          );
    }

    Quaternion slerp (double alpha, const Quaternion &quat) const {
      // check whether one of the two has 0 length
      Scalar s = std::sqrt (squaredNorm() * quat.squaredNorm());

      // division by 0.f is unhealthy
      #ifndef RBDL_USE_CASADI_MATH
        assert (s != 0.);
      #endif

      Scalar angle = acos (dot(quat) / s);
#ifndef RBDL_USE_CASADI_MATH
      if (angle == 0. || std::isnan(angle)) {
        return *this;
      }
      assert(!std::isnan(angle));
#endif

      Scalar d = 1. / std::sin (angle);
      Scalar p0 = std::sin ((1. - alpha) * angle);
      Scalar p1 = std::sin (alpha * angle);

#ifdef RBDL_USE_CASADI_MATH
      return Quaternion (casadi::MX::if_else(casadi::MX::lt(dot (quat), 0.),
                                 Quaternion( ((*this) * p0 - quat * p1) * d),
                                 Quaternion( ((*this) * p0 + quat * p1) * d)) );
#else
      if (dot (quat) < 0.) {
          return Quaternion( ((*this) * p0 - quat * p1) * d);
      } else {
          return Quaternion( ((*this) * p0 + quat * p1) * d);
      }
#endif
    }

    static Quaternion fromAxisAngle (const Vector3d &axis, Scalar angle_rad) {
      Scalar d = axis.norm();
      Scalar s2 = std::sin (angle_rad * 0.5) / d;
      return Quaternion (
          axis[0] * s2,
          axis[1] * s2,
          axis[2] * s2,
          std::cos(angle_rad * 0.5)
          );
    }

    static Quaternion fromMatrix (const Matrix3d &mat) {
      Scalar w = std::sqrt (1. + mat(0,0) + mat(1,1) + mat(2,2)) * 0.5;
      return Quaternion (
          (mat(1,2) - mat(2,1)) / (w * 4.),
          (mat(2,0) - mat(0,2)) / (w * 4.),
          (mat(0,1) - mat(1,0)) / (w * 4.),
          w);
    }

    static Quaternion fromZYXAngles (const Vector3d &zyx_angles) {
      return Quaternion::fromAxisAngle (Vector3d (0., 0., 1.), zyx_angles[0])
        * Quaternion::fromAxisAngle (Vector3d (0., 1., 0.), zyx_angles[1])
        * Quaternion::fromAxisAngle (Vector3d (1., 0., 0.), zyx_angles[2]); 
    }

    static Quaternion fromYXZAngles (const Vector3d &yxz_angles) {
      return Quaternion::fromAxisAngle (Vector3d (0., 1., 0.), yxz_angles[0])
        * Quaternion::fromAxisAngle (Vector3d (1., 0., 0.), yxz_angles[1])
        * Quaternion::fromAxisAngle (Vector3d (0., 0., 1.), yxz_angles[2]);
    }

    static Quaternion fromXYZAngles (const Vector3d &xyz_angles) {
      return Quaternion::fromAxisAngle (Vector3d (0., 0., 01.), xyz_angles[2]) 
        * Quaternion::fromAxisAngle (Vector3d (0., 1., 0.), xyz_angles[1])
        * Quaternion::fromAxisAngle (Vector3d (1., 0., 0.), xyz_angles[0]);
    }

    Matrix3d toMatrix() const {
      Scalar x = (*this)[0];
      Scalar y = (*this)[1];
      Scalar z = (*this)[2];
      Scalar w = (*this)[3];
      return Matrix3d (
          1 - 2*y*y - 2*z*z,
          2*x*y + 2*w*z,
          2*x*z - 2*w*y,

          2*x*y - 2*w*z,
          1 - 2*x*x - 2*z*z,
          2*y*z + 2*w*x,

          2*x*z + 2*w*y,
          2*y*z - 2*w*x,
          1 - 2*x*x - 2*y*y

          /*
             1 - 2*y*y - 2*z*z,
             2*x*y - 2*w*z,
             2*x*z + 2*w*y,

             2*x*y + 2*w*z,
             1 - 2*x*x - 2*z*z,
             2*y*z - 2*w*x,

             2*x*z - 2*w*y,
             2*y*z + 2*w*x,
             1 - 2*x*x - 2*y*y
             */
        );
    }

    Quaternion conjugate() const {
      return Quaternion (
          -(*this)[0],
          -(*this)[1],
          -(*this)[2],
          (*this)[3]);
    }

    Quaternion timeStep (const Vector3d &omega, double dt) {
      Scalar omega_norm = omega.norm();
      return Quaternion::fromAxisAngle (omega / omega_norm, dt * omega_norm) * (*this);
    }

    Vector3d rotate (const Vector3d &vec) const {
      Vector3d vn (vec);
      Quaternion vec_quat (vn[0], vn[1], vn[2], 0.f), res_quat;

      res_quat = vec_quat * (*this);
      res_quat = conjugate() * res_quat;

      return Vector3d (res_quat[0], res_quat[1], res_quat[2]);
    }

    /** \brief Converts a 3d angular velocity vector into a 4d derivative of the
    * components of the quaternion.
    * 
    * \param omega the angular velocity.
    *
    * \return a 4d vector containing the derivatives of the 4 components of the
    * quaternion corresponding to omega.
    *
    */
    Vector4d omegaToQDot(const Vector3d& omega) const {
      Math::Matrix43 m;
      m(0, 0) =  (*this)[3];   m(0, 1) = -(*this)[2];   m(0, 2) =  (*this)[1];
      m(1, 0) =  (*this)[2];   m(1, 1) =  (*this)[3];   m(1, 2) = -(*this)[0];
      m(2, 0) = -(*this)[1];   m(2, 1) =  (*this)[0];   m(2, 2) =  (*this)[3];
      m(3, 0) = -(*this)[0];   m(3, 1) = -(*this)[1];   m(3, 2) = -(*this)[2];
      return Quaternion(0.5 * m * omega);
    }
};

}

}

/* RBDL_QUATERNION_H */
#endif
