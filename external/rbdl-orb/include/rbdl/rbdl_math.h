/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_MATH_H
#define RBDL_MATH_H

#include "rbdl/rbdl_config.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/QR>

#include "rbdl/rbdl_eigenmath.h"

typedef Eigen::Matrix<double, 6, 3> Matrix63_t;
typedef Eigen::Matrix<double, 4, 3> Matrix43_t;

typedef Eigen::VectorXd VectorN_t;
typedef Eigen::MatrixXd MatrixN_t;

namespace RigidBodyDynamics {

/** \brief Math types such as vectors and matrices and utility functions. */
namespace Math {
typedef Vector2_t Vector2d;
typedef Vector3_t Vector3d;
typedef Vector4_t Vector4d;
typedef Matrix3_t Matrix3d;
typedef SpatialVector_t SpatialVector;
typedef SpatialMatrix_t SpatialMatrix;
typedef Matrix63_t Matrix63;
typedef Matrix43_t Matrix43;
typedef VectorN_t VectorNd;
typedef MatrixN_t MatrixNd;
} /* Math */

} /* RigidBodyDynamics */

#include "rbdl/Quaternion.h"
#include "rbdl/SpatialAlgebraOperators.h"

// If we use Eigen3 we have to create specializations of the STL
// std::vector such that the alignment is done properly.
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialVector)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialMatrix)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::Matrix63)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::Matrix43)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialTransform)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialRigidBodyInertia)

  /* RBDL_MATH_H_H */
#endif
