#include <iostream>

#include "rbdl_tests.h"
#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Body.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

/* Tests whether the spatial inertia matches the one specified by its
 * parameters
 */
TEST_CASE (__FILE__"_TestComputeSpatialInertiaFromAbsoluteRadiiGyration", "" ) {
  Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

  Matrix3d inertia_C (
      1.4, 0., 0.,
      0., 2., 0., 
      0., 0., 3.);

  SpatialMatrix reference_inertia (
      4.843, -1.98, -2.145, 0, -1.43, 1.32,
      -1.98, 6.334, -1.716, 1.43, 0, -1.65,
      -2.145, -1.716, 7.059, -1.32, 1.65, 0,
      0, 1.43, -1.32, 1.1, 0, 0,
      -1.43, 0, 1.65, 0, 1.1, 0,
      1.32, -1.65, 0, 0, 0, 1.1
      );

  //	cout << LogOutput.str() << endl;

  auto body_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC
                                (body.mMass, body.mCenterOfMass, body.mInertia);

  CHECK_THAT(reference_inertia, AllCloseMatrix(body_rbi.toMatrix(),
                                               TEST_PREC, TEST_PREC));
  CHECK_THAT(inertia_C, AllCloseMatrix(body.mInertia,
                                       TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_TestBodyConstructorMassComInertia", "" ) {
  double mass = 1.1;
  Vector3d com (1.5, 1.2, 1.3);
  Matrix3d inertia_C (
      8.286, -3.96, -4.29,
      -3.96, 10.668, -3.432,
      -4.29, -3.432, 11.118
      );

  Body body (mass, com, inertia_C);

  SpatialMatrix reference_inertia (
      11.729, -5.94, -6.435, 0, -1.43, 1.32,
      -5.94, 15.002, -5.148, 1.43, 0, -1.65,
      -6.435, -5.148, 15.177, -1.32, 1.65, 0,
      0, 1.43, -1.32, 1.1, 0, 0,
      -1.43, 0, 1.65, 0, 1.1, 0,
      1.32, -1.65, 0, 0, 0, 1.1
      );

  auto body_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC
                                (body.mMass, body.mCenterOfMass, body.mInertia);

  CHECK_THAT(reference_inertia, AllCloseMatrix(body_rbi.toMatrix(),
                                               TEST_PREC, TEST_PREC));
  CHECK_THAT(inertia_C, AllCloseMatrix(body.mInertia,
                                       TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_TestBodyJoinNullbody", "" ) {
  ClearLogOutput();
  Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));
  Body nullbody (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));

  Body joined_body = body;
  joined_body.Join (Xtrans(Vector3d (0., 0., 0.)), nullbody);

  SpatialRigidBodyInertia body_rbi (body.mMass,
                                    body.mCenterOfMass,
                                    body.mInertia);
  SpatialRigidBodyInertia joined_body_rbi (joined_body.mMass,
                                           joined_body.mCenterOfMass,
                                           joined_body.mInertia);

  CHECK (1.1 == body.mMass);
  CHECK_THAT(body.mCenterOfMass, AllCloseVector(joined_body.mCenterOfMass,
                                                TEST_PREC,
                                                TEST_PREC));
  CHECK_THAT(body_rbi.toMatrix(), AllCloseMatrix(joined_body_rbi.toMatrix(),
                                                 TEST_PREC,
                                                 TEST_PREC));
}

TEST_CASE (__FILE__"_TestBodyJoinTwoBodies", "" ) {
  ClearLogOutput();
  Body body_a(1.1, Vector3d (-1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
  Body body_b(1.1, Vector3d (1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));

  Body body_joined (body_a);
  body_joined.Join (Xtrans(Vector3d (0., 0., 0.)), body_b);

  SpatialRigidBodyInertia body_joined_rbi =
    SpatialRigidBodyInertia::createFromMassComInertiaC(body_joined.mMass,
                                                       body_joined.mCenterOfMass,
                                                       body_joined.mInertia);

  SpatialMatrix reference_inertia (
      9.918, 0, 0, 0, -0, 2.86,
      0, 9.062, 0, 0, 0, -0,
      0, 0, 12.98, -2.86, 0, 0,
      0, 0, -2.86, 2.2, 0, 0,
      -0, 0, 0, 0, 2.2, 0,
      2.86, -0, 0, 0, 0, 2.2
      );

  CHECK (2.2 == body_joined.mMass);
  CHECK_THAT (Vector3d (0., 1.3, 0.),
              AllCloseVector(body_joined.mCenterOfMass, 0, 0));
  CHECK_THAT (reference_inertia,
              AllCloseMatrix(body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_TestBodyJoinTwoBodiesDisplaced", "" ) {
  ClearLogOutput();
  Body body_a(1.1, Vector3d (-1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
  Body body_b(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.2, 3.3));

  Body body_joined (body_a);
  body_joined.Join (Xtrans(Vector3d (1.1, 1.3, 0.)), body_b);

  SpatialRigidBodyInertia body_joined_rbi =
    SpatialRigidBodyInertia::createFromMassComInertiaC(body_joined.mMass,
                                                       body_joined.mCenterOfMass,
                                                       body_joined.mInertia);

  SpatialMatrix reference_inertia (
      9.918, 0, 0, 0, -0, 2.86,
      0, 9.062, 0, 0, 0, -0,
      0, 0, 12.98, -2.86, 0, 0,
      0, 0, -2.86, 2.2, 0, 0,
      -0, 0, 0, 0, 2.2, 0,
      2.86, -0, 0, 0, 0, 2.2
      );

  CHECK (2.2 == body_joined.mMass);
  CHECK_THAT (Vector3d (0., 1.3, 0.),
              AllCloseVector(body_joined.mCenterOfMass, 0, 0));
  CHECK_THAT (reference_inertia,
              AllCloseMatrix(body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_TestBodyJoinTwoBodiesRotated", "" ) {
  ClearLogOutput();
  Body body_a(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.2, 3.3));
  Body body_b(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.3, 3.2));

  Body body_joined (body_a);
  body_joined.Join (Xrotx(-M_PI*0.5), body_b);

  SpatialRigidBodyInertia body_joined_rbi (body_joined.mMass,
                                           body_joined.mCenterOfMass,
                                           body_joined.mInertia);

  SpatialMatrix reference_inertia (
      6.2, 0., 0., 0., 0., 0.,
      0., 6.4, 0., 0., 0., 0.,
      0., 0., 6.6, 0., 0., 0.,
      0., 0., 0., 2.2, 0., 0.,
      0., 0., 0., 0., 2.2, 0.,
      0., 0., 0., 0., 0., 2.2
      );

  CHECK (2.2 == body_joined.mMass);
  CHECK_THAT (Vector3d (0., 0., 0.),
              AllCloseVector(body_joined.mCenterOfMass, 0, 0));
  CHECK_THAT (reference_inertia,
              AllCloseMatrix(body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_TestBodyJoinTwoBodiesRotatedAndTranslated", "" ) {
  ClearLogOutput();
  Body body_a(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.2, 3.3));
  Body body_b(1.1, Vector3d (-1., 1., 0.), Vector3d (3.2, 3.1, 3.3));

  Body body_joined (body_a);
  body_joined.Join (Xrotz(M_PI*0.5) * Xtrans(Vector3d (1., 1., 0.)), body_b);

  SpatialRigidBodyInertia body_joined_rbi (body_joined.mMass,
                                           body_joined.mCenterOfMass,
                                           body_joined.mInertia);

  SpatialMatrix reference_inertia (
      6.2, 0., 0., 0., 0., 0.,
      0., 6.4, 0., 0., 0., 0.,
      0., 0., 6.6, 0., 0., 0.,
      0., 0., 0., 2.2, 0., 0.,
      0., 0., 0., 0., 2.2, 0.,
      0., 0., 0., 0., 0., 2.2
      );

  CHECK (2.2 == body_joined.mMass);
  CHECK_THAT (Vector3d (0., 0., 0.),
              AllCloseVector(body_joined.mCenterOfMass, TEST_PREC, TEST_PREC));
  CHECK_THAT (reference_inertia,
              AllCloseMatrix(body_joined_rbi.toMatrix(), TEST_PREC, TEST_PREC));
}

TEST_CASE(__FILE__"_TestBodyConstructorSpatialRigidBodyInertiaMultiplyMotion","")
{
  Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

  SpatialRigidBodyInertia rbi = SpatialRigidBodyInertia(
      body.mMass,
      body.mCenterOfMass * body.mMass,
      body.mInertia
      );

  SpatialVector mv (1.1, 1.2, 1.3, 1.4, 1.5, 1.6);
  SpatialVector fv_matrix = rbi.toMatrix() * mv;
  SpatialVector fv_rbi = rbi * mv;

  CHECK_THAT (fv_matrix, AllCloseVector(fv_rbi, TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_TestBodyConstructorSpatialRigidBodyInertia", "" ) {
  Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

  SpatialRigidBodyInertia rbi = SpatialRigidBodyInertia(
      body.mMass,
      body.mCenterOfMass * body.mMass,
      body.mInertia
      );
  SpatialMatrix spatial_inertia = rbi.toMatrix();

  CHECK_THAT (spatial_inertia,
              AllCloseMatrix(rbi.toMatrix(), TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_TestBodyConstructorCopySpatialRigidBodyInertia", "" ) {
  Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

  SpatialRigidBodyInertia rbi = SpatialRigidBodyInertia(
      body.mMass,
      body.mCenterOfMass * body.mMass,
      body.mInertia
      );

  SpatialRigidBodyInertia rbi_from_matrix;
  rbi_from_matrix.createFromMatrix (rbi.toMatrix());

  //	cout << "Spatial Inertia = " << endl << spatial_inertia << endl;
  //	cout << "rbi = " << endl << rbi.toMatrix() << endl;
  //	cout << "rbi.m = " << rbi.m << endl;
  //	cout << "rbi.h = " << rbi.h.transpose() << endl;
  //	cout << "rbi.I = " << endl << rbi.I << endl;

  CHECK_THAT (rbi.toMatrix(),
              AllCloseMatrix(rbi_from_matrix.toMatrix(), TEST_PREC, TEST_PREC));
}
