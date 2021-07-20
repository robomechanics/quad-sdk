#include <iostream>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"

#include "Human36Fixture.h"

#include "rbdl_tests.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-12;
const double TEST_LAX = 1.0e-7;

struct KinematicsFixture {
  KinematicsFixture () {
    ClearLogOutput();
    model = new Model;

    /* Basically a model like this, where X are the Center of Masses
     * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
     *
     *                X
     *                *
     *              _/
     *            _/  (-Z)
     *      Z    /
     *      *---* 
     *      |
     *      |
     *  Z   |
     *  O---*
     *      Y
     */

    body_a = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
    joint_a = Joint( SpatialVector (0., 0., 1., 0., 0., 0.));

    body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

    body_b = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
    joint_b = Joint ( SpatialVector (0., 1., 0., 0., 0., 0.));

    body_b_id = model->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b,
                               body_b);

    body_c = Body (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
    joint_c = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

    body_c_id = model->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c,
                               body_c);

    body_d = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
    joint_c = Joint ( SpatialVector (1., 0., 0., 0., 0., 0.));

    body_d_id = model->AddBody(body_c_id, Xtrans(Vector3d(0., 0., -1.)), joint_c,
                               body_d);

    Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
    QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
    Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

    ClearLogOutput();
  }

  ~KinematicsFixture () {
    delete model;
  }
  Model *model;

  unsigned int body_a_id, body_b_id, body_c_id, body_d_id;
  Body body_a, body_b, body_c, body_d;
  Joint joint_a, joint_b, joint_c, joint_d;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;
};

struct KinematicsFixture6DoF {
  KinematicsFixture6DoF () {
    ClearLogOutput();
    model = new Model;

    model->gravity = Vector3d  (0., -9.81, 0.);

    /* 
     *
     *          X Contact point (ref child)
     *          |
     *    Base  |
     *   / body |
     *  O-------*
     *           \
     *             Child body
     */

    // base body (3 DoF)
    base = Body (
        1.,
        Vector3d (0.5, 0., 0.),
        Vector3d (1., 1., 1.)
        );
    joint_rotzyx = Joint (
        SpatialVector (0., 0., 1., 0., 0., 0.),
        SpatialVector (0., 1., 0., 0., 0., 0.),
        SpatialVector (1., 0., 0., 0., 0., 0.)
        );
    base_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_rotzyx,
                              base);

    // child body (3 DoF)
    child = Body (
        1.,
        Vector3d (0., 0.5, 0.),
        Vector3d (1., 1., 1.)
        );
    child_id = model->AddBody (base_id, Xtrans (Vector3d (1., 0., 0.)),
                               joint_rotzyx, child);

    Q = VectorNd::Constant (model->mBodies.size() - 1, 0.);
    QDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
    QDDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
    Tau = VectorNd::Constant (model->mBodies.size() - 1, 0.);

    ClearLogOutput();
  }

  ~KinematicsFixture6DoF () {
    delete model;
  }
  Model *model;

  unsigned int base_id, child_id;
  Body base, child;
  Joint joint_rotzyx;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;
};



TEST_CASE_METHOD(KinematicsFixture, __FILE__"_TestPositionNeutral", "") {
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  Vector3d body_position;

  CHECK_THAT (Vector3d (0., 0., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_a_id,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (Vector3d (1., 0., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_b_id,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (Vector3d (1., 1., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_c_id,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (Vector3d (1., 1., -1.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_d_id,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD(KinematicsFixture, __FILE__"_TestPositionBaseRotated90Deg","") {
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices

  Q[0] = 0.5 * M_PI;
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  Vector3d body_position;

  //	cout << LogOutput.str() << endl;
  CHECK_THAT(Vector3d (0., 0., 0.),
             AllCloseVector(
               CalcBodyToBaseCoordinates(*model,
                                         Q,
                                         body_a_id,
                                         Vector3d (0., 0., 0.),
                                         true),
               TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(Vector3d (0., 1., 0.),
             AllCloseVector(
               CalcBodyToBaseCoordinates(*model,
                                         Q,
                                         body_b_id,
                                         Vector3d (0., 0., 0.),
                                         true),
               TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(Vector3d (-1., 1., 0.),
             AllCloseVector(
               CalcBodyToBaseCoordinates(*model,
                                         Q,
                                         body_c_id,
                                         Vector3d (0., 0., 0.),
                                         true),
               TEST_PREC, TEST_PREC)
    );
  CHECK_THAT(Vector3d (-1., 1., -1.),
           AllCloseVector(
             CalcBodyToBaseCoordinates(*model,
                                       Q,
                                       body_d_id,
                                       Vector3d (0., 0., 0.),
                                       true),
             TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD(KinematicsFixture,__FILE__"_TestPositionBaseRotatedNeg45Deg","")
{
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices

  Q[0] = -0.25 * M_PI;
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  Vector3d body_position;

  //	cout << LogOutput.str() << endl;
  CHECK_THAT (Vector3d (0., 0., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_a_id,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (Vector3d (0.707106781186547, -0.707106781186547, 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_b_id,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (Vector3d (sqrt(2.0), 0., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_c_id,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (Vector3d (sqrt(2.0), 0., -1.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_d_id,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD(KinematicsFixture,__FILE__"_TestPositionBodyBRotated90Deg","") {
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  Q[1] = 0.5 * M_PI;
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  Vector3d body_position;

  CHECK_THAT(Vector3d (0., 0., 0.),
             AllCloseVector(
               CalcBodyToBaseCoordinates(*model,
                                         Q,
                                         body_a_id,
                                         Vector3d (0., 0., 0.),
                                         true),
               TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(Vector3d (1., 0., 0.),
             AllCloseVector(
               CalcBodyToBaseCoordinates(*model,
                                         Q,
                                         body_b_id,
                                         Vector3d (0., 0., 0.),
                                         true),
               TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(Vector3d (1., 1., 0.),
             AllCloseVector(
               CalcBodyToBaseCoordinates(*model,
                                         Q,
                                         body_c_id,
                                         Vector3d (0., 0., 0.),
                                         true),
               TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(Vector3d (0., 1., 0.),
             AllCloseVector(
               CalcBodyToBaseCoordinates(*model,
                                         Q,
                                         body_d_id,
                                         Vector3d (0., 0., 0.),
                                         true),
               TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD(KinematicsFixture,
                 __FILE__"_TestPositionBodyBRotatedNeg45Deg", "") {
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  Q[1] = -0.25 * M_PI;
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  Vector3d body_position;

  CHECK_THAT (Vector3d (0., 0., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_a_id,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (Vector3d (1., 0., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_b_id,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (Vector3d (1., 1., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_c_id,
                                          Vector3d (0., 0., 0.),
                                          true),
                TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (Vector3d (1 + 0.707106781186547, 1., -0.707106781186547),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_d_id,
                                          Vector3d (0., 0., 0.),
                                          true),
              TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD(KinematicsFixture,
                 __FILE__"_TestCalcBodyToBaseCoordinates", "") {
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  CHECK_THAT (Vector3d (1., 2., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_c_id,
                                          Vector3d (0., 1., 0.)),
                TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD(KinematicsFixture,
                 __FILE__"_TestCalcBodyToBaseCoordinatesRotated", "") {
  Q[2] = 0.5 * M_PI;

  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  CHECK_THAT (Vector3d (1., 1., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_c_id,
                                          Vector3d (0., 0., 0.), false),
                TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(Vector3d (0., 1., 0.),
             AllCloseVector(
               CalcBodyToBaseCoordinates(*model,
                                         Q,
                                         body_c_id,
                                         Vector3d (0., 1., 0.), false),
               TEST_PREC, TEST_PREC)
  );

  // Rotate the other way round
  Q[2] = -0.5 * M_PI;

  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  CHECK_THAT (Vector3d (1., 1., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_c_id,
                                          Vector3d (0., 0., 0.), false),
                TEST_PREC, TEST_PREC)
  );

  CHECK_THAT (Vector3d (2., 1., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_c_id,
                                          Vector3d (0., 1., 0.), false),
                TEST_PREC, TEST_PREC)
  );

  // Rotate around the base
  Q[0] = 0.5 * M_PI;
  Q[2] = 0.;

  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  CHECK_THAT (Vector3d (-1., 1., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_c_id,
                                          Vector3d (0., 0., 0.), false),
                TEST_PREC, TEST_PREC)
  );

  CHECK_THAT (Vector3d (-2., 1., 0.),
              AllCloseVector(
                CalcBodyToBaseCoordinates(*model,
                                          Q,
                                          body_c_id,
                                          Vector3d (0., 1., 0.), false),
                TEST_PREC, TEST_PREC)
  );

  //	cout << LogOutput.str() << endl;
}

TEST_CASE(__FILE__"TestCalcPointJacobian", "") {
  Model model;
  Body base_body (1., Vector3d (0., 0., 0.), Vector3d (1., 1., 1.));

  unsigned int base_body_id = model.AddBody (0, SpatialTransform(), 
      Joint (
        SpatialVector (0., 0., 0., 1., 0., 0.),
        SpatialVector (0., 0., 0., 0., 1., 0.),
        SpatialVector (0., 0., 0., 0., 0., 1.),
        SpatialVector (0., 0., 1., 0., 0., 0.),
        SpatialVector (0., 1., 0., 0., 0., 0.),
        SpatialVector (1., 0., 0., 0., 0., 0.)
        ),
      base_body);

  VectorNd Q = VectorNd::Constant ((size_t) model.dof_count, 0.);
  VectorNd QDot = VectorNd::Constant ((size_t) model.dof_count, 0.);
  MatrixNd G = MatrixNd::Constant (3, model.dof_count, 0.);
  Vector3d point_position (1.1, 1.2, 2.1);
  Vector3d point_velocity_ref;
  Vector3d point_velocity;

  Q[0] = 1.1;
  Q[1] = 1.2;
  Q[2] = 1.3;
  Q[3] = 0.7;
  Q[4] = 0.8;
  Q[5] = 0.9;

  QDot[0] = -1.1;
  QDot[1] = 2.2;
  QDot[2] = 1.3;
  QDot[3] = -2.7;
  QDot[4] = 1.8;
  QDot[5] = -2.9;

  // Compute the reference velocity
  point_velocity_ref = CalcPointVelocity (model, Q, QDot, base_body_id,
                                          point_position);

  G.setZero();
  CalcPointJacobian (model, Q, base_body_id, point_position, G);

  point_velocity = G * QDot;

  CHECK_THAT (point_velocity_ref,
              AllCloseVector(point_velocity, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD(KinematicsFixture,
                 __FILE__"_TestInverseKinematicSimple", "") {
  std::vector<unsigned int> body_ids;
  std::vector<Vector3d> body_points;
  std::vector<Vector3d> target_pos;

  Q[0] = 0.2;
  Q[1] = 0.1;
  Q[2] = 0.1;

  VectorNd Qres = VectorNd::Zero ((size_t) model->dof_count);

  unsigned int body_id = body_d_id;
  Vector3d body_point = Vector3d (1., 0., 0.);
  Vector3d target (1.3, 0., 0.);

  body_ids.push_back (body_d_id);
  body_points.push_back (body_point);
  target_pos.push_back (target);

  ClearLogOutput();
  bool res = InverseKinematics (*model, Q, body_ids, body_points, target_pos,
                                Qres);
  //	cout << LogOutput.str() << endl;
  CHECK (true == res);

  UpdateKinematicsCustom (*model, &Qres, NULL, NULL);

  Vector3d effector;
  effector = CalcBodyToBaseCoordinates(*model, Qres, body_id, body_point, false);

  CHECK_THAT (target, AllCloseVector(effector, TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD(KinematicsFixture6DoF,
                 __FILE__"TestInverseKinematicUnreachable", "") {
  std::vector<unsigned int> body_ids;
  std::vector<Vector3d> body_points;
  std::vector<Vector3d> target_pos;

  Q[0] = 0.2;
  Q[1] = 0.1;
  Q[2] = 0.1;

  VectorNd Qres = VectorNd::Zero ((size_t) model->dof_count);

  unsigned int body_id = child_id;
  Vector3d body_point = Vector3d (1., 0., 0.);
  Vector3d target (2.2, 0., 0.);

  body_ids.push_back (body_id);
  body_points.push_back (body_point);
  target_pos.push_back (target);

  ClearLogOutput();
  bool res = InverseKinematics (*model, Q, body_ids, body_points,
                                target_pos, Qres, 1.0e-8, 0.9, 1000);
  //	cout << LogOutput.str() << endl;
  CHECK (true == res);

  UpdateKinematicsCustom (*model, &Qres, NULL, NULL);

  Vector3d effector;
  effector = CalcBodyToBaseCoordinates(*model, Qres, body_id, body_point, false);

  CHECK_THAT (Vector3d (2.0, 0., 0.),
              AllCloseVector(effector, TEST_LAX, TEST_LAX));
}

TEST_CASE_METHOD(KinematicsFixture6DoF,
                 __FILE__"TestInverseKinematicTwoPoints", "") {
  std::vector<unsigned int> body_ids;
  std::vector<Vector3d> body_points;
  std::vector<Vector3d> target_pos;

  Q[0] = 0.2;
  Q[1] = 0.1;
  Q[2] = 0.1;

  VectorNd Qres = VectorNd::Zero ((size_t) model->dof_count);

  unsigned int body_id = child_id;
  Vector3d body_point = Vector3d (1., 0., 0.);
  Vector3d target (2., 0., 0.);

  body_ids.push_back (body_id);
  body_points.push_back (body_point);
  target_pos.push_back (target);

  body_ids.push_back (base_id);
  body_points.push_back (Vector3d (0.6, 1.0, 0.));
  target_pos.push_back (Vector3d (0.5, 1.1, 0.));

  ClearLogOutput();
  bool res = InverseKinematics (*model, Q, body_ids, body_points,
                                target_pos, Qres, 1.0e-3, 0.9, 200);
  CHECK (true == res);

  //	cout << LogOutput.str() << endl;
  UpdateKinematicsCustom (*model, &Qres, NULL, NULL);

  Vector3d effector;

  // testing with very low precision
  effector = CalcBodyToBaseCoordinates(*model, Qres, body_ids[0],
                                       body_points[0], false);
  CHECK_THAT (target_pos[0], AllCloseVector(effector, 1.0e-1, 1.0e-1));

  effector = CalcBodyToBaseCoordinates(*model, Qres, body_ids[1],
                                       body_points[1], false);
  CHECK_THAT (target_pos[1], AllCloseVector(effector, 1.0e-1, 1.0e-1));
}

TEST_CASE ( __FILE__"_FixedJointBodyCalcBodyToBase", "" ) {
  // the standard modeling using a null body
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
  model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
  unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)),
                                                 Joint(JointTypeFixed),
                                                 fixed_body);

  VectorNd Q_zero = VectorNd::Zero (model.dof_count);
  Vector3d base_coords = CalcBodyToBaseCoordinates (model, Q_zero, fixed_body_id,
                                                    Vector3d (1., 1., 0.1));

  CHECK_THAT (Vector3d (1., 2., 0.1),
              AllCloseVector(base_coords, TEST_PREC, TEST_PREC));
}

TEST_CASE ( __FILE__"_FixedJointBodyCalcBodyToBaseRotated", "" ) {
  // the standard modeling using a null body
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector(0., 0., 1., 0., 0., 0.));
  model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
  unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(1., 0., 0.)),
                                                 Joint(JointTypeFixed),
                                                 fixed_body);

  VectorNd Q = VectorNd::Zero (model.dof_count);

  ClearLogOutput();
  Q[0] = M_PI * 0.5;
  Vector3d base_coords = CalcBodyToBaseCoordinates (model, Q, fixed_body_id,
                                                    Vector3d (1., 0., 0.));
  //	cout << LogOutput.str() << endl;	

  CHECK_THAT (Vector3d (0., 2., 0.),
              AllCloseVector(base_coords, TEST_PREC, TEST_PREC));
}

TEST_CASE ( __FILE__"_FixedJointBodyCalcBaseToBody", "" ) {
  // the standard modeling using a null body
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
  model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
  unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)),
                                                 Joint(JointTypeFixed),
                                                 fixed_body);

  VectorNd Q_zero = VectorNd::Zero (model.dof_count);
  Vector3d base_coords = CalcBaseToBodyCoordinates (model, Q_zero, fixed_body_id,
                                                    Vector3d (1., 2., 0.1));

  CHECK_THAT (Vector3d (1., 1., 0.1),
              AllCloseVector(base_coords, TEST_PREC, TEST_PREC));
}

TEST_CASE ( __FILE__"_FixedJointBodyCalcBaseToBodyRotated", "" ) {
  // the standard modeling using a null body
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
  model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
  unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(1., 0., 0.)),
                                                 Joint(JointTypeFixed),
                                                 fixed_body);

  VectorNd Q = VectorNd::Zero (model.dof_count);

  ClearLogOutput();
  Q[0] = M_PI * 0.5;
  Vector3d base_coords = CalcBaseToBodyCoordinates (model, Q, fixed_body_id,
                                                    Vector3d (0., 2., 0.));
  // cout << LogOutput.str() << endl;	

  CHECK_THAT (Vector3d (1., 0., 0.),
              AllCloseVector(base_coords, TEST_PREC, TEST_PREC));
}

TEST_CASE ( __FILE__"_FixedJointBodyWorldOrientation", "" ) {
  // the standard modeling using a null body
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
  model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);

  SpatialTransform transform = Xrotz(0.25) * Xtrans (Vector3d (1., 2., 3.));
  unsigned int fixed_body_id = model.AppendBody (transform,
                                                 Joint(JointTypeFixed),
                                                 fixed_body);

  VectorNd Q_zero = VectorNd::Zero (model.dof_count);
  Matrix3d orientation = CalcBodyWorldOrientation (model, Q_zero, fixed_body_id);

  Matrix3d reference = transform.E;

  CHECK_THAT (reference,
              AllCloseMatrix(orientation, TEST_PREC, TEST_PREC));
}

TEST_CASE ( __FILE__"_FixedJointCalcPointJacobian", "" ) {
  // the standard modeling using a null body
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
  model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);

  SpatialTransform transform = Xrotz(0.25) * Xtrans (Vector3d (1., 2., 3.));
  unsigned int fixed_body_id = model.AppendBody (transform,
                                                 Joint(JointTypeFixed),
                                                 fixed_body);

  VectorNd Q = VectorNd::Zero (model.dof_count);
  VectorNd QDot = VectorNd::Zero (model.dof_count);

  Q[0] = 1.1;
  QDot[0] = 1.2;

  Vector3d point_position (1., 0., 0.);

  MatrixNd G = MatrixNd::Zero (3, model.dof_count);	
  CalcPointJacobian (model, Q, fixed_body_id, point_position, G);
  Vector3d point_velocity_jacobian = G * QDot;
  Vector3d point_velocity_reference = CalcPointVelocity (model, Q, QDot,
                                                         fixed_body_id,
                                                         point_position);

  CHECK_THAT (point_velocity_reference,
              AllCloseVector(point_velocity_jacobian, TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD ( Human36,
                   __FILE__"_SpatialJacobianSimple", "" ) {
  randomizeStates();

  unsigned int foot_r_id = model->GetBodyId ("foot_r");
  MatrixNd G (MatrixNd::Zero (6, model->dof_count));

  CalcBodySpatialJacobian (*model, q, foot_r_id, G);

  UpdateKinematicsCustom (*model, &q, &qdot, NULL);
  SpatialVector v_body = SpatialVector(G * qdot);

  CHECK_THAT (model->v[foot_r_id], AllCloseVector(v_body, TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD ( Human36,
                   __FILE__"_SpatialJacobianFixedBody", "" ) {
  randomizeStates();

  unsigned int uppertrunk_id = model->GetBodyId ("uppertrunk");
  MatrixNd G (MatrixNd::Zero (6, model->dof_count));

  CalcBodySpatialJacobian (*model, q, uppertrunk_id, G);

  unsigned int fixed_body_id = uppertrunk_id - model->fixed_body_discriminator;
  unsigned int movable_parent = model->mFixedBodies[fixed_body_id]
                                       .mMovableParent;

  UpdateKinematicsCustom (*model, &q, &qdot, NULL);
  SpatialVector v_body = SpatialVector(G * qdot);

  auto v_fixed_body = model->mFixedBodies[fixed_body_id]
                             .mParentTransform.apply(model->v[movable_parent]);

  CHECK_THAT (v_fixed_body, AllCloseVector(v_body, TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD ( Human36,
                   __FILE__"_CalcPointJacobian6D", "" ) {
  randomizeStates();

  unsigned int foot_r_id = model->GetBodyId ("foot_r");
  Vector3d point_local (1.1, 2.2, 3.3);

  // Compute the 6-D velocity using the 6-D Jacobian
  MatrixNd G (MatrixNd::Zero (6, model->dof_count));
  CalcPointJacobian6D (*model, q, foot_r_id, point_local, G);
  SpatialVector v_foot_0_jac = SpatialVector (G * qdot);

  // Compute the 6-D velocity by transforming the body velocity to the
  // reference point and aligning it with the base coordinate system
  Vector3d r_point = CalcBodyToBaseCoordinates (*model, q, foot_r_id,
                                                point_local);
  SpatialTransform X_foot (Matrix3d::Identity(), r_point);
  UpdateKinematicsCustom (*model, &q, &qdot, NULL);
  SpatialVector v_foot_0_ref = X_foot.apply(model->X_base[foot_r_id]
                                                  .inverse()
                                                  .apply(model->v[foot_r_id]));

  CHECK_THAT (v_foot_0_ref, AllCloseVector(v_foot_0_jac, TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD ( Human36,
                   __FILE__"_CalcPointVelocity6D", "" ) {
  randomizeStates();

  unsigned int foot_r_id = model->GetBodyId ("foot_r");
  Vector3d point_local (1.1, 2.2, 3.3);

  // Compute the 6-D velocity 
  SpatialVector v_foot_0 = CalcPointVelocity6D (*model, q, qdot, foot_r_id,
                                                point_local);

  // Compute the 6-D velocity by transforming the body velocity to the
  // reference point and aligning it with the base coordinate system
  Vector3d r_point = CalcBodyToBaseCoordinates (*model, q, foot_r_id,
                                                point_local);
  SpatialTransform X_foot (Matrix3d::Identity(), r_point);
  UpdateKinematicsCustom (*model, &q, &qdot, NULL);
  SpatialVector v_foot_0_ref = X_foot.apply(model->X_base[foot_r_id]
                                                   .inverse()
                                                   .apply(model->v[foot_r_id]));

  CHECK_THAT (v_foot_0_ref, AllCloseVector(v_foot_0, TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD ( Human36,
                   __FILE__"_CalcPointAcceleration6D", "" ) {
  randomizeStates();

  unsigned int foot_r_id = model->GetBodyId ("foot_r");
  Vector3d point_local (1.1, 2.2, 3.3);

  // Compute the 6-D acceleration 
  SpatialVector a_foot_0 = CalcPointAcceleration6D (*model, q, qdot, qddot,
                                                    foot_r_id, point_local);

  // Compute the 6-D acceleration by adding the coriolis term to the
  // acceleration of the body and transforming the result to the
  // point and align it with the base coordinate system.
  Vector3d r_point = CalcBodyToBaseCoordinates (*model, q, foot_r_id,
                                                point_local);
  Vector3d v_foot_0 = CalcPointVelocity (*model, q, qdot, foot_r_id,
                                         point_local);
  SpatialVector rdot (0., 0., 0., v_foot_0[0], v_foot_0[1], v_foot_0[2]);

  SpatialTransform X_foot (Matrix3d::Identity(), r_point);
  UpdateKinematicsCustom (*model, &q, &qdot, NULL);
  SpatialVector a_foot_0_ref = X_foot.apply(
      model->X_base[foot_r_id].inverse().apply(model->a[foot_r_id]) 
      - crossm(rdot, 
        model->X_base[foot_r_id].inverse().apply(model->v[foot_r_id])
        )
      );

  CHECK_THAT (a_foot_0_ref, AllCloseVector(a_foot_0, TEST_PREC, TEST_PREC));
}
