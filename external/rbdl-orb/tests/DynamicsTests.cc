#include <iostream>
#include <limits>

#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Constraints.h"

#include "rbdl_tests.h"

#include "Fixtures.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-13;

struct DynamicsFixture {
  DynamicsFixture () {
    ClearLogOutput();
    model = new Model;
    model->gravity = Vector3d (0., -9.81, 0.);
  }
  ~DynamicsFixture () {
    delete model;
  }
  Model *model;
};

TEST_CASE_METHOD(DynamicsFixture,
                 __FILE__"_TestCalcDynamicSingleChain", "") {
  Body body(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
  Joint joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body);

  // Initialization of the input vectors
  VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDot = VectorNd::Constant  ((size_t) model->dof_count, 0.);
  VectorNd QDDot = VectorNd::Constant  ((size_t) model->dof_count, 0.);
  VectorNd Tau = VectorNd::Constant  ((size_t) model->dof_count, 0.);

  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  unsigned int i;
  for (i = 0; i < QDDot.size(); i++) {
    LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
  }

  for (i = 0; i < model->a.size(); i++) {
    LOG << "a[" << i << "]     = " << model->a[i] << endl;
  }

  CHECK (-4.905 == QDDot[0]);
}

TEST_CASE_METHOD(DynamicsFixture,
                 __FILE__"_TestCalcDynamicSpatialInertiaSingleChain", "") {
  // This function checks the value for a non-trivial spatial inertia
  Body body(1., Vector3d (1.5, 1., 1.), Vector3d (1., 2., 3.));

  Joint joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body);

  // Initialization of the input vectors
  VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

  ForwardDynamics(*model, Q, QDot, Tau, QDDot);


  unsigned int i;
  for (i = 0; i < QDDot.size(); i++) {
    LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
  }

  for (i = 0; i < model->a.size(); i++) {
    LOG << "a[" << i << "]     = " << model->a[i] << endl;
  }

  CHECK (-2.3544 == QDDot[0]);
}

TEST_CASE_METHOD(DynamicsFixture,
                 __FILE__"_TestCalcDynamicDoubleChain", "") {
  Body body_a (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
  Joint joint_a ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

  Body body_b (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
  Joint joint_b ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

  // Initialization of the input vectors
  VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

//  cout << "--- Double Chain ---" << endl;

  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  unsigned int i;
  for (i = 0; i < QDDot.size(); i++) {
    LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
  }

  for (i = 0; i < model->a.size(); i++) {
    LOG << "a[" << i << "]     = " << model->a[i] << endl;
  }

  //  cout << LogOutput.str() << endl;
  
  CHECK_THAT (-5.88600000000000E+00, IsClose(QDDot[0], TEST_PREC, TEST_PREC));
  CHECK_THAT ( 3.92400000000000E+00, IsClose(QDDot[1], TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD(DynamicsFixture,
                 __FILE__"_TestCalcDynamicTripleChain", "") {
  Body body_a (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
  Joint joint_a ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

  Body body_b (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
  Joint joint_b ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

  Body body_c (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
  Joint joint_c ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(2, Xtrans(Vector3d(1., 0., 0.)), joint_c, body_c);

  // Initialization of the input vectors
  VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

  // cout << "--- Triple Chain ---" << endl;

  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  unsigned int i;
  for (i = 0; i < QDDot.size(); i++) {
    LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
  }

  for (i = 0; i < model->a.size(); i++) {
    LOG << "a[" << i << "]     = " << model->a[i] << endl;
  }

  // cout << LogOutput.str() << endl;

  CHECK_THAT (-6.03692307692308E+00, IsClose(QDDot[0], TEST_PREC, TEST_PREC));
  CHECK_THAT ( 3.77307692307692E+00, IsClose(QDDot[1], TEST_PREC, TEST_PREC));
  CHECK_THAT ( 1.50923076923077E+00, IsClose(QDDot[2], TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD(DynamicsFixture,
                 __FILE__"_TestCalcDynamicDoubleChain3D", "") {
  Body body_a (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
  Joint joint_a ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

  Body body_b (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
  Joint joint_b ( SpatialVector (0., 1., 0., 0., 0., 0.));

  model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

  // Initialization of the input vectors
  VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

  // cout << "--- Double Chain 3D ---" << endl;

  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  unsigned int i;
  for (i = 0; i < QDDot.size(); i++) {
    LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
  }

  for (i = 0; i < model->a.size(); i++) {
    LOG << "a[" << i << "]     = " << model->a[i] << endl;
  }

  // cout << LogOutput.str() << endl;

  CHECK_THAT (-3.92400000000000E+00, IsClose(QDDot[0], TEST_PREC, TEST_PREC));
  CHECK_THAT ( 0.00000000000000E+00, IsClose(QDDot[1], TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD(DynamicsFixture,
                 __FILE__"_TestCalcDynamicSimpleTree3D", "") {
  Body body_a (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
  Joint joint_a ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

  Body body_b1 (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
  Joint joint_b1 ( SpatialVector (0., 1., 0., 0., 0., 0.));

  model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b1, body_b1);

  Body body_c1 (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
  Joint joint_c1 ( SpatialVector (1., 0., 0., 0., 0., 0.));

  model->AddBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c1, body_c1);

  Body body_b2 (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
  Joint joint_b2 ( SpatialVector (0., 1., 0., 0., 0., 0.));

  model->AddBody(1, Xtrans(Vector3d(-0.5, 0., 0.)), joint_b2, body_b2);

  Body body_c2 (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
  Joint joint_c2 ( SpatialVector (1., 0., 0., 0., 0., 0.));

  model->AddBody(4, Xtrans(Vector3d(0., -0.5, 0.)), joint_c2, body_c2);

  // Initialization of the input vectors
  VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

  // cout << "--- SimpleTree ---" << endl;

  ForwardDynamics(*model, Q, QDot, Tau, QDDot);

  unsigned int i;
  for (i = 0; i < QDDot.size(); i++) {
    LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
  }

  for (i = 0; i < model->a.size(); i++) {
    LOG << "a[" << i << "]     = " << model->a[i] << endl;
  }

  // cout << LogOutput.str() << endl;

  CHECK_THAT (-1.60319066147860E+00, IsClose(QDDot[0], TEST_PREC, TEST_PREC));
  CHECK_THAT (-5.34396887159533E-01, IsClose(QDDot[1], TEST_PREC, TEST_PREC));
  CHECK_THAT ( 4.10340466926070E+00, IsClose(QDDot[2], TEST_PREC, TEST_PREC));
  CHECK_THAT ( 2.67198443579767E-01, IsClose(QDDot[3], TEST_PREC, TEST_PREC));
  CHECK_THAT ( 5.30579766536965E+00, IsClose(QDDot[4], TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_TestForwardDynamicsLagrangian", "") {
  Model model;
  Body base_body(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));

  model.AddBody (0, SpatialTransform(), 
      Joint (
        SpatialVector (0., 0., 0., 1., 0., 0.),
        SpatialVector (0., 0., 0., 0., 1., 0.),
        SpatialVector (0., 0., 0., 0., 0., 1.),
        SpatialVector (0., 0., 1., 0., 0., 0.),
        SpatialVector (0., 1., 0., 0., 0., 0.),
        SpatialVector (1., 0., 0., 0., 0., 0.)
        ),
      base_body);

  // Initialization of the input vectors
  VectorNd Q = VectorNd::Zero (model.dof_count);
  VectorNd QDot = VectorNd::Zero (model.dof_count);
  VectorNd Tau = VectorNd::Zero (model.dof_count);
  
  VectorNd QDDot_aba = VectorNd::Zero (model.dof_count);
  VectorNd QDDot_lagrangian = VectorNd::Zero (model.dof_count);

  Q[0] = 1.1;
  Q[1] = 1.2;
  Q[2] = 1.3;
  Q[3] = 0.1;
  Q[4] = 0.2;
  Q[5] = 0.3;

  QDot[0] = 1.1;
  QDot[1] = -1.2;
  QDot[2] = 1.3;
  QDot[3] = -0.1;
  QDot[4] = 0.2;
  QDot[5] = -0.3;

  Tau[0] = 2.1;
  Tau[1] = 2.2;
  Tau[2] = 2.3;
  Tau[3] = 1.1;
  Tau[4] = 1.2;
  Tau[5] = 1.3;

  ForwardDynamics(model, Q, QDot, Tau, QDDot_aba);
  ForwardDynamicsLagrangian(model, Q, QDot, Tau, QDDot_lagrangian);

  CHECK (QDDot_aba.size() == QDDot_lagrangian.size());
  CHECK_THAT (QDDot_aba,
              AllCloseVector(QDDot_lagrangian, TEST_PREC, TEST_PREC)
  );
}

/* 
 * A simple test for a model with 3 rotational dof. The reference value was
 * computed with Featherstones spatial_v1 code. This test was written
 * because my benchmark tool showed up inconsistencies, however this was
 * due to the missing gravity term. But as the test now works, I just leave
 * it here.
 */
TEST_CASE (__FILE__"_TestForwardDynamics3DoFModel", "") {
  Model model;

  model.gravity = Vector3d (0., -9.81, 0.);

  Body null_body (0., Vector3d(0., 0., 0.), Vector3d (0., 0., 0.));
  Body base_body (1., Vector3d(0., 0.5, 0.), Vector3d (1., 1., 1.));

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
  Joint joint_rot_y ( SpatialVector (0., 1., 0., 0., 0., 0.));
  Joint joint_rot_x ( SpatialVector (1., 0., 0., 0., 0., 0.));

  unsigned int base_id_rot_z, base_id_rot_y;

  // we can reuse both bodies and joints as they are copied
  base_id_rot_z = model.AddBody (0, Xtrans (Vector3d(0., 0., 0.)), joint_rot_z,
                                 null_body);
  base_id_rot_y = model.AddBody (base_id_rot_z, Xtrans (Vector3d(0., 0., 0.)),
                                 joint_rot_y, null_body);
  model.AddBody (base_id_rot_y, Xtrans (Vector3d(0., 0., 0.)), joint_rot_x,
                 base_body);

  // Initialization of the input vectors
  VectorNd Q = VectorNd::Constant ((size_t) model.dof_count, 0.);
  VectorNd QDot = VectorNd::Constant ((size_t) model.dof_count, 0.);
  VectorNd Tau = VectorNd::Constant ((size_t) model.dof_count, 0.);
  
  VectorNd QDDot = VectorNd::Constant ((size_t) model.dof_count, 0.);
  VectorNd QDDot_ref = VectorNd::Constant ((size_t) model.dof_count, 0.);

  Q[0] = 1.;

  ClearLogOutput();

  ForwardDynamics (model, Q, QDot, Tau, QDDot);

//  cout << LogOutput.str() << endl;

  QDDot_ref[0] = 3.301932144386186;
  
  CHECK_THAT (QDDot_ref,
              AllCloseVector(QDDot, TEST_PREC, TEST_PREC)
  );
}

/*
 * Another simple 3 dof model test which showed some problems when
 * computing forward dynamics with the Lagrangian formulation. A proplem
 * occured as the CRBA does not update the kinematics of the model, hence
 * invalid body transformations and joint axis were used in the CRBA.
 * Running the CRBA after the InverseDynamics calculation fixes this. This
 * test ensures that the error does not happen when calling ForwardLagrangian.
 */
TEST_CASE (__FILE__"_TestForwardDynamics3DoFModelLagrangian", "") {
  Model model;

  model.gravity = Vector3d (0., -9.81, 0.);

  Body null_body (0., Vector3d(0., 0., 0.), Vector3d (0., 0., 0.));
  Body base_body (1., Vector3d(0., 0.5, 0.), Vector3d (1., 1., 1.));

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
  Joint joint_rot_y ( SpatialVector (0., 1., 0., 0., 0., 0.));
  Joint joint_rot_x ( SpatialVector (1., 0., 0., 0., 0., 0.));

  unsigned int base_id_rot_z, base_id_rot_y;

  // we can reuse both bodies and joints as they are copied
  base_id_rot_z = model.AddBody (0, Xtrans (Vector3d(0., 0., 0.)),
                                 joint_rot_z, null_body);
  base_id_rot_y = model.AddBody (base_id_rot_z, Xtrans (Vector3d(0., 0., 0.)),
                                 joint_rot_y, null_body);
  model.AddBody (base_id_rot_y, Xtrans (Vector3d(0., 0., 0.)), joint_rot_x,
                 base_body);

  // Initialization of the input vectors
  VectorNd Q = VectorNd::Constant ((size_t) model.dof_count, 0.);
  VectorNd QDot = VectorNd::Constant ((size_t) model.dof_count, 0.);
  VectorNd Tau = VectorNd::Constant ((size_t) model.dof_count, 0.);
  
  VectorNd QDDot_ab = VectorNd::Constant ((size_t) model.dof_count, 0.);
  VectorNd QDDot_lagrangian = VectorNd::Constant ((size_t) model.dof_count, 0.);

  Q[1] = 1.;
  ClearLogOutput();

  Q[0] = 0.;
  Q[1] = 1.;
  Q[2] = 0.;
  ForwardDynamicsLagrangian (model, Q, QDot, Tau, QDDot_lagrangian);
  Q[0] = 0.;
  Q[1] = 0.;
  Q[2] = 1.;
  ForwardDynamicsLagrangian (model, Q, QDot, Tau, QDDot_lagrangian);
  ForwardDynamics (model, Q, QDot, Tau, QDDot_ab);

//  cout << QDDot_lagrangian << endl;
//  cout << LogOutput.str() << endl;

  CHECK_THAT (QDDot_ab,
              AllCloseVector(QDDot_lagrangian, TEST_PREC, TEST_PREC)
  );
}

/* 
 * This is a test for a model where I detected incosistencies between the
 * Lagragian method and the ABA.
 */
TEST_CASE (__FILE__"_TestForwardDynamicsTwoLegModelLagrangian", "") {
  Model *model = NULL;

  unsigned int hip_id,
               upper_leg_right_id,
               lower_leg_right_id,
               foot_right_id,
               upper_leg_left_id,
               lower_leg_left_id,
               foot_left_id;
  Body hip_body,
       upper_leg_right_body,
       lower_leg_right_body,
       foot_right_body,
       upper_leg_left_body,
       lower_leg_left_body,
       foot_left_body;

  Joint joint_rot_z, joint_rot_y, joint_rot_x;
  Joint joint_trans_z, joint_trans_y, joint_trans_x;

  ConstraintSet CS_right;
  ConstraintSet CS_left;
  ConstraintSet CS_both;

  model = new Model();

  model->gravity = Vector3d (0., -9.81, 0.);

  joint_rot_z = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));
  joint_rot_y = Joint ( SpatialVector (0., 1., 0., 0., 0., 0.));
  joint_rot_x = Joint ( SpatialVector (1., 0., 0., 0., 0., 0.));

  joint_trans_z = Joint ( SpatialVector (0., 0., 0., 0., 0., 1.));
  joint_trans_y = Joint ( SpatialVector (0., 0., 0., 0., 1., 0.));
  joint_trans_x = Joint ( SpatialVector (0., 0., 0., 1., 0., 0.));

  Body null_body (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));

  // hip
  hip_body = Body (1., Vector3d (0., 0., 0.), Vector3d (1., 1., 1.));

  // lateral right
  upper_leg_right_body = Body (1., Vector3d (0., -0.25, 0.),
                               Vector3d (1., 1., 1.));
  lower_leg_right_body = Body (1., Vector3d (0., -0.25, 0.),
                               Vector3d (1., 1., 1.));
  foot_right_body = Body (1., Vector3d (0.15, -0.1, 0.), Vector3d (1., 1., 1.));

  // lateral left
  upper_leg_left_body = Body (1., Vector3d (0., -0.25, 0.),
                              Vector3d (1., 1., 1.));
  lower_leg_left_body = Body (1., Vector3d (0., -0.25, 0.),
                              Vector3d (1., 1., 1.));
  foot_left_body = Body (1., Vector3d (0.15, -0.1, 0.), Vector3d (1., 1., 1.));

  // temporary value to store most recent body id
  unsigned int temp_id;

  // add hip to the model (planar, 3 DOF)
  temp_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_trans_x,
                            null_body);
  temp_id = model->AddBody (temp_id, Xtrans (Vector3d (0., 0., 0.)),
                            joint_trans_y, null_body);
  hip_id = model->AddBody (temp_id, Xtrans (Vector3d (0., 0., 0.)), joint_rot_z,
                           hip_body);

  //
  // right leg
  //

  // add right upper leg
  temp_id = model->AddBody (hip_id, Xtrans (Vector3d(0., 0., 0.)), joint_rot_z,
                            upper_leg_right_body);
  upper_leg_right_id = temp_id;

  // add the right lower leg (only one DOF)
  temp_id = model->AddBody (temp_id, Xtrans (Vector3d(0., -0.5, 0.)),
                            joint_rot_z, lower_leg_right_body);
  lower_leg_right_id = temp_id;

  // add the right foot (1 DOF)
  temp_id = model->AddBody (temp_id, Xtrans (Vector3d(0., -0.5, 0.)),
                            joint_rot_z, foot_right_body);
  foot_right_id = temp_id;

  //
  // left leg
  //

  // add left upper leg
  temp_id = model->AddBody (hip_id, Xtrans (Vector3d(0., 0., 0.)), joint_rot_z,
                            upper_leg_left_body);
  upper_leg_left_id = temp_id;

  // add the left lower leg (only one DOF)
  temp_id = model->AddBody (temp_id, Xtrans (Vector3d(0., -0.5, 0.)),
                            joint_rot_z, lower_leg_left_body);
  lower_leg_left_id = temp_id;

  // add the left foot (1 DOF)
  temp_id = model->AddBody (temp_id, Xtrans (Vector3d(0., -0.5, 0.)),
                            joint_rot_z, foot_left_body);
  foot_left_id = temp_id;

  LOG << "--- model created (" << model->dof_count << " DOF) ---" << endl;
  
  // contact data
  CS_right.AddContactConstraint(foot_right_id, Vector3d (0., 0., 0.),
                                Vector3d (1., 0., 0.), "foot_right_x");
  CS_right.AddContactConstraint(foot_right_id, Vector3d (0., 0., 0.),
                                Vector3d (0., 1., 0.), "foot_right_y");
  CS_right.AddContactConstraint(foot_right_id, Vector3d (0., 0., 0.),
                                Vector3d (0., 0., 1.), "foot_right_z");

  CS_left.AddContactConstraint(foot_left_id, Vector3d (0., 0., 0.),
                               Vector3d (1., 0., 0.), "foot_left_x");
  CS_left.AddContactConstraint(foot_left_id, Vector3d (0., 0., 0.),
                               Vector3d (0., 1., 0.), "foot_left_y");
  CS_left.AddContactConstraint(foot_left_id, Vector3d (0., 0., 0.),
                               Vector3d (0., 0., 1.), "foot_left_z");
  
  CS_both.AddContactConstraint(foot_right_id, Vector3d (0., 0., 0.),
                               Vector3d (1., 0., 0.), "foot_right_x");
  CS_both.AddContactConstraint(foot_right_id, Vector3d (0., 0., 0.),
                               Vector3d (0., 1., 0.), "foot_right_y");
  CS_both.AddContactConstraint(foot_right_id, Vector3d (0., 0., 0.),
                               Vector3d (0., 0., 1.), "foot_right_z");
  CS_both.AddContactConstraint(foot_left_id, Vector3d (0., 0., 0.),
                               Vector3d (1., 0., 0.), "foot_left_x");
  CS_both.AddContactConstraint(foot_left_id, Vector3d (0., 0., 0.),
                               Vector3d (0., 1., 0.), "foot_left_y");
  CS_both.AddContactConstraint(foot_left_id, Vector3d (0., 0., 0.),
                               Vector3d (0., 0., 1.), "foot_left_z");

  CS_right.Bind(*model);
  CS_left.Bind(*model);
  CS_both.Bind(*model);

  VectorNd Q(model->dof_count);
  VectorNd QDot(model->dof_count);
  VectorNd Tau(model->dof_count);
  VectorNd QDDot(model->dof_count);
  VectorNd QDDotABA(model->dof_count);

  Q[0] = 0.8;
  Q[1] = -7.76326e-06;
  Q[2] = -1.58205e-07;
  Q[3] = 1.57391e-07;
  Q[4] = -1.03029e-09;
  Q[5] = 7.92143e-08;
  Q[6] = 1.57391e-07;
  Q[7] = -1.03029e-09;
  Q[8] = 7.92143e-08;

  QDot[0] = -1.77845e-06;
  QDot[1] = -0.00905283;
  QDot[2] = -0.000184484;
  QDot[3] = 0.000183536;
  QDot[4] = -1.20144e-06;
  QDot[5] = 9.23727e-05;
  QDot[6] = 0.000183536;
  QDot[7] = -1.20144e-06;
  QDot[8] = 9.23727e-05;

  Tau[0] = 0;
  Tau[1] = 0;
  Tau[2] = 0;
  Tau[3] = 0.1;
  Tau[4] = 0.1;
  Tau[5] = 0.1;
  Tau[6] = 0.1;
  Tau[7] = 0.1;
  Tau[8] = 0.1;

  // QDDot =  6.31843e-07 -6.12442e-07  9.22595e-14   3.3712e-07  4.27368e-07 -7.91795e-07   3.3712e-07  4.27368e-07 -7.91795e-07
  // QDDAB = -0.00192794    -9.81419        -0.2    0.198972 -0.00130243    0.100141    0.198972 -0.00130243    0.100141

  ForwardDynamics (*model, Q, QDot, Tau, QDDotABA);
  ClearLogOutput();
  ForwardDynamicsLagrangian (*model, Q, QDot, Tau, QDDot);

//  cout << LogOutput.str() << endl;

  // run it again to make sure the calculations give the same results and
  // no invalid state information lingering in the model structure is being used
  ForwardDynamics (*model, Q, QDot, Tau, QDDotABA);
  ClearLogOutput();
  ForwardDynamicsLagrangian (*model, Q, QDot, Tau, QDDot);

  CHECK_THAT (QDDotABA,
              AllCloseVector(QDDot, TEST_PREC, TEST_PREC)
  );

  delete model;
}

TEST_CASE_METHOD(FixedAndMovableJoint,
                 __FILE__"_TestForwardDynamicsFixedJoint", "") {
  Q_fixed[0] = 1.1;
  Q_fixed[1] = 2.2;

  QDot_fixed[0] = -3.2;
  QDot_fixed[1] = -2.3;

  Tau_fixed[0] = 1.2;
  Tau_fixed[1] = 2.1;

  Q = CreateDofVectorFromReducedVector (Q_fixed);
  QDot = CreateDofVectorFromReducedVector (QDot_fixed);

  QDDot.setZero();

  InverseDynamics (*model_movable, Q, QDot, QDDot, C_movable);
  CompositeRigidBodyAlgorithm (*model_movable, Q, H_movable);

  H_fixed = CreateReducedInertiaMatrix (H_movable);

  C_fixed[0] = C_movable[0];
  C_fixed[1] = C_movable[2];

  VectorNd QDDot_fixed_emulate(2);
  CHECK (LinSolveGaussElimPivot (H_fixed, C_fixed * -1. + Tau_fixed,
                                 QDDot_fixed_emulate));

  ForwardDynamics (*model_fixed, Q_fixed, QDot_fixed, Tau_fixed, QDDot_fixed);

  CHECK_THAT (QDDot_fixed_emulate,
              AllCloseVector(QDDot_fixed, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD(FixedAndMovableJoint,
                 __FILE__"_TestInverseDynamicsFixedJoint", "") {
  Q_fixed[0] = 1.1;
  Q_fixed[1] = 2.2;

  QDot_fixed[0] = -3.2;
  QDot_fixed[1] = -2.3;

  QDDot_fixed[0] = 1.2;
  QDDot_fixed[1] = 2.1;

  Q = CreateDofVectorFromReducedVector (Q_fixed);
  QDot = CreateDofVectorFromReducedVector (QDot_fixed);
  QDDot = CreateDofVectorFromReducedVector (QDDot_fixed);

  InverseDynamics (*model_movable, Q, QDot, QDDot, Tau);
  InverseDynamics (*model_fixed, Q_fixed, QDot_fixed, QDDot_fixed, Tau_fixed);

  VectorNd Tau_2dof (2);
  Tau_2dof[0] = Tau[0];
  Tau_2dof[1] = Tau[2];

  CHECK_THAT (Tau_2dof,
              AllCloseVector(Tau_fixed, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD ( FloatingBase12DoF,
                   __FILE__"_TestForwardDynamicsLagrangianPrealloc", "") {
  for (unsigned int i = 0; i < model->dof_count; i++) {
    Q[i] = static_cast<double>(i + 1) * 0.1;
    QDot[i] = static_cast<double>(i + 1) * 1.1;
    Tau[i] = static_cast<double>(i + 1) * -1.2;
  }

  ForwardDynamicsLagrangian (*model,
      Q,
      QDot,
      Tau,
      QDDot,
      Math::LinearSolverPartialPivLU,
      NULL,
      NULL,
      NULL
      );

  MatrixNd H (MatrixNd::Zero(model->dof_count, model->dof_count));
  VectorNd C (VectorNd::Zero(model->dof_count));
  VectorNd QDDot_prealloc (VectorNd::Zero (model->dof_count));
  ForwardDynamicsLagrangian (*model,
      Q,
      QDot,
      Tau,
      QDDot_prealloc,
      Math::LinearSolverPartialPivLU,
      NULL,
      &H,
      &C
      );

  CHECK_THAT (QDDot,
              AllCloseVector(QDDot_prealloc, 0., 0.)
  );
}

TEST_CASE_METHOD ( FixedBase3DoF,
                   __FILE__"_SolveMInvTimesTau", "") {
  for (unsigned int i = 0; i < model->dof_count; i++) {
    Q[i] = rand() / static_cast<double>(RAND_MAX);
    Tau[i] = rand() / static_cast<double>(RAND_MAX);
  }

  MatrixNd M (MatrixNd::Zero(model->dof_count, model->dof_count));
  CompositeRigidBodyAlgorithm (*model, Q, M);

  VectorNd qddot_solve_llt = M.llt().solve(Tau);

  VectorNd qddot_minv (Q);
  CalcMInvTimesTau (*model, Q, Tau, qddot_minv);

  CHECK_THAT (qddot_solve_llt,
              AllCloseVector(qddot_minv, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD ( FixedBase3DoF,
                   __FILE__"_SolveMInvTimesTauReuse", "") {
  for (unsigned int i = 0; i < model->dof_count; i++) {
    Q[i] = rand() / static_cast<double>(RAND_MAX);
    Tau[i] = rand() / static_cast<double>(RAND_MAX);
  }

  MatrixNd M (MatrixNd::Zero(model->dof_count, model->dof_count));
  CompositeRigidBodyAlgorithm (*model, Q, M);

  VectorNd qddot_solve_llt = M.llt().solve(Tau);

  VectorNd qddot_minv (Q);
  CalcMInvTimesTau (*model, Q, Tau, qddot_minv);

  for (unsigned int j = 0; j < 1; j++) {
    for (unsigned int i = 0; i < model->dof_count; i++) {
      Tau[i] = rand() / static_cast<double>(RAND_MAX);
    }

    CompositeRigidBodyAlgorithm (*model, Q, M);
    qddot_solve_llt = M.llt().solve(Tau);

    CalcMInvTimesTau (*model, Q, Tau, qddot_minv, false);

    CHECK_THAT (qddot_solve_llt,
                AllCloseVector(qddot_minv, TEST_PREC, TEST_PREC)
    );
  }
}

TEST_CASE_METHOD ( FixedBase3DoF,
                   __FILE__"_SolveMInvTimesNonZeroQDotKinematicsUpdate", "") {
  for (unsigned int i = 0; i < model->dof_count; i++) {
    Q[i] = rand() / static_cast<double>(RAND_MAX);
    QDot[i] = rand() / static_cast<double>(RAND_MAX);
    Tau[i] = rand() / static_cast<double>(RAND_MAX);
  }

  UpdateKinematicsCustom(*model, &Q, &QDot, NULL);

  MatrixNd M (MatrixNd::Zero(model->dof_count, model->dof_count));
  CompositeRigidBodyAlgorithm (*model, Q, M);

  VectorNd qddot_solve_llt = M.llt().solve(Tau);

  VectorNd qddot_minv (Q);
  CalcMInvTimesTau (*model, Q, Tau, qddot_minv);

  CHECK_THAT (qddot_solve_llt,
              AllCloseVector(qddot_minv, TEST_PREC, TEST_PREC)
  );
}
