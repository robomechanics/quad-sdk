#include <iostream>

#include "Fixtures.h"
#include "Human36Fixture.h"
#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Constraints.h"

#include "rbdl_tests.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-12;

struct CustomEulerZYXJoint : public CustomJoint {
  CustomEulerZYXJoint () {
    mDoFCount = 3;
    S = MatrixNd::Zero (6,3);
  };

  virtual void jcalc (Model &model,
    unsigned int joint_id,
    const Math::VectorNd &q,
    const Math::VectorNd &qdot
    ) {
    double q0 = q[model.mJoints[joint_id].q_index];
    double q1 = q[model.mJoints[joint_id].q_index + 1];
    double q2 = q[model.mJoints[joint_id].q_index + 2];

    double s0 = sin (q0);
    double c0 = cos (q0);
    double s1 = sin (q1);
    double c1 = cos (q1);
    double s2 = sin (q2);
    double c2 = cos (q2);

    model.X_J[joint_id].E = Matrix3d(
        c0 * c1, s0 * c1, -s1,
        c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
        c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2
        );

    S(0,0) = -s1;
    S(0,2) = 1.;

    S(1,0) = c1 * s2;
    S(1,1) = c2;

    S(2,0) = c1 * c2;
    S(2,1) = - s2;

    double qdot0 = qdot[model.mJoints[joint_id].q_index];
    double qdot1 = qdot[model.mJoints[joint_id].q_index + 1];
    double qdot2 = qdot[model.mJoints[joint_id].q_index + 2];

    model.v_J[joint_id] = S * Vector3d (qdot0, qdot1, qdot2);

    model.c_J[joint_id].set(
        - c1 * qdot0 * qdot1,
        -s1 * s2 * qdot0 * qdot1 + c1 * c2 * qdot0 * qdot2 - s2 * qdot1 * qdot2,
        -s1 * c2 * qdot0 * qdot1 - c1 * s2 * qdot0 * qdot2 - c2 * qdot1 * qdot2,
        0., 0., 0.
        );
  }
  virtual void jcalc_X_lambda_S (Model &model,
    unsigned int joint_id,
    const Math::VectorNd &q
    ) {
    // TODO
    assert (false && "Not yet implemented!");
  }
};

struct CustomJointFixture {
  CustomJointFixture () {
    custom_joint = new CustomEulerZYXJoint();

    Matrix3d inertia = Matrix3d::Identity(3,3);
    body = Body (1., Vector3d (1.1, 1.2, 1.3), inertia);
    reference_body_id = reference_model.AddBody (0,SpatialTransform(), Joint(JointTypeEulerZYX), body);
    custom_body_id = custom_model.AddBodyCustomJoint (0, SpatialTransform(), custom_joint, body);

    q = VectorNd::Zero (reference_model.q_size);    
    qdot = VectorNd::Zero (reference_model.qdot_size);    
    qddot = VectorNd::Zero (reference_model.qdot_size);   
    tau = VectorNd::Zero (reference_model.qdot_size);   
  }

  ~CustomJointFixture () {
    delete custom_joint;
  }

  Model reference_model;
  Model custom_model;

  Body body;
  CustomJoint* custom_joint;

  unsigned int reference_body_id;
  unsigned int custom_body_id;

  VectorNd q;
  VectorNd qdot;
  VectorNd qddot;
  VectorNd tau;
};

TEST_CASE_METHOD ( CustomJointFixture,
                   __FILE__"_UpdateKinematics", "") {
  for (unsigned int i = 0; i < 3; i++) {
    q[i] = i * 0.1;
    qdot[i] = i * 0.15;
    qddot[i] = i * 0.17;
  }

  UpdateKinematics (reference_model, q, qdot, qddot);
  UpdateKinematics (custom_model, q, qdot, qddot);

  CHECK_THAT (reference_model.X_base[reference_body_id].E,
              AllCloseMatrix(custom_model.X_base[custom_body_id].E, 0., 0.)
  );

  CHECK_THAT (reference_model.v[reference_body_id],
              AllCloseVector(custom_model.v[custom_body_id], 0., 0.)
  );

  CHECK_THAT (reference_model.a[reference_body_id],
              AllCloseVector(custom_model.a[custom_body_id], 0., 0.)
  );
}

// TODO: implement test for UpdateKinematicsCustom
// TODO: implement test for Jacobians
// TODO: implement test for InverseDynamics
// TODO: implement test for CompositeRigidBodyAlgorithm
// TODO: implement test for ForwardDynamics
// TODO: implement test for CalcMInvTimestau
// TODO: implement test for ForwardDynamicsContacts
