#include <iostream>

#include "Fixtures.h"
#include "Human36Fixture.h"
#include "rbdl/rbdl_mathutils.h"
#include "rbdl/rbdl_utils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"

#include "rbdl_tests.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

TEST_CASE_METHOD(FloatingBase12DoF,
                 __FILE__"_TestKineticEnergy", "") {
  VectorNd q = VectorNd::Zero(model->q_size);
  VectorNd qdot = VectorNd::Zero(model->q_size);

  for (unsigned int i = 0; i < q.size(); i++) {
    q[i] = 0.1 * i;
    qdot[i] = 0.3 * i;
  }

  MatrixNd H = MatrixNd::Zero (model->q_size, model->q_size);
  CompositeRigidBodyAlgorithm (*model, q, H, true);

  double kinetic_energy_ref = 0.5 * qdot.transpose() * H * qdot;
  double kinetic_energy = Utils::CalcKineticEnergy (*model, q, qdot);

  CHECK (kinetic_energy_ref == kinetic_energy);
}

TEST_CASE(__FILE__"_TestPotentialEnergy", "") {
  Model model;
  Matrix3d inertia = Matrix3d::Zero(3,3);
  Body body (0.5, Vector3d (0., 0., 0.), inertia);
  Joint joint (
      SpatialVector (0., 0., 0., 1., 0., 0.),
      SpatialVector (0., 0., 0., 0., 1., 0.),
      SpatialVector (0., 0., 0., 0., 0., 1.)
      );

  model.AppendBody (Xtrans (Vector3d::Zero()), joint, body);

  VectorNd q = VectorNd::Zero(model.q_size);
  double potential_energy_zero = Utils::CalcPotentialEnergy (model, q);
  CHECK (0. == potential_energy_zero);

  q[1] = 1.;
  double potential_energy_lifted = Utils::CalcPotentialEnergy (model, q);
  CHECK (4.905 == potential_energy_lifted);
}

TEST_CASE(__FILE__"_TestCOMSimple", "") {
  Model model;
  Matrix3d inertia = Matrix3d::Zero(3,3);
  Body body (123., Vector3d (0., 0., 0.), inertia);
  Joint joint (
      SpatialVector (0., 0., 0., 1., 0., 0.),
      SpatialVector (0., 0., 0., 0., 1., 0.),
      SpatialVector (0., 0., 0., 0., 0., 1.)
      );

  model.AppendBody (Xtrans (Vector3d::Zero()), joint, body);

  VectorNd q = VectorNd::Zero(model.q_size);
  VectorNd qdot = VectorNd::Zero(model.qdot_size);

  double mass;
  Vector3d com;
  Vector3d com_velocity;
  Utils::CalcCenterOfMass (model, q, qdot, NULL, mass, com, &com_velocity);

  CHECK (123. == mass);
  CHECK (Vector3d (0., 0., 0.) == com);
  CHECK (Vector3d (0., 0., 0.) == com_velocity);

  q[1] = 1.;
  Utils::CalcCenterOfMass (model, q, qdot, NULL, mass, com, &com_velocity);
  CHECK (Vector3d (0., 1., 0.) == com);
  CHECK (Vector3d (0., 0., 0.) == com_velocity);

  qdot[1] = 1.;
  Utils::CalcCenterOfMass (model, q, qdot, NULL, mass, com, &com_velocity);
  CHECK (Vector3d (0., 1., 0.) == com);
  CHECK (Vector3d (0., 1., 0.) == com_velocity);
}

TEST_CASE(__FILE__"_TestAngularMomentumSimple", "") {
  Model model;
  Matrix3d inertia = Matrix3d::Zero(3,3);
  inertia(0,0) = 1.1;
  inertia(1,1) = 2.2;
  inertia(2,2) = 3.3;

  Body body (0.5, Vector3d (1., 0., 0.), inertia);
  Joint joint (
      SpatialVector (1., 0., 0., 0., 0., 0.),
      SpatialVector (0., 1., 0., 0., 0., 0.),
      SpatialVector (0., 0., 1., 0., 0., 0.)
      );

  model.AppendBody (Xtrans (Vector3d(0., 0., 0.)), joint, body);

  VectorNd q = VectorNd::Zero(model.q_size);
  VectorNd qdot = VectorNd::Zero(model.qdot_size);

  double mass;
  Vector3d com;
  Vector3d angular_momentum;

  qdot << 1., 0., 0.;
  Utils::CalcCenterOfMass (model, q, qdot, NULL, mass, com, NULL, NULL,
                           &angular_momentum);
  CHECK (Vector3d (1.1, 0., 0.) == angular_momentum);

  qdot << 0., 1., 0.;
  Utils::CalcCenterOfMass (model, q, qdot, NULL, mass, com, NULL, NULL,
                           &angular_momentum);
  CHECK (Vector3d (0., 2.2, 0.) == angular_momentum);

  qdot << 0., 0., 1.;
  Utils::CalcCenterOfMass (model, q, qdot, NULL, mass, com, NULL, NULL,
                           &angular_momentum);
  CHECK (Vector3d (0., 0., 3.3) == angular_momentum);
}

TEST_CASE_METHOD (TwoArms12DoF,
                  __FILE__"_TestAngularMomentumSimple2", "") {
  double mass;
  Vector3d com;
  Vector3d angular_momentum;

  Utils::CalcCenterOfMass (*model, q, qdot, NULL, mass, com, NULL, NULL,
                           &angular_momentum);

  CHECK (Vector3d (0., 0., 0.) == angular_momentum);

  qdot[0] = 1.;
  qdot[1] = 2.;
  qdot[2] = 3.;

  Utils::CalcCenterOfMass (*model, q, qdot, NULL, mass, com, NULL, NULL,
                           &angular_momentum);

  // only a rough guess from test calculation
  CHECK_THAT (Vector3d (3.3, 2.54, 1.5),
              AllCloseVector(angular_momentum, 1.0e-1, 1.0e-1)
  );

  qdot[3] = -qdot[0];
  qdot[4] = -qdot[1];
  qdot[5] = -qdot[2];

  ClearLogOutput();
  Utils::CalcCenterOfMass (*model, q, qdot, NULL, mass, com, NULL, NULL,
                           &angular_momentum);

  CHECK (angular_momentum[0] == 0);
  CHECK (angular_momentum[1] < 0);
  CHECK (angular_momentum[2] == 0.);
}

template <typename T>
void TestCoMComputation (
  T &obj
) {

  VectorNd Q = VectorNd::Random (obj.model->dof_count);
  VectorNd QDot = VectorNd::Random (obj.model->dof_count);
  VectorNd QDDot = VectorNd::Random (obj.model->dof_count);

  // compute quantities directly from model
  double mass_expected = 0.0;

  UpdateKinematicsCustom(*obj.model, &Q, NULL, NULL);
  for (unsigned int i = 1; i < obj.model->mBodies.size(); i++) {
    // mass_expected += obj.model->I[i].m;
    mass_expected += obj.model->mBodies[i].mMass;
  }

  double mass_actual = 0.0;
  Vector3d com = Vector3d::Zero();
  Utils::CalcCenterOfMass (
    *obj.model,
    Q, QDot, NULL,
    mass_actual, com,
    NULL, NULL
  );

  CHECK_THAT (mass_expected, IsClose(mass_actual, 1e-7, 1e-7));

  return;
}

TEST_CASE_METHOD( LinearInvertedPendulumModel,
                  __FILE__"_TestCoMComputationLinearInvertedPendulumModel", "")
{
  TestCoMComputation (*this);
}

TEST_CASE_METHOD(FixedJoint2DoF,
                 __FILE__"_TestCoMComputationFixedJoint2DoF", "")
{
  TestCoMComputation (*this);
}

TEST_CASE_METHOD(FixedBase6DoF12DoFFloatingBase,
                 __FILE__"_TestCoMComputationFixedBase6DoF12DoFFloatingBase", "")
{
  TestCoMComputation (*this);
}

TEST_CASE_METHOD(Human36,
                 __FILE__"_TestCoMComputationHuman36", "")
{
  TestCoMComputation (*this);
}

template <typename T>
void TestCoMAccelerationUsingFD (
  T & obj,
  const double TOL = 1e-8
) {
  const double EPS = 1e-8;

  obj.Q = VectorNd::Random (obj.model->dof_count);
  obj.QDot = VectorNd::Random (obj.model->dof_count);
  obj.QDDot = VectorNd::Random (obj.model->dof_count);

  double mass = 0.0;
  Vector3d com (Vector3d::Zero());
  Vector3d com_vec (Vector3d::Zero());
  Vector3d ang_mom (Vector3d::Zero());

  Vector3d com_acc_nom (Vector3d::Zero());
  Vector3d com_acc_fd (Vector3d::Zero());

  Vector3d ch_ang_mom_nom (Vector3d::Zero());
  Vector3d ch_ang_mom_fd (Vector3d::Zero());

  // compute com acceleration nominal
  Utils::CalcCenterOfMass (
    *obj.model,
    obj.Q, obj.QDot, &obj.QDDot,
    mass,
    com, &com_vec, &com_acc_nom,
    &ang_mom, &ch_ang_mom_nom
  );

  // compute com acceleration using finite differences from velocity
  Utils::CalcCenterOfMass (
    *obj.model,
    obj.Q + EPS*obj.QDot,
    obj.QDot + EPS*obj.QDDot,
    NULL,
    mass, com, &com_acc_fd, NULL,
    &ch_ang_mom_fd
  );

  com_acc_fd = (com_acc_fd - com_vec) / EPS;
  ch_ang_mom_fd = (ch_ang_mom_fd - ang_mom) / EPS;

  // check CoM acceleration
  CHECK_THAT (com_acc_nom, AllCloseVector(com_acc_fd, TOL, TOL));
  CHECK_THAT (ch_ang_mom_nom, AllCloseVector(ch_ang_mom_fd, TOL, TOL));

  return;
}

TEST_CASE_METHOD(LinearInvertedPendulumModel,
                __FILE__"_TestCoMAccelerationUsingFDLinearInvertedPendulumModel",
                 "")
{
  TestCoMAccelerationUsingFD (*this, 1e-8);
}

TEST_CASE_METHOD(FixedJoint2DoF,
                 __FILE__"_TestCoMAccelerationUsingFDFixedJoint2DoF", "")
{
  TestCoMAccelerationUsingFD (*this, 1e-7);
}

TEST_CASE_METHOD(FixedBase6DoF12DoFFloatingBase,
            __FILE__"_TestCoMAccelerationUsingFDFixedBase6DoF12DoFFloatingBase",
                 "")
{
  TestCoMAccelerationUsingFD (*this, 1e-6);
}

template <typename T>
void TestZMPComputationForNotMovingSystem(
  T & obj,
  const double TOL = 1e-8
) {
  // Test ZMP against CoM projection for non-moving system (qdot, qddot = 0)
  // for this configurations CoM and ZMP coincide

  obj.Q = VectorNd::Random (obj.model->dof_count);
  obj.QDot = VectorNd::Zero (obj.model->dof_count);
  obj.QDDot = VectorNd::Zero (obj.model->dof_count);

  Vector3d zmp (Vector3d::Zero());
  Utils::CalcZeroMomentPoint (
    *obj.model,
    obj.Q, obj.QDot, obj.QDDot,
    &zmp,
    obj.contact_normal, obj.contact_point
  );

  double mass = 0.0;
  Vector3d com (Vector3d::Zero());
  Utils::CalcCenterOfMass (
    *obj.model,
    obj.Q, obj.QDot, NULL,
    mass, com, NULL, NULL
  );

  // project CoM onto surface
  double distance = (com - obj.contact_point).dot(obj.contact_normal);
  com = com - distance * obj.contact_normal;

  // check ZMP against CoM
  CHECK_THAT (com, AllCloseVector(zmp, TOL, TOL));

  return;
}

TEST_CASE_METHOD(LinearInvertedPendulumModel,
      __FILE__"_TestZMPComputationForNotMovingSystemLinearInvertedPendulumModel",
                 "")
{
  TestZMPComputationForNotMovingSystem (*this, 1e-8);
}

template <typename T>
void TestZMPComputationAgainstTableCartModel(
  T & obj,
  const double TOL = 1e-8
) {
  obj.Q = VectorNd::Random (obj.model->dof_count);
  obj.QDot = VectorNd::Random (obj.model->dof_count);
  obj.QDDot = VectorNd::Random (obj.model->dof_count);

  Vector3d zmp (Vector3d::Zero());
  Utils::CalcZeroMomentPoint (
    *obj.model,
    obj.Q, obj.QDot, obj.QDDot,
    &zmp,
    obj.contact_normal, obj.contact_point
  );

  double mass = 0.0;
  Vector3d com (Vector3d::Zero());
  Utils::CalcCenterOfMass (
    *obj.model,
    obj.Q, obj.QDot, NULL,
    mass, com, NULL, NULL
  );

  com.set(
    obj.Q[0] - com[2]/obj.model->gravity.norm()*obj.QDDot[0],
    obj.Q[1] - com[2]/obj.model->gravity.norm()*obj.QDDot[1],
    0.
  );

  // check ZMP against CoM
  CHECK_THAT (com, AllCloseVector(zmp, TOL, TOL));

  return;
}

TEST_CASE_METHOD(LinearInvertedPendulumModel,
   __FILE__"_TestZMPComputationAgainstTableCartModelLinearInvertedPendulumModel",
                 "")
{
  TestZMPComputationAgainstTableCartModel (*this, 1e-8);
}
