#include <iostream>

#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"

#include "rbdl_tests.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

struct InverseDynamicsFixture {
  InverseDynamicsFixture () {
    ClearLogOutput();
    model = new Model;
    model->gravity = Vector3d  (0., -9.81, 0.);
  }
  ~InverseDynamicsFixture () {
    delete model;
  }
  Model *model;
};

#ifndef USE_SLOW_SPATIAL_ALGEBRA
TEST_CASE_METHOD(InverseDynamicsFixture,
                 __FILE__"_TestInverseForwardDynamicsFloatingBase", "") {
  Body base_body(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));

  model->AddBody (0, SpatialTransform(), 
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
  VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);
  VectorNd TauInv = VectorNd::Constant ((size_t) model->dof_count, 0.);

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

  ForwardDynamics(*model, Q, QDot, Tau, QDDot);
  InverseDynamics(*model, Q, QDot, QDDot, TauInv);

  CHECK_THAT (Tau,
              AllCloseVector(TauInv, TEST_PREC, TEST_PREC)
  );
}
#endif
