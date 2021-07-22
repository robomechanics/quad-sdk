#include <iostream>
#include <cmath>

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

struct ScrewJoint1DofFixedBase {
  ScrewJoint1DofFixedBase() {
    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;

    ClearLogOutput();
    model = new Model;

    /* Single screw joint with a fixed base.  Rotation about Z, translation
     * along X. A rolling log.
     */

    Body body = Body (1., Vector3d (0., 0, 0.), Vector3d (1., 1., 1.));

    Joint joint = Joint (SpatialVector (0., 0., 1., 1., 0., 0.));

    roller = model->AppendBody (Xtrans (Vector3d (0., 0., 0.)), joint, body,
                                "Roller");

    q = VectorNd::Constant ((size_t) model->dof_count, 0.);
    qdot = VectorNd::Constant ((size_t) model->dof_count, 0.);
    qddot = VectorNd::Constant ((size_t) model->dof_count, 0.);
    tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

    epsilon = 1e-8;

    ClearLogOutput();
  }
  ~ScrewJoint1DofFixedBase() {
    delete model;
  }

  RigidBodyDynamics::Model *model;

  RigidBodyDynamics::Math::VectorNd q;
  RigidBodyDynamics::Math::VectorNd qdot;
  RigidBodyDynamics::Math::VectorNd qddot;
  RigidBodyDynamics::Math::VectorNd tau;

  unsigned int roller;
  double epsilon;
};


TEST_CASE_METHOD ( ScrewJoint1DofFixedBase,
                   __FILE__"_UpdateKinematics", "") {
  q[0] = 1;
  qdot[0] = 2;
  qddot[0] = 0;

  UpdateKinematics (*model, q, qdot, qddot);

  CHECK_THAT (Xrot(1,Vector3d(0,0,1)).E,
              AllCloseMatrix(model->X_base[roller].E, 0., 0.)
  );
  CHECK_THAT (Vector3d(1.,0.,0.),
              AllCloseVector(model->X_base[roller].r, 0., 0.)
  );
  CHECK_THAT (SpatialVector(0.,0.,2.,cos(q[0])*2,-sin(q[0])*2.,0.),
              AllCloseVector(model->v[roller], 0., 0.)
  );

  SpatialVector a0(model->a[roller]);
  SpatialVector v0(model->v[roller]);
  
  q[0] = 1+2*epsilon;
  qdot[0] = 2;
  qddot[0] = 0;

  UpdateKinematics (*model, q, qdot, qddot);

  v0 = model->v[roller] - v0;
  v0 /= epsilon;

  //finite diff vs. analytical derivative
  CHECK_THAT (a0,
              AllCloseVector(v0, 1e-5, 1e-5)
  );
}

TEST_CASE_METHOD ( ScrewJoint1DofFixedBase,
                   __FILE__"_Jacobians", "") {
  q[0] = 1;
  qdot[0] = 0;
  qddot[0] = 9;

  Vector3d refPt = Vector3d(1,0,3);
  MatrixNd GrefPt = MatrixNd::Constant(3,1,0.);
  MatrixNd Gexpected = MatrixNd::Constant(3,1,0.);
  Vector3d refPtBaseCoord = Vector3d();

  refPtBaseCoord = CalcBodyToBaseCoordinates(*model, q, roller, refPt);

  CHECK_THAT (Vector3d(1+cos(1), sin(1), 3),
              AllCloseVector(refPtBaseCoord, 0., 0.)
  );
  
  CalcPointJacobian(*model, q, roller, refPt, GrefPt);

  Gexpected(0,0) = 1 - sin(1);
  Gexpected(1,0) = cos(1);
  Gexpected(2,0) = 0;
  
  CHECK_THAT (Gexpected,
              AllCloseMatrix(GrefPt, 0., 0.)
  );
}
