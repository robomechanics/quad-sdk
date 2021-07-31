#include <iostream>

#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Constraints.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Kinematics.h"

#include "rbdl_tests.h"

#include "Fixtures.h"
#include "Human36Fixture.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-11;

struct FixedBase6DoF9DoF {
  FixedBase6DoF9DoF () {
    ClearLogOutput();
    model = new Model;

    model->gravity = Vector3d  (0., -9.81, 0.);

    /* 3 DoF (rot.) joint at base
     * 3 DoF (rot.) joint child origin
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

    // child body 1 (3 DoF)
    child = Body (
        1.,
        Vector3d (0., 0.5, 0.),
        Vector3d (1., 1., 1.)
        );
    child_id = model->AddBody (base_id, Xtrans (Vector3d (0., 0., 0.)),
                               joint_rotzyx, child);

    // child body (3 DoF)
    child_2 = Body (
        1.,
        Vector3d (0., 0.5, 0.),
        Vector3d (1., 1., 1.)
        );
    child_2_id = model->AddBody (child_id, Xtrans (Vector3d (0., 0., 0.)),
                                 joint_rotzyx, child_2);

    Q = VectorNd::Constant (model->mBodies.size() - 1, 0.);
    QDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
    QDDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
    Tau = VectorNd::Constant (model->mBodies.size() - 1, 0.);

    contact_body_id = child_id;
    contact_point = Vector3d  (0.5, 0.5, 0.);
    contact_normal = Vector3d  (0., 1., 0.);

    ClearLogOutput();
  }
  
  ~FixedBase6DoF9DoF () {
    delete model;
  }
  Model *model;

  unsigned int base_id, child_id, child_2_id;

  Body base, child, child_2;

  Joint joint_rotzyx;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;

  unsigned int contact_body_id;
  Vector3d contact_point;
  Vector3d contact_normal;
  ConstraintSet constraint_set;
};

TEST_CASE( __FILE__"_TestExtendedConstraintFunctionsContact ", ""){
  //Make a simple system for which we know the constraint forces
  //by construction and use this to test the newly added generic
  //functions to compute contraint forces, position errors, velocity errors
  //and Baumgarte forces

  Model model;
  model.gravity = Vector3d  (0., -9.81, 0.);
  double m1=1.;
  Body boxBody (m1, Vector3d (0., 0., 0.), Matrix3dIdentity);
  unsigned int boxId = model.AddBody (0, SpatialTransform(),
      Joint (
        SpatialVector (0., 0., 0., 1., 0., 0.),
        SpatialVector (0., 0., 0., 0., 1., 0.),
        SpatialVector (0., 0., 1., 0., 0., 0.)),
      boxBody);

  ConstraintSet cs;
  cs.AddContactConstraint(boxId,Vector3d(-0.5,0,0),Vector3d(0.,1.,0.),
                          "LeftCorner");
  cs.AddContactConstraint(boxId,Vector3d(-0.5,0,0),Vector3d(1.,0.,0.));
  cs.AddContactConstraint(boxId,Vector3d( 0.5,0,0),Vector3d(0.,1.,0.),
                          "RightCorner");
  cs.Bind(model);

  //VectorNd qInit  =  VectorNd::Zero(model.dof_count);
  //qInit[2] = M_PI/3.0;
  VectorNd qdInit = VectorNd::Zero(model.dof_count);
  VectorNd tau    = VectorNd::Zero(model.dof_count);

  VectorNd q =  VectorNd::Zero(model.dof_count);
  q[2] = M_PI/3.0;
  VectorNd qd = VectorNd::Zero(model.dof_count);
  VectorNd qdd = VectorNd::Zero(model.dof_count);

  VectorNd weights = VectorNd::Constant(model.dof_count,1.);

  //CalcAssemblyQ(model,qInit,cs,q,weights); ContactConstraints not defined at position level  
  CalcAssemblyQDot(model,q,qdInit,cs,qd,weights);

  ForwardDynamicsConstraintsDirect(model,q,qd,tau,cs,qdd);

  std::vector< unsigned int > bodyIds;
  std::vector< SpatialTransform > bodyFrames;
  std::vector< SpatialVector > constraintForces;

  VectorNd posErrors, velErrors, bgForces;

  unsigned int gIdxLeft = cs.getGroupIndexByName("LeftCorner");
  unsigned int gIdxRight = cs.getGroupIndexByName("RightCorner");


  CHECK(cs.getGroupSize(0) == 2);
  CHECK(cs.getGroupSize(1) == 1);
  CHECK(cs.getGroupType(0) == ConstraintTypeContact);

  // New functions to test
  //    calcForces
  //    calcPositionError
  //    calcVelocityError
  //    calcBaumgarteStabilizationForces
  //    isBaumgarteStabilizationEnabled
  //    getBaumgarteStabilizationCoefficients
  //
  //    and finally
  //    calcImpulses

  cs.calcForces(gIdxLeft,model,q,qd,bodyIds,bodyFrames,constraintForces);

  //ContactConstraints occur between a point on a body and the ground
  //The body always appears in the 0 index when calcForces is called
  //while the ground appears in the 1 index
  unsigned int idxBody = 0;
  unsigned int idxGround=1;

  CHECK(bodyIds[idxBody]==boxId); //First body is always the model body ContactConstraint
  CHECK(bodyIds[idxGround]==0);     //Second body is always ground for a ContactConstraint

  //Frames associated with the contacting body
  Vector3d r = Vector3d(-0.5,0.,0.);
  CHECK_THAT(bodyFrames[idxBody].r,
             AllCloseVector(r,TEST_PREC,TEST_PREC)
  );
  Matrix3d eye = Matrix3dIdentity;
  for(unsigned int i=0; i<3;++i){
    for(unsigned int j=0; j<3;++j){
      CHECK_THAT(bodyFrames[idxBody].E(i,j),
                 IsClose(eye(i,j),TEST_PREC, TEST_PREC));
    }
  }

  //Frame associated with base frame
  r.setZero();

  CHECK_THAT(bodyFrames[idxGround].r,
             AllCloseVector(r,TEST_PREC,TEST_PREC)
  );
  for(unsigned int i=0; i<3;++i){
    for(unsigned int j=0; j<3;++j){
      CHECK_THAT(bodyFrames[idxGround].E(i,j),
                 IsClose(eye(i,j),TEST_PREC, TEST_PREC));
    }
  }

  double fbody = 9.81*1.0*0.5*cos(q[2]);
  double fground= -9.81*1.0*0.5;
  unsigned int idxFy = 4;
  CHECK_THAT(constraintForces[idxBody  ][idxFy],
             IsClose(fbody, TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintForces[idxGround][idxFy],
             IsClose(fground, TEST_PREC, TEST_PREC));


  VectorNd qErr = q;
  qErr[0] += 1.0;
  VectorNd posErrUpd;
  cs.calcPositionError(gIdxLeft,model,qErr,posErrUpd,true);
  CHECK_THAT(posErrUpd[0], IsClose(0.0, TEST_PREC, TEST_PREC));
  CHECK_THAT(posErrUpd[1], IsClose(0.0, TEST_PREC, TEST_PREC));

  VectorNd qdErr = qd;
  qdErr[0] += 1.0;
  VectorNd velErrUpd;
  cs.calcVelocityError(gIdxLeft,model,q,qdErr,velErrUpd,true);
  CHECK_THAT(velErrUpd[0], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(velErrUpd[1], IsClose(1.0, TEST_PREC, TEST_PREC));

  Vector2d bgParams;
  cs.getBaumgarteStabilizationCoefficients(gIdxLeft,bgParams);
  CHECK_THAT(bgParams[0],IsClose(10., TEST_PREC, TEST_PREC));
  CHECK_THAT(bgParams[1],IsClose(10., TEST_PREC, TEST_PREC));

  bool bgEnabled = cs.isBaumgarteStabilizationEnabled(gIdxLeft);
  CHECK(bgEnabled == false);

  cs.calcBaumgarteStabilizationForces(gIdxLeft,model,posErrUpd,velErrUpd,
                                      bgForces);
  double bgForcesX = -2*bgParams[0]*velErrUpd[1];
  CHECK_THAT(bgForces[1], IsClose(bgForcesX, TEST_PREC, TEST_PREC));


  //Test calcForces but using the resolveAllInBaseFrame option
  cs.calcForces(gIdxLeft,model,q,qd,bodyIds,bodyFrames,constraintForces,true,
                true);

  CHECK(bodyIds[idxBody]  ==0); //First body is always the model body ContactConstraint
  CHECK(bodyIds[idxGround]==0);     //Second body is always ground for a ContactConstraint

  //Frames associated with the contacting body
  Matrix3d rotZ45 = rotz(q[2]);
  r = rotZ45.transpose()*Vector3d(-0.5,0.,0.);
  CHECK_THAT(bodyFrames[idxBody].r,
             AllCloseVector(r, TEST_PREC, TEST_PREC)
  );

  for(unsigned int i=0; i<3;++i){
    for(unsigned int j=0; j<3;++j){
      CHECK_THAT(bodyFrames[idxBody].E(i,j),
                 IsClose(eye(i,j), TEST_PREC, TEST_PREC));
    }
  }

  //Frame associated with base frame
  r.setZero();
  CHECK_THAT(bodyFrames[idxGround].r,
             AllCloseVector(r ,TEST_PREC,TEST_PREC)
  );
  for(unsigned int i=0; i<3;++i){
    for(unsigned int j=0; j<3;++j){
      CHECK_THAT(bodyFrames[idxGround].E(i,j),
                 IsClose(eye(i,j),TEST_PREC, TEST_PREC));
    }
  }

  fbody = 9.81*1.0*0.5;
  fground= -9.81*1.0*0.5;
  idxFy = 4;
  CHECK_THAT(constraintForces[idxBody  ][idxFy],
             IsClose(fbody,TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintForces[idxGround][idxFy],
             IsClose(fground,TEST_PREC, TEST_PREC));


  //Check the computation of impulses
  q.setZero();
  qd.setZero();
  double vx1 = 2.;
  double vy1 = -1.;
  qd[0] = vx1;
  qd[1] = vy1;
  VectorNd qDotPlus;
  qDotPlus.resize(qd.rows());

  ComputeConstraintImpulsesDirect(model,q,qd,cs,qDotPlus);

  for(unsigned int i=0; i<qDotPlus.rows();++i){
    CHECK_THAT(qDotPlus[i],IsClose(0.,TEST_PREC, TEST_PREC));
  }

  std::vector< SpatialVector > constraintImpulses;

  cs.calcImpulses(0,model,q,qDotPlus,bodyIds,bodyFrames,constraintImpulses,
                  false,false);

  //Due to symmetry this contraint group will only get half of the impulse
  CHECK_THAT(constraintImpulses[0][0],
             IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[0][1],
             IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[0][2],
             IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[0][3],
             IsClose(-vx1*m1, TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[0][4],
             IsClose(-vy1*m1*0.5, TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[0][5],
             IsClose(0., TEST_PREC, TEST_PREC));

  CHECK_THAT(constraintImpulses[1][0], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[1][1], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[1][2], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[1][3], IsClose(vx1*m1, TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[1][4],IsClose(vy1*m1*0.5, TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[1][5], IsClose(0., TEST_PREC, TEST_PREC));

cs.calcImpulses(1,model,q,qDotPlus,bodyIds,bodyFrames,constraintImpulses,
                false,false);

  //Due to symmetry this contraint group will only get half of the impulse
  CHECK_THAT(constraintImpulses[0][0], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[0][1], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[0][2], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[0][3], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[0][4],
             IsClose(-vy1*m1*0.5, TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[0][5], IsClose(0., TEST_PREC, TEST_PREC));

  CHECK_THAT(constraintImpulses[1][0], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[1][1], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[1][2], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[1][3], IsClose(0., TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[1][4],IsClose(vy1*m1*0.5, TEST_PREC, TEST_PREC));
  CHECK_THAT(constraintImpulses[1][5], IsClose(0., TEST_PREC, TEST_PREC));
}



// 
// ForwardDynamicsConstraintsDirect 
// 
TEST_CASE ( __FILE__"_TestForwardDynamicsConstraintsDirectSimple", "") {
  Model model;
  model.gravity = Vector3d  (0., -9.81, 0.);
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
  VectorNd QDDot = VectorNd::Constant  ((size_t) model.dof_count, 0.);
  VectorNd Tau = VectorNd::Constant ((size_t) model.dof_count, 0.);

  Q[1] = 1.;
  QDot[0] = 1.;
  QDot[3] = -1.;

  unsigned int contact_body_id = base_body_id;
  Vector3d contact_point ( 0., -1., 0.);

  ConstraintSet constraint_set;


  unsigned int id=11;
  unsigned int autoId =
      constraint_set.AddContactConstraint(contact_body_id, contact_point,
                                     Vector3d (1., 0., 0.),
                                      "ground_xyz",id);
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (0., 1., 0.));
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (0., 0., 1.));

  constraint_set.Bind (model);

  unsigned int index = constraint_set.getGroupIndexByName("ground_xyz");
  CHECK(index==0);
  index = constraint_set.getGroupIndexById(id);
  CHECK(index==0);
  index = constraint_set.getGroupIndexByAssignedId(autoId);
  CHECK(index==0);

  const char* conNameBack = constraint_set.getGroupName(index);
  CHECK(std::strcmp(conNameBack,"ground_xyz")==0);
  unsigned userId = constraint_set.getGroupId(index);
  CHECK(userId == id);

  ClearLogOutput();

//  cout << constraint_set.acceleration.transpose() << endl;
  ForwardDynamicsConstraintsDirect (model, Q, QDot, Tau, constraint_set, QDDot);

//  cout << "A = " << endl << constraint_set.A << endl << endl;
//  cout << "H = " << endl << constraint_set.H << endl << endl;
//  cout << "b = " << endl << constraint_set.b << endl << endl;
//  cout << "x = " << endl << constraint_set.x << endl << endl;
//  cout << constraint_set.b << endl;
//  cout << "QDDot = " << QDDot.transpose() << endl;

  Vector3d point_acceleration = CalcPointAcceleration (model, Q, QDot, QDDot,
                                                       contact_body_id,
                                                       contact_point);

  CHECK_THAT (Vector3d (0., 0., 0.),
              AllCloseVector(point_acceleration, TEST_PREC, TEST_PREC)
  );

  // cout << "LagrangianSimple Logoutput Start" << endl;
  // cout << LogOutput.str() << endl;
  // cout << "LagrangianSimple Logoutput End" << endl;
}

TEST_CASE ( __FILE__"_TestForwardDynamicsConstraintsDirectMoving ", "") {
  Model model;
  model.gravity = Vector3d  (0., -9.81, 0.);
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
  VectorNd QDDot = VectorNd::Constant  ((size_t) model.dof_count, 0.);
  VectorNd Tau = VectorNd::Constant ((size_t) model.dof_count, 0.);

  Q[0] = 0.1;
  Q[1] = 0.2;
  Q[2] = 0.3;
  Q[3] = 0.4;
  Q[4] = 0.5;
  Q[5] = 0.6;
  QDot[0] = 1.1;
  QDot[1] = 1.2;
  QDot[2] = 1.3;
  QDot[3] = -1.4;
  QDot[4] = -1.5;
  QDot[5] = -1.6;

  unsigned int contact_body_id = base_body_id;
  Vector3d contact_point ( 0., -1., 0.);

  ConstraintSet constraint_set;

  constraint_set.AddContactConstraint(contact_body_id, contact_point,
                                      Vector3d (1., 0., 0.),
                                      "ground_xyz");
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (0., 1., 0.));
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (0., 0., 1.));

  constraint_set.Bind (model);

  ClearLogOutput();

  ForwardDynamicsConstraintsDirect (model, Q, QDot, Tau, constraint_set, QDDot);

  Vector3d point_acceleration = CalcPointAcceleration (model, Q, QDot, QDDot,
                                                       contact_body_id,
                                                       contact_point);

  CHECK_THAT ( Vector3d (0., 0., 0.),
               AllCloseVector(point_acceleration, TEST_PREC, TEST_PREC)
  );

  // cout << "LagrangianSimple Logoutput Start" << endl;
  // cout << LogOutput.str() << endl;
  // cout << "LagrangianSimple Logoutput End" << endl;
}

// 
// ForwardDynamicsContacts
// 
TEST_CASE_METHOD (FixedBase6DoF,
                  __FILE__"_ForwardDynamicsContactsSingleContact", "") {
  contact_normal.set (0., 1., 0.);
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       contact_normal);
  ConstraintSet constraint_set_lagrangian = constraint_set.Copy();

  constraint_set_lagrangian.Bind (*model);
  constraint_set.Bind (*model);
  
  Vector3d point_accel_lagrangian, point_accel_contacts;
  
  ClearLogOutput();

  VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  
  ClearLogOutput();
  ForwardDynamicsConstraintsDirect (*model, Q, QDot, Tau,
                                    constraint_set_lagrangian, QDDot_lagrangian);
  ClearLogOutput();
  ForwardDynamicsContactsKokkevis (*model, Q, QDot, Tau, constraint_set,
                                   QDDot_contacts);
//  cout << LogOutput.str() << endl;
  ClearLogOutput();

  point_accel_lagrangian = CalcPointAcceleration (*model, Q, QDot,
                                                  QDDot_lagrangian, contact_body_id, contact_point, true);
  point_accel_contacts = CalcPointAcceleration (*model, Q, QDot,
                                                QDDot_contacts,
                                                contact_body_id, contact_point,
                                                true);

  CHECK_THAT (constraint_set_lagrangian.force[0],
              IsClose(constraint_set.force[0], TEST_PREC, TEST_PREC));
  CHECK_THAT (contact_normal.dot(point_accel_lagrangian),
              IsClose(contact_normal.dot(point_accel_contacts), TEST_PREC, TEST_PREC));
  CHECK_THAT (point_accel_lagrangian,
              AllCloseVector(point_accel_contacts, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (QDDot_lagrangian,
              AllCloseVector(QDDot_contacts, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD (FixedBase6DoF,
                  __FILE__"_ForwardDynamicsContactsSingleContactRotated", "") {
  Q[0] = 0.6;
  Q[3] =   M_PI * 0.6;
  Q[4] = 0.1;

  contact_normal.set (0., 1., 0.);
  
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       contact_normal);
  ConstraintSet constraint_set_lagrangian = constraint_set.Copy();
  
  constraint_set_lagrangian.Bind (*model);
  constraint_set.Bind (*model);
  
  Vector3d point_accel_lagrangian,point_accel_contacts, point_accel_contacts_opt;
  
  ClearLogOutput();

  VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  VectorNd QDDot_contacts_opt = VectorNd::Constant (model->mBodies.size() -1,0.);
  
  ClearLogOutput();
  ForwardDynamicsConstraintsDirect (*model, Q, QDot, Tau,
                                    constraint_set_lagrangian, QDDot_lagrangian);
  ForwardDynamicsContactsKokkevis (*model, Q, QDot, Tau, constraint_set,
                                   QDDot_contacts_opt);

  point_accel_lagrangian = CalcPointAcceleration (*model, Q, QDot,
                                                  QDDot_lagrangian,
                                                  contact_body_id,
                                                  contact_point, true);
  point_accel_contacts_opt = CalcPointAcceleration (*model, Q, QDot,
                                                    QDDot_contacts_opt,
                                                    contact_body_id,
                                                    contact_point, true);

  CHECK_THAT (constraint_set_lagrangian.force[0],
              IsClose(constraint_set.force[0], TEST_PREC, TEST_PREC));
  CHECK_THAT (contact_normal.dot(point_accel_lagrangian),
              IsClose(contact_normal.dot(point_accel_contacts_opt), TEST_PREC,
                      TEST_PREC));
  CHECK_THAT (point_accel_lagrangian,
              AllCloseVector(point_accel_contacts_opt, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (QDDot_lagrangian,
              AllCloseVector(QDDot_contacts_opt, TEST_PREC, TEST_PREC)
  );
}

// 
// Similiar to the previous test, this test compares the results of 
//   - ForwardDynamicsConstraintsDirect
//   - ForwardDynamcsContactsOpt
// for the example model in FixedBase6DoF and a moving state (i.e. a
// nonzero QDot)
// 
TEST_CASE_METHOD (FixedBase6DoF,
                  __FILE__"_ForwardDynamicsContactsSingleContactRotatedMoving",
                  "") {
  Q[0] = 0.6;
  Q[3] =   M_PI * 0.6;
  Q[4] = 0.1;

  QDot[0] = -0.3;
  QDot[1] = 0.1;
  QDot[2] = -0.5;
  QDot[3] = 0.8;

  contact_normal.set (0., 1., 0.);
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       contact_normal);
  ConstraintSet constraint_set_lagrangian = constraint_set.Copy();
  
  constraint_set_lagrangian.Bind (*model);
  constraint_set.Bind (*model);
  
  Vector3d point_accel_lagrangian, point_accel_contacts;
  
  VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  
  ClearLogOutput();
  ForwardDynamicsConstraintsDirect (*model, Q, QDot, Tau,
                                    constraint_set_lagrangian, QDDot_lagrangian);
//  cout << LogOutput.str() << endl;
  ClearLogOutput();
  ForwardDynamicsContactsKokkevis (*model, Q, QDot, Tau, constraint_set,
                                   QDDot_contacts);
//  cout << LogOutput.str() << endl;

  point_accel_lagrangian = CalcPointAcceleration (*model, Q, QDot,
                                                  QDDot_lagrangian,
                                                  contact_body_id,
                                                  contact_point, true);
  point_accel_contacts = CalcPointAcceleration (*model, Q, QDot,
                                                QDDot_contacts,
                                                contact_body_id, contact_point,
                                                true);

  // check whether FDContactsLagrangian and FDContactsOld match
  CHECK_THAT (constraint_set_lagrangian.force[0],
              IsClose(constraint_set.force[0], TEST_PREC, TEST_PREC));
  CHECK_THAT (contact_normal.dot(point_accel_lagrangian),
              IsClose(contact_normal.dot(point_accel_contacts), TEST_PREC,
                      TEST_PREC));
  CHECK_THAT (point_accel_lagrangian,
              AllCloseVector(point_accel_contacts, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (QDDot_lagrangian,
              AllCloseVector(QDDot_contacts, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD (FixedBase6DoF,
                  __FILE__"_ForwardDynamicsContactsOptDoubleContact", "") {
  ConstraintSet constraint_set_lagrangian;

  constraint_set.AddContactConstraint (contact_body_id, Vector3d (1., 0., 0.),
                                       contact_normal);
  constraint_set.AddContactConstraint (contact_body_id, Vector3d (0., 1., 0.),
                                       contact_normal);
  
  constraint_set_lagrangian = constraint_set.Copy();
  constraint_set_lagrangian.Bind (*model);
  constraint_set.Bind (*model);
  
  Vector3d point_accel_lagrangian, point_accel_contacts;
  
  ClearLogOutput();

  VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  
  ClearLogOutput();

  ForwardDynamicsConstraintsDirect (*model, Q, QDot, Tau,
                                    constraint_set_lagrangian, QDDot_lagrangian);
  ForwardDynamicsContactsKokkevis (*model, Q, QDot, Tau, constraint_set,
                                   QDDot_contacts);

  point_accel_lagrangian = CalcPointAcceleration (*model, Q, QDot,
                                                  QDDot_lagrangian,
                                                  contact_body_id,
                                                  contact_point, true);
  point_accel_contacts = CalcPointAcceleration (*model, Q, QDot,
                                                QDDot_contacts,
                                                contact_body_id, contact_point,
                                                true);

  // check whether FDContactsLagrangian and FDContacts match
  CHECK_THAT ( constraint_set_lagrangian.force,
               AllCloseVector(constraint_set.force, TEST_PREC, TEST_PREC)
  );

  // check whether the point accelerations match
  CHECK_THAT (point_accel_lagrangian,
              AllCloseVector(point_accel_contacts, TEST_PREC, TEST_PREC)
  );

  // check whether the generalized accelerations match
  CHECK_THAT (QDDot_lagrangian,
              AllCloseVector(QDDot_contacts, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD (FixedBase6DoF,
                  __FILE__"_ForwardDynamicsContactsOptDoubleContactRepeated", "")
{
  // makes sure that all variables in the constraint set gets reset
  // properly when making repeated calls to ForwardDynamicsContacts.
  ConstraintSet constraint_set_lagrangian;

  constraint_set.AddContactConstraint (contact_body_id, Vector3d (1., 0., 0.),
                                       contact_normal);
  constraint_set.AddContactConstraint (contact_body_id, Vector3d (0., 1., 0.),
                                       contact_normal);
  
  constraint_set_lagrangian = constraint_set.Copy();
  constraint_set_lagrangian.Bind (*model);
  constraint_set.Bind (*model);
  
  Vector3d point_accel_lagrangian, point_accel_contacts;
  
  ClearLogOutput();

  VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  
  ClearLogOutput();

  ForwardDynamicsConstraintsDirect (*model, Q, QDot, Tau,
                                    constraint_set_lagrangian, QDDot_lagrangian);
  // Call ForwardDynamicsContacts multiple times such that old values might
  // be re-used and thus cause erroneus values.
  ForwardDynamicsContactsKokkevis (*model, Q, QDot, Tau, constraint_set,
                                   QDDot_contacts);
  ForwardDynamicsContactsKokkevis (*model, Q, QDot, Tau, constraint_set,
                                   QDDot_contacts);
  ForwardDynamicsContactsKokkevis (*model, Q, QDot, Tau, constraint_set,
                                   QDDot_contacts);

  point_accel_lagrangian = CalcPointAcceleration (*model, Q, QDot,
                                                  QDDot_lagrangian,
                                                  contact_body_id,
                                                  contact_point, true);
  point_accel_contacts = CalcPointAcceleration (*model, Q, QDot,
                                                QDDot_contacts,
                                                contact_body_id, contact_point,
                                                true);

  // check whether FDContactsLagrangian and FDContacts match
  CHECK_THAT (constraint_set_lagrangian.force,
              AllCloseVector(constraint_set.force, TEST_PREC, TEST_PREC)
  );

  // check whether the point accelerations match
  CHECK_THAT (point_accel_lagrangian,
              AllCloseVector(point_accel_contacts, TEST_PREC, TEST_PREC)
  );

  // check whether the generalized accelerations match
  CHECK_THAT (QDDot_lagrangian,
              AllCloseVector(QDDot_contacts, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD (FixedBase6DoF,
                  __FILE__"_ForwardDynamicsContactsOptMultipleContact", "") {
  ConstraintSet constraint_set_lagrangian;

  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (1., 0., 0.));
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (0., 1., 0.));
  
  constraint_set_lagrangian = constraint_set.Copy();
  constraint_set_lagrangian.Bind (*model);
  constraint_set.Bind (*model);

  // we rotate the joints so that we have full mobility at the contact
  // point:
  //
  //  O       X (contact point)
  //   \     /
  //    \   /
  //     \ /
  //      *      
  //

  Q[0] = M_PI * 0.25;
  Q[1] = 0.2;
  Q[3] = M_PI * 0.5;

  VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
  
  ClearLogOutput();
  ForwardDynamicsConstraintsDirect (*model, Q, QDot, Tau,
                                    constraint_set_lagrangian, QDDot_lagrangian);
  ForwardDynamicsContactsKokkevis (*model, Q, QDot, Tau, constraint_set,
                                   QDDot_contacts);


  Vector3d point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot,
                                                  contact_body_id,
                                                  contact_point);


  CHECK_THAT (QDDot_lagrangian,
              AllCloseVector(QDDot_contacts, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT (constraint_set_lagrangian.force,
              AllCloseVector(constraint_set.force, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT (0., IsClose(point_accel_c[0], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_c[1], TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD (FixedBase6DoF9DoF,
 __FILE__"_ForwardDynamicsContactsOptMultipleContactsMultipleBodiesMoving", "") {
  ConstraintSet constraint_set_lagrangian;

  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (1., 0., 0.));
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (0., 1., 0.));
  constraint_set.AddContactConstraint (child_2_id, contact_point, Vector3d (
                                         0., 1., 0.));
  
  constraint_set_lagrangian = constraint_set.Copy();
  constraint_set_lagrangian.Bind (*model);
  constraint_set.Bind (*model);
  
  Q[0] = 0.1;
  Q[1] = -0.1;
  Q[2] = 0.1;
  Q[3] = -0.1;
  Q[4] = -0.1;
  Q[5] = 0.1;

  QDot[0] =  1.; 
  QDot[1] = -1.;
  QDot[2] =  1; 
  QDot[3] = -1.5; 
  QDot[4] =  1.5; 
  QDot[5] = -1.5; 

  VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);

  ClearLogOutput();
  ForwardDynamicsContactsKokkevis (*model, Q, QDot, Tau, constraint_set, QDDot);

  Vector3d point_accel_c, point_accel_2_c;

  point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot,
                                         contact_body_id, contact_point);
  point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot, child_2_id,
                                           contact_point);


  ForwardDynamicsConstraintsDirect (*model, Q, QDot, Tau,
                                    constraint_set_lagrangian, QDDot_lagrangian);

  CHECK_THAT (constraint_set_lagrangian.force,
              AllCloseVector(constraint_set.force, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT (0., IsClose(point_accel_c[0], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_c[1], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_2_c[1], TEST_PREC, TEST_PREC));

  point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian,
                                         contact_body_id, contact_point);
  point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian,
                                           child_2_id, contact_point);

  CHECK_THAT (0., IsClose(point_accel_c[0], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_c[1], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_2_c[1], TEST_PREC, TEST_PREC));

  CHECK_THAT (QDDot_lagrangian,
              AllCloseVector(QDDot, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD (FixedBase6DoF9DoF,
__FILE__"_ForwardDynamicsContactsOptMultipleContactsMultipleBodiesMovingAlternate"
                  , "") {
  ConstraintSet constraint_set_lagrangian;

  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (1., 0., 0.));
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (0., 1., 0.));
  constraint_set.AddContactConstraint (child_2_id, contact_point, Vector3d (
                                         0., 1., 0.));
  
  constraint_set_lagrangian = constraint_set.Copy();
  constraint_set_lagrangian.Bind (*model);
  constraint_set.Bind (*model);

  Q[0] = 0.1;
  Q[1] = -0.3;
  Q[2] = 0.15;
  Q[3] = -0.21;
  Q[4] = -0.81;
  Q[5] = 0.11;
  Q[6] = 0.31;
  Q[7] = -0.91;
  Q[8] = 0.61;

  QDot[0] =  1.3; 
  QDot[1] = -1.7;
  QDot[2] =  3; 
  QDot[3] = -2.5; 
  QDot[4] =  1.5; 
  QDot[5] = -5.5; 
  QDot[6] =  2.5; 
  QDot[7] = -1.5; 
  QDot[8] = -3.5; 

  VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);

  ClearLogOutput();
  ForwardDynamicsContactsKokkevis (*model, Q, QDot, Tau, constraint_set, QDDot);

  Vector3d point_accel_c, point_accel_2_c;

  point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot,
                                         contact_body_id, contact_point);
  point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot, child_2_id,
                                           contact_point);


  ForwardDynamicsConstraintsDirect (*model, Q, QDot, Tau,
                                    constraint_set_lagrangian, QDDot_lagrangian);

  CHECK_THAT (constraint_set_lagrangian.force,
              AllCloseVector(constraint_set.force, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT (0., IsClose(point_accel_c[0], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_c[1], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_2_c[1], TEST_PREC, TEST_PREC));

  point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian,
                                         contact_body_id, contact_point);
  point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian,
                                           child_2_id, contact_point);

  CHECK_THAT (0., IsClose(point_accel_c[0], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_c[1], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_2_c[1], TEST_PREC, TEST_PREC));

  CHECK_THAT (QDDot_lagrangian,
              AllCloseVector(QDDot, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD (FixedBase6DoF12DoFFloatingBase,
                  __FILE__"_ForwardDynamicsContactsMultipleContactsFloatingBase",
                  "") {
  ConstraintSet constraint_set_lagrangian;

  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (1., 0., 0.));
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (0., 1., 0.));
  constraint_set.AddContactConstraint (child_2_id, contact_point,
                                       Vector3d (0., 1., 0.));
  
  constraint_set_lagrangian = constraint_set.Copy();
  constraint_set_lagrangian.Bind (*model);
  constraint_set.Bind (*model);

  VectorNd QDDot_lagrangian = VectorNd::Constant (model->dof_count, 0.);

  Q[0] = 0.1;
  Q[1] = -0.3;
  Q[2] = 0.15;
  Q[3] = -0.21;
  Q[4] = -0.81;
  Q[5] = 0.11;
  Q[6] = 0.31;
  Q[7] = -0.91;
  Q[8] = 0.61;

  QDot[0] =  1.3; 
  QDot[1] = -1.7;
  QDot[2] =  3; 
  QDot[3] = -2.5; 
  QDot[4] =  1.5; 
  QDot[5] = -5.5; 
  QDot[6] =  2.5; 
  QDot[7] = -1.5; 
  QDot[8] = -3.5; 

  ClearLogOutput();
  ForwardDynamicsContactsKokkevis (*model, Q, QDot, Tau, constraint_set, QDDot);

  Vector3d point_accel_c, point_accel_2_c;

  point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot,
                                         contact_body_id, contact_point);
  point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot, child_2_id,
                                           contact_point);


  ClearLogOutput();
  ForwardDynamicsConstraintsDirect (*model, Q, QDot, Tau,
                                    constraint_set_lagrangian, QDDot_lagrangian);

  CHECK_THAT (constraint_set_lagrangian.force,
              AllCloseVector(constraint_set.force, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT (0., IsClose(point_accel_c[0], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_c[1], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_2_c[1], TEST_PREC, TEST_PREC));

  point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian,
                                         contact_body_id, contact_point);
  point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian,
                                           child_2_id, contact_point);

  CHECK_THAT (0., IsClose(point_accel_c[0], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_c[1], TEST_PREC, TEST_PREC));
  CHECK_THAT (0., IsClose(point_accel_2_c[1], TEST_PREC, TEST_PREC));

  CHECK_THAT (QDDot_lagrangian,
              AllCloseVector(QDDot, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD (Human36,
                  __FILE__"_ForwardDynamicsContactsFixedBody", "") {
  VectorNd qddot_lagrangian (VectorNd::Zero(qddot.size()));
  VectorNd qddot_sparse (VectorNd::Zero(qddot.size()));

  randomizeStates();

  ConstraintSet constraint_upper_trunk;
  constraint_upper_trunk.AddContactConstraint (body_id_3dof[BodyUpperTrunk],
                                               Vector3d (1.1, 2.2, 3.3),
                                               Vector3d (1., 0., 0.));
  constraint_upper_trunk.Bind (*model_3dof);

  ForwardDynamicsConstraintsDirect (*model_3dof, q, qdot, tau,
                                    constraint_upper_trunk, qddot_lagrangian);
  ForwardDynamicsConstraintsRangeSpaceSparse (*model_3dof, q, qdot, tau,
                                              constraint_upper_trunk,
                                              qddot_sparse);
  ForwardDynamicsContactsKokkevis (*model_3dof, q, qdot, tau,
                                   constraint_upper_trunk, qddot);

  CHECK_THAT (qddot_lagrangian,
              AllCloseVector(qddot, TEST_PREC, TEST_PREC * qddot_lagrangian.norm() * 10.)
  );
  CHECK_THAT (qddot_lagrangian,
              AllCloseVector(qddot_sparse, TEST_PREC, TEST_PREC * qddot_lagrangian.norm() * 10.)
  );
}

TEST_CASE_METHOD (Human36,
                  __FILE__"_ForwardDynamicsContactsImpulses", "") {
  VectorNd qddot_lagrangian (VectorNd::Zero(qddot.size()));

  for (int i = 0; i < q.size(); i++) {
    q[i] = 0.5 * M_PI * static_cast<double>(rand())
           / static_cast<double>(RAND_MAX);
    qdot[i] = 0.5 * M_PI * static_cast<double>(rand())
              / static_cast<double>(RAND_MAX);
    tau[i] = 0.5 * M_PI * static_cast<double>(rand())
             / static_cast<double>(RAND_MAX);
    qddot_3dof[i] = 0.5 * M_PI * static_cast<double>(rand())
                    / static_cast<double>(RAND_MAX);
  }

  Vector3d heel_point (-0.03, 0., -0.03);

  ConstraintSet constraint_upper_trunk;
  constraint_upper_trunk.AddContactConstraint (body_id_3dof[BodyFootLeft],
                                               heel_point, Vector3d(1., 0., 0.));
  constraint_upper_trunk.AddContactConstraint (body_id_3dof[BodyFootLeft],
                                               heel_point, Vector3d(0., 1., 0.));
  constraint_upper_trunk.AddContactConstraint (body_id_3dof[BodyFootLeft],
                                               heel_point, Vector3d(0., 0., 1.));
  constraint_upper_trunk.AddContactConstraint (body_id_3dof[BodyFootRight],
                                               heel_point, Vector3d(1., 0., 0.));
  constraint_upper_trunk.AddContactConstraint (body_id_3dof[BodyFootRight],
                                               heel_point, Vector3d(0., 1., 0.));
  constraint_upper_trunk.AddContactConstraint (body_id_3dof[BodyFootRight],
                                               heel_point, Vector3d(0., 0., 1.));
  constraint_upper_trunk.Bind (*model_3dof);

  VectorNd qdotplus (VectorNd::Zero (qdot.size()));

  ComputeConstraintImpulsesDirect (*model_3dof, q, qdot,
                                   constraint_upper_trunk, qdotplus);

  Vector3d heel_left_velocity = CalcPointVelocity (*model_3dof, q, qdotplus,
                                                   body_id_3dof[BodyFootLeft],
                                                   heel_point);
  Vector3d heel_right_velocity = CalcPointVelocity (*model_3dof, q, qdotplus,
                                                    body_id_3dof[BodyFootRight],
                                                    heel_point);

  CHECK_THAT (Vector3d(0., 0., 0.),
              AllCloseVector(heel_left_velocity, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT (Vector3d(0., 0., 0.),
              AllCloseVector(heel_right_velocity, TEST_PREC, TEST_PREC)
  );
}
