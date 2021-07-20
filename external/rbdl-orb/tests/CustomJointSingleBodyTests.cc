/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2016-2018 Matthew Millard <millard.matthew@gmail.com>
 */
#include "rbdl_tests.h"

#include <iostream>

#include "Fixtures.h"
#include "Human36Fixture.h"
#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Constraints.h"
#include <vector>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 2.0e-12;
const int NUMBER_OF_MODELS = 2;

//==============================================================================
/*

  The purpose of this test is to test that all of the code in RBDL 
  related to a single CustomJoint functions. To do this we will implement
  joints that already exist in RBDL but using the CustomJoint interface. The 
  test will then numerically compare the results produced by the CustomJoint
  and the equivalent built-in joint in RBDL. The following algorithms will
  be tested:

    UpdateKinematicsCustom
    Jacobians
    InverseDynamics
    CompositeRigidBodyAlgorithm
    ForwardDynamics
    CalcMInvTimestau
    ForwardDynamicsContactsKokkevis

*/
//==============================================================================
//==============================================================================
/*
  As a note, below are the basic fields and functions that every CustomJoint
  class member must provide. Refer to Featherstone's dynamics text if the field 
  names below don't make sense to you.

   1. Extend from CustomJoint:
      
      struct CustomJointClass: public CustomJoint 

   2. Make a default constructor, and initialize member variables
          mDoFCount 
          S
          d_u

      e.g. 

      CustomJointClass()

  3. Implement the method jcalc. This method must populate X_J, v_J, c_J, and S.

        virtual void jcalc
          model.X_J[joint_id]
          model.v_J
          model.c_J
          model.mCustomJoints[joint.custom_joint_index]->S = S

  4. Implement the method jcalc_X_lambda_S. This method must populate X_lambda
      and S.

        virtual void jcalc_X_lambda_S
          model.X_lambda
          model.mCustomJoints[joint.custom_joint_index]->S = S;

 */
//==============================================================================
//Custom Joint Code
//==============================================================================
struct CustomJointTypeRevoluteX : public CustomJoint 
{
  CustomJointTypeRevoluteX(){
    mDoFCount = 1;
    S = MatrixNd::Zero(6,1);
    S(0, 0) = 1.0;
    d_u = MatrixNd::Zero(mDoFCount,1);
  }

  virtual void jcalc (Model &model,
                      unsigned int joint_id,
                      const Math::VectorNd &q,
                      const Math::VectorNd &qdot)
  {
    model.X_J[joint_id] = Xrotx(q[model.mJoints[joint_id].q_index]);
    model.v_J[joint_id][0] = qdot[model.mJoints[joint_id].q_index];
  }

  virtual void jcalc_X_lambda_S ( Model &model,
                                  unsigned int joint_id,
                                  const Math::VectorNd &q)
  {
    model.X_lambda[joint_id] =
        Xrotx (q[model.mJoints[joint_id].q_index])
        * model.X_T[joint_id];


    const Joint &joint = model.mJoints[joint_id];
    model.mCustomJoints[joint.custom_joint_index]->S = S;

  }

};

struct CustomEulerZYXJoint : public CustomJoint 
{
  CustomEulerZYXJoint () 
  {
    mDoFCount = 3;
    S = MatrixNd::Zero (6,3);
    d_u = MatrixNd::Zero(mDoFCount,1);
  }

  virtual void jcalc (Model &model,
                      unsigned int joint_id,
                      const Math::VectorNd &q,
                      const Math::VectorNd &qdot) 
  {
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
                       c0 * c1,                s0 * c1,     -s1,
        c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
        c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2
        );

    S.setZero();
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
        -c1*qdot0*qdot1,
        -s1*s2*qdot0*qdot1 + c1*c2*qdot0*qdot2 - s2*qdot1*qdot2,
        -s1*c2*qdot0*qdot1 - c1*s2*qdot0*qdot2 - c2*qdot1*qdot2,
        0., 0., 0.
        );
  }

  virtual void jcalc_X_lambda_S ( Model &model,
                                  unsigned int joint_id,
                                  const Math::VectorNd &q) 
  {      
      double q0 = q[model.mJoints[joint_id].q_index];
      double q1 = q[model.mJoints[joint_id].q_index + 1];
      double q2 = q[model.mJoints[joint_id].q_index + 2];

      double s0 = sin (q0);
      double c0 = cos (q0);
      double s1 = sin (q1);
      double c1 = cos (q1);
      double s2 = sin (q2);
      double c2 = cos (q2);

      
      model.X_lambda[joint_id] = SpatialTransform ( 
          Matrix3d(
            c0 * c1, s0 * c1, -s1,
            c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
            c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2
            ),
          Vector3d (0., 0., 0.)) * model.X_T[joint_id];
      
      S.setZero();
      S(0,0) = -s1;
      S(0,2) = 1.;

      S(1,0) = c1 * s2;
      S(1,1) = c2;

      S(2,0) = c1 * c2;
      S(2,1) = - s2;

      const Joint &joint = model.mJoints[joint_id];
      model.mCustomJoints[joint.custom_joint_index]->S = S;

    //assert (false && "Not yet implemented!");
  }
};

//==============================================================================
//Test Fixture
//==============================================================================

struct CustomJointSingleBodyFixture {
  CustomJointSingleBodyFixture () {

    reference_model.resize(NUMBER_OF_MODELS);
    custom_model.resize(NUMBER_OF_MODELS);

    body.resize(NUMBER_OF_MODELS);
    custom_joint.resize(NUMBER_OF_MODELS);

    reference_body_id.resize(NUMBER_OF_MODELS);
    custom_body_id.resize(NUMBER_OF_MODELS);

    q.resize(NUMBER_OF_MODELS);
    qdot.resize(NUMBER_OF_MODELS);
    qddot.resize(NUMBER_OF_MODELS);
    tau.resize(NUMBER_OF_MODELS);

    //========================================================
    //Test Model 0: 3dof rotational custom joint vs. standard.
    //========================================================

    custom_joint0 = CustomEulerZYXJoint();

    Matrix3d inertia0 = Matrix3d::Identity(3,3);
    Body body0 = Body (1., Vector3d (1.1, 1.2, 1.3), inertia0);

    Model reference0, custom0;

    unsigned int reference_body_id0 =
       reference0.AddBody ( 0,
                            SpatialTransform(),
                            Joint(JointTypeEulerZYX),
                            body0);

    unsigned int custom_body_id0 =
        custom0.AddBodyCustomJoint (  0,
                                      SpatialTransform(),
                                      &custom_joint0,
                                      body0);

    VectorNd q0     = VectorNd::Zero (reference0.q_size);
    VectorNd qdot0  = VectorNd::Zero (reference0.qdot_size);
    VectorNd qddot0 = VectorNd::Zero (reference0.qdot_size);
    VectorNd tau0   = VectorNd::Zero (reference0.qdot_size);

    reference_model.at(0)   = reference0;
    custom_model.at(0)      = custom0;

    reference_body_id.at(0) = (reference_body_id0);
    custom_body_id.at(0)    = (custom_body_id0);

    body.at(0)          = (body0);
    custom_joint.at(0)  = (&custom_joint0);

    q.at(0)       = (q0);
    qdot.at(0)    = (qdot0);
    qddot.at(0)   = (qddot0);
    tau.at(0)     = (tau0);

    //========================================================
    //Test Model 1: 1 dof rotational custom joint vs. standard.
    //========================================================

    custom_joint1 = CustomJointTypeRevoluteX();

    Model reference1, custom1;

    unsigned int reference_body_id1 =
       reference1.AddBody ( 0,
                            SpatialTransform(),
                            Joint(JointTypeRevoluteX),
                            body0);

    unsigned int custom_body_id1 =
        custom1.AddBodyCustomJoint (0,
                                    SpatialTransform(),
                                    &custom_joint1,
                                    body0);

    VectorNd q1     = VectorNd::Zero (reference1.q_size);
    VectorNd qdot1  = VectorNd::Zero (reference1.qdot_size);
    VectorNd qddot1 = VectorNd::Zero (reference1.qdot_size);
    VectorNd tau1   = VectorNd::Zero (reference1.qdot_size);

    reference_model.at(1)   = (reference1);
    custom_model.at(1)      = (custom1);

    reference_body_id.at(1) = (reference_body_id1);
    custom_body_id.at(1)    = (custom_body_id1);

    body.at(1)          = (body0);
    custom_joint.at(1)  = (&custom_joint1);

    q.at(1)     = (q1);
    qdot.at(1)  = (qdot1);
    qddot.at(1) = (qddot1);
    tau.at(1)   = (tau1);

  }

  /*
  ~CustomJointSingleBodyFixture () {
      delete reference_model;
      delete custom_model;

      delete body;
      delete custom_joint;

      delete reference_body_id;
      delete custom_body_id;

      delete q;
      delete qdot;
      delete qddot;
      delete tau;
  }*/

  vector < Model > reference_model;
  vector < Model > custom_model;

  vector < Body  > body;
  vector < CustomJoint* > custom_joint;

  vector < unsigned int > reference_body_id;
  vector < unsigned int > custom_body_id;

  vector < VectorNd > q;
  vector < VectorNd > qdot;
  vector < VectorNd > qddot;
  vector < VectorNd > tau;
  CustomJointTypeRevoluteX custom_joint1;
  CustomEulerZYXJoint custom_joint0;

};

//==============================================================================
//
// Tests 
//    UpdateKinematicsCustom
//    Jacobians
//    InverseDynamics
//    CompositeRigidBodyAlgorithm
//    ForwardDynamics
//    CalcMInvTimestau
//    ForwardDynamicsContactsKokkevis
//
//==============================================================================

TEST_CASE_METHOD ( CustomJointSingleBodyFixture,
                   __FILE__"_UpdateKinematics", "") {

  VectorNd test;

  for(int idx =0; idx < NUMBER_OF_MODELS; ++idx){

    int dof = reference_model.at(idx).dof_count;
    for (unsigned int i = 0; i < dof ; i++) {
      q.at(idx)[i]      = i * 0.1;
      qdot.at(idx)[i]   = i * 0.15;
      qddot.at(idx)[i]  = i * 0.17;
    }

    UpdateKinematics (reference_model.at(idx),
                      q.at(idx),
                      qdot.at(idx),
                      qddot.at(idx));

    UpdateKinematics (custom_model.at(idx),
                      q.at(idx),
                      qdot.at(idx),
                      qddot.at(idx));

    CHECK_THAT (reference_model.at(idx).X_base[reference_body_id.at(idx)].E,
                AllCloseMatrix(
                  custom_model.at(idx).X_base[    custom_body_id.at(idx)].E,
                  TEST_PREC,
                  TEST_PREC)
    );

    CHECK_THAT (reference_model.at(idx).v[reference_body_id.at(idx)],
         AllCloseVector(custom_model.at(idx).v[   custom_body_id.at(idx)],
                        TEST_PREC,
                        TEST_PREC)
    );

    CHECK_THAT (reference_model.at(idx).a[reference_body_id.at(idx)],
         AllCloseVector(custom_model.at(idx).a[   custom_body_id.at(idx)],
                        TEST_PREC,
                        TEST_PREC)
    );
  }
}

TEST_CASE_METHOD (CustomJointSingleBodyFixture,
                  __FILE__"_UpdateKinematicsCustom", "") {
  
  for(int idx =0; idx < NUMBER_OF_MODELS; ++idx){
    int dof = reference_model.at(idx).dof_count;
    for (unsigned int i = 0; i < dof; i++) {
      q.at(idx)[i]        = i * 9.133758561390194e-01;
      qdot.at(idx)[i]     = i * 6.323592462254095e-01;
      qddot.at(idx)[i]    = i * 9.754040499940952e-02;
    }

    UpdateKinematicsCustom (reference_model.at(idx),
                            &q.at(idx), NULL, NULL);
    UpdateKinematicsCustom (custom_model.at(idx),
                            &q.at(idx), NULL, NULL);


    CHECK_THAT (reference_model.at(idx).X_base[reference_body_id.at(idx)].E,
                AllCloseMatrix(
                  custom_model.at(idx).X_base[   custom_body_id.at(idx)].E,
                  TEST_PREC,
                  TEST_PREC)
    );


    //velocity
    UpdateKinematicsCustom (reference_model.at(idx),
                            &q.at(idx),
                            &qdot.at(idx), 
                            NULL);
    UpdateKinematicsCustom (custom_model.at(idx),
                            &q.at(idx),
                            &qdot.at(idx), 
                            NULL);

    CHECK_THAT (reference_model.at(idx).v[reference_body_id.at(idx)],
             AllCloseVector(custom_model.at(idx).v[   custom_body_id.at(idx)],
                            TEST_PREC,
                            TEST_PREC)
    );


    //All
    UpdateKinematicsCustom (reference_model.at(idx),
                            &q.at(idx),
                            &qdot.at(idx),
                            &qddot.at(idx));

    UpdateKinematicsCustom (custom_model.at(idx),
                            &q.at(idx),
                            &qdot.at(idx),
                            &qddot.at(idx));

    CHECK_THAT (reference_model.at(idx).a[reference_body_id.at(idx)],
         AllCloseVector(custom_model.at(idx).a[   custom_body_id.at(idx)],
                        TEST_PREC,
                        TEST_PREC)
    );
  }

   
}

TEST_CASE_METHOD (CustomJointSingleBodyFixture,
                  __FILE__"_Jacobians", "") {

  for(int idx =0; idx < NUMBER_OF_MODELS; ++idx){
    int dof = reference_model.at(idx).dof_count;

    for (unsigned int i = 0; i < dof; i++) {
      q.at(idx)[i]        = i * 9.133758561390194e-01;
      qdot.at(idx)[i]     = i * 6.323592462254095e-01;
      qddot.at(idx)[i]    = i * 9.754040499940952e-02;
    }

    //position
    UpdateKinematics (reference_model.at(idx),
                      q.at(idx),
                      qdot.at(idx),
                      qddot.at(idx));

    UpdateKinematics (custom_model.at(idx),
                      q.at(idx),
                      qdot.at(idx),
                      qddot.at(idx));

    //Check the Spatial Jacobian
    MatrixNd Gref = MatrixNd(
          MatrixNd::Zero(6,reference_model.at(idx).dof_count));

    MatrixNd Gcus = MatrixNd(
          MatrixNd::Zero(6,reference_model.at(idx).dof_count));

    CalcBodySpatialJacobian ( reference_model.at(idx),
                              q.at(idx),
                              reference_body_id.at(idx),
                              Gref);

    CalcBodySpatialJacobian ( custom_model.at(idx),
                              q.at(idx),
                              custom_body_id.at(idx),
                              Gcus);

    for(int i=0; i<6;++i){
      for(int j=0; j<dof;++j){
        CHECK_THAT (Gref(i,j), IsClose(Gcus(i,j), TEST_PREC, TEST_PREC));
      }
    }

    //Check the 6d point Jacobian
    Vector3d point_position (1.1, 1.2, 2.1);

    CalcPointJacobian6D (reference_model.at(idx),
                         q.at(idx),
                         reference_body_id.at(idx),
                         point_position,
                         Gref);

    CalcPointJacobian6D (custom_model.at(idx),
                         q.at(idx),
                         custom_body_id.at(idx),
                         point_position,
                         Gcus);

    for(int i=0; i<6;++i){
      for(int j=0; j<dof;++j){
        CHECK_THAT (Gref(i,j), IsClose(Gcus(i,j), TEST_PREC, TEST_PREC));
      }
    }



    //Check the 3d point Jacobian
    MatrixNd GrefPt = MatrixNd::Constant (3,
                                          reference_model.at(idx).dof_count,
                                          0.);
    MatrixNd GcusPt = MatrixNd::Constant (3,
                                          reference_model.at(idx).dof_count,
                                          0.);



    CalcPointJacobian (reference_model.at(idx),
                       q.at(idx),
                       reference_body_id.at(idx),
                       point_position,
                       GrefPt);

    CalcPointJacobian (custom_model.at(idx),
                       q.at(idx),
                       custom_body_id.at(idx),
                       point_position,
                       GcusPt);

    for(int i=0; i<3;++i){
      for(int j=0; j<dof;++j){
        CHECK_THAT (GrefPt(i,j), IsClose(GcusPt(i,j), TEST_PREC, TEST_PREC));
      }
    }
  }

}

TEST_CASE_METHOD (CustomJointSingleBodyFixture,
                  __FILE__"_InverseDynamics", "") {

  for(int idx =0; idx < NUMBER_OF_MODELS; ++idx){

    int dof = reference_model.at(idx).dof_count;

    for (unsigned int i = 0; i < dof; i++) {
      q.at(idx)[i]        = i * 9.133758561390194e-01;
      qdot.at(idx)[i]     = i * 6.323592462254095e-01;
      qddot.at(idx)[i]    = i * 9.754040499940952e-02;
    }

    //position

    VectorNd tauRef = VectorNd::Zero (reference_model.at(idx).qdot_size);
    VectorNd tauCus = VectorNd::Zero (reference_model.at(idx).qdot_size);

    InverseDynamics(reference_model.at(idx),
                    q.at(idx),
                    qdot.at(idx),
                    qddot.at(idx),
                    tauRef);

    InverseDynamics(custom_model.at(idx),
                    q.at(idx),
                    qdot.at(idx),
                    qddot.at(idx),
                    tauCus);

    VectorNd tauErr = tauRef-tauCus;

    CHECK_THAT (tauRef,
                AllCloseVector(tauCus, TEST_PREC, TEST_PREC)
    );
  }

}

TEST_CASE_METHOD (CustomJointSingleBodyFixture,
                  __FILE__"_CompositeRigidBodyAlgorithm", "") {

  for(int idx =0; idx < NUMBER_OF_MODELS; ++idx){

    int dof = reference_model.at(idx).dof_count;

    for (unsigned int i = 0; i < dof; i++) {
      q.at(idx)[i]      = (i+0.1) * 9.133758561390194e-01;
      qdot.at(idx)[i]    = (i+0.1) * 6.323592462254095e-01;
      tau.at(idx)[i]     = (i+0.1) * 9.754040499940952e-02;
    }

    MatrixNd h_ref = MatrixNd::Constant (dof, dof, 0.);
    VectorNd c_ref = VectorNd::Constant (dof, 0.);
    VectorNd qddot_zero_ref = VectorNd::Constant (dof, 0.);
    VectorNd qddot_crba_ref = VectorNd::Constant (dof, 0.);

    MatrixNd h_cus = MatrixNd::Constant (dof, dof, 0.);
    VectorNd c_cus = VectorNd::Constant (dof, 0.);
    VectorNd qddot_zero_cus = VectorNd::Constant (dof, 0.);
    VectorNd qddot_crba_cus = VectorNd::Constant (dof, 0.);

    VectorNd qddotRef = VectorNd::Zero (dof);
    VectorNd qddotCus = VectorNd::Zero (dof);

    //Ref
    ForwardDynamics(reference_model.at(idx),
                    q.at(idx),
                    qdot.at(idx),
                    tau.at(idx),
                    qddotRef);

    CompositeRigidBodyAlgorithm (reference_model.at(idx),
                                 q.at(idx),
                                 h_ref);

    InverseDynamics (reference_model.at(idx),
                     q.at(idx),
                     qdot.at(idx),
                     qddot_zero_ref,
                     c_ref);

    LinSolveGaussElimPivot (h_ref,
                            c_ref * -1. + tau.at(idx),
                            qddot_crba_ref);

    //Custom
    ForwardDynamics(custom_model.at(idx),
                    q.at(idx),
                    qdot.at(idx),
                    tau.at(idx),
                    qddotCus);

    CompositeRigidBodyAlgorithm (custom_model.at(idx),
                                 q.at(idx),
                                 h_cus);

    InverseDynamics (custom_model.at(idx),
                     q.at(idx),
                     qdot.at(idx),
                     qddot_zero_cus,
                     c_cus);

    LinSolveGaussElimPivot (h_cus,
                            c_cus * -1. + tau.at(idx),
                            qddot_crba_cus);

    CHECK_THAT(qddot_crba_ref,
               AllCloseVector(qddot_crba_cus, TEST_PREC, TEST_PREC)
    );
  }
}

TEST_CASE_METHOD (CustomJointSingleBodyFixture,
                  __FILE__"_ForwardDynamics", "") {

  for(int idx =0; idx < NUMBER_OF_MODELS; ++idx){

    int dof = reference_model.at(idx).dof_count;

    for (unsigned int i = 0; i < dof; i++) {
      q.at(idx)[i]        = (i+0.1) * 9.133758561390194e-01;
      qdot.at(idx)[i]     = (i+0.1) * 6.323592462254095e-01;
      qddot.at(idx)[i]    = (i+0.1) * 2.323592499940952e-01;
      tau.at(idx)[i]      = (i+0.1) * 9.754040499940952e-02;
    }


    VectorNd qddotRef = VectorNd::Zero (reference_model.at(idx).qdot_size);
    VectorNd qddotCus = VectorNd::Zero (reference_model.at(idx).qdot_size);

    ForwardDynamics(reference_model.at(idx),
                    q.at(idx),
                    qdot.at(idx),
                    tau.at(idx),
                    qddotRef);

    ForwardDynamics(custom_model.at(idx),
                    q.at(idx),
                    qdot.at(idx),
                    tau.at(idx),
                    qddotCus);

    CHECK_THAT (qddotRef,
                AllCloseVector(qddotCus, TEST_PREC, TEST_PREC)
    );
  }

}

TEST_CASE_METHOD (CustomJointSingleBodyFixture,
                  __FILE__"_CalcMInvTimestau", "") {

  for(int idx =0; idx < NUMBER_OF_MODELS; ++idx){

    int dof = reference_model.at(idx).dof_count;

    for (unsigned int i = 0; i < dof; i++) {
      q.at(idx)[i]      = (i+0.1) * 9.133758561390194e-01;
      tau.at(idx)[i]    = (i+0.1) * 9.754040499940952e-02;

    }

    //reference
    VectorNd qddot_minv_ref = VectorNd::Zero(dof);

    CalcMInvTimesTau(reference_model.at(idx),
                     q.at(idx),
                     tau.at(idx),
                     qddot_minv_ref,
                     true);

    //custom
    VectorNd qddot_minv_cus = VectorNd::Zero(dof);

    CalcMInvTimesTau(custom_model.at(idx),
                     q.at(idx),
                     tau.at(idx),
                     qddot_minv_cus,
                     true);
    //check.
    CHECK_THAT(qddot_minv_ref,
               AllCloseVector(qddot_minv_cus, TEST_PREC, TEST_PREC)
    );
  }

}

TEST_CASE_METHOD (CustomJointSingleBodyFixture,
                  __FILE__"_ForwardDynamicsContactsKokkevis", ""){

  for(int idx =0; idx < NUMBER_OF_MODELS; ++idx){

    int dof = reference_model.at(idx).dof_count;

    //Adding a 1 constraint to a system with 1 dof is
    //a no-no
    if(dof > 1){

      for (unsigned int i = 0; i < dof; i++) {
        q.at(idx)[i]     = (i+0.1) * 9.133758561390194e-01;
        qdot.at(idx)[i]    = (i+0.1) * 6.323592462254095e-01;

        tau.at(idx)[i]     = (i+0.1) * 9.754040499940952e-02;
      }

      VectorNd qddot_ref = VectorNd::Zero(dof);
      VectorNd qddot_cus = VectorNd::Zero(dof);

      VectorNd qdot_plus_ref = VectorNd::Zero(dof);
      VectorNd qdot_plus_cus = VectorNd::Zero(dof);

      Vector3d contact_point ( 0., 1., 0.);

      ConstraintSet constraint_set_ref;
      ConstraintSet constraint_set_cus;

      //Reference
      constraint_set_ref.AddContactConstraint( reference_body_id.at(idx),
                                        contact_point,
                                        Vector3d (1., 0., 0.),
                                        "ground_x");

      constraint_set_ref.AddContactConstraint( reference_body_id.at(idx),
                                        contact_point,
                                        Vector3d (0., 1., 0.),
                                        "ground_y");

      constraint_set_ref.Bind (reference_model.at(idx));

      //Custom
      constraint_set_cus.AddContactConstraint( custom_body_id.at(idx),
                                        contact_point,
                                        Vector3d (1., 0., 0.),
                                        "ground_x");

      constraint_set_cus.AddContactConstraint( custom_body_id.at(idx),
                                        contact_point,
                                        Vector3d (0., 1., 0.),
                                        "ground_y");

      constraint_set_cus.Bind (custom_model.at(idx));

      ComputeConstraintImpulsesDirect(reference_model.at(idx),
                                   q.at(idx),
                                   qdot.at(idx),
                                   constraint_set_ref,
                                   qdot_plus_ref);

      ForwardDynamicsContactsKokkevis (reference_model.at(idx),
                                       q.at(idx),
                                       qdot_plus_ref,
                                       tau.at(idx),
                                       constraint_set_ref,
                                       qddot_ref);

      ComputeConstraintImpulsesDirect(custom_model.at(idx),
                                   q.at(idx),
                                   qdot.at(idx),
                                   constraint_set_cus,
                                   qdot_plus_cus);

      ForwardDynamicsContactsKokkevis (custom_model.at(idx),
                                       q.at(idx),
                                       qdot_plus_cus,
                                       tau.at(idx),
                                       constraint_set_cus,
                                       qddot_cus);

      VectorNd qdot_plus_error = qdot_plus_ref - qdot_plus_cus;
      VectorNd qddot_error     = qddot_ref     - qddot_cus;

      CHECK_THAT (  qdot_plus_ref,
                    AllCloseVector(qdot_plus_cus, TEST_PREC, TEST_PREC)
      );

      CHECK_THAT (  qddot_ref,
                    AllCloseVector(qddot_cus, TEST_PREC, TEST_PREC)
      );
    }
  }

}


