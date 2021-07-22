#include "rbdl/rbdl.h"
#include <cassert>
#include "rbdl_tests.h"

#include "PendulumModels.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-11;

// Reduce an angle to the (-pi, pi] range.
static double inRange(double angle) {
  while(angle > M_PI) {
    angle -= 2. * M_PI;
  }
  while(angle <= -M_PI) {
    angle += 2. * M_PI;
  }
  return angle;
}



TEST_CASE(__FILE__"_ForwardDynamicsConstraintsWithExternalForcesCorrectnessTest",
          "")
{
  DoublePerpendicularPendulumAbsoluteCoordinates dba
    = DoublePerpendicularPendulumAbsoluteCoordinates();
  DoublePerpendicularPendulumJointCoordinates dbj
    = DoublePerpendicularPendulumJointCoordinates();

  //1. Set the pendulum modeled using joint coordinates to a specific
  //    state and then compute the spatial acceleration of the body.
  dbj.q[0]  = M_PI/3.0;       //About z0
  dbj.q[1]  = M_PI/6.0;       //About y1
  dbj.qd[0] = M_PI;           //About z0
  dbj.qd[1] = M_PI/2.0;       //About y1
  dbj.tau[0]= 0.;
  dbj.tau[1]= 0.;

  std::vector < SpatialVector > fext_dbj,fext_dba;

  fext_dbj.resize( dbj.model.mBodies.size() );
  fext_dba.resize( dba.model.mBodies.size() );

  for(unsigned int i=0; i<dbj.model.mBodies.size();++i){
    fext_dbj[i]=SpatialVector::Zero();
  }
  for(unsigned int i=0; i<dba.model.mBodies.size();++i){
    fext_dba[i]=SpatialVector::Zero();
  }

  SpatialVector fextB1 = SpatialVector( 7.922073295595544e+00,
                                        9.594924263929030e+00,
                                        6.557406991565868e+00,
                                        3.571167857418955e-01,
                                        8.491293058687772e+00,
                                        9.339932477575505e+00);

  SpatialVector fextB2 = SpatialVector( 6.787351548577734e+00,
                                        7.577401305783335e+00,
                                        7.431324681249162e+00,
                                        3.922270195341682e+00,
                                        6.554778901775567e+00,
                                        1.711866878115618e+00);

  fext_dbj[dbj.idB1] = fextB1;
  fext_dbj[dbj.idB2] = fextB2;
  fext_dba[dba.idB1] = fextB1;
  fext_dba[dba.idB2] = fextB2;

  for(unsigned int i=0; i<dbj.qdd.size();++i){
    dbj.qdd[i]=0.;
  }
  UpdateKinematics(dbj.model,dbj.q,dbj.qd,dbj.qdd);
  ForwardDynamics(dbj.model,dbj.q,dbj.qd,dbj.tau,dbj.qdd,&fext_dbj);

  Vector3d r010 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB1,
                    Vector3d(0.,0.,0.),true);
  Vector3d r020 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB2,
                    Vector3d(0.,0.,0.),true);
  Vector3d r030 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB2,
                    Vector3d(dbj.l2,0.,0.),true);

  SpatialVector v010 = CalcPointVelocity6D(
                        dbj.model,dbj.q,dbj.qd,dbj.idB1,
                        Vector3d(0.,0.,0.),true);
  SpatialVector v020 = CalcPointVelocity6D(
                        dbj.model,dbj.q,dbj.qd,dbj.idB2,
                        Vector3d(0.,0.,0.),true);
  SpatialVector v030 = CalcPointVelocity6D(
                        dbj.model,dbj.q,dbj.qd,dbj.idB2,
                        Vector3d(dbj.l2,0.,0.),true);

  SpatialVector a010 = CalcPointAcceleration6D(
                        dbj.model,dbj.q,dbj.qd,dbj.qdd,
                        dbj.idB1,Vector3d(0.,0.,0.),true);
  SpatialVector a020 = CalcPointAcceleration6D(
                        dbj.model,dbj.q,dbj.qd,dbj.qdd,
                        dbj.idB2,Vector3d(0.,0.,0.),true);
  SpatialVector a030 = CalcPointAcceleration6D(
                        dbj.model,dbj.q,dbj.qd,dbj.qdd,
                        dbj.idB2,Vector3d(dbj.l2,0.,0.),true);

  //2. Set the pendulum modelled using absolute coordinates to the
  //   equivalent state as the pendulum modelled using joint
  //   coordinates. Next

  /*
  dba.q[0]  = dbj.q[0];
  dba.q[1]  = r020[0];
  dba.q[2]  = r020[1];
  dba.q[3]  = r020[2];
  dba.q[4]  = 0;
  dba.q[5]  = 0;
  dba.q[6]  = dbj.q[0]+dbj.q[1];

  dba.qd[0]  = dbj.qd[0];
  dba.qd[1]  = v020[0];
  dba.qd[2]  = v020[1];
  dba.qd[3]  = v020[2];
  dba.qd[4]  = 0;
  dba.qd[5]  = 0;
  dba.qd[6]  = dbj.qd[0]+dbj.qd[1];
  */


  dba.q[0]  = r010[0];
  dba.q[1]  = r010[1];
  dba.q[2]  = r010[2];
  dba.q[3]  = dbj.q[0];
  dba.q[4]  = 0;
  dba.q[5]  = 0;
  dba.q[6]  = r020[0];
  dba.q[7]  = r020[1];
  dba.q[8]  = r020[2];
  dba.q[9]  = dbj.q[0];
  dba.q[10] = dbj.q[1];
  dba.q[11] = 0;

  dba.qd[0]  = v010[3];
  dba.qd[1]  = v010[4];
  dba.qd[2]  = v010[5];
  dba.qd[3]  = dbj.qd[0];
  dba.qd[4]  = 0;
  dba.qd[5]  = 0;
  dba.qd[6]  = v020[3];
  dba.qd[7]  = v020[4];
  dba.qd[8]  = v020[5];
  dba.qd[9]  = dbj.qd[0];
  dba.qd[10] = dbj.qd[1];
  dba.qd[11] = 0;

  VectorNd err(dba.cs.size());
  VectorNd errd(dba.cs.size());

  CalcConstraintsPositionError(dba.model,dba.q,dba.cs,err,true);
  CalcConstraintsVelocityError(dba.model,dba.q,dba.qd,dba.cs,errd,true);

  //The constraint errors at the position and velocity level
  //must be zero before the accelerations can be tested.
  for(unsigned int i=0; i<err.rows();++i){
    CHECK_THAT(0,IsClose(err[i],TEST_PREC, TEST_PREC));
  }
  for(unsigned int i=0; i<errd.rows();++i){
    CHECK_THAT(0,IsClose(errd[i],TEST_PREC, TEST_PREC));
  }

  //Evaluate the accelerations of the constrained pendulum and
  //compare those to the joint-coordinate pendulum
  for(unsigned int i=0; i<dba.tau.rows();++i){
    dba.tau[i] = 0.;
  }

  ForwardDynamicsConstraintsDirect(dba.model,dba.q,dba.qd,
                                   dba.tau,dba.cs,dba.qdd,&fext_dba);
  SpatialVector a010c =
      CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB1,Vector3d(0.,0.,0.),true);
  SpatialVector a020c =
      CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(0.,0.,0.),true);
  SpatialVector a030c =
      CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(dba.l2,0.,0.),true);

  for(unsigned int i=0; i<6;++i){
    CHECK_THAT(a010[i],IsClose(a010c[i],TEST_PREC, TEST_PREC));
    CHECK_THAT(a020[i],IsClose(a020c[i],TEST_PREC, TEST_PREC));
    CHECK_THAT(a030[i],IsClose(a030c[i],TEST_PREC, TEST_PREC));
  }

  ForwardDynamicsConstraintsNullSpace(
        dba.model,dba.q,dba.qd,
        dba.tau,dba.cs,dba.qdd,&fext_dba);

  a010c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB1,Vector3d(0.,0.,0.),true);
  a020c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(0.,0.,0.),true);
  a030c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(dba.l2,0.,0.),true);

  for(unsigned int i=0; i<6;++i){
    CHECK_THAT(a010[i],IsClose(a010c[i],TEST_PREC, TEST_PREC));
    CHECK_THAT(a020[i],IsClose(a020c[i],TEST_PREC, TEST_PREC));
    CHECK_THAT(a030[i],IsClose(a030c[i],TEST_PREC, TEST_PREC));
  }


  ForwardDynamicsConstraintsRangeSpaceSparse(
        dba.model,dba.q,dba.qd,
        dba.tau,dba.cs,dba.qdd, &fext_dba);
  a010c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB1,Vector3d(0.,0.,0.),true);
  a020c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(0.,0.,0.),true);
  a030c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(dba.l2,0.,0.),true);

  for(unsigned int i=0; i<6;++i){
    CHECK_THAT(a010[i],IsClose(a010c[i],TEST_PREC, TEST_PREC));
    CHECK_THAT(a020[i],IsClose(a020c[i],TEST_PREC, TEST_PREC));
    CHECK_THAT(a030[i],IsClose(a030c[i],TEST_PREC, TEST_PREC));
  }

}
