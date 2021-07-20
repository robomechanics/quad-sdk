#include "rbdl/rbdl.h"
#include "rbdl/Constraint.h"
#include "rbdl/Constraints.h"
#include <cassert>

#include "rbdl_tests.h"

#include "PendulumModels.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-11;


class PinJointCustomConstraint : public RigidBodyDynamics::Constraint
{

public:

  PinJointCustomConstraint()
    :Constraint(NULL,ConstraintTypeCustom,5,
                std::numeric_limits<unsigned int>::max())
  {
  }

  PinJointCustomConstraint(//const unsigned int rowInSystem,
      const unsigned int bodyIdPredecessor,
      const unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &bodyFramePredecessor,
      const Math::SpatialTransform &bodyFrameSuccessor,
      unsigned int x0y1z2,
      const char *name = NULL,
      unsigned int userDefinedId=std::numeric_limits<unsigned int>::max(),
      bool enableBaumgarteStabilization=false,
      double baumgarteTStab = 0.1):
        Constraint(name,ConstraintTypeCustom,5,userDefinedId)
  {
    //Configure the parent member variables
    bodyIds.push_back(bodyIdPredecessor);
    bodyIds.push_back(bodyIdSuccessor);
    bodyFrames.push_back(bodyFramePredecessor);
    bodyFrames.push_back(bodyFrameSuccessor);

    enableConstraintErrorFromPositionLevel();
    setBaumgarteTimeConstant(baumgarteTStab);
    setEnableBaumgarteStabilization(enableBaumgarteStabilization);

    //Configure the local variables
    T.resize(sizeOfConstraint);
    for(unsigned int i=0; i<T.size(); ++i){
      T[i].setZero();
    }

    /*A pin joint has constraints about these axis when expressed
      in the frame of the predecessor body:
          0   1   2   3   4   5
         wx  wy  wz  rx  ry  rz
     0: [ 0   1   0   0   0   0 ]
     1: [ 0   0   1   0   0   0 ]
     2: [ 0   0   0   1   0   0 ]
     3: [ 0   0   0   0   1   0 ]
     4: [ 0   0   0   0   0   1 ]
    */
    switch(x0y1z2){
      case 0:
      {
        T[3][1] = 1;
        T[4][2] = 1;
      }break;
      case 1:{
        T[3][0] = 1;
        T[4][2] = 1;
      }break;
      case 2:{
        T[3][0] = 1;
        T[4][1] = 1;
      }break;
      default: {
        cerr << "Invalid AxisOfRotation argument" << endl;
      }
    };

    T[0][3] = 1;
    T[1][4] = 1;
    T[2][5] = 1;

  }

  void bind(const Model &model) override
  {
    //This function does not need to resize any of its own local
    //memory (it makes use of the memory in cache) so this function is blank.
  }

  void calcConstraintJacobian(  Model &model,
                              const double time,
                              const Math::VectorNd &Q,
                              const Math::VectorNd &QDot,
                              Math::MatrixNd &GSysUpd,
                              ConstraintCache &cache,
                              bool updKin) override
  {
    //Set the working matrices for the previous/successor point Jacobians
    //to zero
    cache.mat6NA.setZero(); //predecessor
    cache.mat6NB.setZero(); //successor

    CalcPointJacobian6D(model,Q,bodyIds[0],bodyFrames[0].r,cache.mat6NA,updKin);
    CalcPointJacobian6D(model,Q,bodyIds[1],bodyFrames[1].r,cache.mat6NB,updKin);
    cache.mat6NA = cache.mat6NB-cache.mat6NA;

    cache.stA.r = CalcBodyToBaseCoordinates (model, Q, bodyIds[0],
                                                bodyFrames[0].r, false);
    cache.stA.E = CalcBodyWorldOrientation (model, Q, bodyIds[0], false
                                                ).transpose()* bodyFrames[0].E;

    for(unsigned int i=0; i<T.size(); ++i){
      cache.svecA = cache.stA.apply(T[i]);
      GSysUpd.block(rowInSystem+i,0,1,GSysUpd.cols())
          = cache.svecA.transpose()*cache.mat6NA;
    }
  }

  void calcGamma( Model &model,
                  const double time,
                  const Math::VectorNd &Q,
                  const Math::VectorNd &QDot,
                  const Math::MatrixNd &GSys,
                  Math::VectorNd &gammaSysUpd,
                  ConstraintCache &cache,
                  bool updKin) override
  {
  /*
    Position-level

      phi(q) = 0
      r0P-r0S   = 0 : the points p and q are coincident.
      e1x'e2y   = 0 : the x axis of frame 1 is perp. to y axis of frame 2
      e1x'e2z   = 0 : the x axis of frame 1 is perp. to z axis of frame 2

    Velocity-level
      D_phi(q)_Dq * dq/dt = 0
      [J_r0P0_q             - J_r0Q0_q] dq/dt = 0
      [J_e1x0_q'*e2y0 + e1x1'*J_e2y0_q] dq/dt = 0
      [J_e1x0_q'*e2z0 + e1x1'*J_e2z0_q] dq/dt = 0

      Or equivalently

      Tu[vP - vS] = 0

      where Tu are the directions the constraint is applied in and
      vP and vQ are the spatial velocities of points P and Q. This
      can be re-worked into:

      G(q)*dq/dt = 0

      Where G(q) is the Jacobian of the constraint phi w.r.t. q.

    Acceleration-level

      d/dt(Tu)[vP - vS] + Tu[aP - aS] = 0

      This is equivalent to

      G(q)*d^2q/dt^2 + [D_G(q)_Dq * dq/dt]*dq/dt = 0.

      Gamma is the term on the right. Note that it has no d^2q/dt^2 terms.
      Thus the gamma term can be computed by evaluating

      Gamma = - (vP x Tu)[vP - vS] - Tu[aP* - aS*]

      where aP* and aQ* are the spatial accelerations of points P and Q
      evaluated with d^2q/dt^2 = 0.
  */
    //v0P0
    //Velocity of the point on the predecessor body
    cache.svecA = CalcPointVelocity6D(model, Q, QDot,
                               bodyIds[0],
                               bodyFrames[0].r,updKin);
    //v0S0
    //Velocity of the point on the successor body
    cache.svecB = CalcPointVelocity6D(model, Q, QDot,
                               bodyIds[1],
                               bodyFrames[1].r,updKin);
    //GPQ0
    //Jacobian of r0P0
    cache.svecC = CalcPointAcceleration6D(model, Q, QDot,
                                   cache.vecNZeros,
                                   bodyIds[0],
                                   bodyFrames[0].r,updKin);
    //GSQ0
    //Jacobian of r0S0
    cache.svecD = CalcPointAcceleration6D(model, Q, QDot,
                                   cache.vecNZeros,
                                   bodyIds[1],
                                   bodyFrames[1].r,updKin);
    //r0P0
    cache.stA.r = CalcBodyToBaseCoordinates (model, Q, bodyIds[0],
                                            bodyFrames[0].r, updKin);

    //E0P - rotation matrix from the 0th frame to the P frame
    cache.stA.E = CalcBodyWorldOrientation (model, Q, bodyIds[0], updKin
                                              ).transpose() * bodyFrames[0].E;


    for(unsigned int i=0; i<T.size(); ++i){
      cache.svecE = cache.stA.apply(T[i]); //constraint axis
      cache.svecF = crossm(cache.svecA,cache.svecE);//constraint axis derivative
      gammaSysUpd[rowInSystem+i] =  -cache.svecE.dot(cache.svecD-cache.svecC)
                                    -cache.svecF.dot(cache.svecB-cache.svecA);
    }

  }

  void calcPositionError( Model &model,
                          const double time,
                          const Math::VectorNd &Q,
                          Math::VectorNd &errSysUpd,
                          ConstraintCache &cache,
                          bool updKin) override
  {
    //r0P0
    cache.vec3A = CalcBodyToBaseCoordinates (model, Q, bodyIds[0],
                                                bodyFrames[0].r, false);

    //r0S0
    cache.vec3B = CalcBodyToBaseCoordinates (model, Q, bodyIds[1],
                                                bodyFrames[1].r, false);

    //EP0 : rotation matrix from the predecessor frame to the root frame
    cache.mat3A = CalcBodyWorldOrientation (model, Q, bodyIds[0], false
                                     ).transpose()* bodyFrames[0].E;

    //ES0 : rotation matrix from the successor frame to the root frame
    cache.mat3B = CalcBodyWorldOrientation (model, Q, bodyIds[1], false
                                     ).transpose()* bodyFrames[1].E;

    cache.mat3C = cache.mat3A.transpose()*cache.mat3B;

    cache.svecA[0] = -0.5 * (cache.mat3C(1,2) - cache.mat3C(2,1));
    cache.svecA[1] = -0.5 * (cache.mat3C(2,0) - cache.mat3C(0,2));
    cache.svecA[2] = -0.5 * (cache.mat3C(0,1) - cache.mat3C(1,0));
    cache.svecA.block(3,0,3,1) =
        cache.mat3A.transpose()*(cache.vec3B - cache.vec3A);

    for(unsigned int i=0; i < T.size(); ++i){
      errSysUpd[rowInSystem+i] = T[i].transpose()*cache.svecA;
    }

  }

  void calcVelocityError( Model &model,
                                      const double time,
                                      const Math::VectorNd &Q,
                                      const Math::VectorNd &QDot,
                                      const Math::MatrixNd &GSys,
                                      Math::VectorNd &derrSysUpd,
                                      ConstraintCache &cache,
                                      bool updKin) override
  {
    //Since this is a time-invariant constraint the expression for
    //the velocity error is quite straight forward:
    for(unsigned int i=0; i < sizeOfConstraint;++i){
      derrSysUpd[rowInSystem+i] = 0.;
      for(unsigned int j=0; j<GSys.cols();++j){
        derrSysUpd[rowInSystem+i] += GSys(rowInSystem+i,j)*QDot[j];
      }
    }
  }

  void calcConstraintForces(
         Model &model,
         const double time,
         const Math::VectorNd &Q,
         const Math::VectorNd &QDot,
         const Math::MatrixNd &GSys,
         const Math::VectorNd &LagrangeMultipliersSys,
         std::vector< unsigned int > &constraintBodiesUpd,
         std::vector< Math::SpatialTransform > &constraintBodyFramesUpd,
         std::vector< Math::SpatialVector > &constraintForcesUpd,
         ConstraintCache &cache,
         bool resolveAllInRootFrame,
         bool updKin) override
  {

    constraintBodiesUpd.resize(2);
    constraintBodyFramesUpd.resize(2);

    cache.stA.r = CalcBodyToBaseCoordinates(model,Q,bodyIds[0],bodyFrames[0].r,
                                            updKin);
    //The rotation matrix from the predecessor frame to the base frame
    // 0_E_P
    cache.stA.E = CalcBodyWorldOrientation(model,Q,bodyIds[0],updKin
                                           ).transpose()*bodyFrames[0].E;


    cache.stB.r = CalcBodyToBaseCoordinates(model,Q,bodyIds[1],bodyFrames[1].r,
                                            updKin);
    //The rotation matrix from the successor frame to the base frame
    // 0_E_S
    cache.stB.E = CalcBodyWorldOrientation(model,Q,bodyIds[1],updKin
                                           ).transpose()*bodyFrames[1].E;

    constraintForcesUpd.resize(2);
    constraintForcesUpd[0].setZero();
    constraintForcesUpd[1].setZero();

    //Using Eqn. 8.30 of Featherstone. Note that this force is resolved in the
    //predecessor frame.
    cache.svecB.setZero();
    for(unsigned int i=0; i<sizeOfConstraint;++i){
      cache.svecA =  cache.stA.apply(T[i]);
      cache.svecB += cache.svecA*LagrangeMultipliersSys[rowInSystem+i];
    }

    //cache.vec3A[0] = cache.svecB[3];
    //cache.vec3A[1] = cache.svecB[4];
    //cache.vec3A[2] = cache.svecB[5];
    //double mag = cache.vec3A.norm();


    constraintBodiesUpd.resize(2);
    constraintBodyFramesUpd.resize(2);

    if(resolveAllInRootFrame){
      constraintBodiesUpd[0] = 0;
      constraintBodiesUpd[1] = 0;

      //These forces are returned in the coordinates of the
      //root frame but w.r.t. the respective points of the constaint
      constraintBodyFramesUpd[0].r = cache.stA.r;
      constraintBodyFramesUpd[0].E.Identity();

      constraintBodyFramesUpd[1].r = cache.stB.r;
      constraintBodyFramesUpd[1].E.Identity();


      //The forces applied to the successor body are equal and opposite
      constraintForcesUpd[0] = -cache.svecB;
      constraintForcesUpd[1] = cache.svecB;

    }else{

      constraintBodiesUpd     = bodyIds;
      constraintBodyFramesUpd = bodyFrames;

      constraintForcesUpd[0].block(0,0,3,1) = -cache.stA.E.transpose()
                                                *cache.svecB.block(0,0,3,1);
      constraintForcesUpd[0].block(3,0,3,1) = -cache.stA.E.transpose()
                                                *cache.svecB.block(3,0,3,1);


      constraintForcesUpd[1].block(0,0,3,1) = cache.stB.E.transpose()
                                                *cache.svecB.block(0,0,3,1);
      constraintForcesUpd[1].block(3,0,3,1) = cache.stB.E.transpose()
                                                *cache.svecB.block(3,0,3,1);



    }


  }

private:
  std::vector < SpatialVector > T; //Constraint direction vectors resolved in
                                     //the frame that P is on.
  //SpatialVector err, eT0, eT0Dot, v0P0,  v0S0, dv0P0nl ,dv0S0nl;
  //Vector3d r0P0, r0S0;
  //Matrix3d rmP0, rmS0, rmPS;
  //SpatialTransform xP0;


  //VectorNd errVelBlock;

};



class DoublePerpendicularPendulumCustomConstraint {

public:
  DoublePerpendicularPendulumCustomConstraint()
    : model()
    , cs()
    , q()
    , qd()
    , qdd()
    , tau()
    , l1(1.)
    , l2(1.)
    , m1(1.)
    , m2(1.)
    , idB1(0)
    , idB2(0)
    , X_p1(Xtrans(Vector3d(0., 0., 0.)))
    , X_s1(Xtrans(Vector3d(0., 0., 0.)))
    , X_p2(Xtrans(Vector3d(0.,-l1, 0.)))
    , X_s2(Xtrans(Vector3d(0., 0., 0.))){

    model.gravity = Vector3d(0.,-9.81,0.);
    //Planar pendulum is at 0 when it is hanging down.
    //  x: points to the right
    //  y: points up
    //  z: out of the page
    Body link1 = Body(m1, Vector3d(          0., -l1*0.5,          0.),
                      Matrix3d( m1*l1*l1/3.,          0.,           0.,
                                          0., m1*l1*l1/30.,           0.,
                                          0.,          0.,  m1*l1*l1/3.));


    Body link2 = Body(m2, Vector3d( l2*0.5,          0.,          0.),
                          Matrix3d( m2*l2*l2/30.,          0.,           0.,
                                              0., m2*l2*l2/3.,           0.,
                                              0.,          0.,  m2*l2*l2/3.));

    //Joint joint_free(JointTypeFloatingBase);
    Joint jointEA123T123(SpatialVector(0.,0.,0.,1.,0.,0.),
                         SpatialVector(0.,0.,0.,0.,1.,0.),
                         SpatialVector(0.,0.,0.,0.,0.,1.),
                         SpatialVector(0.,0.,1.,0.,0.,0.),
                         SpatialVector(0.,1.,0.,0.,0.,0.),
                         SpatialVector(1.,0.,0.,0.,0.,0.));

    idB1  = model.AddBody(0, Xtrans(Vector3d(0., 0., 0. )),
                         jointEA123T123, link1);

    idB2  = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.)),
                         jointEA123T123, link2);

    //Make the revolute joints about the y axis using 5 constraints
    //between the end points

    PinJointCustomConstraint zJoint(0,idB1,X_p1,X_s1,2,"Rz",7);
    PinJointCustomConstraint yJoint(idB1,idB2,X_p2,X_s2,1,"Ry",11);

    ccPJZaxis = std::make_shared<PinJointCustomConstraint>(zJoint);
    ccPJYaxis = std::make_shared<PinJointCustomConstraint>(yJoint);

    assignedIdRz = cs.AddCustomConstraint(ccPJZaxis);
    assignedIdRy = cs.AddCustomConstraint(ccPJYaxis);
    cs.Bind(model);

    q   = VectorNd::Zero(model.dof_count);
    qd  = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);

  }

    Model model;
    ConstraintSet cs;
    unsigned int assignedIdRz, assignedIdRy;

    VectorNd q;
    VectorNd qd;
    VectorNd qdd;
    VectorNd tau;

    double l1;
    double l2;
    double m1;
    double m2;

    unsigned int idB1;
    unsigned int idB2;
    unsigned int idB01;
    unsigned int idB02;

    SpatialTransform X_p1;
    SpatialTransform X_s1;
    SpatialTransform X_p2;
    SpatialTransform X_s2;

    std::shared_ptr<PinJointCustomConstraint> ccPJZaxis;
    std::shared_ptr<PinJointCustomConstraint> ccPJYaxis;

};



TEST_CASE(__FILE__"_CustomConstraintCorrectnessTest", "") {

  //Test to add:
  //  Jacobian vs. num Jacobian
  DoublePerpendicularPendulumCustomConstraint dbcc
    = DoublePerpendicularPendulumCustomConstraint();
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

  unsigned int groupIndex = dbcc.cs.getGroupIndexByName("Rz");
  assert(groupIndex==0);
  groupIndex = dbcc.cs.getGroupIndexById(7);
  assert(groupIndex==0);
  groupIndex = dbcc.cs.getGroupIndexByAssignedId(dbcc.assignedIdRy);
  assert(groupIndex==1);


  ForwardDynamics(dbj.model,dbj.q,dbj.qd,dbj.tau,dbj.qdd);

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

  double qError = 1.0;
  double qDotError = 1.0;

  //Pefectly initialize the pendulum made with custom constraints and
  //perturb the initialization a bit.
  dbcc.q[0]  = r010[0];
  dbcc.q[1]  = r010[1]+qError;
  dbcc.q[2]  = r010[2];
  dbcc.q[3]  = dbj.q[0];
  dbcc.q[4]  = 0;
  dbcc.q[5]  = 0;
  dbcc.q[6]  = r020[0];
  dbcc.q[7]  = r020[1];
  dbcc.q[8]  = r020[2];
  dbcc.q[9]  = dbj.q[0];
  dbcc.q[10] = dbj.q[1];
  dbcc.q[11] = 0;

  dbcc.qd[0]  = v010[3];
  dbcc.qd[1]  = v010[4]+qDotError;
  dbcc.qd[2]  = v010[5];
  dbcc.qd[3]  = dbj.qd[0];
  dbcc.qd[4]  = 0;
  dbcc.qd[5]  = 0;
  dbcc.qd[6]  = v020[3];
  dbcc.qd[7]  = v020[4];
  dbcc.qd[8]  = v020[5];
  dbcc.qd[9]  = dbj.qd[0];
  dbcc.qd[10] = dbj.qd[1];
  dbcc.qd[11] = 0;

  VectorNd err(dbcc.cs.size());
  VectorNd errd(dbcc.cs.size());


  CalcConstraintsPositionError(dbcc.model,dbcc.q,dbcc.cs,err,true);
  CalcConstraintsVelocityError(dbcc.model,dbcc.q,dbcc.qd,dbcc.cs,errd,true);


  CHECK(err.norm()  >= qError);
  CHECK(errd.norm() >= qDotError);

  //Solve for the initial q and qdot terms that satisfy the constraints
  VectorNd qAsm,qDotAsm,w;
  qAsm.resize(dbcc.q.rows());
  qDotAsm.resize(dbcc.q.rows());
  w.resize(dbcc.q.rows());
  for(unsigned int i=0; i<w.rows();++i){
    w[i] = 1.0;
  }
  double tol = 1e-8;
  unsigned int maxIter = 100;

  CalcAssemblyQ(dbcc.model,dbcc.q,dbcc.cs,qAsm,w,tol,maxIter);
  for(unsigned int i=0; i<dbcc.q.rows();++i){
    dbcc.q[i] = qAsm[i];
  }

  CalcAssemblyQDot(dbcc.model,dbcc.q,dbcc.qd,dbcc.cs,qDotAsm,w);
  for(unsigned int i=0; i<dbcc.q.rows();++i){
    dbcc.qd[i] = qDotAsm[i];
  }

  CalcConstraintsPositionError(dbcc.model,dbcc.q,dbcc.cs,err,true);
  CalcConstraintsVelocityError(dbcc.model,dbcc.q,dbcc.qd,dbcc.cs,errd,true);


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
  dba.q = dbcc.q;
  dba.qd= dbcc.qd;

  for(unsigned int i=0; i<dbcc.tau.rows();++i){
    dbcc.tau[i] = 0.;
  }
  ForwardDynamicsConstraintsDirect(dbcc.model,dbcc.q,dbcc.qd,
                                   dbcc.tau,dbcc.cs,dbcc.qdd);

  ForwardDynamicsConstraintsDirect(dba.model, dba.q,  dba.qd,
                                   dba.tau,   dba.cs, dba.qdd);

  for(unsigned int i = 0; i < dba.cs.G.rows(); ++i){
    for(unsigned int j=0; j< dba.cs.G.cols();++j){
      CHECK_THAT(dba.cs.G(i,j),IsClose(dbcc.cs.G(i,j),TEST_PREC, TEST_PREC));
    }
  }

  for(unsigned int i = 0; i < dba.cs.gamma.rows(); ++i){
    CHECK_THAT(dba.cs.gamma[i],IsClose(dbcc.cs.gamma[i],TEST_PREC, TEST_PREC));
  }

  //This test cannot be performed with the updated Constraint interface
  //because the constraintAxis field is obsolete. Instead we can test
  //the constraint forces
  //unsigned int idx=0;
  //for(unsigned int i=0; i < dba.cs.loopConstraints.size();++i){
  //  for(unsigned int j=0;
  //      j < dba.cs.loopConstraints[i]->getConstraintSize();++j){
  //    for(unsigned int k=0; k<6; ++k){
  //      CHECK_CLOSE(dba.cs.loopConstraints[i]->getConstraintAxes()[j][k],
  //                  dbcc.cs.constraintAxis[idx][k],TEST_PREC);
  //    }
  //    ++idx;
  //  }
  //}

  std::vector< unsigned int >     bodyIdDba,      bodyIdDbcc;
  std::vector< SpatialTransform > bodyFramesDba,  bodyFramesDbcc;
  std::vector < SpatialVector >   forcesDba,      forcesDbcc;

  bool updKin       = false;
  bool inBaseFrame  = false;

  for(unsigned int i=0; i<dba.cs.constraints.size();++i){
    for(unsigned int j=0; j<2; ++j){
      if(j == 0){
        inBaseFrame = false;
      }else{
        inBaseFrame = true;
      }
      dba.cs.constraints[i]->calcConstraintForces(dba.model, 0., dba.q, dba.qd,
                                          dba.cs.G, dba.cs.force,
                                          bodyIdDba, bodyFramesDba, forcesDba,
                                          dba.cs.cache,inBaseFrame,updKin);

      dbcc.cs.constraints[i]->calcConstraintForces(dbcc.model,0.,dbcc.q,dbcc.qd,
                                          dbcc.cs.G, dbcc.cs.force,
                                          bodyIdDbcc, bodyFramesDbcc,forcesDbcc,
                                          dbcc.cs.cache,inBaseFrame,updKin);

      CHECK( (bodyIdDba.size()    - bodyIdDbcc.size()    ) == 0);
      CHECK( (bodyFramesDba.size()- bodyFramesDbcc.size()) == 0);
      CHECK( (forcesDba.size()    - forcesDbcc.size()    ) == 0);

      for(unsigned int k=0; k < bodyIdDba.size();++k){
        CHECK_THAT( bodyFramesDba[k].r,
                    AllCloseVector(bodyFramesDbcc[k].r, TEST_PREC, TEST_PREC)
        );
        CHECK_THAT(bodyFramesDba[k].E,
                   AllCloseMatrix(bodyFramesDbcc[k].E, TEST_PREC, TEST_PREC)
        );
        CHECK_THAT( forcesDba[k],
                    AllCloseVector(forcesDbcc[k], TEST_PREC, TEST_PREC)
        );
      }
    }
  }


  SpatialVector a010c =
      CalcPointAcceleration6D(dbcc.model,dbcc.q,dbcc.qd,dbcc.qdd,
                          dbcc.idB1,Vector3d(0.,0.,0.),true);
  SpatialVector a020c =
      CalcPointAcceleration6D(dbcc.model,dbcc.q,dbcc.qd,dbcc.qdd,
                          dbcc.idB2,Vector3d(0.,0.,0.),true);
  SpatialVector a030c =
      CalcPointAcceleration6D(dbcc.model,dbcc.q,dbcc.qd,dbcc.qdd,
                          dbcc.idB2,Vector3d(dbcc.l2,0.,0.),true);

  for(unsigned int i=0; i<6;++i){
    CHECK_THAT(a010[i],IsClose(a010c[i],TEST_PREC, TEST_PREC));
    CHECK_THAT(a020[i],IsClose(a020c[i],TEST_PREC, TEST_PREC));
    CHECK_THAT(a030[i],IsClose(a030c[i],TEST_PREC, TEST_PREC));
  }
}
