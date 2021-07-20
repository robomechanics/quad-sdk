#include "rbdl/rbdl.h"
#include "PendulumModels.h"

#include <chrono>
#include <thread>
#include <iostream>

#include "rbdl_tests.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-11;

const bool flag_printTimingData = false;

void calcCuboidInertia(double mass,
                       double xLength,
                       double yLength,
                       double zLength,
                       Vector3d& iner){
    iner[0] = (mass/12.)*(yLength*yLength + zLength*zLength);
    iner[1] = (mass/12.)*(xLength*xLength + zLength*zLength);
    iner[2] = (mass/12.)*(yLength*yLength + xLength*xLength);
}

struct PlanarBipedFloatingBase {
  PlanarBipedFloatingBase()
    : model()
      ,cs()
      ,q()
      ,qd()
      ,qdd()
      ,tau()
      ,pelvisWidth(0.2)
      ,segmentLength(0.5)
      ,pelvisMass(50.0)
      ,segmentMass(1.0)
      ,X_p(Xtrans(Vector3d(0.,0.,-segmentLength)))
      ,X_s(Xtrans(Vector3d(0.,0.,0.))) {

    model.gravity = Vector3d(0.,0.,-9.81);

    Body segment = Body(segmentMass,
                        Vector3d(0, 0., -0.5 * segmentLength),
                        Vector3d(0.,segmentMass*segmentLength*segmentLength
                                    /3.,0.));
    Body pelvis  = Body(pelvisMass, Vector3d(0, 0., 0.),
                        Vector3d(0.,pelvisMass*pelvisWidth*pelvisWidth/3.,0.));

    Joint joint_TxzRy = Joint(SpatialVector(0.,0.,0., 1.,0.,0.),
                              SpatialVector(0.,0.,0., 0.,0.,1.),
                              SpatialVector(0.,1.,0., 0.,0.,0.));

    Joint joint_Ry = Joint(SpatialVector(0.,1.,0., 0.,0.,0.));

    idxPelvis       = model.AddBody(0,
                                    Xtrans(Vector3d(0.,0.,0.)),
                                    joint_TxzRy, pelvis);
    idxLeftUpperLeg  = model.AddBody(idxPelvis,
                                    Xtrans(Vector3d(-pelvisWidth*0.5,0.,0.)),
                                    joint_Ry,segment);
    idxLeftLowerLeg = model.AddBody(idxLeftUpperLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Ry, segment);
    idxRightUpperLeg  = model.AddBody(idxPelvis,
                                    Xtrans(Vector3d( pelvisWidth*0.5,0.,0.)),
                                    joint_Ry,segment);
    idxRightLowerLeg = model.AddBody(idxRightUpperLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Ry, segment);

    cs.resize(2);

    //double stance
    cs[0].AddContactConstraint(idxLeftLowerLeg, Vector3d(0.,0.,-segmentLength),
                                             Vector3d(1.,0.,0.));
    cs[0].AddContactConstraint(idxLeftLowerLeg, Vector3d(0.,0.,-segmentLength),
                                             Vector3d(0.,0.,1.));
    cs[0].AddContactConstraint(idxRightLowerLeg, Vector3d(0.,0.,-segmentLength),
                                              Vector3d(1.,0.,0.));
    cs[0].AddContactConstraint(idxRightLowerLeg, Vector3d(0.,0.,-segmentLength),
                                              Vector3d(0.,0.,1.));

    cs[0].Bind(model);

    //single stance
    cs[1].AddContactConstraint(idxLeftLowerLeg, Vector3d(0.,0.,-segmentLength),
                                             Vector3d(1.,0.,0.));
    cs[1].AddContactConstraint(idxLeftLowerLeg, Vector3d(0.,0.,-segmentLength),
                                             Vector3d(0.,0.,1.));
    cs[1].Bind(model);


    q     = VectorNd::Zero(model.dof_count);
    qd    = VectorNd::Zero(model.dof_count);
    qdd   = VectorNd::Zero(model.dof_count);
    tau   = VectorNd::Zero(model.dof_count);

  }

  Model model;
  std::vector< ConstraintSet > cs;

  VectorNd q;
  VectorNd qd;
  VectorNd qdd;
  VectorNd tau;

  double pelvisWidth;
  double segmentLength;
  double pelvisMass;
  double segmentMass;

  unsigned int idxPelvis ;
  unsigned int idxLeftUpperLeg  ;
  unsigned int idxLeftLowerLeg  ;
  unsigned int idxRightUpperLeg ;
  unsigned int idxRightLowerLeg ;

  SpatialTransform X_p;
  SpatialTransform X_s;

};


struct SpatialBipedFloatingBase {
  SpatialBipedFloatingBase()
    : model()
      ,cs()
      ,q()
      ,qd()
      ,qdd()
      ,tau()
      ,pelvisWidth(0.2)
      ,segmentLength(0.5)
      ,footLength(0.3)
      ,pelvisMass(50.0)
      ,segmentMass(5.0)
      ,footMass(1.0)
      ,X_p(Xtrans(Vector3d(0.,0.,-segmentLength)))
      ,X_s(Xtrans(Vector3d(0.,0.,0.))) {


    //From the biped's perspective
    //x: forward
    //z: up
    //y: left

    model.gravity = Vector3d(0.,0.,-9.81);

    Vector3d segmentInertia, pelvisInertia, footInertia;
    calcCuboidInertia(segmentMass,
                      0.1*segmentLength,
                      0.1*segmentLength,
                      segmentLength,
                      segmentInertia);
    calcCuboidInertia(pelvisMass,
                      0.75*pelvisWidth,
                           pelvisWidth,
                      0.50*pelvisWidth,
                      pelvisInertia);
    calcCuboidInertia(footMass,
                          footLength,
                      0.1*footLength,
                      0.1*footLength,
                      footInertia);

    Body segment = Body(segmentMass,
                        Vector3d(0, 0., -0.5 * segmentLength),
                        segmentInertia);
    Body pelvis  = Body(pelvisMass,
                        Vector3d(0, 0., 0.),
                        pelvisInertia);

    Body foot    = Body(footMass,
                        Vector3d(0.5*footLength,0.,0.),
                        footInertia);

    Joint joint_TxyzRxyz = Joint(SpatialVector(0.,0.,0., 1.,0.,0.),
                                 SpatialVector(0.,0.,0., 0.,1.,0.),
                                 SpatialVector(0.,0.,0., 0.,0.,1.),
                                 SpatialVector(1.,0.,0., 0.,0.,0.),
                                 SpatialVector(0.,1.,0., 0.,0.,0.),
                                 SpatialVector(0.,0.,1., 0.,0.,0.));

    Joint joint_Ry  = Joint(SpatialVector(0.,1.,0., 0.,0.,0.));

    Joint joint_Rxy = Joint(SpatialVector(1.,0.,0., 0.,0.,0.),
                            SpatialVector(0.,1.,0., 0.,0.,0.));
    Joint joint_Rxyz = Joint(SpatialVector(1.,0.,0., 0.,0.,0.),
                             SpatialVector(0.,1.,0., 0.,0.,0.),
                             SpatialVector(0.,0.,1., 0.,0.,0.));


    idxPelvis       = model.AddBody(0,
                                    Xtrans(Vector3d(0.,0.,0.)),
                                    joint_TxyzRxyz, pelvis);

    idxLeftUpperLeg  = model.AddBody(idxPelvis,
                                    Xtrans(Vector3d(-pelvisWidth*0.5,0.,0.)),
                                    joint_Rxyz,segment);
    idxLeftLowerLeg = model.AddBody(idxLeftUpperLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Ry, segment);
    idxLeftFoot     = model.AddBody(idxLeftLowerLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Rxy,
                                    foot);

    idxRightUpperLeg  = model.AddBody(idxPelvis,
                                    Xtrans(Vector3d( pelvisWidth*0.5,0.,0.)),
                                    joint_Rxyz,segment);
    idxRightLowerLeg = model.AddBody(idxRightUpperLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Ry, segment);
    idxRightFoot     = model.AddBody(idxRightLowerLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Rxy,
                                    foot);

    SpatialTransform X_zero = Xtrans(Vector3d(0.,0.,0.));
    bool baumgarteEnabled = false;
    double timeStabilityInverse = 0.1;

    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,1,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,0,1,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,0,0,1),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(1,0,0,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,1,0,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,1,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);

    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,1,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,0,1,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,0,0,1),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(1,0,0,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,1,0,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,1,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);

    cs.Bind(model);
    q     = VectorNd::Zero(model.dof_count);
    qd    = VectorNd::Zero(model.dof_count);
    qdd   = VectorNd::Zero(model.dof_count);
    tau   = VectorNd::Zero(model.dof_count);

  }

  Model model;
  ConstraintSet cs;

  VectorNd q;
  VectorNd qd;
  VectorNd qdd;
  VectorNd tau;

  double pelvisWidth;
  double segmentLength;
  double footLength;
  double pelvisMass;
  double segmentMass;
  double footMass;

  unsigned int idxPelvis ;
  unsigned int idxLeftUpperLeg  ;
  unsigned int idxLeftLowerLeg  ;
  unsigned int idxLeftFoot;
  unsigned int idxRightUpperLeg ;
  unsigned int idxRightLowerLeg ;
  unsigned int idxRightFoot;

  SpatialTransform X_p;
  SpatialTransform X_s;

};


TEST_CASE_METHOD(PlanarBipedFloatingBase,
                 __FILE__"_TestCorrectness", "") {

  //1. Make the simple biped
  //2. Assemble it to a specific q and qdot.

  VectorNd q0, qd0, weights, qddTarget, lambda, qddFwd,qddErr,
           lambdaIdc, lambdaFwd, lambdaErr;
  MatrixNd K;
  std::vector<bool> dofActuated;

  unsigned int n  = unsigned (int (q.rows()));
  //unsigned int na = unsigned (int(q.rows()))-3;
  //unsigned int nc = unsigned (int(cs.name.size()));

  q0.resize(n);
  qd0.resize(n);
  weights.resize(n);
  qddTarget.resize(n);
  qddFwd.resize(n);
  qddErr.resize(n);
  dofActuated.resize(n);



  for(unsigned int i=0; i<n;++i){
    if(i>=3){
      dofActuated[i] = true;
    }else{
      dofActuated[i] = false;
    }
  }

  //A statically stable standing pose
  q0[0] = 0;
  q0[1] = 1.0;
  q0[2] = 0;
  q0[3] = M_PI*0.25;
  q0[4] =-M_PI*0.25;
  q0[5] =-M_PI*0.25;
  q0[6] = M_PI*0.25;

  qd0[0] = 0.;
  qd0[1] = 0.;
  qd0[2] = 0.;
  qd0[3] = 0.;
  qd0[4] = 0.;
  qd0[5] = 0.;
  qd0[6] = 0.;

  for(unsigned int i =0; i< q.rows();++i){
    weights[i]    = 1;
  }

  qddTarget[0]  = 0.;
  qddTarget[1]  = 0.;
  qddTarget[2]  = 0.;
  qddTarget[3]  = 0.;
  qddTarget[4]  = 0.;
  qddTarget[5]  = 0.;
  qddTarget[6]  = 0.;

  //============================================================================
  // Double stance: fully actuated
  //============================================================================

  //cs[0]: double stance
  bool qasm = CalcAssemblyQ(model,q0,cs[0],q,weights);
  CHECK(qasm);
  CalcAssemblyQDot(model,q,qd0,cs[0],qd,weights);


  //6. Call InverseDynamicsConstraints & repeat setps 4 & 5
  VectorNd tauIDC = VectorNd::Zero(tau.size());
  VectorNd qddIDC = VectorNd::Zero(qdd.size());

  cs[0].SetActuationMap(model, dofActuated);

  //Test to see if this model is compatiable with the exact IDC operator
  bool isCompatible = isConstrainedSystemFullyActuated(
                        model,q,qd,cs[0]);
  CHECK(isCompatible);

  //The SimpleMath library fails if it tries to solve this
  //linear system. Eigen has no problems.
  InverseDynamicsConstraints(model,
                             q,qd,qddTarget,
                             cs[0], qddIDC,tauIDC);

  //In this case the target qdd should be reachable exactly.
  for(unsigned int i=0; i<qddIDC.rows();++i){
    CHECK_THAT(qddIDC[i], IsClose(qddTarget[i],TEST_PREC, TEST_PREC));
  }


  lambdaIdc = cs[0].force;
  ForwardDynamicsConstraintsDirect(model,q,qd,tauIDC,cs[0],qddFwd);
  lambdaFwd = cs[0].force;


  qddErr = qddIDC-qddFwd;
  lambdaErr=lambdaIdc-lambdaFwd;
  for(unsigned int i=0; i<q.rows();++i){
    CHECK_THAT(qddIDC[i], IsClose(qddFwd[i], TEST_PREC, TEST_PREC));
  }
  for(unsigned int i=0; i<lambdaFwd.rows();++i){
    CHECK_THAT(lambdaIdc[i], IsClose(lambdaFwd[i], TEST_PREC, TEST_PREC));
  }

  //Check the relaxed method: if you give it a valid qddot as a target
  //this should be reached exactly
  VectorNd tauIDCR(tauIDC.rows());
  VectorNd qddIDCR(tauIDC.rows());
  InverseDynamicsConstraintsRelaxed(model,
                             q,qd,qddTarget,
                             cs[0], qddIDCR,tauIDCR);
  lambdaIdc = cs[0].force;

  //In the very least the non-actuated degrees of freedom should not
  //have corresponding entries in tau that are exactly zero:
  for(unsigned int i=0; i<tauIDCR.rows();++i){
    if(dofActuated[i]==false){
      CHECK_THAT(tauIDCR[i],IsClose(0.,TEST_PREC, TEST_PREC));
    }
  }

  //The resulting solution will not necessarily meet qdd target but it
  //should satisfy the equations of motion:

  ForwardDynamicsConstraintsDirect(model,q,qd,tauIDCR,cs[0],qddFwd);
  lambdaFwd = cs[0].force;

  lambdaErr=lambdaIdc-lambdaFwd;
  for(unsigned int i=0; i<q.rows();++i){
    CHECK_THAT(qddIDCR[i], IsClose(qddFwd[i], TEST_PREC, TEST_PREC));
  }
  for(unsigned int i=0; i<lambdaFwd.rows();++i){
    CHECK_THAT(lambdaIdc[i], IsClose(lambdaFwd[i], TEST_PREC, TEST_PREC));
  }


  //============================================================================
  // Single stance: under actuated because of the point foot
  //============================================================================

  q0.setZero();
  q0[1]=1.0;
  q0[2]=0.;
  qd.setZero();
  qd0.setZero();
  qd0[2] = 0.001; //Will result in the system rotating about the +z axis

  qasm = CalcAssemblyQ(model,q0,cs[1],q,weights);
  CHECK(qasm);
  CalcAssemblyQDot(model,q,qd0,cs[1],qd,weights);


  //6. Call InverseDynamicsConstraints & repeat setps 4 & 5
  tauIDC = VectorNd::Zero(tau.size());
  qddIDC = VectorNd::Zero(qdd.size());

  cs[1].SetActuationMap(model, dofActuated);

  //Test to see if this model is compatiable with the exact IDC operator
  isCompatible = isConstrainedSystemFullyActuated(
                        model,q,qd,cs[1]);
  CHECK(isCompatible==false);

  //Check the relaxed method
  qddTarget.setZero();
  InverseDynamicsConstraintsRelaxed(model,
                             q,qd,qddTarget,
                             cs[1], qddIDCR,tauIDCR);
  lambdaIdc = cs[1].force;

  for(unsigned int i=0; i<tauIDCR.size();++i){
    if(dofActuated[i]==false){
      CHECK_THAT(tauIDCR[i],IsClose(0.,TEST_PREC, TEST_PREC));
    }
  }

  ForwardDynamicsConstraintsDirect(model,q,qd,tauIDCR,cs[1],qddFwd);
  lambdaFwd = cs[1].force;

  lambdaErr=lambdaIdc-lambdaFwd;
  for(unsigned int i=0; i<q.rows();++i){
    CHECK_THAT(qddIDCR[i], IsClose(qddFwd[i], TEST_PREC, TEST_PREC));
  }
  for(unsigned int i=0; i<lambdaFwd.rows();++i){
    CHECK_THAT(lambdaIdc[i], IsClose(lambdaFwd[i], TEST_PREC, TEST_PREC));
  }




  //============================================================================
  //Timing comparision
  //============================================================================
  if(flag_printTimingData){
    unsigned int iterations = 100;

    auto tidcr1 = std::chrono::high_resolution_clock::now();
    for(unsigned int i=0; i<iterations; ++i){
      InverseDynamicsConstraintsRelaxed(model,
                                 q,qd,qddTarget,
                                 cs[0],qddIDC,tauIDC);
    }
    auto tidcr2   = std::chrono::high_resolution_clock::now();
    auto tidcr = std::chrono::duration_cast<std::chrono::microseconds>(tidcr2-tidcr1);


    auto tidc1 = std::chrono::high_resolution_clock::now();
    for(unsigned int i=0; i<iterations; ++i){
      InverseDynamicsConstraints(model,
                                 q,qd,qddTarget,
                                 cs[0],qddIDC,tauIDC);
    }
    auto tidc2   = std::chrono::high_resolution_clock::now();
    auto tidc = std::chrono::duration_cast<std::chrono::microseconds>(tidc2-tidc1);

    auto tfd1 = std::chrono::high_resolution_clock::now();
    for(unsigned int i=0; i<iterations; ++i){
      ForwardDynamicsConstraintsNullSpace(model,q,qd,tauIDC,cs[0],qddFwd);
    }
    auto tfd2 = std::chrono::high_resolution_clock::now();
    auto tfd = std::chrono::duration_cast<std::chrono::microseconds>(tfd2-tfd1);

    std::cout << "Planar Biped Dof: " << model.dof_count << std::endl;
    std::cout << "Cost per evaluation : us, xfd " << std::endl;

    std::cout << "IDC: " << double(tidc.count()) / double(iterations) << " "
                        << double(tidc.count())/double(tfd.count()) << std::endl;

    std::cout << "IDCRelaxed: " << double(tidcr.count()) / double(iterations) << " "
                        << double(tidcr.count())/double(tfd.count()) << std::endl;

    std::cout << "FDNullSp:    " << double(tfd.count()) / double(iterations) << " "
                        << double(tfd.count())/double(tfd.count()) << std::endl;
  }

}


TEST_CASE_METHOD(SpatialBipedFloatingBase,
                 __FILE__"_TestCorrectness2", "") {

  //1. Make the simple spatial biped
  //2. Assemble it to a specific q and qdot.

  VectorNd q0, qd0, weights, qddTarget, lambda, qddFwd,qddErr,
           lambdaFwd,lambdaIdc,lambdaErr;
  MatrixNd K;
  std::vector<bool> dofActuated;

  unsigned int n  = unsigned( int( q.rows()));
  //unsigned int na = unsigned( int( q.rows()))-6;
  unsigned int nc = unsigned( int( cs.name.size()));

  q0.resize(n);
  qd0.resize(n);
  weights.resize(n);
  qddTarget.resize(n);
  qddFwd.resize(n);
  qddErr.resize(n);
  lambda.resize(nc);
  dofActuated.resize(n);

  for(unsigned int i=0; i<n;++i){
    if(i>=6){
      dofActuated[i] = true;
    }else{
      dofActuated[i] = false;
    }
  }

  //A statically stable standing pose which will have a physically
  //consistent solution
  q0[0]  = 0.;         //tx Floating base
  q0[1]  = 0.;         //ty
  q0[2]  = 0.75;       //tx
  q0[3]  = 0.;         //rx
  q0[4]  = 0.;         //ry
  q0[5]  = 0.;         //rz
  q0[6]  = 0.;         //rx Left hip
  q0[7]  =  M_PI*0.25; //ry
  q0[8]  = 0.;         //rz
  q0[9]  = -M_PI*0.25; //ry Left knee
  q0[10] = 0.;         //rx Left ankle
  q0[11] = 0.;         //ry
  q0[12] = 0.;         //rx Right hip
  q0[13] = -M_PI*0.25; //ry
  q0[14] = 0.;         //rz
  q0[15] =  M_PI*0.25; //ry Right knee
  q0[16] = 0.;         //rx Right ankle
  q0[17] = 0.;         //ry

  qd0[0]  = 0.;
  qd0[1]  = 0.;
  qd0[2]  = 0.;
  qd0[3]  = 0.;
  qd0[4]  = 0.;
  qd0[5]  = 0.;
  qd0[6]  = 0.;
  qd0[7]  = 0.;
  qd0[8]  = 0.;
  qd0[9]  = 0.;
  qd0[10] = 0.;
  qd0[11] = 0.;
  qd0[12] = 0.;
  qd0[13] = 0.;
  qd0[14] = 0.;
  qd0[15] = 0.;
  qd0[16] = 0.;
  qd0[17] = 0.;

  for(unsigned int i =0; i< q.rows();++i){
    weights[i]    = 1.;
  }

  qddTarget[0]  = 0.;
  qddTarget[1]  = 0.;
  qddTarget[2]  = 0.;
  qddTarget[3]  = 0.;
  qddTarget[4]  = 0.;
  qddTarget[5]  = 0.;
  qddTarget[6]  = 0.;
  qddTarget[7]  = 0.;
  qddTarget[8]  = 0.;
  qddTarget[9]  = 0.;
  qddTarget[10] = 0.;
  qddTarget[11] = 0.;
  qddTarget[12] = 0.;
  qddTarget[13] = 0.;
  qddTarget[14] = 0.;
  qddTarget[15] = 0.;
  qddTarget[16] = 0.;
  qddTarget[17] = 0.;


  bool qAsm=CalcAssemblyQ(model,q0,cs,q,weights);
  CHECK(qAsm==true);

  CalcAssemblyQDot(model,q,qd0,cs,qd,weights);


  //6. Call InverseDynamicsConstraints

  VectorNd tauIDC = VectorNd::Zero(tau.size());
  VectorNd qddIDC = VectorNd::Zero(qdd.size());
  cs.SetActuationMap(model,dofActuated);

  //Test to see if this model is compatiable with the exact IDC operator
  bool isCompatible = isConstrainedSystemFullyActuated(
                        model,q,qd,cs);
  CHECK(isCompatible);

  //The SimpleMath library fails if it tries to solve this
  //linear system. Eigen has no problems.
  InverseDynamicsConstraints(model,
                             q,qd,qddTarget,
                             cs, qddIDC,tauIDC);
  lambdaIdc = cs.force;

  //In this case the target qdd should be reachable exactly.
  for(unsigned int i=0; i<qddIDC.rows();++i){
    CHECK_THAT(qddIDC[i], IsClose(qddTarget[i],TEST_PREC, TEST_PREC));
  }


  ForwardDynamicsConstraintsDirect(model,q,qd,tauIDC,cs,qddFwd);
  lambdaFwd = cs.force;

  //Check that the solution is physically consistent
  qddErr = qddIDC-qddFwd;
  lambdaErr= lambdaIdc-lambdaFwd;
  for(unsigned int i=0; i<q.rows();++i){
    CHECK_THAT(qddIDC[i], IsClose(qddFwd[i], TEST_PREC, TEST_PREC));
  }
  for(unsigned int i=0; i<lambdaIdc.rows();++i){
    CHECK_THAT(lambdaIdc[i], IsClose(lambdaFwd[i], TEST_PREC, TEST_PREC));
  }

  //Check the relaxed method
  VectorNd tauIDCR(tauIDC.rows());
  VectorNd qddIDCR(qddIDC.rows());
  InverseDynamicsConstraintsRelaxed(model,
                             q,qd,qddTarget,
                             cs, qddIDCR,tauIDCR);
  lambdaIdc = cs.force;

  for(unsigned int i=0; i<tauIDCR.rows();++i){
    if(dofActuated[i]==false){
      CHECK_THAT(tauIDCR[i],IsClose(0.,TEST_PREC, TEST_PREC));
    }
  }

  ForwardDynamicsConstraintsDirect(model,q,qd,tauIDCR,cs,qddFwd);
  lambdaFwd = cs.force;

  //Check that the solution is physically consistent
  qddErr = qddIDCR-qddFwd;
  lambdaErr= lambdaIdc-lambdaFwd;
  for(unsigned int i=0; i<q.rows();++i){
    CHECK_THAT(qddIDCR[i], IsClose(qddFwd[i], TEST_PREC, TEST_PREC));
  }
  for(unsigned int i=0; i<lambdaIdc.rows();++i){
    CHECK_THAT(lambdaIdc[i], IsClose(lambdaFwd[i], TEST_PREC, TEST_PREC));
  }


  //Timing comparision
  if(flag_printTimingData){
    unsigned int iterations = 100;

    auto tidcr1 = std::chrono::high_resolution_clock::now();
    for(unsigned int i=0; i<iterations; ++i){
      InverseDynamicsConstraintsRelaxed(model,
                                 q,qd,qddTarget,
                                 cs,qddIDC,tauIDC);
    }
    auto tidcr2   = std::chrono::high_resolution_clock::now();
    auto tidcr = std::chrono::duration_cast<std::chrono::microseconds>(tidcr2-tidcr1);


    auto tidc1 = std::chrono::high_resolution_clock::now();
    for(unsigned int i=0; i<iterations; ++i){
      InverseDynamicsConstraints(model,
                                 q,qd,qddTarget,
                                 cs,qddIDC,tauIDC);
    }
    auto tidc2   = std::chrono::high_resolution_clock::now();
    auto tidc = std::chrono::duration_cast<std::chrono::microseconds>(tidc2-tidc1);

    auto tfd1 = std::chrono::high_resolution_clock::now();
    for(unsigned int i=0; i<iterations; ++i){
      ForwardDynamicsConstraintsNullSpace(model,q,qd,tauIDC,cs,qddFwd);
    }
    auto tfd2 = std::chrono::high_resolution_clock::now();
    auto tfd = std::chrono::duration_cast<std::chrono::microseconds>(tfd2-tfd1);

    std::cout << "Spatial Biped Dof: " << model.dof_count << std::endl;
    std::cout << "Cost per evaluation : us, xfd " << std::endl;

    std::cout << "IDC: " << double(tidc.count()) / double(iterations) << " "
                        << double(tidc.count())/double(tfd.count()) << std::endl;

    std::cout << "IDCRelaxed: " << double(tidcr.count()) / double(iterations) << " "
                        << double(tidcr.count())/double(tfd.count()) << std::endl;

    std::cout << "FDNullSp:    " << double(tfd.count()) / double(iterations) << " "
                        << double(tfd.count())/double(tfd.count()) << std::endl;
  }

}


TEST_CASE(__FILE__"_CorrectnessTestWithSinglePlanarPendulum", ""){

  //With loop constraints
  SinglePendulumAbsoluteCoordinates spa
    = SinglePendulumAbsoluteCoordinates();

  //Without any joint coordinates
  SinglePendulumJointCoordinates spj
    = SinglePendulumJointCoordinates();


  //1. Set the pendulum modeled using joint coordinates to a specific
  //    state and then compute the spatial acceleration of the body.
  spj.q[0]  = M_PI/3.0;   //About z0
  spj.qd.setZero();
  spj.qdd.setZero();
  spj.tau.setZero();


  InverseDynamics(spj.model,spj.q,spj.qd,spj.qdd,spj.tau);

  Vector3d r010 = CalcBodyToBaseCoordinates(
                    spj.model,spj.q,spj.idB1,
                    Vector3d(0.,0.,0.),true);

  //2. Set the pendulum modelled using absolute coordinates to the
  //   equivalent state as the pendulum modelled using joint
  //   coordinates.

  spa.q[0]  = r010[0];
  spa.q[1]  = r010[1];
  spa.q[2]  = spj.q[0];


  spa.qd.setZero();
  spa.qdd.setZero();

  spa.tau[0]  = 0.;//tx
  spa.tau[1]  = 0.;//ty
  spa.tau[2]  = spj.tau[0];//rz


  //Test
  //  calcPositionError
  //  calcVelocityError

  VectorNd err(spa.cs.size());
  VectorNd errd(spa.cs.size());

  CalcConstraintsPositionError(spa.model,spa.q,spa.cs,err,true);
  CalcConstraintsVelocityError(spa.model,spa.q,spa.qd,spa.cs,errd,true);

  for(unsigned int i=0; i<err.rows();++i){
    CHECK_THAT(0, IsClose(err[i], TEST_PREC, TEST_PREC));
  }
  for(unsigned int i=0; i<errd.rows();++i){
    CHECK_THAT(0, IsClose(errd[i], TEST_PREC, TEST_PREC));
  }

  ForwardDynamicsConstraintsDirect(spa.model,spa.q, spa.qd, spa.tau,spa.cs,
                                   spa.qdd);
  for(unsigned int i=0; i<spa.qdd.rows();++i){
    CHECK_THAT(0., IsClose(spa.qdd[i], TEST_PREC, TEST_PREC));
  }

  std::vector< bool > actuationMap;
  actuationMap.resize(spa.tau.rows());
  for(unsigned int i=0; i<actuationMap.size();++i){
    actuationMap[i]=false;
  }
  actuationMap[2] = true;

  VectorNd qddDesired = VectorNd::Zero(spa.qdd.rows());
  VectorNd qddIdc = VectorNd::Zero(spa.qdd.rows());
  VectorNd tauIdc = VectorNd::Zero(spa.qdd.rows());

  //The IDC operator should be able to statisfy qdd=0 and return a tau
  //vector that matches the hand solution produced above.
  spa.cs.SetActuationMap(spa.model,actuationMap);

  //Test to see if this model is compatiable with the exact IDC operator
  bool isCompatible = isConstrainedSystemFullyActuated(
                        spa.model,spa.q,spa.qd,spa.cs);
  CHECK(isCompatible);

  InverseDynamicsConstraints(spa.model,spa.q,spa.qd,qddDesired,
                                          spa.cs, qddIdc,tauIdc);

  for(unsigned int i=0; i<qddIdc.rows();++i){
    CHECK_THAT(0.,IsClose(qddIdc[i],TEST_PREC, TEST_PREC));
  }
  for(unsigned int i=0; i<tauIdc.rows();++i){
    CHECK_THAT(spa.tau[i],IsClose(tauIdc[i],TEST_PREC, TEST_PREC));
  }

  InverseDynamicsConstraintsRelaxed(spa.model,spa.q,spa.qd,qddDesired,
                                    spa.cs, qddIdc,tauIdc);


  for(unsigned int i=0; i<tauIdc.rows();++i){
    if(actuationMap[i]==false){
      CHECK_THAT(tauIdc[i],IsClose(0.,TEST_PREC, TEST_PREC));
    }
  }

  VectorNd qddFwd=VectorNd::Zero(qddIdc.rows());
  ForwardDynamicsConstraintsDirect(spa.model,spa.q,spa.qd,tauIdc,spa.cs,qddFwd);

  //Check that the solution is physically consistent
  VectorNd qddErr = qddIdc-qddFwd;
  for(unsigned int i=0; i<qddIdc.rows();++i){
    CHECK_THAT(qddIdc[i], IsClose(qddFwd[i], TEST_PREC, TEST_PREC));
  }

}



TEST_CASE(__FILE__"_CorrectnessTestWithDoublePerpendicularPendulum", ""){

  //With loop constraints
  DoublePerpendicularPendulumAbsoluteCoordinates dba
    = DoublePerpendicularPendulumAbsoluteCoordinates();

  //Without any joint coordinates
  DoublePerpendicularPendulumJointCoordinates dbj
    = DoublePerpendicularPendulumJointCoordinates();


  //1. Set the pendulum modeled using joint coordinates to a specific
  //    state and then compute the spatial acceleration of the body.
  dbj.q[0]  = M_PI/3.0;   //About z0
  dbj.q[1]  = M_PI/6.0;         //About y1
  dbj.qd.setZero();
  dbj.qdd.setZero();
  dbj.tau.setZero();


  InverseDynamics(dbj.model,dbj.q,dbj.qd,dbj.qdd,dbj.tau);
  //ForwardDynamics(dbj.model,dbj.q,dbj.qd,dbj.tau,dbj.qdd);

  Vector3d r010 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB1,
                    Vector3d(0.,0.,0.),true);
  Vector3d r020 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB2,
                    Vector3d(0.,0.,0.),true);
  Vector3d r030 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB2,
                    Vector3d(dbj.l2,0.,0.),true);

  //2. Set the pendulum modelled using absolute coordinates to the
  //   equivalent state as the pendulum modelled using joint
  //   coordinates. Next


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

  dba.qd.setZero();
  dba.qdd.setZero();

  dba.tau[0]  = 0.;//tx
  dba.tau[1]  = 0.;//ty
  dba.tau[2]  = 0.;//tz
  dba.tau[3]  = dbj.tau[0];//rz
  dba.tau[4]  = 0.;//ry
  dba.tau[5]  = 0.;//rx
  dba.tau[6]  = 0.;//tx
  dba.tau[7]  = 0.;//ty
  dba.tau[8]  = 0.;//tz
  dba.tau[9]  = 0.;//rz
  dba.tau[10] = dbj.tau[1];//ry
  dba.tau[11] = 0.;//rx



  VectorNd err(dba.cs.size());
  VectorNd errd(dba.cs.size());

  CalcConstraintsPositionError(dba.model,dba.q,dba.cs,err,true);
  CalcConstraintsVelocityError(dba.model,dba.q,dba.qd,dba.cs,errd,true);

  for(unsigned int i=0; i<err.rows();++i){
    CHECK_THAT(0, IsClose(err[i], TEST_PREC, TEST_PREC));
  }
  for(unsigned int i=0; i<errd.rows();++i){
    CHECK_THAT(0, IsClose(errd[i], TEST_PREC, TEST_PREC));
  }

  ForwardDynamicsConstraintsDirect(dba.model,dba.q, dba.qd, dba.tau,dba.cs,
                                   dba.qdd);
  for(unsigned int i=0; i<dba.qdd.rows();++i){
    CHECK_THAT(0., IsClose(dba.qdd[i], TEST_PREC, TEST_PREC));
  }

  std::vector< bool > actuationMap;
  actuationMap.resize(dba.tau.rows());
  for(unsigned int i=0; i<actuationMap.size();++i){
    actuationMap[i]=false;
  }
  actuationMap[3] = true;
  actuationMap[10] = true;

  VectorNd qddDesired = VectorNd::Zero(dba.qdd.rows());
  VectorNd qddIdc = VectorNd::Zero(dba.qdd.rows());
  VectorNd tauIdc = VectorNd::Zero(dba.qdd.rows());

  //The IDC operator should be able to statisfy qdd=0 and return a tau
  //vector that matches the hand solution produced above.

  dba.cs.SetActuationMap(dba.model,actuationMap);

  //Test to see if this model is compatiable with the exact IDC operator
  bool isCompatible = isConstrainedSystemFullyActuated(
                        dba.model,dba.q,dba.qd,dba.cs);
  CHECK(isCompatible);

  InverseDynamicsConstraints(dba.model,dba.q,dba.qd,qddDesired,
                                          dba.cs, qddIdc,tauIdc);

  for(unsigned int i=0; i<qddIdc.rows();++i){
    CHECK_THAT(0.,IsClose(qddIdc[i],TEST_PREC, TEST_PREC));
  }
  for(unsigned int i=0; i<tauIdc.rows();++i){
    CHECK_THAT(dba.tau[i],IsClose(tauIdc[i],TEST_PREC, TEST_PREC));
  }

  InverseDynamicsConstraintsRelaxed(dba.model,dba.q,dba.qd,qddDesired,
                                          dba.cs, qddIdc,tauIdc);

  for(unsigned int i=0; i<tauIdc.rows();++i){
    if(actuationMap[i]==false){
      CHECK_THAT(tauIdc[i],IsClose(0.,TEST_PREC, TEST_PREC));
    }
  }

  //Check if the result is physically consistent.
  VectorNd qddCheck(qddIdc.rows());
  ForwardDynamicsConstraintsDirect(dba.model,dba.q,dba.qd, tauIdc, dba.cs, qddCheck);

  for(unsigned int i=0; i<qddIdc.rows();++i){
    CHECK_THAT(qddCheck[i],IsClose(qddIdc[i],TEST_PREC, TEST_PREC));
  }

}

TEST_CASE(__FILE__"_CorrectnessTestWithUnderactuatedCartPendulum", ""){
  //This model is not compatible with InverseDynamicsConstraints
  //which is valuable: all other models tested have been.
  //This model is identical to the one presented in Sec. 3 of Henning's thesis

  Model model;
  model.gravity = Vector3d(0.,0.,-9.81);

  double m1 = 10;
  double l1 = 1.;
  double h1 = 0.5;
  double J1xx = m1*(h1*h1+h1*h1)/12.;
  double J1yy = m1*(l1*l1+h1*h1)/12.;
  double J1zz = m1*(l1*l1+h1*h1)/12.;

  double m2 = 1.;
  double l2 = 1.;
  double J2xx = m2*(l1*l1)/3.;
  double J2yy = m2*(l1*l1)/3.;
  double J2zz = m2*(l1*l1)/30.;


  Body cart1 = Body(m1, Vector3d(0., 0., 0.),
                    Matrix3d(   J1xx,   0.,  0.,
                                  0., J1yy,  0.,
                                  0.,   0.,  J1zz));

  Body link2 = Body(m2, Vector3d( 0., -l2*0.5, 0.),
                    Matrix3d( J2xx,   0. ,   0.,
                                 0., J2yy,   0.,
                                 0.,  0. ,  J2zz));

  Joint jointXYRz (SpatialVector(0.,0.,0., 1.,0.,0.),
                   SpatialVector(0.,0.,0., 0.,0.,1.),
                   SpatialVector(0.,1.,0, 0.,0.,0.));

  Joint jointRy (SpatialVector(0.,1.,0., 0.,0.,0.));

  unsigned int idB1 = model.AddBody(0, SpatialTransform(), jointXYRz, cart1);
  unsigned int idB2 = model.AddBody(idB1, SpatialTransform(), jointRy, link2);

  ConstraintSet cs;
  SpatialTransform X;
  X.r.setZero();
  X.E = Matrix3d::Identity();
  unsigned int cp1=cs.AddLoopConstraint(0,idB1,X,X,SpatialVector(0,0,0,0,0,1));
  unsigned int cp2=cs.AddLoopConstraint(0,idB1,X,X,SpatialVector(0,1,0,0,0,0));

  cs.Bind(model);

  //This system should not be compatible with the InverseDynamicsConstraints
  //operator:

  VectorNd q,qd,qdd,qddDesired, qddFwd,tau;
  q.resize(model.q_size);
  qd.resize(model.qdot_size);
  qdd.resize(model.qdot_size);
  qddFwd.resize(model.qdot_size);
  qddDesired.resize(model.qdot_size);
  tau.resize(model.qdot_size);

  q.setZero();
  qd.setZero();
  qdd.setZero();
  qddFwd.setZero();
  qddDesired.setZero();
  tau.setZero();

  std::vector< bool > actuation;
  actuation.resize(model.qdot_size);
  for(unsigned int i=0; i<actuation.size();++i){
    actuation[i] = false;
  }
  actuation[model.qdot_size-1] = true;
  cs.SetActuationMap(model,actuation);

  bool isCompatible =
      isConstrainedSystemFullyActuated(model,q,qd,cs);
  CHECK(isCompatible == false);

  //pose 1
  InverseDynamicsConstraintsRelaxed(model,q,qd,qddDesired,cs,qdd,tau);

  for(unsigned int i=0; i<tau.rows();++i){
    if(actuation[i]==false){
      CHECK_THAT(tau[i],IsClose(0.,TEST_PREC, TEST_PREC));
    }
  }

  //Check if the result is physically consistent.
  ForwardDynamicsConstraintsDirect(model,q,qd, tau, cs, qddFwd);
  for(unsigned int i=0; i<qddFwd.rows();++i){
    CHECK_THAT(qdd[i],IsClose(qddFwd[i],TEST_PREC, TEST_PREC));
  }

  //pose 2
  q[3] = M_PI*0.5;
  InverseDynamicsConstraintsRelaxed(model,q,qd,qddDesired,cs,qdd,tau);
  for(unsigned int i=0; i<qddDesired.rows();++i){
    CHECK_THAT(qdd[i],IsClose(qddDesired[i],TEST_PREC, 0.01));
  }

  for(unsigned int i=0; i<tau.rows();++i){
    if(actuation[i]==false){
      CHECK_THAT(tau[i],IsClose(0.,TEST_PREC, TEST_PREC));
    }
  }

  //Check if the result is physically consistent.
  ForwardDynamicsConstraintsDirect(model,q,qd, tau, cs, qddFwd);
  for(unsigned int i=0; i<qddFwd.rows();++i){
    CHECK_THAT(qdd[i],IsClose(qddFwd[i],TEST_PREC, TEST_PREC));
  }

  //pose 2 and some movement
  VectorNd qd0(qd.rows());
  qd0[0] = 0.5;
  qd0[1] = 0.125;
  qd0[2] = 0.25;
  qd0[3] = 1.1;
  VectorNd w = VectorNd::Constant(qd0.rows(),1.);


  //Make sure that we are on the constraint manifold
  CalcAssemblyQDot(model,q,qd0,cs,qd,w);

  //Add a non-zero desired acceleration in the actuation domain
  qddDesired[3] = 2.2;
  InverseDynamicsConstraintsRelaxed(model,q,qd,qddDesired,cs,qdd,tau);

  for(unsigned int i=0; i<tau.rows();++i){
    if(actuation[i]==false){
      CHECK_THAT(tau[i],IsClose(0.,TEST_PREC, TEST_PREC));
    }
  }

  //Check if the result is physically consistent.
  ForwardDynamicsConstraintsDirect(model,q,qd, tau, cs, qddFwd);
  for(unsigned int i=0; i<qddFwd.rows();++i){
    CHECK_THAT(qdd[i],IsClose(qddFwd[i],TEST_PREC, TEST_PREC));
  }


}

