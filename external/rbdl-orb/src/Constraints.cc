/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <sstream>
#include <string>
#include <limits>
#include <assert.h>
//The ConstraintCache input to each function contains all of the working
//memory necessary for this constraint. So nothing appears here.

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/rbdl_errors.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Joint.h"
#include "rbdl/Body.h"
#include "rbdl/Constraints.h"
#include "rbdl/Constraint_Contact.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Kinematics.h"

namespace RigidBodyDynamics
{

using namespace Math;

void SolveLinearSystem (
  const MatrixNd& A,
  const VectorNd& b,
  VectorNd& x,
  LinearSolver ls
);

unsigned int GetMovableBodyId (Model& model, unsigned int id);

//==============================================================================


//==============================================================================
unsigned int ConstraintSet::AddContactConstraint (
  unsigned int body_id,
  const Vector3d &body_point,
  const Vector3d &world_normal,
  const char *constraint_name,
  unsigned int userDefinedId)
{
  assert (bound == false);


  unsigned int insertAtRowInG = size();
  unsigned int rowsInG = insertAtRowInG+1;

  std::string nameStr;
  if(constraint_name != NULL) {
    nameStr = constraint_name;
  }
  //Go through all existing ContactConstraints,
  //if there is a BodyToGroundPosition
  //constraint at body_id with the identical body_point, then append the
  //constraint.
  //
  // Why am I bothering to do this? To save on computation.
  // Every individual ContactConstraint evaluates a point Jacobian.
  // Thus 3 individual constraints evaluates a point Jacobian 3 times. If these
  // are all grouped together then the point Jacobian is only evaluated once.
  bool constraintAppended = false;

  if(contactConstraints.size() > 0) {
    unsigned int i = unsigned(contactConstraints.size()-1);
    if(contactConstraints[i]->getBodyIds()[0] == body_id) {
      Vector3d pointErr = body_point -
                          contactConstraints[i]->getBodyFrames()[0].r;

      if(pointErr.norm() < std::numeric_limits<double>::epsilon()*100
         && contactConstraints[i]->getUserDefinedId() == userDefinedId) {
        constraintAppended = true;
        contactConstraints[i]->appendNormalVector(world_normal);
      }
    }
  }

  if(constraintAppended == false) {

    ContactConstraint con = ContactConstraint(body_id,body_point, world_normal,
                            constraint_name,userDefinedId);

    contactConstraints.push_back(std::make_shared<ContactConstraint>(con));
    unsigned int idx = unsigned(contactConstraints.size()-1);
    contactConstraints[idx]->addToConstraintSet(insertAtRowInG);
    constraints.emplace_back(contactConstraints[idx]);

  }

  constraintType.push_back (ConstraintTypeContact);
  name.push_back (nameStr);


  err.conservativeResize(rowsInG);
  err[insertAtRowInG] = 0.;
  errd.conservativeResize(rowsInG);
  errd[insertAtRowInG] = 0.;

  force.conservativeResize (rowsInG);
  force[insertAtRowInG] = 0.;

  impulse.conservativeResize (rowsInG);
  impulse[insertAtRowInG] = 0.;

  v_plus.conservativeResize (rowsInG);
  v_plus[insertAtRowInG] = 0.;

  d_multdof3_u = std::vector<Math::Vector3d>( rowsInG,
                 Math::Vector3d::Zero());

  //Set up access maps
  if(nameStr.size() > 0) {
    std::pair< std::map<std::string, unsigned int>::iterator, bool > iter;
    iter = nameGroupMap.insert(std::pair<std::string, unsigned int>(
                                 name[name.size()-1],
                                 unsigned(constraints.size()-1)));
    //if(iter.second == false){
    //  std::cerr << "Error: optional name is not unique."
    //            << std::endl;
    //  assert(0);
    //  abort();
    //}

  }
  if(userDefinedId < std::numeric_limits<unsigned int>::max()) {
    std::pair< std::map<unsigned int, unsigned int>::iterator, bool > iter;
    iter =userDefinedIdGroupMap.insert( std::pair<unsigned int, unsigned int>(
                                          userDefinedId,
                                          unsigned(constraints.size()-1)));
    //if(iter.second == false){
    //  std::cerr << "Error: optional userDefinedId is not unique."
    //            << std::endl;
    //  assert(0);
    //  abort();
    //}
  }

  std::pair< std::map<unsigned int, unsigned int>::iterator, bool > iter;
  iter = idGroupMap.insert(std::pair<unsigned int, unsigned int>(
                             unsigned(rowsInG-1),
                             unsigned(constraints.size()-1)));
  if(iter.second == false) {
    std::cerr << "Error: Constraint row entry in system is not unique."
              << " (This should not be possible: contact the "
              "maintainer of this code.)"
              << std::endl;
    assert(0);
    abort();
  }

  return rowsInG-1;

}

//==============================================================================
unsigned int ConstraintSet::AddLoopConstraint (
  unsigned int idPredecessor,
  unsigned int idSuccessor,
  const Math::SpatialTransform &XPredecessor,
  const Math::SpatialTransform &XSuccessor,
  const Math::SpatialVector &constraintAxisInPredecessor,
  bool enableBaumgarteStabilization,
  double stabilizationTimeConstant,
  const char *constraintName,
  unsigned int userDefinedId)
{
  assert (bound == false);


  unsigned int insertAtRowInG = unsigned(size());
  unsigned int rowsInG = insertAtRowInG+1;

  double tol = std::numeric_limits<double>::epsilon()*100.;
  bool constraintAppended = false;
  unsigned int idx = unsigned(loopConstraints.size());

  if(loopConstraints.size() > 0) {
    idx = idx-1;
    if(loopConstraints[idx]->getBodyIds()[0] == idPredecessor &&
        loopConstraints[idx]->getBodyIds()[1] == idSuccessor) {

      bool framesNumericallyIdentical=true;
      SpatialTransform frameErrorPre, frameErrorSuc;

      frameErrorPre.r=XPredecessor.r-loopConstraints[idx]->getBodyFrames()[0].r;
      frameErrorPre.E=XPredecessor.E-loopConstraints[idx]->getBodyFrames()[0].E;

      frameErrorSuc.r=XSuccessor.r  -loopConstraints[idx]->getBodyFrames()[1].r;
      frameErrorSuc.E=XSuccessor.E  -loopConstraints[idx]->getBodyFrames()[1].E;

      //Using this awkward element by element comparison to maintain
      //compatibility with SimpleMath.
      for(unsigned int i=0; i<frameErrorPre.r.size(); ++i) {
        if(fabs(frameErrorPre.r[i]) > tol || fabs(frameErrorSuc.r[i]) > tol) {
          framesNumericallyIdentical=false;
        }
        for(unsigned int j=0; j<frameErrorPre.E.cols(); ++j) {
          if(fabs(frameErrorPre.E(i,j))>tol || fabs(frameErrorSuc.E(i,j))>tol) {
            framesNumericallyIdentical=false;
          }
        }
      }

      if(framesNumericallyIdentical
         && loopConstraints[idx]->getUserDefinedId() == userDefinedId) {
        constraintAppended = true;
        loopConstraints[idx]->appendConstraintAxis(constraintAxisInPredecessor);
      }
    }
  }

  if(constraintAppended==false) {

    LoopConstraint loopCon( idPredecessor, idSuccessor,
                            XPredecessor,  XSuccessor,
                            constraintAxisInPredecessor,
                            enableBaumgarteStabilization,
                            stabilizationTimeConstant,
                            constraintName,
                            userDefinedId);

    loopConstraints.push_back(std::make_shared<LoopConstraint>(loopCon));
    idx = unsigned(loopConstraints.size()-1);
    loopConstraints[idx]->addToConstraintSet(insertAtRowInG);
    constraints.emplace_back(loopConstraints[idx]);
  }

  constraintType.push_back(ConstraintTypeLoop);

  //Update all of the struct arrays so that they have the correct number
  //of elements
  std::string nameStr;
  if (constraintName != NULL) {
    nameStr = constraintName;
  }

  name.push_back (nameStr);


  err.conservativeResize(rowsInG);
  err[insertAtRowInG] = 0.;
  errd.conservativeResize(rowsInG);
  errd[insertAtRowInG] = 0.;

  force.conservativeResize (rowsInG);
  force[insertAtRowInG] = 0.;

  impulse.conservativeResize (rowsInG);
  impulse[insertAtRowInG] = 0.;

  v_plus.conservativeResize (rowsInG);
  v_plus[insertAtRowInG] = 0.;

  d_multdof3_u = std::vector<Math::Vector3d>(rowsInG, Math::Vector3d::Zero());

  //Set up access maps
  if(nameStr.size() > 0) {
    std::pair< std::map<std::string, unsigned int>::iterator, bool > iter;
    iter = nameGroupMap.insert(std::pair<std::string, unsigned int>(
                                 name[name.size()-1],
                                 unsigned(constraints.size()-1)));
    //if(iter.second == false){
    //  std::cerr << "Error: optional name is not unique."
    //            << std::endl;
    //  assert(0);
    //  abort();
    //}

  }

  if(userDefinedId < std::numeric_limits<unsigned int>::max()) {
    std::pair< std::map<unsigned int, unsigned int>::iterator, bool > iter;
    iter =userDefinedIdGroupMap.insert( std::pair<unsigned int, unsigned int>(
                                          userDefinedId,
                                          unsigned(constraints.size()-1)));
    //if(iter.second == false){
    //  std::cerr << "Error: optional userDefinedId is not unique."
    //            << std::endl;
    //  assert(0);
    //  abort();
    //}
  }

  std::pair< std::map<unsigned int, unsigned int>::iterator, bool > iter;
  iter = idGroupMap.insert(std::pair<unsigned int, unsigned int>(
                             unsigned(rowsInG-1),
                             unsigned(constraints.size()-1)));
  if(iter.second == false) {
    std::stringstream errormsg;
    errormsg << "Error: Constraint row entry in system is not unique."
             << " (This should not be possible: contact the "
             "maintainer of this code.)"
             << std::endl;
    throw Errors::RBDLError(errormsg.str());
  }

  return rowsInG-1;
}



//==============================================================================
unsigned int ConstraintSet::AddCustomConstraint(
  std::shared_ptr<Constraint> customConstraint)
{
  unsigned int insertAtRowInG = unsigned(size());
  unsigned int rowsInG = insertAtRowInG+customConstraint->getConstraintSize();
  unsigned int cIndex = constraints.size();

  constraints.emplace_back(customConstraint);
  constraints[cIndex]->addToConstraintSet(insertAtRowInG);

  //Resize constraint set system variables
  std::string nameStr("");
  if(customConstraint->getName() != NULL) {
    nameStr = customConstraint->getName();
  }

  err.conservativeResize(     rowsInG);
  errd.conservativeResize(    rowsInG);
  force.conservativeResize (  rowsInG);
  impulse.conservativeResize (rowsInG);
  v_plus.conservativeResize ( rowsInG);
  d_multdof3_u = std::vector<Math::Vector3d>(rowsInG, Math::Vector3d::Zero());

  for(unsigned int i=0; i<customConstraint->getConstraintSize(); ++i) {
    //The list of names, constraint types, and ids must have the same
    //number of entries as G has rows.
    name.push_back (nameStr);
    constraintType.push_back (ConstraintTypeCustom);


    err[      insertAtRowInG+i ] = 0.;
    errd[     insertAtRowInG+i ] = 0.;
    force[    insertAtRowInG+i ] = 0.;
    impulse[  insertAtRowInG+i ] = 0.;
    v_plus[   insertAtRowInG+i ] = 0.;
  }

  //Set up access maps
  if(nameStr.size() > 0) {
    std::pair< std::map<std::string, unsigned int>::iterator, bool > iter;
    iter = nameGroupMap.insert(std::pair<std::string, unsigned int>(
                                 name[name.size()-1],
                                 unsigned(constraints.size()-1)));
    if(iter.second == false) {
      throw Errors::RBDLError("Error: optional name is not unique.\n");
    }

  }
  if(customConstraint->getUserDefinedId()
      < std::numeric_limits<unsigned int>::max()) {
    std::pair< std::map<unsigned int, unsigned int>::iterator, bool > iter;
    iter =userDefinedIdGroupMap.insert( std::pair<unsigned int, unsigned int>(
                                          customConstraint->getUserDefinedId(),
                                          unsigned(constraints.size()-1)));
    if(iter.second == false) {
      throw Errors::RBDLError("Error: optional userDefinedId is not unique.\n");
    }

  }

  std::pair< std::map<unsigned int, unsigned int>::iterator, bool > iter;
  iter = idGroupMap.insert(std::pair<unsigned int, unsigned int>(
                             unsigned(rowsInG-1),
                             unsigned(constraints.size()-1)));
  if(iter.second == false) {
    std::stringstream errormsg;
    errormsg << "Error: Constraint row entry into system is not unique."
             << " (This should not be possible: contact the "
             "maintainer of this code.)"
             << std::endl;
    throw Errors::RBDLError(errormsg.str());
  }

  return rowsInG-1;

}


//==============================================================================
bool ConstraintSet::Bind (const Model &model)
{
  assert (bound == false);

  if (bound) {
    throw Errors::RBDLError("Error: binding an already bound constraint set!\n");
  }
  for(unsigned int i=0; i<constraints.size(); ++i) {
    constraints[i]->bind(model);
  }

  cache.vecNZeros = VectorNd::Zero(model.qdot_size);
  cache.vecNA.resize(model.qdot_size,1);
  cache.vecNB.resize(model.qdot_size,1);
  cache.vecNC.resize(model.qdot_size,1);
  cache.vecND.resize(model.qdot_size,1);

  cache.mat3NA.resize(3, model.qdot_size);
  cache.mat3NB.resize(3, model.qdot_size);
  cache.mat3NC.resize(3, model.qdot_size);
  cache.mat3ND.resize(3, model.qdot_size);

  cache.mat6NA.resize(6, model.qdot_size);
  cache.mat6NB.resize(6, model.qdot_size);
  cache.mat6NC.resize(6, model.qdot_size);
  cache.mat6ND.resize(6, model.qdot_size);


  unsigned int n_constr = size();

  H.conservativeResize (model.dof_count, model.dof_count);
  H.setZero();
  C.conservativeResize (model.dof_count);
  C.setZero();
  gamma.conservativeResize (n_constr);
  gamma.setZero();
  G.conservativeResize (n_constr, model.dof_count);
  G.setZero();
  A.conservativeResize (model.dof_count + n_constr, model.dof_count + n_constr);
  A.setZero();
  b.conservativeResize (model.dof_count + n_constr);
  b.setZero();
  x.conservativeResize (model.dof_count + n_constr);
  x.setZero();


  S.conservativeResize(model.dof_count, model.dof_count);
  S.setZero();
  W.conservativeResize(model.dof_count, model.dof_count);
  W.Identity(model.dof_count, model.dof_count);

  // HouseHolderQR crashes if matrix G has more rows than columns.
  GT_qr = Eigen::HouseholderQR<Math::MatrixNd> (G.transpose());
  GT_qr_Q = MatrixNd::Zero (model.dof_count, model.dof_count);
  Y = MatrixNd::Zero (model.dof_count, G.rows());
  Z = MatrixNd::Zero (model.dof_count, model.dof_count - G.rows());
  qddot_y = VectorNd::Zero (model.dof_count);
  qddot_z = VectorNd::Zero (model.dof_count);

  K.conservativeResize (n_constr, n_constr);
  K.setZero();
  a.conservativeResize (n_constr);
  a.setZero();
  QDDot_t.conservativeResize (model.dof_count);
  QDDot_t.setZero();
  f_t.resize (n_constr, SpatialVector::Zero());
  point_accel_0.resize (n_constr, Vector3d::Zero());

  QDDot_0.conservativeResize (model.dof_count);
  QDDot_0.setZero();


  f_ext_constraints.resize (model.mBodies.size(), SpatialVector::Zero());


  d_pA =std::vector<SpatialVector> (model.mBodies.size(),SpatialVector::Zero());
  d_a = std::vector<SpatialVector> (model.mBodies.size(),SpatialVector::Zero());
  d_u = VectorNd::Zero (model.mBodies.size());

  d_IA = std::vector<SpatialMatrix> (model.mBodies.size()
                                     , SpatialMatrix::Identity());
  d_U = std::vector<SpatialVector> (model.mBodies.size(),SpatialVector::Zero());
  d_d = VectorNd::Zero (model.mBodies.size());

  d_multdof3_u = std::vector<Math::Vector3d> (model.mBodies.size()
                 , Math::Vector3d::Zero());

  bound = true;

  return bound;
}

//==============================================================================
void ConstraintSet::SetActuationMap(const Model &model,
                                    const std::vector<bool> &actuatedDofUpd)
{

  assert(actuatedDofUpd.size() == model.dof_count);

  unsigned int n  = unsigned( int( model.dof_count ));
  unsigned int nc = unsigned( int( name.size() ));
  unsigned int na = 0; //actuated dofs
  unsigned int nu = 0; //unactuated dofs

  for(unsigned int i=0; i<actuatedDofUpd.size(); ++i) {
    if(actuatedDofUpd[i]) {
      ++na;
    }
  }
  nu = n-na;

  S.conservativeResize(na,model.dof_count);
  S.setZero();
  P.conservativeResize(n-na,model.dof_count);
  P.setZero();
  W.conservativeResize(na,na);
  W.setZero();
  Winv.conservativeResize(na,na);
  Winv.setZero();
  WinvSC.conservativeResize(na);
  WinvSC.setZero();

  u.resize(na);
  v.resize(nu);

  unsigned int j=0;
  unsigned int k=0;
  for(unsigned int i=0; i<model.dof_count; ++i) {
    if(actuatedDofUpd[i]) {
      S(j,i) = 1.;
      ++j;
    } else {
      P(k,i) = 1.;
      ++k;
    }
  }

  unsigned int dim = n+n+nc+na;

  //Null space method variable initialization
  dim = na+nu;
  F.conservativeResize(dim,dim);
  F.setZero();

  Ful.conservativeResize(na,na);
  Ful.setZero();
  Fur.conservativeResize(na,nu);
  Fur.setZero();
  Fll.conservativeResize(nu,na);
  Fll.setZero();
  Flr.conservativeResize(nu,nu);
  Flr.setZero();

  g.conservativeResize(n);

  Ru.conservativeResize(nc,nc);
  py.conservativeResize(nc);
  pz.conservativeResize(n-nc);

  GT.conservativeResize(n,nc);
  GTu.conservativeResize(na,nc);
  GTl.conservativeResize(nu,nc);

  GPT.conservativeResize(nc,nu);

}

void ConstraintSet::clear()
{
  force.setZero();
  impulse.setZero();

  H.setZero();
  C.setZero();
  gamma.setZero();
  G.setZero();
  A.setZero();
  b.setZero();
  x.setZero();

  //Constraint cache
  cache.vecNZeros.setZero();

  cache.vecNA.setZero();
  cache.vecNB.setZero();
  cache.vecNC.setZero();
  cache.vecND.setZero();


  cache.mat3NA.setZero();
  cache.mat3NB.setZero();
  cache.mat3NC.setZero();
  cache.mat3ND.setZero();

  cache.vec3A.setZero();
  cache.vec3B.setZero();
  cache.vec3C.setZero();
  cache.vec3D.setZero();
  cache.vec3E.setZero();
  cache.vec3F.setZero();

  cache.svecA.setZero();
  cache.svecB.setZero();
  cache.svecC.setZero();
  cache.svecD.setZero();
  cache.svecE.setZero();
  cache.svecF.setZero();

  cache.stA.E.Identity();
  cache.stA.r.setZero();
  cache.stB.E.Identity();
  cache.stB.r.setZero();
  cache.stC.E.Identity();
  cache.stC.r.setZero();
  cache.stD.E.Identity();
  cache.stD.r.setZero();

  cache.mat3A.setZero();
  cache.mat3B.setZero();
  cache.mat3C.setZero();
  cache.mat3D.setZero();
  cache.mat3E.setZero();
  cache.mat3F.setZero();


  //Kokkevis Cache
  QDDot_t.setZero();
  a.setZero();
  K.setZero();
  for(unsigned int i=0; i<point_accel_0.size(); ++i) {
    point_accel_0[i].setZero();
  }
  for(unsigned int i=0; i<f_t.size(); ++i) {
    f_t[i].setZero();
  }

  QDDot_0.setZero();

  unsigned int i;
  for (i = 0; i < f_t.size(); i++) {
    f_t[i].setZero();
  }

  for (i = 0; i < f_ext_constraints.size(); i++) {
    f_ext_constraints[i].setZero();
  }

  for (i = 0; i < point_accel_0.size(); i++) {
    point_accel_0[i].setZero();
  }

  for (i = 0; i < d_pA.size(); i++) {
    d_pA[i].setZero();
  }

  for (i = 0; i < d_a.size(); i++) {
    d_a[i].setZero();
  }

  d_u.setZero();
}


//==============================================================================
RBDL_DLLAPI
void SolveConstrainedSystemDirect (
  Math::MatrixNd &H,
  const Math::MatrixNd &G,
  const Math::VectorNd &c,
  const Math::VectorNd &gamma,
  Math::VectorNd &lambda,
  Math::MatrixNd &A,
  Math::VectorNd &b,
  Math::VectorNd &x,
  Math::LinearSolver &linear_solver
)
{
  // Build the system: Copy H
  A.block(0, 0, c.rows(), c.rows()) = H;

  // Copy G and G^T
  A.block(0, c.rows(), c.rows(), gamma.rows()) = G.transpose();
  A.block(c.rows(), 0, gamma.rows(), c.rows()) = G;

  // Build the system: Copy -C + \tau
  b.block(0, 0, c.rows(), 1) = c;
  b.block(c.rows(), 0, gamma.rows(), 1) = gamma;

  LOG << "A = " << std::endl << A << std::endl;
  LOG << "b = " << std::endl << b << std::endl;

  switch (linear_solver) {
  case (LinearSolverPartialPivLU) :
    x = A.partialPivLu().solve(b);
    break;
  case (LinearSolverColPivHouseholderQR) :
    x = A.colPivHouseholderQr().solve(b);
    break;
  case (LinearSolverHouseholderQR) :
    x = A.householderQr().solve(b);
    break;
  default:
    LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
    assert (0);
    break;
  }

  LOG << "x = " << std::endl << x << std::endl;
}

//==============================================================================
RBDL_DLLAPI
void SolveConstrainedSystemRangeSpaceSparse (
  Model &model,
  Math::MatrixNd &H,
  const Math::MatrixNd &G,
  const Math::VectorNd &c,
  const Math::VectorNd &gamma,
  Math::VectorNd &qddot,
  Math::VectorNd &lambda,
  Math::MatrixNd &K,
  Math::VectorNd &a,
  Math::LinearSolver linear_solver
)
{
  SparseFactorizeLTL (model, H);

  MatrixNd Y (G.transpose());

  for (unsigned int i = 0; i < Y.cols(); i++) {
    VectorNd Y_col = Y.block(0,i,Y.rows(),1);
    SparseSolveLTx (model, H, Y_col);
    Y.block(0,i,Y.rows(),1) = Y_col;
  }

  VectorNd z (c);
  SparseSolveLTx (model, H, z);

  K = Y.transpose() * Y;

  a = gamma - Y.transpose() * z;

  lambda = K.llt().solve(a);

  qddot = c + G.transpose() * lambda;
  SparseSolveLTx (model, H, qddot);
  SparseSolveLx (model, H, qddot);
}

//==============================================================================
RBDL_DLLAPI
void SolveConstrainedSystemNullSpace (
  Math::MatrixNd &H,
  const Math::MatrixNd &G,
  const Math::VectorNd &c,
  const Math::VectorNd &gamma,
  Math::VectorNd &qddot,
  Math::VectorNd &lambda,
  Math::MatrixNd &Y,
  Math::MatrixNd &Z,
  Math::VectorNd &qddot_y,
  Math::VectorNd &qddot_z,
  Math::LinearSolver &linear_solver
)
{
  switch (linear_solver) {
  case (LinearSolverPartialPivLU) :
    qddot_y = (G * Y).partialPivLu().solve (gamma);
    break;
  case (LinearSolverColPivHouseholderQR) :
    qddot_y = (G * Y).colPivHouseholderQr().solve (gamma);
    break;
  case (LinearSolverHouseholderQR) :
    qddot_y = (G * Y).householderQr().solve (gamma);
    break;
  default:
    LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
    assert (0);
    break;
  }

  qddot_z = (Z.transpose()*H*Z).llt().solve(Z.transpose()*(c - H*Y*qddot_y));

  qddot = Y * qddot_y + Z * qddot_z;

  switch (linear_solver) {
  case (LinearSolverPartialPivLU) :
    lambda = (G * Y).partialPivLu().solve (Y.transpose() * (H * qddot - c));
    break;
  case (LinearSolverColPivHouseholderQR) :
    lambda = (G*Y).colPivHouseholderQr().solve (Y.transpose()*(H*qddot - c));
    break;
  case (LinearSolverHouseholderQR) :
    lambda = (G * Y).householderQr().solve (Y.transpose() * (H * qddot - c));
    break;
  default:
    LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
    assert (0);
    break;
  }
}


//==============================================================================
RBDL_DLLAPI
void CalcConstraintsPositionError (
  Model& model,
  const Math::VectorNd &Q,
  ConstraintSet &CS,
  Math::VectorNd& err,
  bool update_kinematics
)
{
  assert(err.size() == CS.size());

  if(update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  for(unsigned int i=0; i<CS.constraints.size(); ++i) {
    CS.constraints[i]->calcPositionError(model,0,Q,err, CS.cache,
                                         update_kinematics);
  }
}

//==============================================================================
RBDL_DLLAPI
void CalcConstraintsJacobian (
  Model &model,
  const Math::VectorNd &Q,
  ConstraintSet &CS,
  Math::MatrixNd &G,
  bool update_kinematics
)
{
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  for(unsigned int i=0; i<CS.constraints.size(); ++i) {
    CS.constraints[i]->calcConstraintJacobian(model,0,Q,CS.cache.vecNZeros,G,
        CS.cache,update_kinematics);
  }
}

//==============================================================================
RBDL_DLLAPI
void CalcConstraintsVelocityError (
  Model& model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  ConstraintSet &CS,
  Math::VectorNd& err,
  bool update_kinematics
)
{


  CalcConstraintsJacobian (model, Q, CS, CS.G, update_kinematics);

  for(unsigned int i=0; i<CS.constraints.size(); ++i) {
    CS.constraints[i]->calcVelocityError(model,0,Q,QDot,CS.G,err,CS.cache,
                                         update_kinematics);
  }

}

//==============================================================================
RBDL_DLLAPI
void CalcConstrainedSystemVariables (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  const Math::VectorNd &Tau,
  ConstraintSet &CS,
  std::vector<Math::SpatialVector> *f_ext
)
{
  // Compute C
  NonlinearEffects(model, Q, QDot, CS.C, f_ext);
  assert(CS.H.cols() == model.dof_count && CS.H.rows() == model.dof_count);

  // Compute H
  CS.H.setZero();
  CompositeRigidBodyAlgorithm(model, Q, CS.H, false);

  // Compute G
  // We have to update model.X_base as they are not automatically computed
  // by NonlinearEffects()
  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    model.X_base[i] = model.X_lambda[i] * model.X_base[model.lambda[i]];
  }
  CalcConstraintsJacobian (model, Q, CS, CS.G, false);

  // Compute position error for Baumgarte Stabilization.
  CalcConstraintsPositionError (model, Q, CS, CS.err, false);

  // Compute velocity error for Baugarte stabilization.
  CalcConstraintsVelocityError (model, Q, QDot, CS, CS.errd, false);
  //CS.errd = CS.G * QDot;

  // Compute gamma
  unsigned int prev_body_id = 0;
  Vector3d prev_body_point = Vector3d::Zero();
  Vector3d gamma_i = Vector3d::Zero();

  CS.QDDot_0.setZero();
  UpdateKinematicsCustom(model, NULL, NULL, &CS.QDDot_0);


  for(unsigned int i=0; i<CS.constraints.size(); ++i) {
    CS.constraints[i]->calcGamma(model,0,Q,QDot,CS.G,CS.gamma,CS.cache);
    if(CS.constraints[i]->isBaumgarteStabilizationEnabled()) {
      CS.constraints[i]->addInBaumgarteStabilizationForces(
        CS.err,CS.errd,CS.gamma);
    }
  }


}

//==============================================================================
RBDL_DLLAPI
bool CalcAssemblyQ (
  Model &model,
  Math::VectorNd QInit, // Note: passed by value intentionally
  ConstraintSet &cs,
  Math::VectorNd &Q,
  const Math::VectorNd &weights,
  double tolerance,
  unsigned int max_iter
)
{

  if(Q.size() != model.q_size) {
    throw Errors::RBDLDofMismatchError("Incorrect Q vector size.\n");
  }
  if(QInit.size() != model.q_size) {
    throw Errors::RBDLDofMismatchError("Incorrect QInit vector size.\n");
  }
  if(weights.size() != model.dof_count) {
    throw Errors::RBDLDofMismatchError("Incorrect weights vector size.\n");
  }

  // Initialize variables.
  MatrixNd constraintJac (cs.size(), model.dof_count);
  MatrixNd A = MatrixNd::Zero (cs.size() + model.dof_count, cs.size()
                               + model.dof_count);
  VectorNd b = VectorNd::Zero (cs.size() + model.dof_count);
  VectorNd x = VectorNd::Zero (cs.size() + model.dof_count);
  VectorNd d = VectorNd::Zero (model.dof_count);
  VectorNd e = VectorNd::Zero (cs.size());

  // The top-left block is the weight matrix and is constant.
  for(unsigned int i = 0; i < weights.size(); ++i) {
    A(i,i) = weights[i];
  }

  // Check if the error is small enough already. If so, just return the initial
  // guess as the solution.
  CalcConstraintsPositionError (model, QInit, cs, e);
  if (e.norm() < tolerance) {
    Q = QInit;
    return true;
  }

  // We solve the linearized problem iteratively.
  // Iterations are stopped if the maximum is reached.
  for(unsigned int it = 0; it < max_iter; ++it) {
    // Compute the constraint jacobian and build A and b.
    constraintJac.setZero();
    CalcConstraintsJacobian (model, QInit, cs, constraintJac);
    A.block (model.dof_count, 0, cs.size(), model.dof_count) = constraintJac;
    A.block (0, model.dof_count, model.dof_count, cs.size())
      = constraintJac.transpose();
    b.block (model.dof_count, 0, cs.size(), 1) = -e;

    // Solve the sistem A*x = b.
    SolveLinearSystem (A, b, x, cs.linear_solver);

    // Extract the d = (Q - QInit) vector from x.
    d = x.block (0, 0, model.dof_count, 1);

    // Update solution.
    for (size_t i = 0; i < model.mJoints.size(); ++i) {
      // If the joint is spherical, translate the corresponding components
      // of d into a modification in the joint quaternion.
      if (model.mJoints[i].mJointType == JointTypeSpherical) {
        Quaternion quat = model.GetQuaternion(i, QInit);
        Vector3d omega = d.block<3,1>(model.mJoints[i].q_index,0);
        // Convert the 3d representation of the displacement to 4d and sum it
        // to the components of the quaternion.
        quat += quat.omegaToQDot(omega);
        // The quaternion needs to be normalized after the previous sum.
        quat /= quat.norm();
        model.SetQuaternion(i, quat, QInit);
      }
      // If the current joint is not spherical, simply add the corresponding
      // components of d to QInit.
      else {
        unsigned int qIdx = model.mJoints[i].q_index;
        for(size_t j = 0; j < model.mJoints[i].mDoFCount; ++j) {
          QInit[qIdx + j] += d[qIdx + j];
        }
      }
    }

    // Update the errors.
    CalcConstraintsPositionError (model, QInit, cs, e);

    // Check if the error and the step are small enough to end.
    if (e.norm() < tolerance && d.norm() < tolerance) {
      Q = QInit;
      return true;
    }
  }

  // Return false if maximum number of iterations is exceeded.
  Q = QInit;
  return false;
}

//==============================================================================
RBDL_DLLAPI
void CalcAssemblyQDot (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotInit,
  ConstraintSet &cs,
  Math::VectorNd &QDot,
  const Math::VectorNd &weights
)
{
  if(QDot.size() != model.dof_count) {
    throw Errors::RBDLDofMismatchError("Incorrect QDot vector size.\n");
  }
  if(Q.size() != model.q_size) {
    throw Errors::RBDLDofMismatchError("Incorrect Q vector size.\n");
  }
  if(QDotInit.size() != QDot.size()) {
    throw Errors::RBDLDofMismatchError("Incorrect QDotInit vector size.\n");
  }
  if(weights.size() != QDot.size()) {
    throw Errors::RBDLDofMismatchError("Incorrect weight vector size.\n");
  }

  // Initialize variables.
  MatrixNd constraintJac = MatrixNd::Zero(cs.size(), model.dof_count);
  MatrixNd A = MatrixNd::Zero(cs.size() + model.dof_count, cs.size()
                              + model.dof_count);
  VectorNd b = VectorNd::Zero(cs.size() + model.dof_count);
  VectorNd x = VectorNd::Zero(cs.size() + model.dof_count);

  // The top-left block is the weight matrix and is constant.
  for(unsigned int i = 0; i < weights.size(); ++i) {
    A(i,i) = weights[i];
    b[i] = weights[i] * QDotInit[i];
  }
  CalcConstraintsJacobian (model, Q, cs, constraintJac);
  A.block (model.dof_count, 0, cs.size(), model.dof_count) = constraintJac;
  A.block (0, model.dof_count, model.dof_count, cs.size())
    = constraintJac.transpose();

  // Solve the sistem A*x = b.
  SolveLinearSystem (A, b, x, cs.linear_solver);

  // Copy the result to the output variable.
  QDot = x.block (0, 0, model.dof_count, 1);
}

//==============================================================================
RBDL_DLLAPI
void ForwardDynamicsConstraintsDirect (
  Model &model,
  const VectorNd &Q,
  const VectorNd &QDot,
  const VectorNd &Tau,
  ConstraintSet &CS,
  VectorNd &QDDot,
  std::vector<Math::SpatialVector> *f_ext
)
{
  LOG << "-------- " << __func__ << " --------" << std::endl;

  CalcConstrainedSystemVariables (model, Q, QDot, Tau, CS, f_ext);

  SolveConstrainedSystemDirect (CS.H, CS.G, Tau - CS.C, CS.gamma
                                , CS.force, CS.A, CS.b, CS.x, CS.linear_solver);

  // Copy back QDDot
  for (unsigned int i = 0; i < model.dof_count; i++) {
    QDDot[i] = CS.x[i];
  }

  // Copy back contact forces
  for (unsigned int i = 0; i < CS.size(); i++) {
    CS.force[i] = -CS.x[model.dof_count + i];
  }
}

//==============================================================================
RBDL_DLLAPI
void ForwardDynamicsConstraintsRangeSpaceSparse (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  const Math::VectorNd &Tau,
  ConstraintSet &CS,
  Math::VectorNd &QDDot,
  std::vector<Math::SpatialVector> *f_ext)
{

  CalcConstrainedSystemVariables (model, Q, QDot, Tau, CS, f_ext);

  SolveConstrainedSystemRangeSpaceSparse (model, CS.H, CS.G, Tau - CS.C
                                          , CS.gamma, QDDot, CS.force, CS.K, CS.a, CS.linear_solver);
}

//==============================================================================
RBDL_DLLAPI
void ForwardDynamicsConstraintsNullSpace (
  Model &model,
  const VectorNd &Q,
  const VectorNd &QDot,
  const VectorNd &Tau,
  ConstraintSet &CS,
  VectorNd &QDDot,
  std::vector<Math::SpatialVector> *f_ext
)
{

  LOG << "-------- " << __func__ << " --------" << std::endl;

  CalcConstrainedSystemVariables (model, Q, QDot, Tau, CS, f_ext);

  CS.GT_qr.compute (CS.G.transpose());
  CS.GT_qr.householderQ().evalTo (CS.GT_qr_Q);

  CS.Y = CS.GT_qr_Q.block (0,0,QDot.rows(), CS.G.rows());
  CS.Z = CS.GT_qr_Q.block (0,CS.G.rows(),QDot.rows(), QDot.rows() - CS.G.rows());

  SolveConstrainedSystemNullSpace (CS.H, CS.G, Tau - CS.C, CS.gamma, QDDot
                                   , CS.force, CS.Y, CS.Z, CS.qddot_y, CS.qddot_z, CS.linear_solver);

}

//==============================================================================
RBDL_DLLAPI
void ComputeConstraintImpulsesDirect (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotMinus,
  ConstraintSet &CS,
  Math::VectorNd &QDotPlus
)
{

  // Compute H
  UpdateKinematicsCustom (model, &Q, NULL, NULL);
  CompositeRigidBodyAlgorithm (model, Q, CS.H, false);

  // Compute G
  CalcConstraintsJacobian (model, Q, CS, CS.G, false);

  SolveConstrainedSystemDirect (CS.H, CS.G, CS.H * QDotMinus, CS.v_plus
                                , CS.impulse, CS.A, CS.b, CS.x, CS.linear_solver);

  // Copy back QDotPlus
  for (unsigned int i = 0; i < model.dof_count; i++) {
    QDotPlus[i] = CS.x[i];
  }

  // Copy back constraint impulses
  for (unsigned int i = 0; i < CS.size(); i++) {
    CS.impulse[i] = CS.x[model.dof_count + i];
  }

}

//==============================================================================
RBDL_DLLAPI
void ComputeConstraintImpulsesRangeSpaceSparse (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotMinus,
  ConstraintSet &CS,
  Math::VectorNd &QDotPlus
)
{

  // Compute H
  UpdateKinematicsCustom (model, &Q, NULL, NULL);
  CompositeRigidBodyAlgorithm (model, Q, CS.H, false);

  // Compute G
  CalcConstraintsJacobian (model, Q, CS, CS.G, false);

  SolveConstrainedSystemRangeSpaceSparse (model, CS.H, CS.G, CS.H * QDotMinus
                                          , CS.v_plus, QDotPlus, CS.impulse, CS.K, CS.a, CS.linear_solver);

}

//==============================================================================
RBDL_DLLAPI
void ComputeConstraintImpulsesNullSpace (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotMinus,
  ConstraintSet &CS,
  Math::VectorNd &QDotPlus
)
{

  // Compute H
  UpdateKinematicsCustom (model, &Q, NULL, NULL);
  CompositeRigidBodyAlgorithm (model, Q, CS.H, false);

  // Compute G
  CalcConstraintsJacobian (model, Q, CS, CS.G, false);

  CS.GT_qr.compute(CS.G.transpose());
  CS.GT_qr_Q = CS.GT_qr.householderQ();

  CS.Y = CS.GT_qr_Q.block (0,0,QDotMinus.rows(), CS.G.rows());
  CS.Z = CS.GT_qr_Q.block (0,CS.G.rows(),QDotMinus.rows(), QDotMinus.rows()
                           - CS.G.rows());

  SolveConstrainedSystemNullSpace (CS.H, CS.G, CS.H * QDotMinus, CS.v_plus
                                   , QDotPlus, CS.impulse, CS.Y, CS.Z, CS.qddot_y, CS.qddot_z
                                   , CS.linear_solver);
}

//==============================================================================
/** @brief Compute only the effects of external forces on the generalized
    accelerations

    This function is a reduced version of ForwardDynamics() which only
    computes the effects of the external forces on the generalized
    accelerations.

 */
RBDL_DLLAPI
void ForwardDynamicsApplyConstraintForces (
  Model &model,
  const VectorNd &Tau,
  ConstraintSet &CS,
  VectorNd &QDDot
)
{
  LOG << "-------- " << __func__ << " --------" << std::endl;
  assert (QDDot.size() == model.dof_count);

  unsigned int i = 0;

  for (i = 1; i < model.mBodies.size(); i++) {
    model.IA[i] = model.I[i].toMatrix();;
    model.pA[i] = crossf(model.v[i],model.I[i] * model.v[i]);

    if (CS.f_ext_constraints[i] != SpatialVector::Zero()) {
      LOG << "External force (" << i << ") = "
          << model.X_base[i].toMatrixAdjoint() * CS.f_ext_constraints[i]
          << std::endl;
      model.pA[i] -= model.X_base[i].toMatrixAdjoint()*CS.f_ext_constraints[i];
    }
  }

  // ClearLogOutput();

  LOG << "--- first loop ---" << std::endl;

  for (i = model.mBodies.size() - 1; i > 0; i--) {
    unsigned int q_index = model.mJoints[i].q_index;

    if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      unsigned int lambda = model.lambda[i];
      model.multdof3_u[i] = Vector3d (Tau[q_index],
                                      Tau[q_index + 1],
                                      Tau[q_index + 2])
                            - model.multdof3_S[i].transpose() * model.pA[i];

      if (lambda != 0) {
        SpatialMatrix Ia = model.IA[i] - (model.multdof3_U[i]
                                          * model.multdof3_Dinv[i]
                                          * model.multdof3_U[i].transpose());

        SpatialVector pa = model.pA[i] + Ia * model.c[i]
                           + model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_u[i];

        model.IA[lambda].noalias() += (model.X_lambda[i].toMatrixTranspose()
                                       * Ia * model.X_lambda[i].toMatrix());
        model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);

        LOG << "pA[" << lambda << "] = " << model.pA[lambda].transpose()
            << std::endl;
      }
    } else if (model.mJoints[i].mDoFCount == 1
               && model.mJoints[i].mJointType != JointTypeCustom) {
      model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);

      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        SpatialMatrix Ia = model.IA[i]
                           - model.U[i] * (model.U[i] / model.d[i]).transpose();
        SpatialVector pa =  model.pA[i] + Ia * model.c[i]
                            + model.U[i] * model.u[i] / model.d[i];

        model.IA[lambda].noalias() += (model.X_lambda[i].toMatrixTranspose()
                                       * Ia * model.X_lambda[i].toMatrix());
        model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);

        LOG << "pA[" << lambda << "] = "
            << model.pA[lambda].transpose() << std::endl;
      }
    } else if(model.mJoints[i].mJointType == JointTypeCustom) {

      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
      unsigned int lambda = model.lambda[i];
      VectorNd tau_temp = VectorNd::Zero(dofI);

      for(int z=0; z<dofI; ++z) {
        tau_temp[z] = Tau[q_index+z];
      }

      model.mCustomJoints[kI]->u = tau_temp
                                   - (model.mCustomJoints[kI]->S.transpose()
                                      * model.pA[i]);

      if (lambda != 0) {
        SpatialMatrix Ia = model.IA[i]
                           - (   model.mCustomJoints[kI]->U
                                 * model.mCustomJoints[kI]->Dinv
                                 * model.mCustomJoints[kI]->U.transpose());

        SpatialVector pa = model.pA[i] + Ia * model.c[i]
                           + ( model.mCustomJoints[kI]->U
                               * model.mCustomJoints[kI]->Dinv
                               * model.mCustomJoints[kI]->u );

        model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose()
                                      * Ia * model.X_lambda[i].toMatrix();

        model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);

        LOG << "pA[" << lambda << "] = " << model.pA[lambda].transpose()
            << std::endl;
      }
    }
  }

  model.a[0] = SpatialVector (0., 0., 0.,
                              -model.gravity[0],
                              -model.gravity[1],
                              -model.gravity[2]);

  for (i = 1; i < model.mBodies.size(); i++) {
    unsigned int q_index = model.mJoints[i].q_index;
    unsigned int lambda = model.lambda[i];
    SpatialTransform X_lambda = model.X_lambda[i];

    model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];
    LOG << "a'[" << i << "] = " << model.a[i].transpose() << std::endl;

    if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      Vector3d qdd_temp = model.multdof3_Dinv[i] *
                          (model.multdof3_u[i]
                           - model.multdof3_U[i].transpose() * model.a[i]);

      QDDot[q_index] = qdd_temp[0];
      QDDot[q_index + 1] = qdd_temp[1];
      QDDot[q_index + 2] = qdd_temp[2];
      model.a[i] = model.a[i] + model.multdof3_S[i] * qdd_temp;
    } else if (model.mJoints[i].mDoFCount == 1
               && model.mJoints[i].mJointType != JointTypeCustom) {
      QDDot[q_index] = (1./model.d[i]) * (
                         model.u[i] - model.U[i].dot(model.a[i]));
      model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
      VectorNd qdd_temp = VectorNd::Zero(dofI);

      qdd_temp = model.mCustomJoints[kI]->Dinv
                 * (model.mCustomJoints[kI]->u
                    - model.mCustomJoints[kI]->U.transpose()
                    * model.a[i]);

      for(int z=0; z<dofI; ++z) {
        QDDot[q_index+z] = qdd_temp[z];
      }

      model.a[i] = model.a[i] + (model.mCustomJoints[kI]->S * qdd_temp);
    }
  }

  LOG << "QDDot = " << QDDot.transpose() << std::endl;
}

//==============================================================================
/** @brief Computes the effect of external forces on the generalized
            accelerations.

    This function is essentially similar to ForwardDynamics() except that it
    tries to only perform computations of variables that change due to
    external forces defined in f_t.
 */
RBDL_DLLAPI
void ForwardDynamicsAccelerationDeltas (
  Model &model,
  ConstraintSet &CS,
  VectorNd &QDDot_t,
  const unsigned int body_id,
  const std::vector<SpatialVector> &f_t
)
{
  LOG << "-------- " << __func__ << " ------" << std::endl;

  assert (CS.d_pA.size() == model.mBodies.size());
  assert (CS.d_a.size() == model.mBodies.size());
  assert (CS.d_u.size() == model.mBodies.size());

  // TODO reset all values (debug)
  for (unsigned int i = 0; i < model.mBodies.size(); i++) {
    CS.d_pA[i].setZero();
    CS.d_a[i].setZero();
    CS.d_u[i] = 0.;
    CS.d_multdof3_u[i].setZero();
  }
  for(unsigned int i=0; i<model.mCustomJoints.size(); i++) {
    model.mCustomJoints[i]->d_u.setZero();
  }

  for (unsigned int i = body_id; i > 0; i--) {
    if (i == body_id) {
      CS.d_pA[i] = -model.X_base[i].applyAdjoint(f_t[i]);
    }

    if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      CS.d_multdof3_u[i] = - model.multdof3_S[i].transpose() * (CS.d_pA[i]);

      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        CS.d_pA[lambda] =   CS.d_pA[lambda]
                            + model.X_lambda[i].applyTranspose (
                              CS.d_pA[i] + (model.multdof3_U[i]
                                            * model.multdof3_Dinv[i]
                                            * CS.d_multdof3_u[i]));
      }
    } else if(model.mJoints[i].mDoFCount == 1
              && model.mJoints[i].mJointType != JointTypeCustom) {
      CS.d_u[i] = - model.S[i].dot(CS.d_pA[i]);
      unsigned int lambda = model.lambda[i];

      if (lambda != 0) {
        CS.d_pA[lambda] = CS.d_pA[lambda]
                          + model.X_lambda[i].applyTranspose (
                            CS.d_pA[i] + model.U[i] * CS.d_u[i] / model.d[i]);
      }
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {

      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
      //CS.
      model.mCustomJoints[kI]->d_u =
        - model.mCustomJoints[kI]->S.transpose() * (CS.d_pA[i]);
      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        CS.d_pA[lambda] =
          CS.d_pA[lambda]
          + model.X_lambda[i].applyTranspose (
            CS.d_pA[i] + (   model.mCustomJoints[kI]->U
                             * model.mCustomJoints[kI]->Dinv
                             * model.mCustomJoints[kI]->d_u)
          );
      }
    }
  }

  for (unsigned int i = 0; i < f_t.size(); i++) {
    LOG << "f_t[" << i << "] = " << f_t[i].transpose() << std::endl;
  }

  for (unsigned int i = 0; i < model.mBodies.size(); i++) {
    LOG << "i = " << i << ": d_pA[i] " << CS.d_pA[i].transpose() << std::endl;
  }
  for (unsigned int i = 0; i < model.mBodies.size(); i++) {
    LOG << "i = " << i << ": d_u[i] = " << CS.d_u[i] << std::endl;
  }

  QDDot_t[0] = 0.;
  CS.d_a[0] = model.a[0];

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    unsigned int q_index = model.mJoints[i].q_index;
    unsigned int lambda = model.lambda[i];

    SpatialVector Xa = model.X_lambda[i].apply(CS.d_a[lambda]);

    if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      Vector3d qdd_temp = model.multdof3_Dinv[i]
                          * (CS.d_multdof3_u[i] - model.multdof3_U[i].transpose() * Xa);

      QDDot_t[q_index] = qdd_temp[0];
      QDDot_t[q_index + 1] = qdd_temp[1];
      QDDot_t[q_index + 2] = qdd_temp[2];
      model.a[i] = model.a[i] + model.multdof3_S[i] * qdd_temp;
      CS.d_a[i] = Xa + model.multdof3_S[i] * qdd_temp;
    } else if (model.mJoints[i].mDoFCount == 1
               && model.mJoints[i].mJointType != JointTypeCustom) {

      QDDot_t[q_index] = (CS.d_u[i] - model.U[i].dot(Xa) ) / model.d[i];
      CS.d_a[i] = Xa + model.S[i] * QDDot_t[q_index];
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
      VectorNd qdd_temp = VectorNd::Zero(dofI);

      qdd_temp = model.mCustomJoints[kI]->Dinv
                 * (model.mCustomJoints[kI]->d_u
                    - model.mCustomJoints[kI]->U.transpose() * Xa);

      for(int z=0; z<dofI; ++z) {
        QDDot_t[q_index+z] = qdd_temp[z];
      }

      model.a[i] = model.a[i] + model.mCustomJoints[kI]->S * qdd_temp;
      CS.d_a[i] = Xa + model.mCustomJoints[kI]->S * qdd_temp;
    }

    LOG << "QDDot_t[" << i - 1 << "] = " << QDDot_t[i - 1] << std::endl;
    LOG << "d_a[i] = " << CS.d_a[i].transpose() << std::endl;
  }
}

inline void set_zero (std::vector<SpatialVector> &spatial_values)
{
  for (unsigned int i = 0; i < spatial_values.size(); i++) {
    spatial_values[i].setZero();
  }
}

//==============================================================================
RBDL_DLLAPI
void ForwardDynamicsContactsKokkevis (
  Model &model,
  const VectorNd &Q,
  const VectorNd &QDot,
  const VectorNd &Tau,
  ConstraintSet &CS,
  VectorNd &QDDot
)
{
  LOG << "-------- " << __func__ << " ------" << std::endl;

  assert (CS.f_ext_constraints.size() == model.mBodies.size());
  assert (CS.QDDot_0.size() == model.dof_count);
  assert (CS.QDDot_t.size() == model.dof_count);
  assert (CS.f_t.size() == CS.size());
  assert (CS.point_accel_0.size() == CS.size());
  assert (CS.K.rows() == CS.size());
  assert (CS.K.cols() == CS.size());
  assert (CS.force.size() == CS.size());
  assert (CS.a.size() == CS.size());

  if(CS.constraints.size() != CS.contactConstraints.size()) {
    std::stringstream errormsg;
    errormsg << "Incompatible constraint types: all constraints"
             << " must be ContactConstraints for the Kokkevis method"
             << std::endl;
    throw Errors::RBDLError(errormsg.str());
  }

  Vector3d point_accel_t;

  unsigned int ci = 0; //constraint index:
  // the row index in the constraint Jacobian.

  // The default acceleration only needs to be computed once
  {
    SUPPRESS_LOGGING;
    ForwardDynamics(model, Q, QDot, Tau, CS.QDDot_0);
  }

  LOG << "=== Initial Loop Start ===" << std::endl;
  // we have to compute the standard accelerations first as we use them to
  // compute the effects of each test force
  unsigned int bi = 0;
  for(bi =0; bi < CS.contactConstraints.size(); ++bi) {
    {
      SUPPRESS_LOGGING;
      UpdateKinematicsCustom(model, NULL, NULL, &CS.QDDot_0);
    }
    {
      LOG << "body_id = "
          << CS.contactConstraints[bi]->getBodyIds()[0]
          << std::endl;
      LOG << "point = "
          << CS.contactConstraints[bi]->getBodyFrames()[0].r
          << std::endl;
      LOG << "QDDot_0 = " << CS.QDDot_0.transpose() << std::endl;
    }
    {
      SUPPRESS_LOGGING;
      CS.contactConstraints[bi]->calcPointAccelerations(
        model,Q,QDot,CS.QDDot_0,CS.point_accel_0,false);
      CS.contactConstraints[bi]->calcPointAccelerationError(
        CS.point_accel_0,CS.a);
    }
  }

  // K: ContactConstraints
  unsigned int cj=0;
  unsigned int movable_body_id = 0;
  Vector3d point_global;

  for (bi = 0; bi < CS.contactConstraints.size(); bi++) {

    LOG << "=== Testforce Loop Start ===" << std::endl;

    ci = CS.contactConstraints[bi]->getConstraintIndex();

    movable_body_id = GetMovableBodyId(model,
                                       CS.contactConstraints[bi]->getBodyIds()[0]);

    // assemble the test force
    LOG << "point_global = " << point_global.transpose() << std::endl;

    CS.contactConstraints[bi]->calcPointForceJacobian(
      model,Q,CS.cache,CS.f_t,false);

    for(unsigned int j = 0; j<CS.contactConstraints[bi]
        ->getConstraintNormalVectors().size(); ++j) {

      CS.f_ext_constraints[movable_body_id] = CS.f_t[ci+j];

      LOG << "f_t[" << movable_body_id << "] = " << CS.f_t[ci+j].transpose()
          << std::endl;
      {
        ForwardDynamicsAccelerationDeltas(model, CS, CS.QDDot_t
                                          , movable_body_id, CS.f_ext_constraints);

        LOG << "QDDot_0 = " << CS.QDDot_0.transpose() << std::endl;
        LOG << "QDDot_t = " << (CS.QDDot_t + CS.QDDot_0).transpose()
            << std::endl;
        LOG << "QDDot_t - QDDot_0 = " << (CS.QDDot_t).transpose() << std::endl;
      }

      CS.f_ext_constraints[movable_body_id].setZero();

      CS.QDDot_t += CS.QDDot_0;
      // compute the resulting acceleration
      {
        SUPPRESS_LOGGING;
        UpdateKinematicsCustom(model, NULL, NULL, &CS.QDDot_t);
      }

      for(unsigned int dj = 0;
          dj < CS.contactConstraints.size(); dj++) {

        cj = CS.contactConstraints[dj]->getConstraintIndex();
        {
          SUPPRESS_LOGGING;
          CS.contactConstraints[dj]->calcPointAccelerations(
            model,Q,QDot,CS.QDDot_t,point_accel_t,false);
        }

        LOG << "point_accel_0  = " << CS.point_accel_0[ci+j].transpose()
            << std::endl;
        LOG << "point_accel_t = " << point_accel_t.transpose() << std::endl;
        for(unsigned int k=0;
            k < CS.contactConstraints[dj]
            ->getConstraintNormalVectors().size(); ++k) {

          CS.K(ci+j,cj+k) = CS.contactConstraints[dj]
                            ->getConstraintNormalVectors()[k].dot(
                              point_accel_t - CS.point_accel_0[cj+k]);
        }
      }
    }
  }




  LOG << "K = " << std::endl << CS.K << std::endl;
  LOG << "a = " << std::endl << CS.a << std::endl;

  switch (CS.linear_solver) {
  case (LinearSolverPartialPivLU) :
    CS.force = CS.K.partialPivLu().solve(CS.a);
    break;
  case (LinearSolverColPivHouseholderQR) :
    CS.force = CS.K.colPivHouseholderQr().solve(CS.a);
    break;
  case (LinearSolverHouseholderQR) :
    CS.force = CS.K.householderQr().solve(CS.a);
    break;
  default:
    LOG << "Error: Invalid linear solver: " << CS.linear_solver << std::endl;
    assert (0);
    break;
  }

  LOG << "f = " << CS.force.transpose() << std::endl;


  for(bi=0; bi<CS.contactConstraints.size(); ++bi) {
    unsigned int body_id =
      CS.contactConstraints[bi]->getBodyIds()[0];
    unsigned int movable_body_id = body_id;

    if (model.IsFixedBodyId(body_id)) {
      unsigned int fbody_id = body_id - model.fixed_body_discriminator;
      movable_body_id = model.mFixedBodies[fbody_id].mMovableParent;
    }
    ci = CS.contactConstraints[bi]->getConstraintIndex();

    for(unsigned int k=0;
        k<CS.contactConstraints[bi]->getConstraintSize(); ++k) {
      CS.f_ext_constraints[movable_body_id] -= CS.f_t[ci+k] * CS.force[ci+k];
      LOG << "f_ext[" << movable_body_id << "] = "
          << CS.f_ext_constraints[movable_body_id].transpose() << std::endl;
    }

  }



  {
    SUPPRESS_LOGGING;
    ForwardDynamicsApplyConstraintForces (model, Tau, CS, QDDot);
  }

  LOG << "QDDot after applying f_ext: " << QDDot.transpose() << std::endl;
}


//==============================================================================

RBDL_DLLAPI
bool isConstrainedSystemFullyActuated(
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  ConstraintSet &CS,
  std::vector<Math::SpatialVector> *f_ext)
{

  LOG << "-------- " << __func__ << " ------" << std::endl;


  assert (CS.S.cols()    == QDot.rows());

  unsigned int n  = unsigned(    CS.H.rows());
  unsigned int nc = unsigned( CS.name.size());
  unsigned int na = unsigned(    CS.S.rows());
  unsigned int nu = n-na;


  CalcConstrainedSystemVariables(model,Q,QDot,VectorNd::Zero(QDot.rows()),CS,
                                 f_ext);

  CS.GPT = CS.G*CS.P.transpose();

  CS.GPT_full_qr.compute(CS.GPT);
  unsigned int r = unsigned(CS.GPT_full_qr.rank());

  bool isCompatible = false;
  if(r == (n-na)) {
    isCompatible = true;
  } else {
    isCompatible = false;
  }

  return isCompatible;

}

RBDL_DLLAPI
void InverseDynamicsConstraints(
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  const Math::VectorNd &QDDotDesired,
  ConstraintSet &CS,
  Math::VectorNd &QDDotOutput,
  Math::VectorNd &TauOutput,
  std::vector<Math::SpatialVector> *f_ext)
{

  LOG << "-------- " << __func__ << " ------" << std::endl;

  assert (QDot.size()         == QDDotDesired.size());
  assert (QDDotDesired.size() == QDot.size());
  assert (QDDotOutput.size()  == QDot.size());
  assert (TauOutput.size()    == CS.H.rows());

  assert (CS.S.cols()     == QDDotDesired.rows());

  unsigned int n  = unsigned(    CS.H.rows());
  unsigned int nc = unsigned( CS.name.size());
  unsigned int na = unsigned(    CS.S.rows());
  unsigned int nu = n-na;

  TauOutput.setZero();
  CalcConstrainedSystemVariables(model,Q,QDot,TauOutput,CS,f_ext);

  // This implementation follows the projected KKT system described in
  // Eqn. 5.20 of Henning Koch's thesis work. Note that this will fail
  // for under actuated systems
  //  [ SMS'      SMP'    SJ'    I][      u]   [ -SC    ]
  //  [ PMS'      PMP'    PJ'     ][      v] = [ -PC    ]
  //  [ JS'        JP'     0      ][-lambda]   [ -gamma ]
  //  [ I                         ][   -tau]   [  v*     ]
  //double alpha = 0.1;

  CS.Ful = CS.S*CS.H*CS.S.transpose();
  CS.Fur = CS.S*CS.H*CS.P.transpose();
  CS.Fll = CS.P*CS.H*CS.S.transpose();
  CS.Flr = CS.P*CS.H*CS.P.transpose();

  CS.GTu = CS.S*(CS.G.transpose());
  CS.GTl = CS.P*(CS.G.transpose());

  //Exploiting the block triangular structure
  //u:
  //I u = S*qdd*
  CS.u = CS.S*QDDotDesired;
  // v
  //(JP')v = -gamma - (JS')u
  //Using GT

  //This fails using SimpleMath and I'm not sure how to fix it
  SolveLinearSystem( CS.GTl.transpose(),
                     CS.gamma - CS.GTu.transpose()*CS.u,
                     CS.v, CS.linear_solver);

  // lambda
  SolveLinearSystem(CS.GTl,
                    -CS.P*CS.C
                    - CS.Fll*CS.u
                    - CS.Flr*CS.v,
                    CS.force,
                    CS.linear_solver);

  for(unsigned int i=0; i<CS.force.rows(); ++i) {
    CS.force[i] *= -1.0;
  }

  //Evaluating qdd
  QDDotOutput = CS.S.transpose()*CS.u + CS.P.transpose()*CS.v;

  //Evaluating tau
  TauOutput = -CS.S.transpose()*( -CS.S*CS.C
                                  -( CS.Ful*CS.u
                                     +CS.Fur*CS.v
                                     -CS.GTu*CS.force));




}

RBDL_DLLAPI
void InverseDynamicsConstraintsRelaxed(
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  const Math::VectorNd &QDDotControls,
  ConstraintSet &CS,
  Math::VectorNd &QDDotOutput,
  Math::VectorNd &TauOutput,
  std::vector<Math::SpatialVector> *f_ext)
{
  LOG << "-------- " << __func__ << " --------" << std::endl;

  //Check that the input vectors and matricies are sized appropriately
  assert(Q.size()               == model.q_size);
  assert(QDot.size()            == model.qdot_size);
  assert(QDDotControls.size()    == model.dof_count);
  assert(CS.S.cols()            == model.dof_count);
  assert(CS.W.rows()            == CS.W.cols());
  assert(CS.W.rows()            == CS.S.rows());
  assert(QDDotOutput.size()     == model.dof_count);

  TauOutput.setZero();
  CalcConstrainedSystemVariables(model,Q,QDot,TauOutput,CS,f_ext);

  unsigned int n  = unsigned(    CS.H.rows());
  unsigned int nc = unsigned( CS.name.size());
  unsigned int na = unsigned(    CS.S.rows());
  unsigned int nu = n-na;

  //MM 2020/5/29:
  //  The updates I made to Henning's original formulation have
  //  almost certainly made the sensitivity of the resulting qdd
  //  w.r.t. the controls poorly scaled. At least this is a suspicion
  //  of mine looking at how an OCP is behaving when using this operator.
  //  Reverting to the original formulation.
  //MM: Update to Henning's formulation s.t. the relaxed IDC operator will
  //    more closely satisfy QDDotControls if it is possible.
  //double diag = 0.;//100.*CS.H.maxCoeff();
  //double diagInv = 0.;
  //for(unsigned int i=0; i<CS.H.rows(); ++i) {
  //  for(unsigned int j=0; j<CS.H.cols(); ++j) {
  //    if(fabs(CS.H(i,j)) > diag) {
  //      diag = fabs(CS.H(i,j));
  //    }
  //  }
  //}
  //diag = diag*100.;
  //diagInv = 1.0/diag;
  //for(unsigned int i=0; i<CS.W.rows(); ++i) {
  //  CS.W(i,i)    = diag;
  //  CS.Winv(i,i) = diagInv;
  //}

  CS.W = 100.0*CS.S*CS.H*CS.S.transpose();
  CS.Winv = CS.W.inverse();
  CS.WinvSC = CS.Winv * CS.S * CS.C;

  //CS.W = CS.S*CS.H*CS.S.transpose();

  CS.F.block(  0,  0, na, na) = CS.S*CS.H*CS.S.transpose() + CS.W;
  CS.F.block(  0, na, na, nu) = CS.S*CS.H*CS.P.transpose();
  CS.F.block( na,  0, nu, na) = CS.P*CS.H*CS.S.transpose();
  CS.F.block( na, na, nu, nu) = CS.P*CS.H*CS.P.transpose();

  CS.GT.block(  0, 0,na, nc) = CS.S*(CS.G.transpose());
  CS.GT.block( na, 0,nu, nc) = CS.P*(CS.G.transpose());

  CS.GT_qr.compute (CS.GT);
  CS.GT_qr.householderQ().evalTo (CS.GT_qr_Q);

  //GT = [Y  Z] * [ R ]
  //              [ 0 ]

  CS.R  = CS.GT_qr_Q.transpose()*CS.GT;
  CS.Ru = CS.R.block(0,0,nc,nc);

  CS.Y = CS.GT_qr_Q.block( 0, 0,  n, nc    );
  CS.Z = CS.GT_qr_Q.block( 0, nc, n, (n-nc));

  //MM: Update to Henning's formulation s.t. the relaxed IDC operator will
  //    exactly satisfy QDDotControls if it is possible.
  //
  //Modify QDDotControls so that SN is cancelled.
  //
  //    +SC - WS(qdd*)
  //
  // Add a term to cancel off SN
  //
  //    +SC - WS( qdd* + (S' W^-1 S)N )
  //

  CS.u = CS.S*CS.C - CS.W*(CS.S*(QDDotControls
                                 +(CS.S.transpose()*CS.WinvSC)));
  //CS.u =  CS.S*CS.C - CS.W*(CS.S*QDDotControls);
  CS.v =  CS.P*CS.C;

  for(unsigned int i=0; i<CS.S.rows(); ++i) {
    CS.g[i] = CS.u[i];
  }
  unsigned int j=CS.S.rows();
  for(unsigned int i=0; i<CS.P.rows(); ++i) {
    CS.g[j] = CS.v[i];
    ++j;
  }

  //nc x nc system
  SolveLinearSystem(CS.Ru.transpose(), CS.gamma, CS.py, CS.linear_solver);

  //(n-nc) x (n-nc) system
  SolveLinearSystem(CS.Z.transpose()*CS.F*CS.Z,
                    CS.Z.transpose()*(-CS.F*CS.Y*CS.py-CS.g),
                    CS.pz,
                    CS.linear_solver);

  //nc x nc system
  SolveLinearSystem(CS.Ru,
                    CS.Y.transpose()*(CS.g + CS.F*CS.Y*CS.py + CS.F*CS.Z*CS.pz),
                    CS.force, CS.linear_solver);

  //Eqn. 32d, the equation for qdd, is in error. Instead
  // p = Ypy + Zpz = [v,w]
  // qdd = S'v + P'w
  QDDotOutput = CS.Y*CS.py + CS.Z*CS.pz;
  for(unsigned int i=0; i<CS.S.rows(); ++i) {
    CS.u[i] = QDDotOutput[i];
  }
  j = CS.S.rows();
  for(unsigned int i=0; i<CS.P.rows(); ++i) {
    CS.v[i] = QDDotOutput[j];
    ++j;
  }

  QDDotOutput = CS.S.transpose()*CS.u
                +CS.P.transpose()*CS.v;

  TauOutput = (CS.S.transpose()*CS.W*CS.S)*(
                QDDotControls+(CS.S.transpose()*CS.WinvSC)
                -QDDotOutput);
  //TauOutput =  CS.S.transpose()*CS.W*CS.S*(QDDotControls - QDDotOutput);


}

void SolveLinearSystem (
  const MatrixNd& A,
  const VectorNd& b,
  VectorNd& x,
  LinearSolver ls
)
{
  if(A.rows() != b.size() || A.cols() != x.size()) {
    throw Errors::RBDLSizeMismatchError("Mismatching sizes.\n");
  }

  // Solve the system A*x = b.
  switch (ls) {
  case (LinearSolverPartialPivLU) :
    x = A.partialPivLu().solve(b);
    break;
  case (LinearSolverColPivHouseholderQR) :
    x = A.colPivHouseholderQr().solve(b);
    break;
  case (LinearSolverHouseholderQR) :
    x = A.householderQr().solve(b);
    break;
  default:
    std::ostringstream errormsg;
    errormsg << "Error: Invalid linear solver: " << ls << std::endl;
    throw Errors::RBDLError(errormsg.str());
    break;
  }
}

//==============================================================================
unsigned int GetMovableBodyId (Model& model, unsigned int id)
{
  if(model.IsFixedBodyId(id)) {
    unsigned int fbody_id = id - model.fixed_body_discriminator;
    return model.mFixedBodies[fbody_id].mMovableParent;
  } else {
    return id;
  }
}

//==============================================================================

void ConstraintSet::calcForces(
  unsigned int groupIndex,
  Model& model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  std::vector< unsigned int > &constraintBodyIdsUpd,
  std::vector< Math::SpatialTransform > &constraintBodyFramesUpd,
  std::vector< Math::SpatialVector > &constraintForcesUpd,
  bool resolveAllInRootFrame,
  bool updKin)
{
  assert(groupIndex <= unsigned(constraints.size()-1));

  if (updKin) {
    UpdateKinematicsCustom (model, &Q, &QDot, NULL);
  }

  constraints[groupIndex]->calcConstraintForces(model,0.,Q,QDot,G,force,
      constraintBodyIdsUpd,
      constraintBodyFramesUpd,
      constraintForcesUpd,
      cache,
      resolveAllInRootFrame,
      updKin);
}

//==============================================================================

void ConstraintSet::calcImpulses(
  unsigned int groupIndex,
  Model& model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  std::vector< unsigned int > &constraintBodyIdsUpd,
  std::vector< Math::SpatialTransform > &constraintBodyFramesUpd,
  std::vector< Math::SpatialVector > &constraintImpulsesUpd,
  bool resolveAllInRootFrame,
  bool updKin)
{
  assert(groupIndex <= unsigned(constraints.size()-1));

  if (updKin) {
    UpdateKinematicsCustom (model, &Q, &QDot, NULL);
  }

  //The transformation is identical to resolve the impulses
  //to the desired frame
  constraints[groupIndex]->calcConstraintForces(model,0.,Q,QDot,G,impulse,
      constraintBodyIdsUpd,
      constraintBodyFramesUpd,
      constraintImpulsesUpd,
      cache,
      resolveAllInRootFrame,
      updKin);

  //But due to Martin's choice of signs on the Lagrange multipliers vs
  //impulses the signs are opposite
  for(unsigned int i=0; i<constraintImpulsesUpd.size(); ++i) {
    constraintImpulsesUpd[i] *= -1.0;
  }
}

//==============================================================================

void ConstraintSet::calcPositionError(
  unsigned int groupIndex,
  Model& model,
  const Math::VectorNd &Q,
  Math::VectorNd &posErrUpd,
  bool updKin)
{
  assert(groupIndex <= unsigned(constraints.size()-1));

  if (updKin) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  //Update the position errors for this constraint in the system level
  //error vector
  constraints[groupIndex]->calcPositionError(model,0.,Q,err,cache,updKin);

  //Pick out the position errors for this constraint from the system level
  //error vector.
  constraints[groupIndex]->getPositionError(err,posErrUpd);

}
//==============================================================================

void ConstraintSet::calcVelocityError(
  unsigned int groupIndex,
  Model& model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  Math::VectorNd &velErrUpd,
  bool updKin)
{
  assert(groupIndex <= unsigned(constraints.size()-1));

  if (updKin) {
    UpdateKinematicsCustom (model, &Q, &QDot, NULL);
  }

  //Update the constraint Jacobian of this constraint
  constraints[groupIndex]->calcConstraintJacobian(model,0.,Q,QDot,G,cache,
      updKin);

  //Update the velocity-level errors of this constraint
  constraints[groupIndex]->calcVelocityError(model,0.,Q,QDot,G,errd,cache,
      updKin);

  //Pick out the sub vector of velocity errors for this constraint from
  //the system error vector.
  constraints[groupIndex]->getVelocityError(errd,velErrUpd);

}
//==============================================================================

void ConstraintSet::calcBaumgarteStabilizationForces(
  unsigned int groupIndex,
  Model& model,
  const Math::VectorNd &positionError,
  const Math::VectorNd &velocityError,
  Math::VectorNd &baumgarteForces)
{
  assert(groupIndex <= unsigned(constraints.size()-1));
  assert(positionError.rows() == constraints[groupIndex]->getConstraintSize());
  assert(velocityError.rows() == constraints[groupIndex]->getConstraintSize());

  constraints[groupIndex]->getBaumgarteStabilizationForces(positionError,
      velocityError,
      baumgarteForces);

}

} /* namespace RigidBodyDynamics */
