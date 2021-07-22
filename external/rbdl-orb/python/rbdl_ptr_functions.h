/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2015 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 *
 * This file defines functions that allows calling of the RBDL algorithms
 * by providing input and output as raw double arrays. It eliminates the
 * need of copying from Numpy values into temporary RBDL (C++) vectors and
 * matrices. However it requires C++11 and must be compiled with -std=c++11
 * (or -std=c++0x on older compilers).
 */

#include <rbdl/rbdl_math.h>
#include <rbdl/Dynamics.h>
#include <rbdl/Constraints.h>

#include <rbdl/rbdl_utils.h>

namespace RigidBodyDynamics
{

namespace Math
{

// PTR_DATA_ROW_MAJOR :
// Specifies whether the data that is provided via raw double pointers is
// stored as row major. Eigen uses column major by default and therefore
// this has to be properly mapped.
#define PTR_DATA_ROW_MAJOR 1

typedef Eigen::Ref<Eigen::VectorXd> VectorNdRef;

#ifdef PTR_DATA_ROW_MAJOR
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
  MatrixNdRowMaj;
  typedef Eigen::Ref<MatrixNdRowMaj> MatrixNdRef;
#else
  typedef Eigen::Ref<Eigen::MatrixXd> MatrixNdRef;
#endif


RBDL_DLLAPI inline VectorNdRef VectorFromPtr (double *ptr, unsigned int n)
{
  #ifdef EIGEN_CORE_H
  return Eigen::Map<VectorNd> (ptr, n, 1);
  #else
  std::ostringstream errormsg;
  errormsg << __func__ << " not defined for used math library!" << std::endl;
  throw Errors::RBDLError(errormsg.str());
  return VectorNd::Constant (1,1./0.);
  #endif
}

RBDL_DLLAPI inline MatrixNdRef MatrixFromPtr (double *ptr, unsigned int rows,
    unsigned int cols, bool row_major = true)
{
  #ifdef EIGEN_CORE_H
  #ifdef PTR_DATA_ROW_MAJOR
  return Eigen::Map<MatrixNdRowMaj> (ptr, rows, cols);
  #else
  return Eigen::Map<MatrixNd> (ptr, rows, cols);
  #endif
  #else
  std::ostringstream errormsg;
  errormsg << __func__ << " not defined for used math library!" << std::endl;
  throw Errors::RBDLError(errormsg.str());
  return MatrixNd::Constant (1,1, 1./0.);
  #endif
}

}

RBDL_DLLAPI
void UpdateKinematicsCustomPtr (Model &model,
                                const double *q_ptr,
                                const double *qdot_ptr,
                                const double *qddot_ptr
                               )
{
  LOG << "-------- " << __func__ << " --------" << std::endl;

  using namespace RigidBodyDynamics::Math;

  unsigned int i;

  if (q_ptr) {
    VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);

    for (i = 1; i < model.mBodies.size(); i++) {
      unsigned int lambda = model.lambda[i];

      VectorNd QDot_zero (VectorNd::Zero (model.q_size));

      jcalc (model, i, (Q), QDot_zero);

      model.X_lambda[i] = model.X_J[i] * model.X_T[i];

      if (lambda != 0) {
        model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
      } else {
        model.X_base[i] = model.X_lambda[i];
      }
    }
  }

  if (qdot_ptr) {
    VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
    VectorNdRef QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.q_size);

    for (i = 1; i < model.mBodies.size(); i++) {
      unsigned int lambda = model.lambda[i];

      jcalc (model, i, Q, QDot);

      if (lambda != 0) {
        model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
        model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
      } else {
        model.v[i] = model.v_J[i];
        model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
      }
      // LOG << "v[" << i << "] = " << model.v[i].transpose() << std::endl;
    }

  }

  if (qddot_ptr) {
    VectorNdRef QDDot = VectorFromPtr(const_cast<double*>(qddot_ptr), model.q_size);

    for (i = 1; i < model.mBodies.size(); i++) {
      unsigned int q_index = model.mJoints[i].q_index;

      unsigned int lambda = model.lambda[i];

      if (lambda != 0) {
        model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];
      } else {
        model.a[i] = model.c[i];
      }

      if( model.mJoints[i].mJointType != JointTypeCustom) {
        if (model.mJoints[i].mDoFCount == 1) {
          model.a[i] = model.a[i] + model.S[i] * (QDDot)[q_index];
        } else if (model.mJoints[i].mDoFCount == 3) {
          Vector3d omegadot_temp ((QDDot)[q_index],
                                  (QDDot)[q_index + 1],
                                  (QDDot)[q_index + 2]);
          model.a[i] = model.a[i]
                       + model.multdof3_S[i] * omegadot_temp;
        }
      } else {
        unsigned int k = model.mJoints[i].custom_joint_index;

        const CustomJoint* custom_joint = model.mCustomJoints[k];
        unsigned int joint_dof_count = custom_joint->mDoFCount;

        model.a[i] = model.a[i]
                     + (  (model.mCustomJoints[k]->S)
                          *(QDDot.block(q_index, 0, joint_dof_count, 1)));
      }
    }
  }

}






RBDL_DLLAPI
void CalcPointJacobianPtr (
  Model &model,
  const double *q_ptr,
  unsigned int body_id,
  const Math::Vector3d &point_position,
  double * G_ptr,
  bool update_kinematics
)
{
  LOG << "-------- " << __func__ << " --------" << std::endl;

  using namespace RigidBodyDynamics::Math;

  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustomPtr (model, q_ptr, NULL, NULL);
  }

  VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  MatrixNdRef G = MatrixFromPtr(const_cast<double*>(G_ptr), 3, model.qdot_size);

  SpatialTransform point_trans =
    SpatialTransform (Matrix3d::Identity(),
                      CalcBodyToBaseCoordinates ( model,
                          Q,
                          body_id,
                          point_position,
                          false));

  assert (G.rows() == 3 && G.cols() == model.qdot_size );

  unsigned int reference_body_id = body_id;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
  }

  unsigned int j = reference_body_id;

  // e[j] is set to 1 if joint j contributes to the jacobian that we are
  // computing. For all other joints the column will be zero.
  while (j != 0) {
    unsigned int q_index = model.mJoints[j].q_index;

    if(model.mJoints[j].mJointType != JointTypeCustom) {
      if (model.mJoints[j].mDoFCount == 1) {
        G.block(0,q_index, 3, 1) =
          point_trans.apply(
            model.X_base[j].inverse().apply(
              model.S[j])).block(3,0,3,1);
      } else if (model.mJoints[j].mDoFCount == 3) {
        G.block(0, q_index, 3, 3) =
          ((point_trans
            * model.X_base[j].inverse()).toMatrix()
           * model.multdof3_S[j]).block(3,0,3,3);
      }
    } else {
      unsigned int k = model.mJoints[j].custom_joint_index;

      G.block(0, q_index, 3, model.mCustomJoints[k]->mDoFCount) =
        ((point_trans
          * model.X_base[j].inverse()).toMatrix()
         * model.mCustomJoints[k]->S).block(
          3,0,3,model.mCustomJoints[k]->mDoFCount);
    }

    j = model.lambda[j];
  }
}




RBDL_DLLAPI
void CalcPointJacobian6DPtr (
  Model &model,
  const double *q_ptr,
  unsigned int body_id,
  const Math::Vector3d &point_position,
  double *G_ptr,
  bool update_kinematics
)
{
  LOG << "-------- " << __func__ << " --------" << std::endl;

  using namespace RigidBodyDynamics::Math;

  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustomPtr (model, q_ptr, NULL, NULL);
  }

  VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  MatrixNdRef G = MatrixFromPtr(const_cast<double*>(G_ptr), 6, model.qdot_size);

  SpatialTransform point_trans =
    SpatialTransform (Matrix3d::Identity(),
                      CalcBodyToBaseCoordinates (model,
                          Q,
                          body_id,
                          point_position,
                          false));


  assert (G.rows() == 6 && G.cols() == model.qdot_size );

  unsigned int reference_body_id = body_id;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
  }

  unsigned int j = reference_body_id;

  while (j != 0) {
    unsigned int q_index = model.mJoints[j].q_index;

    if(model.mJoints[j].mJointType != JointTypeCustom) {
      if (model.mJoints[j].mDoFCount == 1) {
        G.block(0,q_index, 6, 1)
          = point_trans.apply(
              model.X_base[j].inverse().apply(
                model.S[j])).block(0,0,6,1);
      } else if (model.mJoints[j].mDoFCount == 3) {
        G.block(0, q_index, 6, 3)
          = ((point_trans
              * model.X_base[j].inverse()).toMatrix()
             * model.multdof3_S[j]).block(0,0,6,3);
      }
    } else {
      unsigned int k = model.mJoints[j].custom_joint_index;

      G.block(0, q_index, 6, model.mCustomJoints[k]->mDoFCount)
        = ((point_trans
            * model.X_base[j].inverse()).toMatrix()
           * model.mCustomJoints[k]->S).block(
            0,0,6,model.mCustomJoints[k]->mDoFCount);
    }

    j = model.lambda[j];
  }
}




RBDL_DLLAPI
void CalcBodySpatialJacobianPtr (
  Model &model,
  const double *q_ptr,
  unsigned int body_id,
  double *G_ptr,
  bool update_kinematics
)
{
  LOG << "-------- " << __func__ << " --------" << std::endl;

  using namespace RigidBodyDynamics::Math;

  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustomPtr (model, q_ptr, NULL, NULL);
  }

  MatrixNdRef G = MatrixFromPtr(const_cast<double*>(G_ptr), 6, model.q_size);

  assert (G.rows() == 6 && G.cols() == model.qdot_size );

  unsigned int reference_body_id = body_id;

  SpatialTransform base_to_body;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id   = body_id
                              - model.fixed_body_discriminator;

    reference_body_id       = model
                              .mFixedBodies[fbody_id]
                              .mMovableParent;

    base_to_body = model.mFixedBodies[fbody_id]
                   .mParentTransform
                   * model.X_base[reference_body_id];
  } else {
    base_to_body = model.X_base[reference_body_id];
  }

  unsigned int j = reference_body_id;

  while (j != 0) {
    unsigned int q_index = model.mJoints[j].q_index;

    if(model.mJoints[j].mJointType != JointTypeCustom) {
      if (model.mJoints[j].mDoFCount == 1) {
        G.block(0,q_index,6,1) =
          base_to_body.apply(
            model.X_base[j]
            .inverse()
            .apply(model.S[j])
          );
      } else if (model.mJoints[j].mDoFCount == 3) {
        G.block(0,q_index,6,3) =
          (base_to_body * model.X_base[j].inverse()
          ).toMatrix() * model.multdof3_S[j];
      }
    } else if(model.mJoints[j].mJointType == JointTypeCustom) {
      unsigned int k = model.mJoints[j].custom_joint_index;

      G.block(0,q_index,6,model.mCustomJoints[k]->mDoFCount ) =
        (base_to_body * model.X_base[j].inverse()
        ).toMatrix() * model.mCustomJoints[k]->S;
    }

    j = model.lambda[j];
  }
}




RBDL_DLLAPI
void InverseDynamicsPtr (
  Model &model,
  const double *q_ptr,
  const double *qdot_ptr,
  const double *qddot_ptr,
  const double *tau_ptr,
  std::vector<Math::SpatialVector> *f_ext
)
{
  LOG << "-------- " << __func__ << " --------" << std::endl;

  using namespace RigidBodyDynamics::Math;


  VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  VectorNdRef QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.qdot_size);
  VectorNdRef QDDot = VectorFromPtr(const_cast<double*>(qddot_ptr), model.qdot_size);
  VectorNdRef Tau = VectorFromPtr(const_cast<double*>(tau_ptr), model.qdot_size);

  // Reset the velocity of the root body
  model.v[0].setZero();
  model.a[0].set (0., 0., 0., -model.gravity[0], -model.gravity[1],
                  -model.gravity[2]);

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    unsigned int q_index = model.mJoints[i].q_index;
    unsigned int lambda = model.lambda[i];

    jcalc (model, i, Q, QDot);

    model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
    model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);

    if(model.mJoints[i].mJointType != JointTypeCustom) {
      if (model.mJoints[i].mDoFCount == 1) {
        model.a[i] =  model.X_lambda[i].apply(model.a[lambda])
                      + model.c[i]
                      + model.S[i] * QDDot[q_index];
      } else if (model.mJoints[i].mDoFCount == 3) {
        model.a[i] =  model.X_lambda[i].apply(model.a[lambda])
                      + model.c[i]
                      + model.multdof3_S[i] * Vector3d (QDDot[q_index],
                          QDDot[q_index + 1],
                          QDDot[q_index + 2]);
      }
    } else if(model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int k = model.mJoints[i].custom_joint_index;
      VectorNd customJointQDDot(model.mCustomJoints[k]->mDoFCount);
      for(int z=0; z<model.mCustomJoints[k]->mDoFCount; ++z) {
        customJointQDDot[z] = QDDot[q_index+z];
      }
      model.a[i] =  model.X_lambda[i].apply(model.a[lambda])
                    + model.c[i]
                    + model.mCustomJoints[k]->S * customJointQDDot;
    }

    if (!model.mBodies[i].mIsVirtual) {
      model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i],
                   model.I[i] * model.v[i]);
    } else {
      model.f[i].setZero();
    }
  }

  if (f_ext != NULL ) {
    for (unsigned int i = 1; i < model.mBodies.size(); i++) {
      unsigned int lambda = model.lambda[i];
      model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
      model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
    }
  }

  for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
    if(model.mJoints[i].mJointType != JointTypeCustom) {
      if (model.mJoints[i].mDoFCount == 1) {
        Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f[i]);
      } else if (model.mJoints[i].mDoFCount == 3) {
        Tau.block<3,1>(model.mJoints[i].q_index, 0)
          = model.multdof3_S[i].transpose() * model.f[i];
      }
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int k = model.mJoints[i].custom_joint_index;
      Tau.block(model.mJoints[i].q_index,0,
                model.mCustomJoints[k]->mDoFCount, 1)
        = model.mCustomJoints[k]->S.transpose() * model.f[i];
    }

    if (model.lambda[i] != 0) {
      model.f[model.lambda[i]] = model.f[model.lambda[i]] +
                                 model.X_lambda[i].applyTranspose(model.f[i]);
    }
  }
}

// FIXME(yycho0108): {Matrix,Vector}NdRef conversions will fail.
// Consider other options for specifying the correct function signature.
void SolveLinearSystem (
        const RigidBodyDynamics::Math::MatrixNd& A,
        const RigidBodyDynamics::Math::VectorNd& b,
        RigidBodyDynamics::Math::VectorNd& x,
        RigidBodyDynamics::Math::LinearSolver ls
        )
{
    if(A.rows() != b.size() || A.cols() != x.size()) {
        throw Errors::RBDLSizeMismatchError("Mismatching sizes.\n");
    }

    // Solve the system A*x = b.
    switch (ls) {
        case RigidBodyDynamics::Math::LinearSolverPartialPivLU :
            x = A.partialPivLu().solve(b);
            break;
        case RigidBodyDynamics::Math::LinearSolverColPivHouseholderQR :
            x = A.colPivHouseholderQr().solve(b);
            break;
        case RigidBodyDynamics::Math::LinearSolverHouseholderQR :
            x = A.householderQr().solve(b);
            break;
        default:
            std::ostringstream errormsg;
            errormsg << "Error: Invalid linear solver: " << ls << std::endl;
            throw Errors::RBDLError(errormsg.str());
            break;
    }
}

RBDL_DLLAPI
void InverseDynamicsConstraintsPtr (
  Model &model,
  const double *q_ptr,
  const double *qdot_ptr,
  const double *qddot_ptr,
  ConstraintSet &CS,
  const double *qddot_out_ptr,
  const double *tau_ptr,
  std::vector<Math::SpatialVector> *f_ext
)
{
  LOG << "-------- " << __func__ << " ------" << std::endl;

  using namespace RigidBodyDynamics::Math;

  unsigned int n  = unsigned(    CS.H.rows());
  unsigned int nc = unsigned( CS.name.size());
  unsigned int na = unsigned(    CS.S.rows());
  unsigned int nu = n-na;

  VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  VectorNdRef QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.qdot_size);
  VectorNdRef QDDotDesired = VectorFromPtr(const_cast<double*>(qddot_ptr), model.qdot_size);
  VectorNdRef QDDotOutput = VectorFromPtr(const_cast<double*>(qddot_out_ptr), model.qdot_size);
  VectorNdRef TauOutput = VectorFromPtr(const_cast<double*>(tau_ptr), model.qdot_size);

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
void InverseDynamicsConstraintsRelaxedPtr(
  Model &model,
  const double *q_ptr,
  const double *qdot_ptr,
  const double *qddot_ptr,
  ConstraintSet &CS,
  const double *qddot_out_ptr,
  const double *tau_ptr,
  std::vector<Math::SpatialVector> *f_ext)
{
  LOG << "-------- " << __func__ << " --------" << std::endl;

  using namespace RigidBodyDynamics::Math;
  VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  VectorNdRef QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.qdot_size);
  VectorNdRef QDDotControls = VectorFromPtr(const_cast<double*>(qddot_ptr), model.qdot_size);
  VectorNdRef QDDotOutput = VectorFromPtr(const_cast<double*>(qddot_out_ptr), model.qdot_size);
  VectorNdRef TauOutput = VectorFromPtr(const_cast<double*>(tau_ptr), model.qdot_size);

  TauOutput.setZero();
  CalcConstrainedSystemVariables(model,Q,QDot,TauOutput,CS,f_ext);

  unsigned int n  = unsigned(    CS.H.rows());
  unsigned int nc = unsigned( CS.name.size());
  unsigned int na = unsigned(    CS.S.rows());
  unsigned int nu = n-na;


  //MM: Update to Henning's formulation s.t. the relaxed IDC operator will
  //    more closely satisfy QDDotControls if it is possible.
  double diag = 0.;//100.*CS.H.maxCoeff();
  double diagInv = 0.;
  for(unsigned int i=0; i<CS.H.rows(); ++i) {
    for(unsigned int j=0; j<CS.H.cols(); ++j) {
      if(fabs(CS.H(i,j)) > diag) {
        diag = fabs(CS.H(i,j));
      }
    }
  }
  diag = diag*100.;
  diagInv = 1.0/diag;
  for(unsigned int i=0; i<CS.W.rows(); ++i) {
    CS.W(i,i)    = diag;
    CS.Winv(i,i) = diagInv;
  }

  CS.WinvSC = CS.Winv * CS.S * CS.C;

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



}

RBDL_DLLAPI
bool isConstrainedSystemFullyActuated (
        Model &model,
        const double* q_ptr,
        const double* qdot_ptr,
        ConstraintSet& CS,
        std::vector<Math::SpatialVector> *f_ext){
  LOG << "-------- " << __func__ << " ------" << std::endl;

  using namespace RigidBodyDynamics::Math;

  unsigned int n  = unsigned(    CS.H.rows());
  unsigned int nc = unsigned( CS.name.size());
  unsigned int na = unsigned(    CS.S.rows());
  unsigned int nu = n-na;

  VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  VectorNdRef QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.qdot_size);

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
void NonlinearEffectsPtr (
  Model &model,
  const double *q_ptr,
  const double *qdot_ptr,
  const double *tau_ptr
)
{
  LOG << "-------- " << __func__ << " --------" << std::endl;

  using namespace RigidBodyDynamics::Math;

  VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  VectorNdRef QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.q_size);
  VectorNdRef Tau = VectorFromPtr(const_cast<double*>(tau_ptr), model.q_size);

  SpatialVector spatial_gravity (0., 0., 0., -model.gravity[0], -model.gravity[1],
                                 -model.gravity[2]);

  // Reset the velocity of the root body
  model.v[0].setZero();
  model.a[0] = spatial_gravity;

  for (unsigned int i = 1; i < model.mJointUpdateOrder.size(); i++) {
    jcalc (model, model.mJointUpdateOrder[i], Q, QDot);
  }

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    if (model.lambda[i] == 0) {
      model.v[i] = model.v_J[i];
      model.a[i] = model.X_lambda[i].apply(spatial_gravity);
    }  else {
      model.v[i] = model.X_lambda[i].apply(model.v[model.lambda[i]]) + model.v_J[i];
      model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
      model.a[i] = model.X_lambda[i].apply(model.a[model.lambda[i]]) + model.c[i];
    }

    if (!model.mBodies[i].mIsVirtual) {
      model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i],
                   model.I[i] * model.v[i]);
    } else {
      model.f[i].setZero();
    }
  }

  for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
    if(model.mJoints[i].mJointType != JointTypeCustom) {
      if (model.mJoints[i].mDoFCount == 1) {
        Tau[model.mJoints[i].q_index]
          = model.S[i].dot(model.f[i]);
      } else if (model.mJoints[i].mDoFCount == 3) {
        Tau.block<3,1>(model.mJoints[i].q_index, 0)
          = model.multdof3_S[i].transpose() * model.f[i];
      }
    } else if(model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int k = model.mJoints[i].custom_joint_index;
      Tau.block(model.mJoints[i].q_index,0,
                model.mCustomJoints[k]->mDoFCount, 1)
        = model.mCustomJoints[k]->S.transpose() * model.f[i];
    }

    if (model.lambda[i] != 0) {
      model.f[model.lambda[i]] = model.f[model.lambda[i]] +
                                 model.X_lambda[i].applyTranspose(model.f[i]);
    }
  }
}




RBDL_DLLAPI
inline void CompositeRigidBodyAlgorithmPtr (
  Model& model,
  const double *q_ptr,
  double *H_ptr,
  bool update_kinematics = true
)
{
  using namespace RigidBodyDynamics::Math;

  VectorNdRef&& Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  MatrixNdRef&& H = MatrixFromPtr(H_ptr, model.qdot_size, model.qdot_size);

  assert (H.rows() == model.dof_count && H.cols() == model.dof_count);

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    if (update_kinematics) {
      jcalc_X_lambda_S (model, i, Q);
    }
    model.Ic[i] = model.I[i];
  }


  for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
    if (model.lambda[i] != 0) {
      model.Ic[model.lambda[i]] = model.Ic[model.lambda[i]] +
                                  model.X_lambda[i].applyTranspose(model.Ic[i]);
    }

    unsigned int dof_index_i = model.mJoints[i].q_index;

    if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {

      SpatialVector F             = model.Ic[i] * model.S[i];
      H(dof_index_i, dof_index_i) = model.S[i].dot(F);

      unsigned int j = i;
      unsigned int dof_index_j = dof_index_i;

      while (model.lambda[j] != 0) {
        F = model.X_lambda[j].applyTranspose(F);
        j = model.lambda[j];
        dof_index_j = model.mJoints[j].q_index;

        if(model.mJoints[j].mJointType != JointTypeCustom) {
          if (model.mJoints[j].mDoFCount == 1) {
            H(dof_index_i,dof_index_j) = F.dot(model.S[j]);
            H(dof_index_j,dof_index_i) = H(dof_index_i,dof_index_j);
          } else if (model.mJoints[j].mDoFCount == 3) {
            Vector3d H_temp2 =
              (F.transpose() * model.multdof3_S[j]).transpose();
            LOG << F.transpose() << std::endl
                << model.multdof3_S[j] << std::endl;
            LOG << H_temp2.transpose() << std::endl;

            H.block<1,3>(dof_index_i,dof_index_j) = H_temp2.transpose();
            H.block<3,1>(dof_index_j,dof_index_i) = H_temp2;
          }
        } else if (model.mJoints[j].mJointType == JointTypeCustom) {
          unsigned int k      = model.mJoints[j].custom_joint_index;
          unsigned int dof    = model.mCustomJoints[k]->mDoFCount;
          VectorNd H_temp2    =
            (F.transpose() * model.mCustomJoints[k]->S).transpose();

          LOG << F.transpose()
              << std::endl
              << model.mCustomJoints[j]->S << std::endl;

          LOG << H_temp2.transpose() << std::endl;

          H.block(dof_index_i,dof_index_j,1,dof) = H_temp2.transpose();
          H.block(dof_index_j,dof_index_i,dof,1) = H_temp2;
        }
      }
    } else if (model.mJoints[i].mDoFCount == 3
               && model.mJoints[i].mJointType != JointTypeCustom) {
      Matrix63 F_63 = model.Ic[i].toMatrix() * model.multdof3_S[i];
      H.block<3,3>(dof_index_i, dof_index_i) = model.multdof3_S[i].transpose() * F_63;

      unsigned int j = i;
      unsigned int dof_index_j = dof_index_i;

      while (model.lambda[j] != 0) {
        F_63 = model.X_lambda[j].toMatrixTranspose() * (F_63);
        j = model.lambda[j];
        dof_index_j = model.mJoints[j].q_index;

        if(model.mJoints[j].mJointType != JointTypeCustom) {
          if (model.mJoints[j].mDoFCount == 1) {
            Vector3d H_temp2 = F_63.transpose() * (model.S[j]);

            H.block<3,1>(dof_index_i,dof_index_j) = H_temp2;
            H.block<1,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
          } else if (model.mJoints[j].mDoFCount == 3) {
            Matrix3d H_temp2 = F_63.transpose() * (model.multdof3_S[j]);

            H.block<3,3>(dof_index_i,dof_index_j) = H_temp2;
            H.block<3,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
          }
        } else if (model.mJoints[j].mJointType == JointTypeCustom) {
          unsigned int k = model.mJoints[j].custom_joint_index;
          unsigned int dof = model.mCustomJoints[k]->mDoFCount;

          MatrixNd H_temp2 = F_63.transpose() * (model.mCustomJoints[k]->S);

          H.block(dof_index_i,dof_index_j,3,dof) = H_temp2;
          H.block(dof_index_j,dof_index_i,dof,3) = H_temp2.transpose();
        }
      }
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int kI = model.mJoints[i].custom_joint_index;
      unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;

      MatrixNd F_Nd = model.Ic[i].toMatrix()
                      * model.mCustomJoints[kI]->S;

      H.block(dof_index_i, dof_index_i,dofI,dofI)
        = model.mCustomJoints[kI]->S.transpose() * F_Nd;

      unsigned int j = i;
      unsigned int dof_index_j = dof_index_i;

      while (model.lambda[j] != 0) {
        F_Nd = model.X_lambda[j].toMatrixTranspose() * (F_Nd);
        j = model.lambda[j];
        dof_index_j = model.mJoints[j].q_index;

        if(model.mJoints[j].mJointType != JointTypeCustom) {
          if (model.mJoints[j].mDoFCount == 1) {
            MatrixNd H_temp2 = F_Nd.transpose() * (model.S[j]);
            H.block(   dof_index_i,  dof_index_j,
                       H_temp2.rows(),H_temp2.cols()) = H_temp2;
            H.block(dof_index_j,dof_index_i,
                    H_temp2.cols(),H_temp2.rows()) = H_temp2.transpose();
          } else if (model.mJoints[j].mDoFCount == 3) {
            MatrixNd H_temp2 = F_Nd.transpose() * (model.multdof3_S[j]);
            H.block(dof_index_i,   dof_index_j,
                    H_temp2.rows(),H_temp2.cols()) = H_temp2;
            H.block(dof_index_j,   dof_index_i,
                    H_temp2.cols(),H_temp2.rows()) = H_temp2.transpose();
          }
        } else if (model.mJoints[j].mJointType == JointTypeCustom) {
          unsigned int k   = model.mJoints[j].custom_joint_index;
          unsigned int dof = model.mCustomJoints[k]->mDoFCount;

          MatrixNd H_temp2 = F_Nd.transpose() * (model.mCustomJoints[k]->S);

          H.block(dof_index_i,dof_index_j,3,dof) = H_temp2;
          H.block(dof_index_j,dof_index_i,dof,3) = H_temp2.transpose();
        }
      }
    }
  }
}




RBDL_DLLAPI
void ForwardDynamicsPtr (
  Model &model,
  const double *q_ptr,
  const double *qdot_ptr,
  const double *tau_ptr,
  const double *qddot_ptr,
  std::vector<Math::SpatialVector> *f_ext
)
{
  LOG << "-------- " << __func__ << " --------" << std::endl;

  using namespace RigidBodyDynamics::Math;

  VectorNdRef&& Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  VectorNdRef&& QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.q_size);
  VectorNdRef&& QDDot = VectorFromPtr(const_cast<double*>(qddot_ptr),
                                      model.q_size);
  VectorNdRef&& Tau = VectorFromPtr(const_cast<double*>(tau_ptr), model.q_size);


  SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1],
                                 model.gravity[2]);

  unsigned int i = 0;

  LOG << "Q          = " << Q.transpose() << std::endl;
  LOG << "QDot       = " << QDot.transpose() << std::endl;
  LOG << "Tau        = " << Tau.transpose() << std::endl;
  LOG << "---" << std::endl;

  // Reset the velocity of the root body
  model.v[0].setZero();

  for (i = 1; i < model.mBodies.size(); i++) {

    unsigned int lambda = model.lambda[i];

    jcalc (model, i, Q, QDot);

    if (lambda != 0) {
      model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
    } else {
      model.X_base[i] = model.X_lambda[i];
    }

    model.v[i] = model.X_lambda[i].apply( model.v[lambda]) + model.v_J[i];

    /*
       LOG << "X_J (" << i << "):" << std::endl << X_J << std::endl;
       LOG << "v_J (" << i << "):" << std::endl << v_J << std::endl;
       LOG << "v_lambda" << i << ":" << std::endl << model.v.at(lambda) << std::endl;
       LOG << "X_base (" << i << "):" << std::endl << model.X_base[i] << std::endl;
       LOG << "X_lambda (" << i << "):" << std::endl << model.X_lambda[i] << std::endl;
       LOG << "SpatialVelocity (" << i << "): " << model.v[i] << std::endl;
       */
    model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
    model.I[i].setSpatialMatrix (model.IA[i]);

    model.pA[i] = crossf(model.v[i],model.I[i] * model.v[i]);

    if (f_ext != NULL && (*f_ext)[i] != SpatialVector::Zero()) {
      LOG << "External force (" << i << ") = " << model.X_base[i].toMatrixAdjoint()
          * (*f_ext)[i] << std::endl;
      model.pA[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
    }
  }

  // ClearLogOutput();

  LOG << "--- first loop ---" << std::endl;

  for (i = model.mBodies.size() - 1; i > 0; i--) {
    unsigned int q_index = model.mJoints[i].q_index;

    if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {

      model.U[i] = model.IA[i] * model.S[i];
      model.d[i] = model.S[i].dot(model.U[i]);
      model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);
      //      LOG << "u[" << i << "] = " << model.u[i] << std::endl;

      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        SpatialMatrix Ia =    model.IA[i]
                              - model.U[i]
                              * (model.U[i] / model.d[i]).transpose();

        SpatialVector pa =  model.pA[i]
                            + Ia * model.c[i]
                            + model.U[i] * model.u[i] / model.d[i];

        model.IA[lambda].noalias()
        += model.X_lambda[i].toMatrixTranspose()
           * Ia * model.X_lambda[i].toMatrix();
        model.pA[lambda].noalias()
        += model.X_lambda[i].applyTranspose(pa);

        LOG << "pA[" << lambda << "] = "
            << model.pA[lambda].transpose() << std::endl;
      }
    } else if (model.mJoints[i].mDoFCount == 3
               && model.mJoints[i].mJointType != JointTypeCustom) {
      model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];
      model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose()
                                * model.multdof3_U[i]).inverse().eval();
      VectorNd tau_temp(Tau.block(q_index,0,3,1));
      model.multdof3_u[i] = tau_temp
                            - model.multdof3_S[i].transpose() * model.pA[i];

      // LOG << "multdof3_u[" << i << "] = "
      //                      << model.multdof3_u[i].transpose() << std::endl;
      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        SpatialMatrix Ia = model.IA[i]
                           - model.multdof3_U[i]
                           * model.multdof3_Dinv[i]
                           * model.multdof3_U[i].transpose();
        SpatialVector pa = model.pA[i]
                           + Ia
                           * model.c[i]
                           + model.multdof3_U[i]
                           * model.multdof3_Dinv[i]
                           * model.multdof3_u[i];

        model.IA[lambda].noalias()
        += model.X_lambda[i].toMatrixTranspose()
           * Ia
           * model.X_lambda[i].toMatrix();

        model.pA[lambda].noalias()
        += model.X_lambda[i].applyTranspose(pa);

        LOG << "pA[" << lambda << "] = "
            << model.pA[lambda].transpose()
            << std::endl;
      }
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int kI   = model.mJoints[i].custom_joint_index;
      unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;
      model.mCustomJoints[kI]->U =
        model.IA[i] * model.mCustomJoints[kI]->S;

      std::cout << "IA*S: " << model.IA[i] * model.mCustomJoints[kI]->S << std::endl;
      std::cout << "IA: " << model.IA[i] << std::endl;
      std::cout << "S[i] " << model.mCustomJoints[kI]->S << " U[i] " <<
                model.mCustomJoints[kI]->U << std::endl;
      std::cout << "S*U: " << model.mCustomJoints[kI]->S.transpose()
                * model.mCustomJoints[kI]->U << std::endl;

      model.mCustomJoints[kI]->Dinv
        = (model.mCustomJoints[kI]->S.transpose()
           * model.mCustomJoints[kI]->U).inverse().eval();

      VectorNd tau_temp(Tau.block(q_index,0,dofI,1));
      model.mCustomJoints[kI]->u = tau_temp
                                   - model.mCustomJoints[kI]->S.transpose() * model.pA[i];

      std::cout << "u: " << model.mCustomJoints[kI]->u << std::endl;

      //      LOG << "multdof3_u[" << i << "] = "
      //      << model.multdof3_u[i].transpose() << std::endl;
      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        SpatialMatrix Ia = model.IA[i]
                           - (model.mCustomJoints[kI]->U
                              * model.mCustomJoints[kI]->Dinv
                              * model.mCustomJoints[kI]->U.transpose());
        SpatialVector pa =  model.pA[i]
                            + Ia * model.c[i]
                            + (model.mCustomJoints[kI]->U
                               * model.mCustomJoints[kI]->Dinv
                               * model.mCustomJoints[kI]->u);

        model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose()
                                      * Ia
                                      * model.X_lambda[i].toMatrix();
        model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);

        LOG << "pA[" << lambda << "] = "
            << model.pA[lambda].transpose()
            << std::endl;
      }
    }
  }

  //  ClearLogOutput();

  model.a[0] = spatial_gravity * -1.;

  for (i = 1; i < model.mBodies.size(); i++) {
    unsigned int q_index = model.mJoints[i].q_index;
    unsigned int lambda = model.lambda[i];
    SpatialTransform X_lambda = model.X_lambda[i];

    model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];
    LOG << "a'[" << i << "] = " << model.a[i].transpose() << std::endl;

    if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {
      QDDot[q_index] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
      model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
    } else if (model.mJoints[i].mDoFCount == 3
               && model.mJoints[i].mJointType != JointTypeCustom) {
      Vector3d qdd_temp = model.multdof3_Dinv[i] * (model.multdof3_u[i] -
                          model.multdof3_U[i].transpose() * model.a[i]);
      QDDot[q_index] = qdd_temp[0];
      QDDot[q_index + 1] = qdd_temp[1];
      QDDot[q_index + 2] = qdd_temp[2];
      model.a[i] = model.a[i] + model.multdof3_S[i] * qdd_temp;
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int kI = model.mJoints[i].custom_joint_index;
      unsigned int dofI=model.mCustomJoints[kI]->mDoFCount;

      std::cout << "DINV: " << model.mCustomJoints[kI]->Dinv << std::endl;
      std::cout << "u: " << model.mCustomJoints[kI]->u << std::endl;
      std::cout << "U: " << model.mCustomJoints[kI]->U.transpose() << std::endl;
      std::cout << "a: " << model.a[i] << std::endl;

      VectorNd qdd_temp = model.mCustomJoints[kI]->Dinv
                          * (  model.mCustomJoints[kI]->u
                               - model.mCustomJoints[kI]->U.transpose()
                               * model.a[i]);

      for(int z=0; z<dofI; ++z) {
        QDDot[q_index+z] = qdd_temp[z];
      }

      model.a[i] = model.a[i]
                   + model.mCustomJoints[kI]->S * qdd_temp;
    }
  }

  LOG << "QDDot = " << QDDot.transpose() << std::endl;
}






RBDL_DLLAPI bool InverseKinematicsPtr (
  Model &model,
  const double *qinit_ptr,
  const std::vector<unsigned int>& body_id,
  const std::vector<Math::Vector3d>& body_point,
  const std::vector<Math::Vector3d>& target_pos,
  const double *qres_ptr,
  double step_tol,
  double dlambda,
  unsigned int max_iter)
{

  using namespace RigidBodyDynamics::Math;

//  assert (Qinit.size() == model.q_size);
  assert (body_id.size() == body_point.size());
  assert (body_id.size() == target_pos.size());

//  for (unsigned int k = 0; k < body_id.size(); k++) {
//      assert(model.IsBodyId (body_id[k]));
//  }


  VectorNdRef&& Qinit = VectorFromPtr(const_cast<double*>(qinit_ptr),
                                      model.q_size);
  VectorNdRef&& Qres = VectorFromPtr(const_cast<double*>(qres_ptr), model.q_size);


  MatrixNd J = MatrixNd::Zero(3 * body_id.size(), model.qdot_size);
  VectorNd e = VectorNd::Zero(3 * body_id.size());

  Qres = Qinit;

  for (unsigned int ik_iter = 0; ik_iter < max_iter; ik_iter++) {

    UpdateKinematicsCustomPtr (model, qres_ptr, NULL, NULL);

    for (unsigned int k = 0; k < body_id.size(); k++) {
      MatrixNd G (MatrixNd::Zero(3, model.qdot_size));
      CalcPointJacobian (model, Qres, body_id[k], body_point[k], G, false);
      Vector3d point_base =
        CalcBodyToBaseCoordinates (model, Qres, body_id[k], body_point[k], false);
      LOG << "current_pos = " << point_base.transpose() << std::endl;

      for (unsigned int i = 0; i < 3; i++) {
        for (unsigned int j = 0; j < model.qdot_size; j++) {
          unsigned int row = k * 3 + i;
          LOG << "i = " << i << " j = " << j << " k = " << k << " row = "
              << row << " col = " << j << std::endl;
          J(row, j) = G (i,j);
        }

        e[k * 3 + i] = target_pos[k][i] - point_base[i];
      }
    }

    LOG << "J = " << J << std::endl;
    LOG << "e = " << e.transpose() << std::endl;

    // abort if we are getting "close"
    if (e.norm() < step_tol) {
      LOG << "Reached target close enough after " << ik_iter << " steps" << std::endl;
      return true;
    }

    MatrixNd JJTe_lambda2_I =
      J * J.transpose()
      + dlambda*dlambda * MatrixNd::Identity(e.size(), e.size());

    VectorNd z (body_id.size() * 3);

    bool solve_successful = LinSolveGaussElimPivot (JJTe_lambda2_I, e, z);
    assert (solve_successful);

    LOG << "z = " << z << std::endl;

    VectorNd delta_theta = J.transpose() * z;
    LOG << "change = " << delta_theta << std::endl;

    Qres = Qres + delta_theta;
    LOG << "Qres = " << Qres.transpose() << std::endl;

    if (delta_theta.norm() < step_tol) {

      LOG << "reached convergence after " << ik_iter << " steps" << std::endl;
      return true;
    }

    VectorNd test_1 (z.size());
    VectorNd test_res (z.size());

    test_1.setZero();

    for (unsigned int i = 0; i < z.size(); i++) {
      test_1[i] = 1.;

      VectorNd test_delta = J.transpose() * test_1;

      test_res[i] = test_delta.squaredNorm();

      test_1[i] = 0.;
    }

    LOG << "test_res = " << test_res.transpose() << std::endl;
  }

  return false;
}



RBDL_DLLAPI bool InverseKinematicsCSPtr (
  Model &model,
  const double *qinit_ptr,
  InverseKinematicsConstraintSet &CS,
  const double *qres_ptr
)
{

  using namespace RigidBodyDynamics::Math;


  VectorNdRef&& Qinit = VectorFromPtr(const_cast<double*>(qinit_ptr),
                                      model.q_size);
  VectorNdRef&& Qres = VectorFromPtr(const_cast<double*>(qres_ptr), model.q_size);


  assert (Qinit.size() == model.q_size);
  assert (Qres.size() == Qinit.size());

  CS.J = MatrixNd::Zero(CS.num_constraints, model.qdot_size);
  CS.e = VectorNd::Zero(CS.num_constraints);

  double mass;
  Qres = Qinit;


  for (CS.num_steps = 0; CS.num_steps < CS.max_steps; CS.num_steps++) {
    UpdateKinematicsCustomPtr (model, qres_ptr, NULL, NULL);

    for (unsigned int k = 0; k < CS.constraint_row_index.size(); k++) {
      CS.G = MatrixNd::Zero(6, model.qdot_size);
      CalcPointJacobian6D (model, Qres, CS.body_ids[k], CS.body_points[k], CS.G,
                           false);
      Vector3d point_base = CalcBodyToBaseCoordinates (model, Qres, CS.body_ids[k],
                            CS.body_points[k], false);
      Matrix3d R = CalcBodyWorldOrientation(model, Qres, CS.body_ids[k], false);

      Vector3d angular_velocity = R.transpose()*CalcAngularVelocityfromMatrix(
                                    R*CS.target_orientations[k].transpose());
      //assign offsets and Jacobians
      if (CS.constraint_type[k] ==
          InverseKinematicsConstraintSet::ConstraintTypeFull) {
        for (unsigned int i = 0; i < 3; i++) {
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row + 3] = CS.constraint_weight.at(k)*(CS.target_positions[k][i] -
                          point_base[i]);
          CS.e[row] = CS.constraint_weight.at(k)*angular_velocity[i];
          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row + 3, j) = CS.constraint_weight.at(k)*CS.G (i + 3,j);
            CS.J(row, j) = CS.constraint_weight.at(k)*CS.G (i,j);
          }
        }
      } else if (CS.constraint_type[k] ==
                 InverseKinematicsConstraintSet::ConstraintTypeOrientation) {
        for (unsigned int i = 0; i < 3; i++) {
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row] = CS.constraint_weight.at(k)*angular_velocity[i];

          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row, j) = CS.constraint_weight.at(k)*CS.G (i,j);
          }
        }
      } else if (CS.constraint_type[k] ==
                 InverseKinematicsConstraintSet::ConstraintTypePosition) {
        for (unsigned int i = 0; i < 3; i++) {
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row] = CS.constraint_weight.at(k)*(CS.target_positions[k][i] -
                                                  point_base[i]);
          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row, j) = CS.constraint_weight.at(k)*CS.G (i + 3,j);
          }
        }
      } else if (CS.constraint_type[k] ==
                 InverseKinematicsConstraintSet::ConstraintTypePositionXY) {
        for (unsigned int i = 0; i < 2; i++) {
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row] = CS.constraint_weight.at(k)*(CS.target_positions[k][i] -
                                                  point_base[i]);
          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row, j) = CS.constraint_weight.at(k)*CS.G (i + 3,j);
          }
        }
      } else if (CS.constraint_type[k] ==
                 InverseKinematicsConstraintSet::ConstraintTypePositionZ) {

        unsigned int row = CS.constraint_row_index[k];
        CS.e[row] = CS.constraint_weight.at(k)*(CS.target_positions[k][2] -
                                                point_base[2]);
        for (unsigned int j = 0; j < model.qdot_size; j++) {
          CS.J(row, j) = CS.constraint_weight.at(k)*CS.G (2 + 3,j);
        }

      } else if (CS.constraint_type[k] ==
                 InverseKinematicsConstraintSet::ConstraintTypePositionCoMXY) {
        Utils::CalcCenterOfMass (model, Qres, Qres, NULL, mass, point_base, NULL, NULL,
                                 NULL, NULL, false);
        CalcPointJacobian6D (model, Qres, CS.body_ids[k], point_base, CS.G, false);

        for (unsigned int i = 0; i < 2; i++) {
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row] = CS.constraint_weight.at(k)*(CS.target_positions[k][i] -
                                                  point_base[i]);
          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row, j) = CS.constraint_weight.at(k)*CS.G (i + 3,j);
          }
        }
      } else {
        assert (false && !"Invalid inverse kinematics constraint");
      }
    }

    LOG << "J = " << CS.J << std::endl;
    LOG << "e = " << CS.e.transpose() << std::endl;
    CS.error_norm = CS.e.norm();

    // abort if we are getting "close"
    if (CS.error_norm < CS.step_tol) {
      LOG << "Reached target close enough after " << CS.num_steps << " steps" <<
          std::endl;
      return true;
    }

    //     // "task space" from puppeteer
    //     MatrixNd Ek = MatrixNd::Zero (CS.e.size(), CS.e.size());
    //
    //     for (size_t ei = 0; ei < CS.e.size(); ei ++) {
    // //      Ek(ei,ei) = CS.error_norm * CS.error_norm * 0.5 + CS.lambda;
    //       Ek(ei,ei) = CS.e[ei]*CS.e[ei] * 0.5 + CS.lambda;
    //     }
    //
    //     MatrixNd JJT_Ek_wnI = CS.J * CS.J.transpose() + Ek;
    //
    //     VectorNd delta_theta = CS.J.transpose() * JJT_Ek_wnI.colPivHouseholderQr().solve (CS.e);
    //
    //     LOG << "change = " << delta_theta << std::endl;


    // "joint space" from puppeteer

    double Ek = 0.;

    for (size_t ei = 0; ei < CS.e.size(); ei ++) {
      Ek += CS.e[ei] * CS.e[ei] * 0.5;
    }

    VectorNd ek = CS.J.transpose() * CS.e;
    MatrixNd Wn = MatrixNd::Zero (Qres.size(), Qres.size());

//    assert (ek.size() == QNdres.size());

    for (size_t wi = 0; wi < Qres.size(); wi++) {
      Wn(wi, wi) = ek[wi] * ek[wi] * 0.5 + CS.lambda;
      //      Wn(wi, wi) = Ek + 1.0e-3;
    }

    MatrixNd A = CS.J.transpose() * CS.J + Wn;
    VectorNd delta_theta = A.colPivHouseholderQr().solve(CS.J.transpose() * CS.e);

    Qres = Qres + delta_theta;

    if (delta_theta.norm() < CS.step_tol) {
      LOG << "reached convergence after " << CS.num_steps << " steps" << std::endl;
      return true;
    }
  }

  return false;
}




RBDL_DLLAPI void ForwardDynamicsConstraintsDirectPtr (
  Model &model,
  const double *q_ptr,
  const double *qdot_ptr,
  const double *tau_ptr,
  ConstraintSet &CS,
  const double *qddot_ptr,
  std::vector<Math::SpatialVector> *f_ext
)
{
  LOG << "-------- " << __func__ << " --------" << std::endl;

  using namespace RigidBodyDynamics::Math;

  VectorNdRef&& Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  VectorNdRef&& QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.q_size);
  VectorNdRef&& QDDot = VectorFromPtr(const_cast<double*>(qddot_ptr),
                                      model.q_size);
  VectorNdRef&& Tau = VectorFromPtr(const_cast<double*>(tau_ptr), model.q_size);


  CalcConstrainedSystemVariables (model, Q, QDot, Tau, CS, f_ext);

  SolveConstrainedSystemDirect (CS.H, CS.G, Tau - CS.C, CS.gamma,
                                CS.force, CS.A, CS.b, CS.x, CS.linear_solver);

  // Copy back QDDot
  for (unsigned int i = 0; i < model.dof_count; i++) {
    QDDot[i] = CS.x[i];
  }

  // Copy back contact forces
  for (unsigned int i = 0; i < CS.size(); i++) {
    CS.force[i] = -CS.x[model.dof_count + i];
  }
}



RBDL_DLLAPI void CalcCenterOfMass (
  Model &model,
  const double *q_ptr,
  const double *qdot_ptr,
  const double *qddot_ptr,
  double &mass,
  Math::Vector3d &com,
  Math::Vector3d *com_velocity,
  Math::Vector3d *com_acceleration,
  Math::Vector3d *angular_momentum,
  Math::Vector3d *change_of_angular_momentum,
  bool update_kinematics)
{

  using namespace RigidBodyDynamics::Math;



  VectorNdRef&& q     = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  VectorNdRef&& qdot  = VectorFromPtr(const_cast<double*>(qdot_ptr),
                                      model.q_size);
  VectorNdRef&& qddot = VectorFromPtr(const_cast<double*>(qddot_ptr),
                                      model.q_size);

  // If we want to compute com_acceleration or change of angular momentum
  // we must have qddot provided.
  assert( (com_acceleration == NULL && change_of_angular_momentum == NULL)
          || (qddot_ptr != NULL) );

  if (update_kinematics) {
    UpdateKinematicsCustomPtr (model, q_ptr, qdot_ptr, qddot_ptr);
  }

  for (size_t i = 1; i < model.mBodies.size(); i++) {
    model.Ic[i] = model.I[i];
    model.hc[i] = model.Ic[i].toMatrix() * model.v[i];
    model.hdotc[i] = model.Ic[i] * model.a[i] + crossf(model.v[i],
                     model.Ic[i] * model.v[i]);
  }

  if (qddot_ptr && (com_acceleration || change_of_angular_momentum)) {
    for (size_t i = 1; i < model.mBodies.size(); i++) {
      model.hdotc[i] = model.Ic[i] * model.a[i] + crossf(model.v[i],
                       model.Ic[i] * model.v[i]);
    }
  }

  SpatialRigidBodyInertia Itot (0., Vector3d (0., 0., 0.), Matrix3d::Zero(3,3));
  SpatialVector htot (SpatialVector::Zero(6));
  SpatialVector hdot_tot (SpatialVector::Zero(6));

  for (size_t i = model.mBodies.size() - 1; i > 0; i--) {
    unsigned int lambda = model.lambda[i];

    if (lambda != 0) {
      model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].applyTranspose (
                           model.Ic[i]);
      model.hc[lambda] = model.hc[lambda] + model.X_lambda[i].applyTranspose (
                           model.hc[i]);
    } else {
      Itot = Itot + model.X_lambda[i].applyTranspose (model.Ic[i]);
      htot = htot + model.X_lambda[i].applyTranspose (model.hc[i]);
    }
  }

  if (qddot_ptr && (com_acceleration || change_of_angular_momentum)) {
    for (size_t i = model.mBodies.size() - 1; i > 0; i--) {
      unsigned int lambda = model.lambda[i];

      if (lambda != 0) {
        model.hdotc[lambda] = model.hdotc[lambda] + model.X_lambda[i].applyTranspose (
                                model.hdotc[i]);
      } else {
        hdot_tot = hdot_tot + model.X_lambda[i].applyTranspose (model.hdotc[i]);
      }
    }
  }

  mass = Itot.m;
  com = Itot.h / mass;
  LOG << "mass = " << mass << " com = " << com.transpose() << " htot = " <<
      htot.transpose() << std::endl;

  if (com_velocity) {
    *com_velocity = Vector3d (htot[3] / mass, htot[4] / mass, htot[5] / mass);
  }

  if (angular_momentum) {
    htot = Xtrans (com).applyAdjoint (htot);
    angular_momentum->set (htot[0], htot[1], htot[2]);
  }

  if (com_acceleration) {
    *com_acceleration = Vector3d (hdot_tot[3] / mass, hdot_tot[4] / mass,
                                  hdot_tot[5] / mass);
  }

  if (change_of_angular_momentum) {
    hdot_tot = Xtrans (com).applyAdjoint (hdot_tot);
    change_of_angular_momentum->set (hdot_tot[0], hdot_tot[1], hdot_tot[2]);
  }
}

}

