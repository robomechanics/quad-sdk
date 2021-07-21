#include <iostream>

#include "Fixtures.h"
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

const double TEST_PREC = 1.0e-12;

TEST_CASE_METHOD (FloatingBase12DoF,
                  __FILE__"_TestSparseFactorizationLTL", "") {
  for (unsigned int i = 0; i < model->q_size; i++) {
    Q[i] = static_cast<double> (i + 1) * 0.1;
  }

  MatrixNd H (MatrixNd::Zero (model->qdot_size, model->qdot_size));

  CompositeRigidBodyAlgorithm (*model, Q, H);

  MatrixNd L (H);
  SparseFactorizeLTL (*model, L);
  MatrixNd LTL = L.transpose() * L;

  CHECK_THAT (H,
              AllCloseMatrix(LTL, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE_METHOD (FloatingBase12DoF,
                  __FILE__"_TestSparseSolveLx", "") {
  for (unsigned int i = 0; i < model->q_size; i++) {
    Q[i] = static_cast<double> (i + 1) * 0.1;
  }

  MatrixNd H (MatrixNd::Zero (model->qdot_size, model->qdot_size));

  CompositeRigidBodyAlgorithm (*model, Q, H);

  MatrixNd L (H);
  SparseFactorizeLTL (*model, L);
  VectorNd x = L * Q; 

  SparseSolveLx (*model, L, x);

  CHECK_THAT (Q, AllCloseVector(x, TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD (FloatingBase12DoF,
                  __FILE__"_TestSparseSolveLTx", "") {
  for (unsigned int i = 0; i < model->q_size; i++) {
    Q[i] = static_cast<double> (i + 1) * 0.1;
  }

  MatrixNd H (MatrixNd::Zero (model->qdot_size, model->qdot_size));

  CompositeRigidBodyAlgorithm (*model, Q, H);

  MatrixNd L (H);
  SparseFactorizeLTL (*model, L);
  VectorNd x = L.transpose() * Q; 

  SparseSolveLTx (*model, L, x);

  CHECK_THAT (Q, AllCloseVector(x, TEST_PREC, TEST_PREC));
}

TEST_CASE_METHOD (FixedBase6DoF12DoFFloatingBase,
                  __FILE__"_ForwardDynamicsContactsSparse", "") {
  ConstraintSet constraint_set_var1;

  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (1., 0., 0.));
  constraint_set.AddContactConstraint (contact_body_id, contact_point,
                                       Vector3d (0., 1., 0.));
  constraint_set.AddContactConstraint (child_2_id, contact_point,
                                       Vector3d (0., 1., 0.));
  
  constraint_set_var1 = constraint_set.Copy();
  constraint_set_var1.Bind (*model);
  constraint_set.Bind (*model);

  VectorNd QDDot_var1 = VectorNd::Constant (model->dof_count, 0.);

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
  
  ClearLogOutput();
  ForwardDynamicsConstraintsRangeSpaceSparse (*model, Q, QDot, Tau,
                                              constraint_set_var1, QDDot_var1);

  CHECK_THAT (QDDot, AllCloseVector(QDDot_var1, TEST_PREC, TEST_PREC));
}

TEST_CASE ( __FILE__"_TestSparseFactorizationMultiDof", "") {
  Model model_emulated;
  Model model_3dof;

  Body body (1., Vector3d (1., 2., 1.),
             Matrix3d (1., 0., 0, 0., 1., 0., 0., 0., 1.));
  Joint joint_emulated (
      SpatialVector (0., 1., 0., 0., 0., 0.),
      SpatialVector (1., 0., 0., 0., 0., 0.),
      SpatialVector (0., 0., 1., 0., 0., 0.)
      );
  Joint joint_3dof (JointTypeEulerYXZ);

  Joint joint_rot_y (
      SpatialVector (0., 1., 0., 0., 0., 0.)
      );

  model_emulated.AppendBody (SpatialTransform (Matrix3d::Identity(),
                                               Vector3d::Zero()),
                             joint_rot_y, body);
  unsigned int multdof_body_id_emulated = model_emulated.AppendBody (
    SpatialTransform (Matrix3d::Identity(), Vector3d::Zero()), joint_emulated,
    body);
  model_emulated.AppendBody (SpatialTransform (Matrix3d::Identity(),
                                               Vector3d::Zero()),
                             joint_emulated, body);
  model_emulated.AddBody (multdof_body_id_emulated, SpatialTransform (
                            Matrix3d::Identity(), Vector3d::Zero()),
                          joint_rot_y, body);
  model_emulated.AppendBody (SpatialTransform (Matrix3d::Identity(),
                                               Vector3d::Zero()),
                             joint_emulated, body);

  model_3dof.AppendBody (SpatialTransform (Matrix3d::Identity(),
                                           Vector3d::Zero()), joint_rot_y, body);
  unsigned int multdof_body_id_3dof = model_3dof.AppendBody (
    SpatialTransform (Matrix3d::Identity(), Vector3d::Zero()), joint_3dof, body);
  model_3dof.AppendBody (SpatialTransform (Matrix3d::Identity(),
                                           Vector3d::Zero()), joint_3dof, body);
  model_3dof.AddBody (multdof_body_id_3dof, SpatialTransform (
                        Matrix3d::Identity(), Vector3d::Zero()), joint_rot_y,
                      body);
  model_3dof.AppendBody (SpatialTransform (Matrix3d::Identity(),
                                           Vector3d::Zero()), joint_3dof, body);

  VectorNd q (VectorNd::Zero (model_emulated.q_size));
  VectorNd qdot (VectorNd::Zero (model_emulated.qdot_size));
  VectorNd qddot_emulated (VectorNd::Zero (model_emulated.qdot_size));
  VectorNd qddot_3dof (VectorNd::Zero (model_emulated.qdot_size));
  VectorNd tau (VectorNd::Zero (model_emulated.qdot_size));

  for (int i = 0; i < q.size(); i++) {
    q[i] = 1.1 * (static_cast<double>(i + 1));
    qdot[i] = 0.55* (static_cast<double>(i + 1));
    qddot_emulated[i] = 0.23 * (static_cast<double>(i + 1));
    qddot_3dof[i] = 0.22 * (static_cast<double>(i + 1));
    tau[i] = 2.1 * (static_cast<double>(i + 1));
  }

  MatrixNd H_emulated (MatrixNd::Zero (q.size(), q.size()));
  MatrixNd H_3dof (MatrixNd::Zero (q.size(), q.size()));

  CompositeRigidBodyAlgorithm (model_emulated, q, H_emulated);
  CompositeRigidBodyAlgorithm (model_3dof, q, H_3dof);

  VectorNd b (VectorNd::Zero (q.size()));
  VectorNd x_emulated (VectorNd::Zero (q.size()));
  VectorNd x_3dof (VectorNd::Zero (q.size()));

  for (unsigned int i = 0; i < b.size(); i++) {
    b[i] = static_cast<double> (i + 1) * 2.152;
  }
  b = H_emulated * b;

  SparseFactorizeLTL (model_emulated, H_emulated);
  SparseFactorizeLTL (model_3dof, H_3dof);

  CHECK_THAT (H_emulated,
              AllCloseMatrix(H_3dof, TEST_PREC, TEST_PREC)
  );

  x_emulated = b;
  SparseSolveLx (model_emulated, H_emulated, x_emulated); 
  x_3dof = b;
  SparseSolveLx (model_3dof, H_3dof, x_3dof); 

  CHECK_THAT (x_emulated,
              AllCloseVector(x_3dof, 1.0e-9, 1.0e-9)
  );

  x_emulated = b;
  SparseSolveLTx (model_emulated, H_emulated, x_emulated);  
  x_3dof = b;
  SparseSolveLTx (model_3dof, H_3dof, x_3dof);  

  CHECK_THAT (x_emulated,
              AllCloseVector(x_3dof, 1.0e-9, 1.0e-9)
  );
}

TEST_CASE ( __FILE__"_TestSparseFactorizationMultiDofAndFixed", "") {
  Model model_emulated;
  Model model_3dof;

  Body body (1., Vector3d (1., 2., 1.),
             Matrix3d (1., 0., 0, 0., 1., 0., 0., 0., 1.));
  Joint joint_emulated (
      SpatialVector (0., 1., 0., 0., 0., 0.),
      SpatialVector (1., 0., 0., 0., 0., 0.),
      SpatialVector (0., 0., 1., 0., 0., 0.)
      );
  Joint joint_3dof (JointTypeEulerYXZ);

  Joint joint_rot_y (
      SpatialVector (0., 1., 0., 0., 0., 0.)
      );

  SpatialTransform translate_x (Matrix3d::Identity(), Vector3d (1., 0., 0.));

  model_emulated.AppendBody (SpatialTransform(Matrix3d::Identity(),
                                              Vector3d::Zero()), joint_rot_y,
                             body);
  unsigned int multdof_body_id_emulated = model_emulated.AppendBody (
    translate_x, joint_emulated, body);
  model_emulated.AppendBody (translate_x, joint_emulated, body);
  model_emulated.AddBody(multdof_body_id_emulated, translate_x,
                         Joint(JointTypeFixed), body);
  model_emulated.AppendBody (translate_x, joint_emulated, body);

  model_3dof.AppendBody (SpatialTransform(Matrix3d::Identity(),
                                          Vector3d::Zero()), joint_rot_y, body);
  unsigned int multdof_body_id_3dof = model_3dof.AppendBody (translate_x,
                                                             joint_3dof, body);
  model_3dof.AppendBody (translate_x, joint_3dof, body);
  model_3dof.AddBody (multdof_body_id_3dof, translate_x, Joint(JointTypeFixed),
                      body);
  model_3dof.AppendBody (translate_x, joint_3dof, body);

  VectorNd q (VectorNd::Zero (model_emulated.q_size));
  VectorNd qdot (VectorNd::Zero (model_emulated.qdot_size));
  VectorNd qddot_emulated (VectorNd::Zero (model_emulated.qdot_size));
  VectorNd qddot_3dof (VectorNd::Zero (model_emulated.qdot_size));
  VectorNd tau (VectorNd::Zero (model_emulated.qdot_size));

  for (int i = 0; i < q.size(); i++) {
    q[i] = 1.1 * (static_cast<double>(i + 1));
    qdot[i] = 0.55* (static_cast<double>(i + 1));
    qddot_emulated[i] = 0.23 * (static_cast<double>(i + 1));
    qddot_3dof[i] = 0.22 * (static_cast<double>(i + 1));
    tau[i] = 2.1 * (static_cast<double>(i + 1));
  }

  MatrixNd H_emulated (MatrixNd::Zero (q.size(), q.size()));
  MatrixNd H_3dof (MatrixNd::Zero (q.size(), q.size()));

  CompositeRigidBodyAlgorithm (model_emulated, q, H_emulated);
  CompositeRigidBodyAlgorithm (model_3dof, q, H_3dof);

  VectorNd b (VectorNd::Zero (q.size()));
  VectorNd x_emulated (VectorNd::Zero (q.size()));
  VectorNd x_3dof (VectorNd::Zero (q.size()));

  for (unsigned int i = 0; i < b.size(); i++) {
    b[i] = static_cast<double> (i + 1) * 2.152;
  }
  b = H_emulated * b;

  SparseFactorizeLTL (model_emulated, H_emulated);
  SparseFactorizeLTL (model_3dof, H_3dof);

  CHECK_THAT (H_emulated,
              AllCloseMatrix(H_3dof, TEST_PREC, TEST_PREC)
  );

  x_emulated = b;
  SparseSolveLx (model_emulated, H_emulated, x_emulated); 
  x_3dof = b;
  SparseSolveLx (model_3dof, H_3dof, x_3dof); 

  CHECK_THAT (x_emulated,
              AllCloseVector(x_3dof, 1.0e-9, 1.0e-9)
  );

  x_emulated = b;
  SparseSolveLTx (model_emulated, H_emulated, x_emulated);  
  x_3dof = b;
  SparseSolveLTx (model_3dof, H_3dof, x_3dof);  

  CHECK_THAT (x_emulated,
              AllCloseVector(x_3dof, 1.0e-9, 1.0e-9)
  );
}
