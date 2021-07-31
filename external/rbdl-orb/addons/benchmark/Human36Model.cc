#include "Human36Model.h"

#include "rbdl/rbdl.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

enum SegmentName {
  SegmentPelvis = 0,
  SegmentThigh,
  SegmentShank,
  SegmentFoot,
  SegmentMiddleTrunk,
  SegmentUpperTrunk,
  SegmentUpperArm,
  SegmentLowerArm,
  SegmentHand,
  SegmentHead,
  SegmentNameLast
};

double SegmentLengths[SegmentNameLast] = {
  0.1457,
  0.4222,
  0.4403,
  0.1037,
  0.2155,
  0.2421,
  0.2817,
  0.2689,
  0.0862,
  0.2429
};

double SegmentMass[SegmentNameLast] = {
  0.8154,
  10.3368,
  3.1609,
  1.001,
  16.33,
  15.96,
  1.9783,
  1.1826,
  0.4453,
  5.0662
};

double SegmentCOM[SegmentNameLast][3] = {
  { 0., 0.,  0.0891},
  { 0., 0., -0.1729},
  { 0., 0., -0.1963},
  { 0.1254, 0., -0.0516},
  { 0., 0.,  0.1185},
  { 0., 0.,  0.1195},
  { 0., 0., -0.1626},
  { 0., 0., -0.1230},
  { 0., 0., -0.0680},
  { 0., 0.,  1.1214}
};

double SegmentRadiiOfGyration[SegmentNameLast][3] = {
  { 0.0897, 0.0855, 0.0803},
  { 0.1389, 0.0629, 0.1389},
  { 0.1123, 0.0454, 0.1096},
  { 0.0267, 0.0129, 0.0254},
  { 0.0970, 0.1009, 0.0825},
  { 0.1273, 0.1172, 0.0807},
  { 0.0803, 0.0758, 0.0445},
  { 0.0742, 0.0713, 0.0325},
  { 0.0541, 0.0442, 0.0346},
  { 0.0736, 0.0634, 0.0765}
};

Body create_body (SegmentName segment) {
  Matrix3d inertia_C (Matrix3d::Zero());
  inertia_C(0,0) = pow(SegmentRadiiOfGyration[segment][0] * SegmentLengths[segment], 2) * SegmentMass[segment];
  inertia_C(1,1) = pow(SegmentRadiiOfGyration[segment][1] * SegmentLengths[segment], 2) * SegmentMass[segment];
  inertia_C(2,2) = pow(SegmentRadiiOfGyration[segment][2] * SegmentLengths[segment], 2) * SegmentMass[segment];

  return RigidBodyDynamics::Body (
      SegmentMass[segment],
      RigidBodyDynamics::Math::Vector3d (
        SegmentCOM[segment][0],
        SegmentCOM[segment][1],
        SegmentCOM[segment][2]
        ),
      inertia_C);
}

void generate_human36model (RigidBodyDynamics::Model *model) {
  Body pelvis_body = create_body (SegmentPelvis);
  Body thigh_body = create_body (SegmentThigh);
  Body shank_body = create_body (SegmentShank);
  Body foot_body = create_body (SegmentFoot);
  Body middle_trunk_body = create_body (SegmentMiddleTrunk);
  Body upper_trunk_body = create_body (SegmentUpperTrunk);
  Body upperarm_body = create_body (SegmentUpperArm);
  Body lowerarm_body = create_body (SegmentLowerArm);
  Body hand_body = create_body (SegmentHand);
  Body head_body = create_body (SegmentHead);

  Joint free_flyer (
      SpatialVector (0., 0., 0., 1., 0., 0.),
      SpatialVector (0., 0., 0., 0., 1., 0.),
      SpatialVector (0., 0., 0., 0., 0., 1.),
      SpatialVector (0., 1., 0., 0., 0., 0.),
      SpatialVector (0., 0., 1., 0., 0., 0.),
      SpatialVector (1., 0., 0., 0., 0., 0.)
      );

  Joint rot_yxz (
      SpatialVector (0., 1., 0., 0., 0., 0.),
      SpatialVector (1., 0., 0., 0., 0., 0.),
      SpatialVector (0., 0., 1., 0., 0., 0.)
      );

  Joint rot_yz (
      SpatialVector (0., 1., 0., 0., 0., 0.),
      SpatialVector (0., 0., 1., 0., 0., 0.)
      );

  Joint rot_y (
      SpatialVector (0., 1., 0., 0., 0., 0.)
      );

  Joint fixed (JointTypeFixed);

  model->gravity = Vector3d (0., 0., -9.81);

  unsigned int pelvis_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), free_flyer, pelvis_body, "pelvis");

  // right leg
  model->AddBody (pelvis_id, Xtrans(Vector3d(0., -0.0872, 0.)), rot_yxz, thigh_body, "thigh_r");
  model->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentThigh])), rot_y, shank_body, "shank_r");
  model->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentShank])), rot_yz, foot_body, "foot_r");

  // left leg
  model->AddBody (pelvis_id, Xtrans(Vector3d(0., 0.0872, 0.)), rot_yxz, thigh_body, "thigh_l");
  model->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentThigh])), rot_y, shank_body, "shank_l");
  model->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentShank])), rot_yz, foot_body, "foot_l");

  // trunk
  model->AddBody (pelvis_id, Xtrans(Vector3d(0., 0., SegmentLengths[SegmentPelvis])), rot_yxz, middle_trunk_body, "middletrunk");
  unsigned int uppertrunk_id = model->AppendBody (Xtrans(Vector3d(0., 0., SegmentLengths[SegmentMiddleTrunk])), fixed, upper_trunk_body, "uppertrunk");

  // right arm
  model->AddBody (uppertrunk_id, Xtrans(Vector3d(0., -0.1900, SegmentLengths[SegmentUpperTrunk])), rot_yxz, upperarm_body, "upperarm_r");
  model->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentUpperArm])), rot_y, lowerarm_body, "lowerarm_r");
  model->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentLowerArm])), rot_yz, hand_body, "hand_r");

  // left arm
  model->AddBody (uppertrunk_id, Xtrans(Vector3d(0.,  0.1900, SegmentLengths[SegmentUpperTrunk])), rot_yxz, upperarm_body, "upperarm_l");
  model->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentUpperArm])), rot_y, lowerarm_body, "lowerarm_l");
  model->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentLowerArm])), rot_yz, hand_body, "hand_l");

  // head	
  model->AddBody (uppertrunk_id, Xtrans(Vector3d(0., 0.1900, SegmentLengths[SegmentUpperTrunk])), rot_yxz, upperarm_body, "head");
}
