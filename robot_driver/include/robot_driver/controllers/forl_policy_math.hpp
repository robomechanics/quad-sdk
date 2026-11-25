#ifndef FORL_POLICY_MATH_H
#define FORL_POLICY_MATH_H

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <array>

// Conventions of FORL (forlax) Go2 joystick policies, trained in MuJoCo MJX on
// the Go2 model of mujoco_menagerie, and their mapping to Quad-SDK. Pure
// functions with no ROS dependency so they can be unit tested.
//
// Policy joint order (MuJoCo model order): FR, FL, RR, RL, each
//   (hip abduction, thigh, calf).
// Quad-SDK joint order: FL, RL, FR, RR, each (abad, hip, knee).
// Joint angle zero and sign are the Unitree URDF's in both.
//
// Observation (45): body angular velocity (3, rad/s), projected gravity (3,
//   unit vector, (0, 0, -1) when upright), joint position minus default pose
//   (12, rad), joint velocity (12, rad/s), previous action (12), velocity
//   command (3: vx, vy, yaw rate). No scaling.
// Action (12): in [-1, 1]; joint target = default pose + action_scale * action.
namespace forl {

constexpr int kNumJoints = 12;
constexpr int kObsDim = 45;

using Vector12d = Eigen::Matrix<double, kNumJoints, 1>;
using ObsVector = Eigen::Matrix<float, kObsDim, 1>;

/// Quad-SDK leg index of each policy leg (FR, FL, RR, RL).
constexpr std::array<int, 4> kQuadLegOfPolicyLeg = {2, 0, 3, 1};

/// Reorder a Quad-SDK joint vector into the policy's joint order.
inline Vector12d quadToPolicyOrder(const Vector12d& quad) {
  Vector12d policy;
  for (int leg = 0; leg < 4; ++leg) {
    for (int j = 0; j < 3; ++j) {
      policy(3 * leg + j) = quad(3 * kQuadLegOfPolicyLeg[leg] + j);
    }
  }
  return policy;
}

/// Reorder a policy joint vector into Quad-SDK's joint order.
inline Vector12d policyToQuadOrder(const Vector12d& policy) {
  Vector12d quad;
  for (int leg = 0; leg < 4; ++leg) {
    for (int j = 0; j < 3; ++j) {
      quad(3 * kQuadLegOfPolicyLeg[leg] + j) = policy(3 * leg + j);
    }
  }
  return quad;
}

/// Default joint pose in policy order from one leg's (abad, hip, knee).
inline Vector12d defaultPose(const Eigen::Vector3d& leg_angles) {
  Vector12d pose;
  for (int leg = 0; leg < 4; ++leg) {
    pose.segment<3>(3 * leg) = leg_angles;
  }
  return pose;
}

/// World gravity direction expressed in the body frame.
inline Eigen::Vector3d projectedGravity(const Eigen::Quaterniond& q_world_body) {
  return q_world_body.normalized().conjugate() * Eigen::Vector3d(0.0, 0.0, -1.0);
}

/// Clip a (vx, vy, yaw rate) command to the symmetric trained range.
inline Eigen::Vector3d clipCommand(const Eigen::Vector3d& cmd,
                                   const Eigen::Vector3d& limits) {
  return cmd.cwiseMin(limits).cwiseMax(-limits);
}

/// Assemble the policy observation. Joint vectors are in Quad-SDK order;
/// prev_action and default_pose are in policy order.
inline ObsVector assembleObservation(const Eigen::Vector3d& body_ang_vel,
                                     const Eigen::Vector3d& projected_gravity,
                                     const Vector12d& joint_pos_quad,
                                     const Vector12d& joint_vel_quad,
                                     const Vector12d& prev_action,
                                     const Eigen::Vector3d& command,
                                     const Vector12d& default_pose) {
  Eigen::Matrix<double, kObsDim, 1> obs;
  obs << body_ang_vel, projected_gravity,
      quadToPolicyOrder(joint_pos_quad) - default_pose,
      quadToPolicyOrder(joint_vel_quad), prev_action, command;
  return obs.cast<float>();
}

/// Joint position targets in Quad-SDK order from a policy action. The action
/// is clipped to [-1, 1], the range the policy's tanh output covers.
inline Vector12d actionToTargets(const Vector12d& action,
                                 const Vector12d& default_pose,
                                 double action_scale) {
  const Vector12d clipped = action.cwiseMin(1.0).cwiseMax(-1.0);
  return policyToQuadOrder(default_pose + action_scale * clipped);
}

}  // namespace forl

#endif  // FORL_POLICY_MATH_H
