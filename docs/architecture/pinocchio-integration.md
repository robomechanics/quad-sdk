---
title: Pinocchio Integration
tags:
  - architecture
  - kinematics
  - dynamics
---

# Pinocchio Integration

The kinematics and dynamics layer was rewritten on top of [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) when the stack moved to ROS 2. This page describes the new API (`QuadKD2`), why it was chosen, and how to use it from controllers and planners.

## Why Pinocchio

The ROS 1 stack used [RBDL](https://rbdl.github.io/). RBDL works, but for a hardware-deployment-grade quadruped stack we needed:

- **Active maintenance and a healthy community** — Pinocchio is maintained by the Stack-of-Tasks group and used by quadruped/humanoid teams worldwide
- **Auto-differentiation** — first-class CasADi/CppAD bindings let us reuse derivatives in NMPC
- **Speed** — competitive with hand-rolled solvers for typical quadruped inertia structures
- **URDF-driven model building** — fewer custom YAML mappings to maintain per robot

The RBDL source files are retained in-tree for reference but are not linked by the current build.

## QuadKD vs. QuadKD2

| | `QuadKD` (legacy) | `QuadKD2` (current) |
|---|---|---|
| Backend | RBDL | Pinocchio |
| Model construction | Hard-coded | From URDF |
| Joint ordering | Internal (leg0 abad/hip/knee, …) | URDF order, with mapping helpers |
| Per-loop call | implicit | explicit `updateFromPinocchio(q, [v])` |
| Built into ROS 2 build | no | yes |

If you're porting downstream code, the FK/IK/Jacobian function names follow the same convention (`proximalLinkToDistalLinkFK/IK<CoordFrame>`) so most call sites change by class name only.

## Naming convention

```
QuadKD::<proximal>To<distal>{FK|IK}<CoordFrame>(...)
```

Examples:

| Function | Returns |
|---|---|
| `worldToFootFKWorldFrame(q, leg, p_foot)` | foot position in world frame |
| `legbaseToFootIKLegbaseFrame(p_foot, leg, q_joints)` | joint angles for desired foot location |
| `bodyToFootFKBodyFrame(q, leg, p_foot)` | foot position in body frame |

## Joint ordering

Pinocchio orders joints in the order they appear in the URDF. Quad-SDK's internal convention is `leg0 (abad, hip, knee), leg1, leg2, leg3` (12 entries). When the URDF order differs (it usually does — manufacturer URDFs put body before legs and may name joints arbitrarily), the per-robot YAML provides a mapping:

```yaml
/**:
  ros__parameters:
    leg_0:
      joints:
        abad:
          name: "8"      # joint index in URDF
          sign: 1.0      # direction of positive rotation
          offset: 0.0    # zero-offset in radians
        hip: { name: "9", sign: 1.0, offset: 0.0 }
        knee: { name: "10", sign: 1.0, offset: 0.0 }
    # ... leg_1, leg_2, leg_3
```

`QuadKD2` consumes these mappings and converts between Quad-SDK internal ordering and Pinocchio ordering transparently.

## State management

Pinocchio decouples the **model** (built once from URDF) from the **data** (stores frame placements, FK/Jacobian results, updated each loop):

```cpp
QuadKD2 kd(urdf_path, joint_mapping);

// In the control loop, once per tick:
kd.updateFromPinocchio(q, v);    // or position-only overload

// Then any FK / Jacobian / dynamics calls use cached results:
Eigen::Vector3d p_foot;
kd.worldToFootFKWorldFrame(q, /*leg=*/0, p_foot);
```

!!! warning "Forgetting to update"
    `updateFromPinocchio()` **must** be called before any FK/Jacobian evaluation, or you'll get the previous tick's data. This is the most common porting bug.

## Generalized state assembly

Two helpers convert between body-pose + joint-angles and the generalized configuration `q` (and velocity `v`):

```cpp
Eigen::VectorXd q = kd.assembleQfromBodyAndJoints(body_pose, joint_angles);
Eigen::VectorXd qv = kd.assembleQVFromBodyAndJoints(body_pose, body_twist,
                                                   joint_angles, joint_vels);
```

Use these instead of hand-stacking — they get the floating-base quaternion and ordering right.

## Foot velocity Jacobians

Three flavors depending on the angular-velocity convention you want for the body:

| Function | Body angular velocity expressed in |
|---|---|
| `getJacobianGenCoord(q, leg)` | generalized coordinates |
| `getJacobianWorldAngVel(q, leg)` | world frame |
| `getJacobianBodyAngVel(q, leg)` | body frame |

Each returns the matrix `J` such that `v_foot = J · q̇_func`. Pick based on the controller's body-twist convention.

## Use in NMPC

The `nmpc_controller` builds its CasADi expression graph from the same Pinocchio model via the CasADi bindings, so the dynamics seen by the optimizer match the dynamics seen by the controller exactly. This is the main practical reason we picked Pinocchio over alternatives.

## Migration tips

If you're porting controller code from `QuadKD`:

1. Replace the class name and add `kd.updateFromPinocchio(q, v)` at the top of the control loop.
2. If you assembled `q` by hand, switch to `assembleQfromBodyAndJoints`.
3. Confirm joint sign/offset mapping in the per-robot YAML — manufacturer URDFs are inconsistent here, and a wrong sign manifests as a robot that drifts on stand.
4. Run the unit tests (`colcon test --packages-select quad_utils`) — there are FK/IK round-trip tests that will catch mapping mistakes.

[:octicons-arrow-right-24: Browse the planning packages](../packages/global_body_planner.md)
