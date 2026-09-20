# underbrush3: four-vine v90 comparison

Run the existing quad_gazebo launch with `scenario:=underbrush3`. This adds
four transverse vines at (x, z) = (0.60, 0.18), (1.03, 0.12),
(1.03, 0.18), (1.36, 0.15) m, each centered at y=0. It is independent
of UNDERBRUSH_CORD_MODEL. Existing scenarios are unchanged.

Reference: saved v90 run 2026-09-11_17-08-07_v90_widepatch_3d_handoff1s_s42_20260911,
params/env.yaml and manager_based/go2_unitree/scene/vine_spawner.py.

Six cylindrical links: length 0.18 m, radius 0.012 m, mass 0.1 kg each,
correct cylinder inertia. No angular restoring springs. Angular damping is zero on all three rotational axes. The training spawner
authors DriveAPI damping=15 on ordinary SphericalJoint primitives, but
the installed Omni PhysX integration does not support those drives. Copying
15 to Gazebo hinges would therefore introduce additional damping. Initial chain length 1.08 m;
rest anchor separation 1.13 m. The 0.05 m offset corresponds to v90's
suspended-vine configuration. The last link has a world-aligned prismatic
endpoint constraint with displacement range [-0.10, 0.20] relative to its
initial frame, rest target +0.05, stiffness 50 N/m, damping 10 N s/m.
VineAxialSpring applies clip(50*(0.05-q)-10*v, -40, 40) N. Native axial
spring/damping are zero to avoid double counting. The 40 N drive cap is
not a cap on total robot-vine contact force.

## Explicit approximations

This is a comparison fixture, not an exact PhysX reproduction or a calibrated
outdoor vegetation model. Each spherical joint is represented by three
co-located revolute axes: two bends limited separately to +/-35 degrees,
and an unrestricted twist. Separate angular limits differ from a spherical
cone, and local-axis damping differs at finite rotations. The 12 intermediate hinge carriers each have mass 0.0001 kg and isotropic
inertia 0.0005 kg m^2. The endpoint slider remains 0.0001 kg with inertia
1e-7 kg m^2. The hinge inertia is numerical rotational regularization: it
changes rotational dynamics and is not an exact implementation of PhysX joint
armature or a calibrated vegetation property. Contact/friction and constraint-solver behavior
remain engine dependent and have not been force-deflection calibrated.

A runtime fixed detachable joint closes the endpoint constraint after the
Gazebo trees are constructed; it does not implement vine breakage. There is
no duplicated cylinder at the closure. The right endpoint has no extra
universal joint. Collision surfaces use the engine's default material.

Validation: xacro expansion and gz sdf -k passed; plugin built; isolated
CPU-only Gazebo server completed 3000 steps at dt=0.001 without reported
load/physics errors. No robot encounter or Isaac/Gazebo force-response
comparison has yet been performed. This test ran on a separate GZ_PARTITION
without a ROS bridge; ongoing training and user simulation were not controlled.

Damping correction audit: installed omni.physx.tests 107.3.26,
omni/physxtests/tests/PhysicsJoint.py:1052 explicitly notes unsupported
spherical drives and creates a separate D6 drive for its test. The training
vine spawner does not create that additional drive. See also NVIDIA
https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/rigid_bodies_articulations/joints.html
This identifies a configuration/implementation mismatch; it does not by
itself demonstrate successful robot recovery after correction.

## Contact-crash correction (2026-09-13)

The original intermediate-link inertia of 1e-7 kg m^2 reproduced a DART
BoxedLcpConstraintSolver `isSymmetric` assertion under a 10 kg falling-sphere
contact test with the four-vine fixture. Unloaded simulation did not reveal
the failure. Replacing the hinges with universal or ball joints alone also
failed. Increasing only the endpoint-slider mass/inertia also failed.
Intermediate-link inertia of 1e-5 passed the 10 kg case but failed at 20 kg;
5e-4 is used for the contact-tested correction. Geometry, masses, angular
limits, angular damping, axial spring and four-vine placement are unchanged.

Reproducer worlds, variant logs, load-test scripts and pose checks are in
`/home/rml/underbrush/outputs/vine_crash_audit/`. These are isolated contact
tests, not a demonstration of robot recovery or proof against every possible
constraint-solver failure.

Validation of the corrected model: four load cases (1 kg, 10 kg offset,
20 kg central, 10 kg front vine) each completed 30,000 steps at 1 ms with
exit code 0. A separate 30,000-step pose-monitoring run received 12,000
pose messages without nonfinite values. Expanded installed/source model
links, joints and plugins match the tested model; `gz sdf -k` passes.
