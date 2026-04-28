# conflict_based_search

The source code is released under a [MIT License](../LICENSE).

**Author:** David Ologan

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

**Maintainer:** David Ologan (dologan@andrew.cmu.edu)

Tested under [ROS2] Jazzy on Ubuntu 24.04. This is research code; expect frequent changes and no fitness for any particular purpose.

Multi-robot global planner for the ROS 2 Quad-SDK. This package is a port of
the ROS 1 `conflict_based_search` node from the
[`devel_cbs`](https://github.com/robomechanics/quad-sdk/tree/devel_cbs)
branch of `multi-robot-quad-sdk`, with two notable algorithmic improvements:

1. **OBB-based collision checking** in place of the original point-distance
   threshold (in both the high-level conflict detector and the low-level RRT
   validity check).
2. **RRT-Connect tree reuse** across CBS expansions, so each per-robot
   replan warm-starts from the trees built on the previous service call
   instead of running RRT from scratch.

The low-level planner is still RRT-Connect (as implemented by `GBPL`), and the
existing single-robot pipeline is unchanged when no constraints are supplied.

## Architecture

```
┌────────────────────────────────────────────────────────────────────────┐
│                       conflict_based_search_node                       │
│  ┌──────────────┐                                                      │
│  │  Open list   │   priority queue keyed on summed plan cost           │
│  │ (CBS nodes)  │                                                      │
│  └─────┬────────┘                                                      │
│        │                                                               │
│        │ pop best node                                                 │
│        ▼                                                               │
│  ┌──────────────┐    findFirstConflict (OBB-OBB SAT, time-aligned)     │
│  │ Conflict?    │───────no──────► publish plans, done                  │
│  └─────┬────────┘                                                      │
│        │ yes                                                           │
│        ▼                                                               │
│  for each robot involved in conflict:                                  │
│      build new RobotPlanConstraints (forbidden poses + window)         │
│      call /<robot>/plan_with_constraints (warm_start = true)           │
│      push child node onto open list                                    │
└────────────────────────────────────────────────────────────────────────┘
              │ async service call                  ▲
              ▼                                     │
┌─────────────────────────────────────────┐         │
│   /<robot>/global_body_planner          │         │
│   (one per spawned robot)               │         │
│                                         │         │
│   plan_with_constraints service:        │         │
│     ① stash constraints on PlannerConfig│ replan  │
│     ② setWarmStart(req.warm_start)      │ result  │
│     ③ pruneByConstraints on cached trees│         │
│     ④ RRT-Connect resumes from trees    │         │
│     ⑤ return RobotPlan + path_length    │─────────┘
└─────────────────────────────────────────┘
```

## What was ported from ROS 1

| ROS 1 (devel_cbs)                                       | ROS 2 (this branch)                                          |
| ------------------------------------------------------- | ------------------------------------------------------------ |
| `conflict_based_search/conflict_based_search.h/.cpp`    | `include/conflict_based_search/conflict_based_search.hpp` + `src/conflict_based_search.cpp` |
| `conflict_based_search/conflict_based_search_node.cpp`  | `src/conflict_based_search_node.cpp`                         |
| `conflict_based_search/conflict_based_search.yaml`      | `config/conflict_based_search.yaml` (ROS 2 param schema)     |
| `quad_utils/launch/multi_robot.launch`                  | `conflict_based_search/launch/multi_robot.py` (Python launch)|
| `quad_msgs/msg/RobotPlanConflicts.msg` (flat float[])   | `quad_msgs/msg/RobotPlanConstraints.msg` (typed parallel arrays) |
| `global_body_planner/srv/ExampleService.srv`            | `quad_msgs/srv/PlanWithConstraints.srv` (with `warm_start`)  |
| `addServiceCallback(global_body_planner::ExampleService)` (advertised as `/<robot>/add_two_ints`) | `GlobalBodyPlanner::planWithConstraintsCallback` (advertised as `/<robot>/plan_with_constraints`) |
| Catkin/`package.xml` format 2                           | `ament_cmake`/`package.xml` format 3                         |

The high-level CBS algorithm is structurally the same: priority queue ordered
by total path length, expand by branching on the first detected conflict, add
one child per involved robot with a fresh constraint, replan only that
robot's path.

## Improvement 1 — better collision checking

### What was broken in the ROS 1 version

The original `doPlansCollide` used a single Euclidean center-of-mass distance
threshold (`threshold = 0.33` m) between time-aligned states. This had two
practical problems:

1. **Body shape ignored.** Quad-SDK robots are not spheres — Spirit's body is
   roughly 0.6 × 0.3 m. A single-radius check either over-constrains across
   the body width (false positives) or under-constrains along the body
   length (real collisions slip through).
2. **Only one sample per plan-A timestep.** Because plan A and plan B are
   discretised independently, two robots sweeping past each other can each
   hold a non-colliding sample at every aligned time, even when the actual
   trajectory crosses.

### What this branch does

Both layers now use the **separating axis theorem on oriented bounding
boxes** (OBB-OBB SAT) defined by `(body_length, body_width, body_height)`
and yaw extracted from the body pose. The implementation lives in
`planning_utils::obbIntersect` (planner side) and
`ConflictBasedSearch::obbsOverlap` (CBS side). Z-extent is checked as a
1D interval reject before the planar SAT.

In `findFirstConflict` the CBS node also does a **mid-segment swept check**:
between consecutive states `s_k` and `s_{k+1}` of plan A it samples a
midpoint pose and tests OBB overlap there. This catches the
"sweep-past-each-other" failure mode without paying the cost of full
continuous collision detection. If you need full CCD a follow-up could
upgrade this midpoint check to a binary subdivision until the gap is below
a per-second translation threshold.

The planner-side check is wired in through
`PlannerConfig::dynamic_constraints` — a vector of `TimedPoseConstraint`
populated by `planWithConstraintsCallback` from the request message and
cleared again at the end of the call. `isValidState` consults it after the
existing geometric and traversability checks, so single-robot operation is
unaffected when the vector is empty.

## Improvement 2 — RRT-Connect tree reuse

### What was broken in the ROS 1 version

Every `addServiceCallback` call triggered `triggerReset()` followed by
`callPlanner()`, which built two fresh `PlannerClass` trees and ran
RRT-Connect from scratch. With even a handful of CBS expansions this
dominates the wall-clock cost — every constraint added by the high-level
search invalidates *all* prior planning work.

### What this branch does

The persistent state is held by a `GBPL gbpl_` member on `GlobalBodyPlanner`
(was a stack-local). `GBPL` itself owns two `unique_ptr<PlannerClass>` —
`Ta_cache_` and `Tb_cache_` — that survive across `findPlan` calls.

When the service callback sets `gbpl_.setWarmStart(true)`:

1. `findPlan` reuses `Ta_cache_` and `Tb_cache_` instead of constructing new
   trees.
2. Before the RRT-Connect loop runs, both trees call
   `pruneByConstraints(planner_config)`, which:
   - finds every cached vertex whose body OBB overlaps any new constraint, and
   - BFSes through `getSuccessors` to mark the entire descendant subtree as
     invalid (so paths-from-root that cross a forbidden pose are rejected).
3. `getNearestNeighbor`, `neighborhoodN`, and `neighborhoodDist` skip
   vertices in the `invalid_vertices_` set, so RRT-Connect only extends
   from valid parents and only returns paths through valid vertices.
4. The anytime-restart inside `findPlan` is suppressed when warm-starting,
   so the cache is not reset mid-search.

Empirically this means a CBS expansion that adds a single new constraint
typically only costs the time of a few extra `extend → connect` iterations
plus an O(|V|) prune, instead of an entire fresh RRT-Connect solve.

`triggerReset()` invalidates the cache (fresh start/goal pair → previous
trees are no longer admissible). This keeps the original single-robot
behaviour intact: the spin-loop call to `callPlanner` continues to run cold.

## Files changed outside this package

This package depends on a small surface added to the rest of the workspace.
None of the existing single-robot launch paths were modified.

* `quad_msgs/msg/RobotPlanConstraints.msg` — typed time-windowed constraint
  vector, replaces the flat `RobotPlanConflicts` from ROS 1.
* `quad_msgs/srv/PlanWithConstraints.srv` — request/response carrying the
  constraint message, a `warm_start` flag, and the resulting plan.
* `quad_msgs/CMakeLists.txt` — registers the new message and service.
* `global_body_planner/include/global_body_planner/planning_utils.hpp`
  - `TimedPoseConstraint` struct.
  - `dynamic_constraints` vector on `PlannerConfig`.
  - `obbIntersect` and `failsRobotConstraint` declarations.
* `global_body_planner/src/planning_utils.cpp`
  - `isValidState` now checks `dynamic_constraints` via OBB-OBB SAT.
  - `obbIntersect` and `failsRobotConstraint` implementations.
* `global_body_planner/include/global_body_planner/planner_class.hpp`
  - `invalid_vertices_` member.
  - `markVertexInvalid`, `isVertexInvalid`, `resetInvalidVertices`,
    `pruneByConstraints` methods.
* `global_body_planner/src/planner_class.cpp`
  - Neighbor lookups skip invalid vertices, with a graceful fallback when
    every vertex has been pruned.
  - `pruneByConstraints` walks `getSuccessors` so descendants of an invalid
    vertex are also marked.
* `global_body_planner/include/global_body_planner/gbpl.hpp`
  - `setWarmStart`, `invalidateCache` API.
  - `Ta_cache_` / `Tb_cache_` `unique_ptr<PlannerClass>` members.
* `global_body_planner/src/gbpl.cpp`
  - `findPlan` builds or reuses cached trees based on `warm_start_`.
  - The anytime-horizon restart is skipped when warm-starting (so we don't
    discard the cache mid-search).
  - Fixed a latent shadowing bug where `t_start_current_solve` inside the
    restart branch shadowed the outer variable.
* `global_body_planner/include/global_body_planner/global_body_planner.hpp`
  - `planWithConstraintsCallback` declaration.
  - `plan_with_constraints_srv_` member, persistent `GBPL gbpl_` member.
* `global_body_planner/src/global_body_planner.cpp`
  - `plan_with_constraints` service is advertised in the constructor.
  - `triggerReset` invalidates the GBPL cache.
  - `callPlanner` now uses the persistent `gbpl_` instance.
  - `planWithConstraintsCallback` translates the constraint message into
    `TimedPoseConstraint`s, configures warm-start, runs the planner, and
    returns the resulting plan.

## Usage

### End-to-end Gazebo workflow (three terminals)

The CBS node is *not* a replacement for `quad_gazebo.py`. It replaces only
the planning stack — `multi_robot.py` is a thin wrapper around
`quad_plan.py` that adds CBS on top, so the launch arguments are
identical apart from one extra per-robot field (`goal_state`). The
expected order is: launch the simulator with multiple robots, stand each
robot, then launch this package.

The defaults in both launches are now a **six-robot hexagon-swap
scenario** on the `big_flat` world (~22×22 m). Robots spawn at the
vertices of a regular hexagon at radius 7 m and swap with the
diametrically-opposite vertex — every pair of straight-line paths
crosses at the origin, giving CBS fifteen pairwise conflicts to
resolve.

The Gazebo GUI is **off by default** because rendering six robots at
once is heavyweight. RViz is the primary viewer; pass `gui:=true` to
bring the Gazebo GUI back if you need it.

```bash
# Terminal 1 — Gazebo headless (server only) + RViz, six robots in hex.
# All defaults; no robot_configs override needed.
ros2 launch quad_utils quad_gazebo.py
```

Wait until each robot has settled on the ground in RViz, then publish
stand commands once per namespace:

```bash
# Terminal 2 — control mode 1 = READY (stand). Repeat per robot namespace.
for n in 1 2 3 4 5 6; do
  ros2 topic pub --once /robot_${n}/control/mode std_msgs/msg/UInt8 "data: 1"
done
```

Once the robots are standing and their `state/ground_truth` topics are
publishing valid odometry, bring up the planning stack and the CBS
coordinator together. The default `robot_configs` already encodes the
hexagon-swap goals, so no override is needed:

```bash
# Terminal 3 — per-robot global/local planner + central CBS.
ros2 launch conflict_based_search multi_robot.py
```

The default robot/goal pairing is:

| Robot | Start (hex vertex)   | Goal (opposite vertex) |
| ----- | -------------------- | ---------------------- |
| robot_1 | ( 7.00,  0.00)     | (-7.00,  0.00) |
| robot_2 | ( 3.50,  6.06)     | (-3.50, -6.06) |
| robot_3 | (-3.50,  6.06)     | ( 3.50, -6.06) |
| robot_4 | (-7.00,  0.00)     | ( 7.00,  0.00) |
| robot_5 | (-3.50, -6.06)     | ( 3.50,  6.06) |
| robot_6 | ( 3.50, -6.06)     | (-3.50,  6.06) |

For smaller scenarios (two-robot head-to-head, four-corner diagonal
swap, etc.) override `robot_configs:='[ ... ]'` on both launches. The
shape is unchanged from single-robot Quad-SDK except that
`multi_robot.py` requires an additional `goal_state` per entry.

CBS will keep expanding until either a conflict-free set of plans is
found (in which case it publishes one `RobotPlan` per robot on
`/<robot>/global_plan`, where the local NMPC controller picks them up) or
`max_iterations` is hit.

### Things to know before the first run

* **JSON shape differs from quad_gazebo.py but matches quad_plan.py.**
  `quad_gazebo.py` wants `controller` + `init_pose` per robot;
  `multi_robot.py` wants `controller_mode` + `twist_input` +
  `goal_state` (just like `quad_plan.py`, with `goal_state` mandatory
  here). The robot `name` and `type` fields are the same in both.
* **`reference` is always forced to `gbpl`.** CBS calls
  `/<robot>/plan_with_constraints`, which only exists when each robot
  is running its `global_body_planner_node`. If you supply a
  `"reference": "twist"` entry the launch will warn and rewrite it.
* **Stand before launching CBS.** `requestInitialPlans()` blocks on each
  robot's state estimate; if you launch CBS before standing, it sits in
  `waitForServices` until you do.
* **Per-robot goals must differ to actually exercise CBS.** With every
  robot heading to the same XY there is nothing to resolve — CBS will
  publish identical-looking plans on the first iteration. The
  `goal_state` field is what `multi_robot.py` adds on top of
  `quad_plan.py` to make this convenient at the launch layer.
* **Optional MuJoCo back-end.** The same workflow works against
  `quad_mujoco.py` instead of `quad_gazebo.py`; only the simulator
  launch changes.

### Relationship to `quad_plan.py`

`multi_robot.py` is a wrapper. Internally it:

1. Validates `robot_configs` (each entry must have `goal_state`, and
   `reference` is rewritten to `gbpl` if needed).
2. Hands the (validated) JSON to `quad_plan.py` via
   `IncludeLaunchDescription`, which spawns the per-robot
   `global_body_planner` + `local_planner` + `body_force_estimator`
   stacks identically to single-robot operation.
3. Adds one `conflict_based_search_node` that targets every robot named
   in the JSON.

The `goal_state` propagation goes one level deeper: `quad_plan.py`
forwards it to `planning.py`, which (when `reference == 'gbpl'`)
overrides `global_body_planner.goal_state` on the spawned planner node.
Single-robot users of `quad_plan.py` can also use this — pass
`"goal_state": [x, y]` in their JSON to override the yaml default for
that robot. Omitting it preserves the old behaviour.

### Standalone (no simulator)

The `conflict_based_search_node` only needs each robot's
`plan_with_constraints` service to be available. To run only the planner
side without spinning the simulator, launch each robot's
`global_body_planner_node` directly under its namespace and start the
CBS node by hand:

```bash
ros2 run global_body_planner global_body_planner_node \
  --ros-args -r __ns:=/robot_1 \
  --params-file <robot_1.yaml>

ros2 run global_body_planner global_body_planner_node \
  --ros-args -r __ns:=/robot_2 \
  --params-file <robot_2.yaml>

ros2 run conflict_based_search conflict_based_search_node \
  --ros-args --params-file install/conflict_based_search/share/conflict_based_search/config/conflict_based_search.yaml
```

## Parameters

See `config/conflict_based_search.yaml` for the full list, but the ones you
are most likely to tune are:

| Param            | Type      | Meaning                                                               |
| ---------------- | --------- | --------------------------------------------------------------------- |
| `robot_names`    | `string[]`| Namespaces under which each per-robot planner advertises its service. |
| `body_length`    | `double`  | Body length used for the OBB collision check.                         |
| `body_width`     | `double`  | Body width.                                                           |
| `body_height`    | `double`  | Body height.                                                          |
| `warm_start`     | `bool`    | Reuse cached RRT trees on each replan. Set `false` to recover the original ROS 1 cold-start behaviour. |
| `max_iterations` | `int`     | Hard cap on CBS expansions before falling back to the best-known plan. |

## Design rationale (why these two improvements?)

### Why OBB SAT for collision checking

The starting point was a single COM-COM Euclidean distance threshold
(`threshold = 0.33` m) — a sphere approximation. Spirit's body is
~`0.6 × 0.3 × 0.2` m (length × width × height). A sphere is the *worst*
possible primitive for a quadruped:

- If the radius is set to half the body length, the planner refuses to put
  two robots side-by-side even when there is plenty of width clearance —
  big false-positive rate, plans get over-constrained and CBS expansion
  count explodes.
- If the radius is set to half the body width, real collisions slip through
  along the length axis — false-negative rate, CBS converges to plans that
  are not actually conflict-free.

I considered the other primitives in the usual order of cost and accuracy:

| Primitive               | Pros                                                    | Cons                                                                    | Verdict                  |
| ----------------------- | ------------------------------------------------------- | ----------------------------------------------------------------------- | ------------------------ |
| Sphere (current ROS 1)  | Trivial cost, no yaw needed                             | Geometry mismatch for any non-spherical body                            | rejected (the problem)   |
| Capsule / sweep-sphere  | Closed-form, captures length anisotropy                 | Pads corners outward; quadrupeds are flat-bottomed boxes, not pills     | rejected (corners worse) |
| Multi-sphere (n along body) | Same primitive, just N copies                       | Same geometry mismatch, just more checks; jittery at corners            | rejected (no real fix)   |
| **OBB-OBB SAT (chosen)**| Exact for boxes, ~16 dot products in 2D, yaw-aware      | Needs body yaw (already available from velocity direction)              | **chosen**               |
| Convex polygon          | Per-robot custom shape                                  | More math, more code, marginal accuracy gain over OBB for quadrupeds    | overkill                 |
| GJK + EPA               | Works for arbitrary convex shapes                       | Heavy dependency, only wins when shapes are non-trivial                 | overkill                 |
| FCL (full library)      | Industrial-grade, supports CCD                          | Big external dep, too much machinery for 2-4 box-shaped robots          | overkill                 |

OBB SAT is also the right primitive *both* in CBS (where the robots' yaws
come from their plans) and in the planner-side validity check (where yaw
is approximated from the candidate state's planar velocity, matching the
planner's existing convention for the rotation matrix in `isValidState`).
Sharing the same primitive across both layers means CBS's "this conflicts"
verdict matches the planner's "this state is invalid" verdict, which avoids
the failure mode where CBS sees a conflict the planner does not know how to
avoid.

The mid-segment swept check (one extra OBB test at the midpoint between
consecutive plan-A states) was added because the discretized state-by-state
check has a known failure: two robots passing each other at speed can each
hold a non-colliding sample at every time step while the segment between
samples actually crosses. Doing full continuous OBB-OBB collision detection
(rotating + translating boxes) is doable but the math is more involved.
Single-midpoint swept check costs ~50% more compute and catches the
overwhelmingly common case; recursive subdivision can replace it later if
adversarial geometry shows up.

### Why lazy-invalidation tree reuse for RRT-Connect

The starting point: every CBS service call ran `triggerReset()` then a fresh
`callPlanner()`, which rebuilt both RRT trees from scratch. With even a
handful of CBS expansions, the dominant cost was *redoing the same
planning work* — the topology of feasible space hadn't changed, only one
new pose was forbidden.

The user's constraint was "keep the global body planner an RRT-Connect and
don't restructure the code entirely." That ruled out:

| Alternative                                        | Why it was rejected                                                          |
| -------------------------------------------------- | ---------------------------------------------------------------------------- |
| Multi-Constrained-RRT-Connect (MCRRT) as a separate planner | Would require a parallel planner implementation maintained alongside RRT-Connect — explicit "don't restructure" violation. |
| Dynamic RRT (DRRT) with full subtree deletion      | Physically removes invalid vertices, requiring careful index/predecessor bookkeeping in `GraphClass`. Lazy invalidation is algorithmically equivalent with much less code churn. |
| RRT* with explicit cost rewiring                   | CBS only needs feasibility, not optimality. Rewiring slows every call for no benefit here. |
| Plan-level caching keyed on constraint set         | Constraint sets are usually unique per CBS expansion, so the cache hit rate would be ~0. Not worth the bookkeeping. |
| Restart from scratch (status quo)                  | The thing being fixed.                                                       |

Lazy invalidation (mark, skip, propagate to descendants) is the minimal
possible change to RRT-Connect. The graph data structure is identical; only
the *visibility* of vertices to neighbor queries changes. The two pieces
that needed to be right:

1. **Propagation through descendants.** A valid vertex with an invalid
   *ancestor* sits on a path-from-root that crosses a forbidden pose, so
   the path is forbidden. `pruneByConstraints` runs a BFS through
   `getSuccessors` to mark every descendant of any directly-invalid vertex.
   Without this step you would extract paths that thread through the
   constraint.

2. **Skipping in neighbor lookup.** With invalid vertices filtered out of
   `getNearestNeighbor` / `neighborhoodN`, RRT-Connect only ever extends
   from valid parents. Combined with the descendant propagation, the path
   returned by `extractPath` is guaranteed to stay clear of every
   constraint.

A graceful fallback is built into `getNearestNeighbor`: if every cached
vertex has been invalidated (e.g. constraints completely cover the previous
free space), return the unfiltered nearest neighbor so the caller can still
extend rather than crashing on an empty result.

The win in practice: the first CBS call pays the full cold-start cost
(unchanged from before). Each subsequent call typically pays only the
prune (O(|V| × |C|)) plus a handful of `extend → connect` iterations until
a new connection is found, instead of an entire fresh RRT-Connect run.

## Future work — within this package

Grouped roughly in priority order. Items earlier in each group are higher
leverage relative to their effort.

### High-level CBS algorithm

* **Replan only over the conflict window.** Today each child node replans a
  robot's *entire* path from start to goal. If the conflict is at second 5
  of a 10-second plan, half of the work is wasted. Splice the existing
  plan up to the start of the conflict window, plan a detour through it,
  and splice back. The `start_index_` machinery already in
  `GlobalBodyPlanner` can be repurposed for this.
* **Disjoint-set / improved CBS (ICBS, CBS-DS).** Detect cardinal vs
  semi-cardinal conflicts and prefer expanding cardinal ones first — order
  of magnitude smaller search trees on the standard MAPF benchmarks and
  the same idea applies here.
* **Prioritized planning fallback.** When CBS hits `max_iterations`, fall
  back to a fixed priority order (replan robot 2 with robot 1's plan as a
  hard constraint, etc). Suboptimal but always returns *something*.
* **Meta-agents for tightly-coupled robots.** When two robots conflict
  repeatedly at the same locale, plan them jointly in a coupled state
  space rather than expanding constraint pairs forever.
* **Conflict caching across CBS runs.** If the same robot pair conflicts
  in nearly the same place every time the user runs the system, persist
  that knowledge and seed the constraint set on next launch.

### Collision detection

* **Recursive segment subdivision.** Replace the single-midpoint swept check
  with binary subdivision until segment translation is below an
  epsilon — full CCD without the closed-form OBB sweep math.
* **Time-aware planner-side check.** The constraint message already
  carries `t_start, t_end`. Right now `failsRobotConstraint` ignores the
  window. To make it time-aware, propagate the candidate state's
  plan-relative time through `isValidState` (it is computable from the
  parent's `g_value` plus the action duration during `extend`) and only
  reject when the candidate's time falls inside the window. Two robots
  could then occupy the same XY at different times — much less
  over-constrained.
* **Spatial index of constraints.** When the constraint set grows, the
  per-state O(|C|) check becomes the bottleneck. A 2D grid hash or AABB
  tree over constraint poses gets it to O(log |C|) per state.
* **Velocity-direction yaw is unstable near v ≈ 0.** Pick up the body's
  actual orientation from `quad_msgs::msg::RobotState` rather than
  inferring yaw from velocity, at least for the CBS-side check (where the
  state messages already contain orientation).

### Plan reuse

* **Spatial index over tree vertices.** `pruneByConstraints` is O(|V| × |C|)
  and `getNearestNeighbor` is O(|V|). A kd-tree or grid hash over vertex
  positions cuts both to O(log |V|).
* **Detect already-feasible cached plan.** If a service call comes in but
  none of the new constraints invalidate the cached path, skip RRT entirely
  and re-extract the previous path. Track the indices of the most recently
  connected Ta/Tb vertices so the cheap path-validity check is constant
  time.
* **Cache keyed on (start, goal) pair.** Today changing the goal (via
  `triggerReset`) drops the cache; for some scenarios (rotating goal among
  a small set of stations) keying the cache lets the system reuse trees
  across goal changes.
* **Periodic terrain-revalidation.** Long-lived vertices may have been
  validated against a stale grid map. When the terrain message updates,
  invalidate vertices whose validity is height-dependent.

### Service / async architecture

* **Parallel per-robot service calls.** Within a single CBS expansion the
  per-robot replans are independent given their constraint sets. Issue all
  of them concurrently (separate callback group + multi-threaded executor)
  instead of round-robin.
* **Drop the busy-wait in `callPlanWithConstraints`.** The `spin_some` +
  `sleep_for(5ms)` loop is a workaround for ROS 2's
  "can't-spin-from-inside-a-callback" rule. A `MutuallyExclusive` callback
  group for the algorithm and a `Reentrant` group for the clients
  eliminates the workaround.
* **Cancellation.** `run()` has no way to be interrupted. If the user
  changes the goal or kills a robot mid-search, CBS keeps grinding.
* **Streaming / anytime output.** Publish the best-known plan after every
  CBS expansion, not only on convergence — even a partial improvement is
  useful for the consumer.

### API / message design

* `RobotPlanConstraints` uses parallel `float64[]` arrays for ease of port.
  A future cleanup should swap to `geometry_msgs/PoseStamped[]` plus a
  shared body-extents block (or one block per pose, for heterogeneous
  robots).
* Body extents are duplicated between `global_body_planner.yaml`
  (`robot_l/w/h`) and `conflict_based_search.yaml`
  (`body_length/width/height`). Source them from a single per-robot yaml
  the way the rest of `quad_utils/launch/planning.py` does.
* `warm_start` could default-on at the planner level, with the service
  flag only an override.
* Per-robot heterogeneous body sizes are not consumed today. The
  `RobotPlanConstraints.length/width/height` fields are wired all the way
  through; what's missing is having CBS look up each constrained robot's
  *own* extents (from a config or a side-channel topic) instead of using
  one set globally.

### Robustness / failure modes

* If a replan returns the same plan (planner couldn't satisfy the new
  constraint), CBS will expand the same conflict forever. Detect with a
  per-(robot, conflict_signature) seen-before set and either back off,
  increase per-call planning time, or mark the conflict as unresolvable.
* If two goals are physically incompatible (1-wide corridor, opposite
  ends), `max_iterations` triggers silently. Should expose a
  `goal_infeasible` signal upstream so the mission layer can react.
* The CBS time-alignment uses absolute `header.stamp` from each plan's
  RobotState. If the per-robot planner clocks drift or use different
  epochs, time alignment breaks. Should normalize to plan-relative time
  (subtract `global_plan_timestamp` from each state stamp) before
  comparing.
* `dynamic_constraints` lives on `PlannerConfig` as global mutable state.
  If a future refactor ever runs the spin-loop `callPlanner` and the
  service callback concurrently, this races. Either gate with a mutex,
  or pass constraints in as an explicit parameter to `findPlan`.

### Testing (currently zero)

* Unit tests for `obbIntersect` (axis-aligned identical, axis-aligned
  separated, rotated overlapping, rotated edge-touching, height
  separation).
* Unit tests for `pruneByConstraints` correctly propagating to descendants
  (build a small tree, mark one vertex invalid, assert the subtree is also
  marked).
* Integration test: launch two robots whose straight-line paths cross,
  assert the published plans satisfy the CBS conflict-free property.
* Regression test: launch one robot, assert single-robot plans are
  *byte-identical* to the pre-port output (since CBS additions should be
  no-ops with empty constraints).

## Roadmap for autonomous multi-robot deployment

The CBS port is the first piece of multi-robot infrastructure on top of
single-robot Quad-SDK. To go from "two simulated quadrupeds resolve their
plans before motion" to "a fleet of robots executing missions in the
field", the roadmap below is what I would prioritize. Items are grouped
by deployment phase. Items in the **CBS-specific** sections above are not
repeated here.

### Phase 0 — Demo readiness (1–2 weeks)

These are the gaps that today block a clean two-robot Gazebo demo of the
ported code.

* **Integration tests in CI.** A pre-recorded terrain map, two robots with
  crossing goals, success criterion = published plans pass the CBS
  conflict-free check. Without this, every change to the planner is a
  potential silent regression.
* **Time-aware planner-side constraints.** As above, but called out
  separately because demos almost always have robots passing through the
  same XY at different times. Without time awareness CBS over-constrains
  and runs out of iterations on otherwise-easy scenarios.
* **Per-robot body extents wired through the launch.** Today every robot
  is assumed Spirit-sized in CBS. Easy fix; needed for any mixed-fleet
  demo (Spirit + A1 + Go2).
* **Plan-relative time normalization** (see Robustness section) — bug
  waiting to happen the first time the system runs across more than one
  host.
* **Cross-robot controller-spawner serialisation (current state).**
  ros2_control's spawner uses a single process-wide lock to serialise
  controller load / activation calls. The six-robot demo currently
  serialises *within each robot* — `joint_controller` only spawns
  after `joint_state_broadcaster` exits via `OnProcessExit`, plus
  bumped 120 s / 180 s timeouts on each spawner. Worst-case lock
  contention is N (six simultaneous `joint_state_broadcaster` spawners)
  rather than 2N. This is sufficient for six robots in our testing.
  At 8+ robots the per-robot chain may still race; the next
  enhancement is to additionally serialise *across* robots — chain
  each robot's bringup so robot N's `joint_state_broadcaster` only
  spawns after robot N-1's `joint_controller` exits, cutting
  contention to 1 at a time. Adds linear startup latency (~3 s ×
  N) but eliminates the race entirely. Defer until 8-robot scaling
  is needed.

CBS as it stands is one-shot: it solves once at startup and then the
robots execute. Real deployments do not work that way.

* **Continuous replanning.** The CBS loop should re-run when:
  - any robot's pose deviates beyond a threshold from its plan,
  - the shared map updates,
  - a new goal is published,
  - an in-execution conflict is detected by a runtime collision check.
  The existing `GlobalBodyPlanner` already has a replanning loop and
  start-index machinery — extend it to the multi-robot case rather than
  starting from scratch.
* **Plan stitching.** When CBS produces a new plan mid-execution, the
  robot must transition smoothly from the old plan to the new one rather
  than jumping. The local NMPC controller already accepts plan updates;
  just need to be careful about the splice index and continuity of yaw /
  velocity references.
* **Runtime reactive avoidance layer.** CBS gives global guarantees
  *modulo* execution accuracy. A faster local layer (ORCA / RVO /
  control-barrier-functions) running on each robot at controller rate
  protects against transient disturbances and uncoordinated dynamic
  agents (humans). This is layered, not replacing CBS.
* **Decentralization option.** A single CBS server is a single point of
  failure and does not scale past ~5 robots. Alternatives:
  - **Prioritized + broadcast:** every robot has a fixed priority,
    broadcasts its plan, and lower-priority robots treat all higher
    priority plans as constraints. Trivially decentralized,
    suboptimal-but-fast.
  - **Distributed CBS:** known algorithms exist; significant work.
  - **Hybrid:** central CBS for global coordination on the order of
    seconds, local reactive on the order of milliseconds.
* **Fault tolerance.** A robot dropping out (network partition, fall,
  E-stop) must not deadlock the CBS server. Heartbeat, dropout detection,
  remove-from-constraint-set semantics. Conversely, the CBS server dying
  must not deadlock the robots — they should continue with their last
  valid plan and trigger a degraded mode.

### Phase 2 — Shared world model (1–3 months, parallel with Phase 1)

The single-robot pipeline trusts a single `GridMap` published once.
Multi-robot execution needs:

* **Shared map server.** One source of truth for terrain + obstacles, fed
  by every robot's perception. The existing `terrain_map_publisher` is
  the single-robot scaffold; multi-robot needs map merging from
  per-robot grid_map contributions.
* **Inter-robot perception** — robots seeing each other directly via
  AprilTags / visual fiducials / lidar segmentation, fed back as
  constraint updates rather than relying solely on plan-based prediction.
  This catches the case where a robot's *actual* pose has diverged from
  its plan.
* **Multi-robot localization.** Either a shared external frame (mocap,
  RTK GPS, AprilTag world) or distributed multi-robot SLAM. Without a
  consistent frame the CBS constraints are meaningless — every check
  becomes "I think the other robot is at X but actually it's at Y".
* **Dynamic obstacles.** Humans, animals, vehicles. Out of scope for the
  current planner but a real deployment requirement. The same
  constraint-injection machinery used by CBS can carry "dynamic obstacle
  trajectory" constraints from a perception module.

### Phase 3 — Mission management (3–6 months)

* **Goal assignment / task allocation.** Today goals are hardcoded in the
  launch JSON. A real fleet has missions: "patrol these waypoints",
  "deliver from A to B", "cooperatively map this region". Hungarian /
  market-based / auction algorithms over the fleet, with CBS as the
  motion-planning back-end.
* **Temporal dependencies between robots.** "Robot A must complete its
  segment before robot B starts" — temporal CBS or a behaviour-tree layer
  on top of motion planning.
* **Energy / battery management.** Coordinate sit/stand cycles, queue at
  charging stations, refuse missions whose energy budget exceeds
  reserves. The legged platform's stand/sit/safety state machine is
  already in `robot_driver`; just needs fleet-level orchestration.
* **Behaviour trees / state machines.** Mission planning is hierarchical;
  the leaf actions are CBS goals. The `BehaviorTree.CPP` or
  `py_trees_ros` ecosystem already integrates with ROS 2.
* **Human-in-the-loop interface.** Operator dashboard for assigning
  goals, viewing per-robot plan + constraint set + conflict history,
  intervening (pause / resume / override) per-robot.

### Phase 4 — Safety & reliability (3–6 months, parallel with Phase 3)

* **E-stop coordination.** A single E-stop must reach every robot
  reliably and deterministically. DDS QoS settings, dedicated E-stop
  topic with `RELIABLE` + `TRANSIENT_LOCAL`.
* **Safety zones.** Velocity caps in shared regions, "no-go zones" the
  planner cannot cross under any circumstance, geo-fenced operational
  envelope.
* **Watchdogs at every layer.** CBS server, per-robot planner, per-robot
  controller. Stale heartbeat → degraded mode → safe sit + comms-loss
  recovery procedure.
* **Plan validation gate.** Before any plan reaches a robot, an
  independent validator (different code path from the planner) confirms
  the plan satisfies the CBS conflict-free property and the geometric /
  kinodynamic constraints. Catches planner regressions before they reach
  hardware.
* **Replay-debugger.** Every CBS run records: robot states, terrain map,
  goals, every service request/response, every detected conflict, every
  expansion. Offline replay through the same CBS code reproduces the run
  bit-for-bit. Indispensable for diagnosing field failures.

### Phase 5 — Production deployment (6+ months)

* **Containerised robot images.** Per-robot Docker / OCI image with
  pinned versions of every package, deployed via OTA update. Reproducible
  field deployments, atomic rollback on regression.
* **Monitoring stack.** Prometheus / Grafana dashboards: per-robot CBS
  service latency, conflict count, replan frequency, planner solve time,
  battery, network round-trip, perception health.
* **Multi-host launch.** Today everything runs in one ros2 launch
  invocation. A real deployment has the central CBS server on a base
  station, each robot on its own ROS 2 domain or DDS partition,
  coordinated network discovery.
* **DDS QoS tuning.** `BEST_EFFORT` for high-frequency state, `RELIABLE`
  for plans / services / E-stop. `TRANSIENT_LOCAL` for late-joiner
  scenarios. This is the difference between "works on my desk" and
  "works on a 5-robot real-time deployment over wifi".
* **Hardware-in-the-loop test rig.** Real robots talking to a simulated
  central CBS server (and vice versa) so the network and integration
  layer is exercised against the same code that ships.

### Legged-robot–specific extensions

These items are specific to quadrupeds and would slot in across multiple
phases:

* **Foothold-level conflict checking.** CBS today is body-only. Two
  robots can have non-conflicting bodies but step on the same foothold
  at the same time. The local planner already produces a
  `MultiFootPlanDiscrete`; lift that into the CBS conflict detector.
* **Fall recovery in the multi-robot context.** If robot A falls, robot
  B's plan may now run through where A is. Fast invalidation +
  emergency-replan path.
* **Adaptive complexity per-region.** NMPC in dense / contested regions,
  simpler controller in open regions. The
  `nmpc_controller.enable_adaptive_complexity` flag already exists; the
  switching policy needs to be tied to the local conflict density that
  CBS sees.
* **Coordinated transitions.** Two robots taking off / landing
  simultaneously is a worse interaction than steady walking. Treat the
  flight phase of `Action` as a higher-priority constraint or simply
  forbid leaping in shared regions.
* **Heterogeneous fleets.** Quad-SDK already supports Spirit / A1 / Go1
  / Go2 / Spot / Vision60 / B2 single-robot. The multi-robot stack
  should not assume Spirit-sized bodies anywhere — see the Per-robot
  body extents item above.

### Suggested first six things to land

If the question is "what's the highest-ROI sequence given finite
engineering time", I would land them in this order:

1. **Integration test in CI** for the existing two-robot scenario.
   Without it, all of the work below is unsafe to merge.
2. **Time-aware planner-side constraint check.** Removes the dominant
   over-constraint failure mode in CBS.
3. **Plan-relative time normalization** in the CBS conflict detector.
   Removes a real bug that will bite the first multi-host test.
4. **Continuous replanning loop** wrapping the CBS one-shot. This is the
   gateway to actual execution, not just static planning.
5. **Runtime reactive avoidance layer** (ORCA-style). Closes the loop
   on execution-time disturbances that CBS alone cannot handle.
6. **Decentralized priority-based fallback.** Removes the single-server
   bottleneck for the common case of small fleets, and gives the system
   a graceful-degradation mode if the CBS server goes down.
