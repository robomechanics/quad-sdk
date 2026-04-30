# NMPC Controller

## Overview

This package implements a nonlinear model predictive controller (NMPC) for agile quadrupedal locomotion. It produces ground reaction force (GRF) and body-state plans that track a reference trajectory subject to a given contact schedule. The controller is formulated as a direct-collocation trajectory optimization problem using [CasADi] and solved by [IPOPT] through a custom C++ interface. Multiple dynamics models are supported; the default is a centroidal-dynamics model.

This package is a **library**, not a standalone node. It is linked by `local_planner` and invoked each planning cycle.

### License

The source code is released under a [MIT License](../LICENSE).

**Author:** Yanhao Yang

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

**Maintainers:** Yanhao Yang (yanhaoy@andrew.cmu.edu), Joe Norby (jnorby@andrew.cmu.edu), Jiming Ren (jimingre@andrew.cmu.edu)

Tested under [ROS2] Jazzy on Ubuntu 24.04. This is research code; expect frequent changes and no fitness for any particular purpose.

### Publications

If you use this work in an academic context, please cite:

* J. Norby, Y. Yang, A. Tajbakhsh et al., "Quad-SDK: Full stack software framework for agile quadrupedal locomotion," *ICRA Workshop on Legged Robots*, 2022. ([paper])

```bibtex
@inproceedings{abs:norby-quad-sdk-2022,
  author    = {Norby, Joseph and Yang, Yanhao and Tajbakhsh, Ardalan and Ren, Jiming and Yim, Justin K. and Stutt, Alexandra and Yu, Qishun and Flowers, Nikolai and Johnson, Aaron M.},
  title     = {Quad-{SDK}: Full Stack Software Framework for Agile Quadrupedal Locomotion},
  booktitle = {ICRA Workshop on Legged Robots},
  year      = {2022}
}
```

## Dependencies

* [CasADi] (for symbolic generation and NLP assembly)
* [IPOPT] with Coin-OR HSL (linked from `/usr/local/lib` by default; see `CMakeLists.txt`)
* `quad_utils`, `quad_msgs`, `grid_map_core`, Eigen3

## Build

```bash
colcon build --packages-select nmpc_controller
```

The `src/gen/` directory contains CasADi-generated C code that is compiled into the library.

## Unit Tests

```bash
colcon test --packages-select nmpc_controller
colcon test-result --verbose
```

## Usage

The controller is consumed as a C++ class. Typical use from `local_planner`:

```cpp
#include "nmpc_controller/nmpc_controller.h"

auto nmpc = std::make_shared<NMPCController>(node, system_id);
nmpc->computePlan(current_state, reference_traj, contact_schedule, grf_plan, state_plan);
```

Parameters are loaded from the host node's parameter namespace under `nmpc_controller:`.

## Config

### `config/nmpc_controller.yaml`

**Solver / cost weights:**

* `panic_weights` (double, default: `200.0`) — linear penalty on state slack variables.
* `constraint_panic_weights` (double, default: `20.0`) — linear penalty on constraint slack variables.
* `Q_temporal_factor` (double, default: `100.0`) — temporal scaling of state cost weights.
* `R_temporal_factor` (double, default: `1.0`) — temporal scaling of control cost weights.
* `friction_coefficient` (double, default: `0.3`) — friction cone coefficient for GRFs.

**Horizon / complexity scheduling:**

* `enable_variable_horizon` (bool, default: `false`) — allow the horizon length to shrink as the goal approaches.
* `min_horizon_length` (int, default: `10`) — minimum horizon when variable-horizon is enabled.
* `enable_mixed_complexity` (bool, default: `false`) — use different dynamics models across the horizon.
* `enable_adaptive_complexity` (bool, default: `false`) — auto-select complex elements when a simple model's constraints are violated.
* `fixed_complex_idxs` (int[], default: `[0]`) — fixed complex-element indices (set `[]` for none).
* `fixed_complex_head` (int, default: `0`) — number of complex elements at the head.
* `fixed_complex_tail` (int, default: `0`) — number of complex elements at the tail.

**Per-model dimensions and bounds** (loaded under each model's sub-namespace):

* `x_dim`, `u_dim`, `g_dim` — state, control, and constraint dimensions.
* `x_weights`, `u_weights` — diagonal cost weights on state and control.
* `x_lb`, `x_ub`, `x_lb_soft`, `x_ub_soft` — hard and soft state bounds.
* `u_lb`, `u_ub` — control (GRF) bounds.
* `g_lb`, `g_ub` — general constraint bounds.

See the YAML for model-specific defaults.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[paper]: https://leggedrobots.org/assets/pdfs/paper22.pdf
[ROS2]: https://docs.ros.org/en/jazzy/
[CasADi]: https://web.casadi.org/
[IPOPT]: https://coin-or.github.io/Ipopt/
