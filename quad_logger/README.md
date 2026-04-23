# Quad Logger

## Overview

This package contains data-logging and post-processing utilities for Quad-SDK runs: rosbag recording helpers, Python parsers for `quad_msgs` payloads, and plotting scripts for state, control, and EKF comparison. MATLAB-based plotting scripts from prior iterations are preserved alongside the Python ones.

### License

The source code is released under a [MIT License](../LICENSE).

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

Tested under [ROS2] Jazzy on Ubuntu 24.04.

## Build

```bash
colcon build --packages-select quad_logger
```

## Usage

Logging is typically invoked via the bundled launch file, which starts a `ros2 bag record` process with the standard Quad-SDK topic set:

```bash
ros2 launch quad_utils logging.py
```

Bag files are written to the current working directory with a timestamped prefix.

## Scripts

### Python (ROS2)

Run these after sourcing the workspace. All Python parsers read MCAP-format ROS2 bags.

* **`scripts/save_log.py`** — wraps `ros2 bag record` with Quad-SDK's topic preset and a timestamped filename.
* **`scripts/parse_quad_bag.py`** — reads a bag and returns a Python dict keyed by topic with deserialized message sequences.
* **`scripts/process_log.py`** — end-to-end helper that parses a bag and produces the default set of plots.
* **`scripts/plot_state.py`** — plots body pose/twist, joint states, and foot states from `state/ground_truth` or `state/estimate`.
* **`scripts/plot_control.py`** — plots motor commands and ground reaction forces.
* **`scripts/plot_local_plan.py`** — plots NMPC local plan trajectories vs. realized state.
* **`scripts/plot_ekf_comparison.py`** — overlays EKF estimate against ground truth for state-estimation evaluation.

Example:

```bash
python3 src/quad-sdk/quad_logger/scripts/process_log.py /path/to/bag_dir
```

### MATLAB

The MATLAB scripts (`parseQuadBag.m`, `plotState.m`, `plotControl.m`, `animateData.m`, etc.) are retained for users with existing MATLAB analysis pipelines. They predate the ROS2 migration and currently read ROS1 `.bag` files; use the Python equivalents for ROS2 MCAP bags.

### Legacy (`scripts/ros1/`)

Contains ROS1-era parsers (`bag_reader.py`, `read_bag.py`, `mouse_interface.py`) kept for reference. These will not work with ROS2 bags without modification.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
