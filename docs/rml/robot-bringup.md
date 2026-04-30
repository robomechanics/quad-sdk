---
title: Robot Bringup
password: R0b0mech
tags:
  - rml
  - hardware
  - bringup
---

# :material-robot-industrial: Robot Bringup

End-to-end procedure for taking an RML Unitree from "powered off" to "tracking commanded velocity" with the full Quad-SDK planning stack. Allow ~10 minutes the first time per session.

## Compile

### On the robot (arm64)

NatNet (the mocap driver dependency) is x86-only. Skip it when building on the robot:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip mocap4r2_optitrack_driver
```

### On the remote laptop (x86_64)

Build everything:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Docker container (MPC / Underbrush)

Build the image once:

```bash
cd ~/ros2_ws/src/quad-sdk/.devcontainer
docker build -t quad-sdk:latest .
```

Run a fresh container:

```bash
cd ~/ros2_ws/src/quad-sdk/.devcontainer
./run.sh quad-sdk quad-sdk:latest
```

Restart a persistent container later:

```bash
docker start -i quad-sdk
```

For VS Code, open the repo and use **Reopen in Container** from the command palette — the `.devcontainer` config is in-tree.

## SSH

```bash
ssh unitree@192.168.8.18   # Theodore (.18 / .19 / .20 = Theodore / Alvin / Simon)
# Password: 123
```

## The 5-terminal launch procedure

Steps 1, 3, 4, 5 run on the **remote laptop**. Step 2 runs **on the robot via SSH**.

!!! danger "Check launch flags before deploying on hardware"
    Default flags target simulation. Before running on a real robot, **explicitly verify** the following on the relevant launches (`robot_driver.py`, `quad_plan.py`, `remote_driver.py`):

    | Flag | Sim default | Hardware value |
    |---|---|---|
    | `use_sim_time` | `true` | **`false`** — robot uses wall clock, not Gazebo's `/clock` |
    | `is_hardware` | `false` | **`true`** — switches the driver to the manufacturer SDK instead of Gazebo bridge |
    | `mocap` | `false` | **`true`** — turns on the OptiTrack/NatNet pose feed for the EKF |

    Forgetting any of these is the most common cause of "the robot stands but doesn't track" symptoms. Pass them explicitly on the command line, e.g.:

    ```bash
    ros2 launch quad_utils robot_driver.py \
        use_sim_time:=false is_hardware:=true mocap:=true
    ```

!!! tip "Adjust per setup"
    `robot_driver.py` and `quad_plan.py` accept args that depend on which robot is running and which controller (learned vs MPC) you want. Default args target Go2 with inverse-dynamics control.

=== "1. Mocap (remote)"

    Only needed if mocap is in the loop:

    ```bash
    cd /root/ros2_ws/src/quad-sdk/quad_utils/scripts
    source launch_remote_env.sh
    cd ros2_ws
    ros2 launch mocap4r2_optitrack_driver mocap.py
    ```

=== "2. Robot driver (on robot, via SSH)"

    SSH in from the remote, then launch the robot interface:

    ```bash
    ssh unitree@192.168.8.18
    cd /root/ros2_ws/src/quad-sdk/quad_utils/scripts
    source launch_robot_env.sh
    cd ros2_ws
    ros2 launch quad_utils robot_driver.py
    ```

=== "3. RViz (remote)"

    ```bash
    cd /root/ros2_ws/src/quad-sdk/quad_utils/scripts
    source launch_remote_env.sh
    cd ros2_ws
    ros2 launch quad_utils remote_driver.launch
    ```

=== "4. Stand (remote)"

    Once the driver is up and you see joint state coming back, ramp the robot to stand:

    ```bash
    source launch_remote_env.sh
    ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 1"
    ```

=== "5. Plan / joystick (remote)"

    ```bash
    source launch_remote_env.sh
    ros2 launch quad_utils quad_plan.py
    ```

    Then walk:

    ```bash
    ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 2"
    ```

## Special arguments

```bash
# Sit the robot
ros2 topic pub /robot_1/control/mode std_msgs/UInt8 "data: 0"
```

```bash
# Restart from safety mode after a hard fall
# (Kill terminal 1 BEFORE touching the robot. Sit the robot. Then:)
ros2 topic pub /robot_1/control/restart_flag std_msgs/Bool "data: true"
```

```bash
# PlotJuggler in a 6th terminal
ros2 launch quad_utils remote_driver.launch live_plot:=true
```

!!! danger "Recovering from a hard fall"
    1. **Kill terminal 1 first** (the driver). Do not touch the robot while terminal 1 is alive — the controller is still trying to drive joints.
    2. Sit the robot manually.
    3. Then publish the restart_flag.
    4. Re-launch terminal 1.

## Common failure modes

??? warning "Robot stays in safety after restart_flag"
    You probably forgot to sit the robot first. Power-cycle if it gets stuck.

??? warning "Mocap pose shows but the robot doesn't track"
    Check that step 1 (mocap launch) is on the same ROS_DOMAIN_ID as the rest of the stack, and that the rigid body name in Motive matches what the EKF subscribes to.

??? warning "ros2 launch hangs on the robot"
    Time sync. Run `chronyc sources` on the robot — if the lab laptop isn't `^*` the cross-host transient discovery hangs. See [Time sync](time-sync.md).

??? warning "Build hits a 'NatNet' / 'optitrack' compile error on the robot"
    You forgot `--packages-skip mocap4r2_optitrack_driver`.

[:octicons-arrow-right-24: Hardware design](hardware-design.md)
