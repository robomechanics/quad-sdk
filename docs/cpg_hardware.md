# Go2 Conga hardware runbook

The contact-free Conga controller is an **experimental Go2 hardware candidate**. The packaged policy has passed offline replay and x86 Gazebo integration. Physical Go2 motion, onboard ARM64 execution, and hardware timing are unverified. Use a lab-approved support rig with an operator at the robot and a verified independent stop for the first hardware session.

The destination laptop can build and simulate from this Quad-SDK checkout alone. For hardware, run both the driver and its 50 Hz CPU worker on the same Linux computer with access to the robot’s Unitree Ethernet subnet. This can be the onboard computer or a directly connected laptop on a verified wired link. Issue first-session commands from a second shell on that same computer, over SSH if needed. Keep the known-good walking checkout and launch command available for rollback.

## Build and preflight

1. Check `uname -m`, `cat /etc/os-release`, and `ls /opt/ros`. Follow the [native Conga build instructions](cpg_gazebo.md#build) on Ubuntu 24.04 / ROS 2 Jazzy, or use the [standalone CPU image](cpg_gazebo.md#fresh-clone-and-docker) on x86 Linux. Rebuild for the destination architecture. For an onboard computer on a different ROS/OS combination, keep its working installation and verify a compatible build on that combination before connecting this controller.
2. In the native checkout, source Jazzy and the overlay printed by the build script. Check that the package prefixes resolve to this installation and replay its installed policy.

   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/conga-host/setup.bash
   ros2 pkg prefix quad_utils
   ros2 interface show quad_msgs/msg/CpgSensorFrame
   ros2 launch quad_utils conga_driver.py --show-args
   "$HOME/venvs/go2-cpg/bin/python" scripts/test_conga_runtime.py \
     "$(ros2 pkg prefix --share robot_driver)/conga"
   ```

   In the standalone image, the overlay is sourced by `/conga_entrypoint.sh` and the policy Python is `/opt/conga-venv/bin/python`. Run the replay and headless Gazebo check before using the hardware launch. Gazebo commands can reach a real driver on the same ROS domain, so stop simulation and its command publishers before switching to hardware.

3. Identify the **robot-facing** Linux network interface with `ip -br link` and `ip -br -4 addr` on the computer running the driver. Verify the wired port has the address assigned for your robot's Unitree subnet (normally `192.168.123.0/24` in the lab). Confirm the address with the robot's network configuration. The ROS domain below is for the operator topics. Unitree low-level DDS uses domain 0 on the interface passed to the hardware launch. [Unitree DDS networking](https://github.com/unitreerobotics/xr_teleoperate/wiki/CycloneDDS-and-UnitreeSDK-%28en%29)
4. Check that there is one low-level controller and that old mode, joystick, and velocity publishers are stopped. Set the same dedicated ROS domain and local discovery in the driver and operator shells.

   ```bash
   export ROS_DOMAIN_ID=43
   export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
   ```

## Start in SAFETY

Place the robot in a supported, seated posture and confirm the independent stop. The launch first loads the policy, then waits up to five seconds for advancing Unitree low-state packets. It releases the Unitree motion service and takes low-level control after that check. Replace `enp3s0` below with the interface identified above.

```bash
ros2 launch quad_utils conga_driver.py \
  activate_hardware:=true mcu_iface:=enp3s0 \
  trace:="$HOME/conga_hardware_trace.jsonl"
```

For the standalone image on Linux, expose the host NIC with host networking. Set discovery in the container and issue ROS commands from a second shell using `docker exec conga-robot /conga_entrypoint.sh` before each command.

```bash
mkdir -p "$HOME/conga-logs"
docker run --rm --init --name conga-robot --network host \
  -e ROS_DOMAIN_ID=43 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v "$HOME/conga-logs:/logs" quad-sdk:conga \
  ros2 launch quad_utils conga_driver.py activate_hardware:=true \
  mcu_iface:=enp3s0 trace:=/logs/conga_hardware_trace.jsonl
```

Before requesting READY, use the operator shell to verify SAFETY (`data: 4`), an advancing sensor stream, and fresh frames with 12 joint positions and velocities and a plausible IMU orientation. Use a dedicated terminal for faults.

```bash
ros2 topic echo --once /robot_1/state/control_mode
ros2 topic hz /robot_1/cpg/sensors
ros2 topic echo --once --qos-reliability best_effort /robot_1/cpg/sensors
ros2 topic echo /robot_1/cpg/fault
```

If there is no sensor stream or the driver keeps retrying motion-service release, stop the launch and verify the NIC, subnet, and ownership of low-level control. The preflight worker message confirms that the policy loaded. The live sensor topic confirms that the driver is receiving robot data.

## Request motion and stop

| Command | Behavior |
| --- | --- |
| `control/mode` **1** | Ramp from fresh measured joint positions to the standing pose, enter READY, then start Conga after startup settling. |
| `cmd_vel` | Forward 0–0.10 m/s and yaw ±0.10 rad/s. Lateral and backward commands are clamped to zero. Zero or stale input removes travel while the dance continues. |
| `cpg/stand` **true** | Finish the scheduled movement and hold a standing target. Relaunch to dance again. |
| `control/mode` **0** | From READY, request the sit transition. Leaving READY ends the active dance session. |
| `control/mode` **4** | Apply software damping, including during a stand or sit transition. An active-session stop or fault requires a relaunch before another READY request. |

Have the mode 4 command ready before intentionally requesting READY. Remain supported for the first trial and start at neutral. Check that `/robot_1/cpg/status` advances. Watch robot posture and motor response directly. The contact-free scheduler assumes support from the planned phase, and hardware visualization has no measured horizontal odometry in this configuration.

```bash
# Deliberate start while supported
ros2 topic pub --once /robot_1/control/mode std_msgs/msg/UInt8 '{data: 1}'

# Stand request
ros2 topic pub --once /robot_1/cpg/stand std_msgs/msg/Bool '{data: true}'

# Software stop
ros2 topic pub --once /robot_1/control/mode std_msgs/msg/UInt8 '{data: 4}'
```

Damping can let the robot sag or fall. Use the independent stop and support rig for a loss of control. If a fault occurs, secure the robot, keep the launch log and JSONL trace, and resolve the cause before relaunching. Check the age and continuity of `CpgSensorFrame`, worker round-trip times, and any deadline, clock, or joint-range fault. The worker has a 20 ms hardware reply deadline. The trace logs nominal command gains before the driver cap. The driver uses Kp 25 and Kd 0.5 during policy control and scales both when the sampled 18 Nm cap or speed envelope binds. Sampled torque limiting does not bound physical torque between samples.

For normal shutdown, secure and support the robot before exiting the launch. For rollback, stop the Conga driver and worker, then restore your site's known-good Unitree motion service or start the previously verified walking installation according to the lab procedure. Keep the two low-level drivers mutually exclusive.
