# MBLinkProtocol

These files contain the protocol for communication between the mainboard and any higher-level controller.

### Integration

Dependencies:
- mavlink
- Eigen 3.3+

When messages are received, the networking code should call `unpack()` with every received message, and the parsed data is placed in a key-value format in `rxdata` which is documented below. 

### Setting and getting mainboard params

The `setParam`, `getParam` can be used to get and set params. `getParam` waits a little while to send a request and wait for a response. the `setRetry` function sets a param and then reads it again to ensure the value was set.

### Received data key-value pairs

Full mavlink messages can be processed and converted to logical key/value pairs by calling `unpack()` on a mavlink message. Alternately, `parse()` can process a buffer with one or more mavlink messages (for example, received over UDP) and unpack all of them.

The keys are `std::string` and the values are `Eigen::VectorXf`. When data is added (by calling `unpack()`) the `VectorXf` is resized appropriately.

| key | size | meaning |
| ------ | ------ | ------ |
| `joint_cmd` | 12 | Commanded joint currents [A] |
| `joint_position` | 12 | Current joint positions [rad for revolute joints] |
| `joint_velocity` | 12 | Current joint velocities [rad/s for revolute joints] |
| `joint_current` | 12 | Current joint currents [A] |
| `joint_temperature` | 12 | Current joint temperature (estimated) [C] |
| `joint_voltage` | 12 | Current joint voltage [V] |
| `joint_residual` | 8 | Sagittal plane estimated external forces acting at the toe `[fz0, fx0, fz1, ..., fx3]` [N] |
| `imu_euler` | 3 | Euler angles xyz (to transform from body to world frame, `pw = Rz*Ry*Rx*pb`) [rad] |
| `imu_linear_acceleration` | 3 | Cartesian linear acceleration [m/s^2] |
| `imu_angular_velocity` | 3 | Raw IMU angular velocity xyz [rad/s] |
| `twist_linear` | 3 | Proprioceptive body-frame velocity estimate [m/s] |
| `voltage` | 2 | System voltage from the BMS, joint mean [V] |
| `joy_axes` | 4 | Raw (mainboard-connected) joystick axes in [-1,1] |
| `joy_twist` | 4 | Scaled joystick values for (x_vel,y_vel,yaw_vel,z_pose) |
| `phase` | 4 | Swing phase for each leg |
| `swing_mode` | 4 | Discrete swing mode for each leg |
| `mode` | 2 | MAVlink base mode, custom mode |
| `contacts` | 4 | Boolean array of leg contact states |
| `behavior` | 3 | Low-level SDK behaviorId, behaviorMode, and status |
| `slope_est` | 2 | roll, pitch terrain slope estimate |
| `se2twist_des` | 3 | The filtered SE(2) twist currently being used as a setpoint by the low-level controllers |
| `user` | 10 | User data sent using `mavlinkUserTxPack()` in the low-level SDK | 
| `param_value` | 0 or 1 | A parameter value is sometimes stored (use `getParam()` instead of directly reading this) |
| `debug_timings` | 4 | Reserved |
| `debug_legH` | 3 | Reserved |
| `y` | 21 | Current state vector (reserved); last element is mainboard time in seconds |

### Functions to send data to the mainboard

The `send*()` functions are for sending continuous asynchronous commands (such as commanded velocities, poses, etc.) and the `set*()` commands are for discrete selections such as selecting a behavior, setting a parameter value, etc. These pack a message for sending in the `msg` member variables.

To actually send the messages to the mainboard, the `queueMessage()` function must be implemented. The virtual `queueMessage()` function does nothing in the `MBLinkProtocol` class (but is specialized below).

# MBLink communication library

This builds on the protocol and adds a UDP communication library. It also adds an optional python binding using pybind11 (v2.5.0).

### Building

To build the library only,
```
cd mblink
mkdir -p build
cd build
cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DPYBIND=ON
make
```
`PYBIND` is `ON` by default and builds python bindings. Note that your compiler must be compatible with the python it detects. You can also specify the python executable to use using PYTHON_EXECUTABLE.

If building on Windows with Visual Studio Build Tools, use
```
cmake .. -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DPYBIND=ON
nmake
```

### How it works

Calling `start()` starts a receiver thread that sets a flag when new data is received. Calling `get()` will safely copy this received data for the main thread using a mutex. This enables very simple application code that looks like this:
```C
mb.start(...);
mb.rxstart();
mb.setRetry("UPST_LOOP_DELAY", 5) // the rate of loop below will be ~1000/n Hz
while (!shouldStop)
{
	data = mb.get(); // will block (yielding CPU) till there is new data
	// use data to produce control data
	u = myController(data["y"])
	mb.sendSE2Twist(u)
}
mb.rxstop(); // will stop the receiver thread
```
