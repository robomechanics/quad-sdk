"""ROS-independent codec for robomechanics/quad-sdk devel_ros2 Go2 messages.

Verified against commit 67d2b64749e13cd924a778be0eb1e7914b014582.
Uses raw IMU/joints/FootContact, not the stack's privileged simulation state.
"""
import math

# Quad-SDK FL, RL, FR, RR -> Unitree FR, FL, RR, RL, hip/thigh/calf.
QUAD_TO_SDK = (3,4,5,9,10,11,0,1,2,6,7,8)
QUAD_NAMES = ('8','0','1','9','2','3','10','4','5','11','6','7')
QUAD_NAMED = tuple(f'{leg}_{j}_joint' for leg in ('FL','RL','FR','RR')
                   for j in ('hip','thigh','calf'))


def decode_state(joints, imu, contact, timestamp, force_scale=1., force_offset=0.):
    """Message objects are duck-typed so the codec can be tested without ROS.

    FootContact.foot_force_raw has *raw sensor units*. Scale/offset must be
    measured before hardware use; scale=1 is only the simulation convention.
    """
    if force_scale <= 0 or not math.isfinite(force_scale) or not math.isfinite(force_offset):
        raise ValueError('Invalid foot-force calibration')
    names = list(joints.name)
    if len(names) != 12 or len(set(names)) != 12:
        raise ValueError('Expected 12 uniquely named Go2 joints')
    expected = QUAD_NAMES if set(names) == set(QUAD_NAMES) else QUAD_NAMED
    if set(names) != set(expected) or len(joints.position) != 12 or len(joints.velocity) != 12:
        raise ValueError('Unknown Go2 joint names or missing joint state')
    q, qd = [0.]*12, [0.]*12
    for name, sdk in zip(expected, QUAD_TO_SDK):
        index = names.index(name)
        q[sdk], qd[sdk] = float(joints.position[index]), float(joints.velocity[index])
    if len(contact.foot_force_raw) != 4:
        raise ValueError('Expected four raw foot-force readings')
    forces = [max(0., (float(contact.foot_force_raw[i])-force_offset)*force_scale) for i in (0,2,1,3)]
    result = dict(time=float(timestamp), q=q, qd=qd,
                  quaternion=[imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z],
                  gyro=[imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z], foot_force=forces)
    if not all(math.isfinite(v) for values in result.values() for v in (values if isinstance(values,list) else [values])):
        raise ValueError('Nonfinite Quad-SDK sensor message')
    return result


def encode_command(target_sdk, kp_sdk, kd_sdk, array_type, leg_type, motor_type, stamp=None):
    arrays = (target_sdk,kp_sdk,kd_sdk)
    if any(len(a)!=12 or not all(math.isfinite(v) for v in a) for a in arrays):
        raise ValueError('Invalid joint command')
    message = array_type()
    if stamp is not None: message.header.stamp = stamp
    for leg in range(4):
        command = leg_type()
        for sdk in QUAD_TO_SDK[leg*3:leg*3+3]:
            motor = motor_type()
            motor.pos_setpoint = float(target_sdk[sdk])
            motor.vel_setpoint = 0.
            motor.kp, motor.kd = float(kp_sdk[sdk]),float(kd_sdk[sdk])
            motor.torque_ff = 0.  # Nominal support force is already a position bias.
            command.motor_commands.append(motor)
        message.leg_commands.append(command)
    return message
