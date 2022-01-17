#!/usr/bin/python

import rosbag
import rospy
import yaml
import numpy as np

from tf.transformations import euler_from_quaternion

topic_type_dict = {}
msg_types = ['nav_msgs/Odometry', 'sensor_msgs/Imu', 'geometry_msgs/PoseStamped',
        'quadrotor_msgs/PositionCommand', 'quadrotor_msgs/TRPYCommand',
        'quadrotor_msgs/SO3Command', 'sensor_msgs/Range',
        'geometry_msgs/PoseWithCovarianceStamped']
var_types = ['x', 'y', 'z', 'vx', 'vy', 'vz',
        'acc_x', 'acc_y', 'acc_z',
        'roll', 'pitch', 'yaw',
        'ang_vel_x', 'ang_vel_y', 'ang_vel_z']

def read_bag(bagfile):
    global inbag
    inbag = rosbag.Bag(bagfile, 'r')
    return read_topic_type()

def read_topic_type():
    info_dict = yaml.load(inbag._get_yaml_info())
    for x in info_dict['topics']:
         topic_type_dict[x['topic']] = x['type']
    return topic_type_dict

def read_msg(topics):
    data = {}
    if len(topics) > 0:
        for topic, msg, type in inbag.read_messages():
         if topics.count(topic):
            if topic_type_dict[topic] == 'nav_msgs/Odometry':
                data = update_odometry(data, topic, msg)
            elif topic_type_dict[topic] == 'sensor_msgs/Imu':
                data = update_imu(data, topic, msg)
            elif topic_type_dict[topic] == 'geometry_msgs/PoseStamped':
                data = update_pose(data, topic, msg.pose, msg.header)
            elif topic_type_dict[topic] == 'quadrotor_msgs/PositionCommand':
                data = update_pose_cmd(data, topic, msg)
            elif topic_type_dict[topic] == 'geometry_msgs/PoseWithCovarianceStamped':
                data = update_pose(data, topic, msg.pose.pose, msg.header)

    return data

def update_odometry(data, topic, msg):
       quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
               msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
       [r, p, y] = euler_from_quaternion(quat)

       if topic in data:
           data[topic]['x'] = np.append(data[topic]['x'], msg.pose.pose.position.x)
           data[topic]['y'] = np.append(data[topic]['y'], msg.pose.pose.position.y)
           data[topic]['z'] = np.append(data[topic]['z'], msg.pose.pose.position.z)
           data[topic]['vx'] = np.append(data[topic]['vx'], msg.twist.twist.linear.x)
           data[topic]['vy'] = np.append(data[topic]['vy'], msg.twist.twist.linear.y)
           data[topic]['vz'] = np.append(data[topic]['vz'], msg.twist.twist.linear.z)
           data[topic]['roll'] = np.append(data[topic]['roll'], r)
           data[topic]['pitch'] = np.append(data[topic]['pitch'], p)
           data[topic]['yaw'] = np.append(data[topic]['yaw'], y)
           data[topic]['ang_vel_x'] = np.append(data[topic]['ang_vel_x'], msg.twist.twist.angular.x)
           data[topic]['ang_vel_y'] = np.append(data[topic]['ang_vel_y'], msg.twist.twist.angular.y)
           data[topic]['ang_vel_z'] = np.append(data[topic]['ang_vel_z'], msg.twist.twist.angular.z)
           data[topic]['t'] = np.append(data[topic]['t'], msg.header.stamp.to_sec())
       else:
           data[topic] = {}
           data[topic]['x'] = np.array([msg.pose.pose.position.x])
           data[topic]['y'] = np.array([msg.pose.pose.position.y])
           data[topic]['z'] = np.array([msg.pose.pose.position.z])
           data[topic]['vx'] = np.array([msg.twist.twist.linear.x])
           data[topic]['vy'] = np.array([msg.twist.twist.linear.y])
           data[topic]['vz'] = np.array([msg.twist.twist.linear.z])
           data[topic]['ang_vel_x'] = np.array([msg.twist.twist.angular.x])
           data[topic]['ang_vel_y'] = np.array([msg.twist.twist.angular.y])
           data[topic]['ang_vel_z'] = np.array([msg.twist.twist.angular.z])
           data[topic]['roll'] = np.array([r])
           data[topic]['pitch'] = np.array([p])
           data[topic]['yaw'] = np.array([y])
           data[topic]['t'] = np.array([msg.header.stamp.to_sec()])
       return data

def update_pose(data, topic, msg, header):
       quat = [msg.orientation.x, msg.orientation.y,
               msg.orientation.z, msg.orientation.w]
       [r, p, y] = euler_from_quaternion(quat)

       if topic in data:
           data[topic]['x'] = np.append(data[topic]['x'], msg.position.x)
           data[topic]['y'] = np.append(data[topic]['y'], msg.position.y)
           data[topic]['z'] = np.append(data[topic]['z'], msg.position.z)
           data[topic]['roll'] = np.append(data[topic]['roll'], r)
           data[topic]['pitch'] = np.append(data[topic]['pitch'], p)
           data[topic]['yaw'] = np.append(data[topic]['yaw'], y)
           data[topic]['t'] = np.append(data[topic]['t'], header.stamp.to_sec())
       else:
           data[topic] = {}
           data[topic]['x'] = np.array([msg.position.x])
           data[topic]['y'] = np.array([msg.position.y])
           data[topic]['z'] = np.array([msg.position.z])
           data[topic]['roll'] = np.array([r])
           data[topic]['pitch'] = np.array([p])
           data[topic]['yaw'] = np.array([y])
           data[topic]['t'] = np.array([header.stamp.to_sec()])

       return data


def update_imu(data, topic, msg):
       quat = [msg.orientation.x, msg.orientation.y,
               msg.orientation.z, msg.orientation.w]
       [r, p, y] = euler_from_quaternion(quat)

       if topic in data:
           data[topic]['acc_x'] = np.append(data[topic]['acc_x'], msg.linear_acceleration.x)
           data[topic]['acc_y'] = np.append(data[topic]['acc_y'], msg.linear_acceleration.y)
           data[topic]['acc_z'] = np.append(data[topic]['acc_z'], msg.linear_acceleration.z)
           data[topic]['ang_vel_x'] = np.append(data[topic]['ang_vel_x'], msg.angular_velocity.x)
           data[topic]['ang_vel_y'] = np.append(data[topic]['ang_vel_y'], msg.angular_velocity.y)
           data[topic]['ang_vel_z'] = np.append(data[topic]['ang_vel_z'], msg.angular_velocity.z)
           data[topic]['roll'] = np.append(data[topic]['roll'], r)
           data[topic]['pitch'] = np.append(data[topic]['pitch'], p)
           data[topic]['yaw'] = np.append(data[topic]['yaw'], y)
           data[topic]['t'] = np.append(data[topic]['t'], msg.header.stamp.to_sec())
       else:
           data[topic] = {}
           data[topic]['acc_x'] = np.array([msg.linear_acceleration.x])
           data[topic]['acc_y'] = np.array([msg.linear_acceleration.y])
           data[topic]['acc_z'] = np.array([msg.linear_acceleration.z])
           data[topic]['ang_vel_x'] = np.array([msg.angular_velocity.x])
           data[topic]['ang_vel_y'] = np.array([msg.angular_velocity.y])
           data[topic]['ang_vel_z'] = np.array([msg.angular_velocity.z])
           data[topic]['roll'] = np.array([r])
           data[topic]['pitch'] = np.array([p])
           data[topic]['yaw'] = np.array([y])
           data[topic]['t'] = np.array([msg.header.stamp.to_sec()])
       return data

def update_pose_cmd(data, topic, msg):
       if topic in data:
           data[topic]['x'] = np.append(data[topic]['x'], msg.position.x)
           data[topic]['y'] = np.append(data[topic]['y'], msg.position.y)
           data[topic]['z'] = np.append(data[topic]['z'], msg.position.z)
           data[topic]['vx'] = np.append(data[topic]['vx'], msg.velocity.x)
           data[topic]['vy'] = np.append(data[topic]['vy'], msg.velocity.y)
           data[topic]['vz'] = np.append(data[topic]['vz'], msg.velocity.z)
           data[topic]['acc_x'] = np.append(data[topic]['acc_x'], msg.acceleration.x)
           data[topic]['acc_y'] = np.append(data[topic]['acc_y'], msg.acceleration.y)
           data[topic]['acc_z'] = np.append(data[topic]['acc_z'], msg.acceleration.z)
           data[topic]['yaw'] = np.append(data[topic]['yaw'], msg.yaw)
           data[topic]['t'] = np.append(data[topic]['t'], msg.header.stamp.to_sec())
       else:
           data[topic] = {}
           data[topic]['x'] = np.array([msg.position.x])
           data[topic]['y'] = np.array([msg.position.y])
           data[topic]['z'] = np.array([msg.position.z])
           data[topic]['vx'] = np.array([msg.velocity.x])
           data[topic]['vy'] = np.array([msg.velocity.y])
           data[topic]['vz'] = np.array([msg.velocity.z])
           data[topic]['acc_x'] = np.array([msg.acceleration.x])
           data[topic]['acc_y'] = np.array([msg.acceleration.y])
           data[topic]['acc_z'] = np.array([msg.acceleration.z])
           data[topic]['yaw'] = np.array([msg.yaw])
           data[topic]['t'] = np.array([msg.header.stamp.to_sec()])
       return data


def update_trpy_cmd(data, topic, msg):
       if topic in data:
           data[topic]['roll'] = np.append(data[topic]['roll'], msg.roll)
           data[topic]['pitch'] = np.append(data[topic]['pitch'], msg.pitch)
           data[topic]['yaw'] = np.append(data[topic]['yaw'], msg.yaw)
           data[topic]['t'] = np.append(data[topic]['t'], msg.header.stamp.to_sec())
       else:
           data[topic] = {}
           data[topic]['roll'] = np.array([msg.roll])
           data[topic]['pitch'] = np.array([msg.pitch])
           data[topic]['yaw'] = np.array([msg.yaw])
           data[topic]['t'] = np.array([msg.header.stamp.to_sec()])
       return data

def update_so3_cmd(data, topic, msg):
       quat = [msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w]
       [r, p, y] = euler_from_quaternion(quat)

       if topic in data:
           data[topic]['yaw'] = np.append(data[topic]['yaw'], y)
           data[topic]['ang_vel_x'] = np.append(data[topic]['ang_vel_x'], msg.angular_velocity.x)
           data[topic]['ang_vel_y'] = np.append(data[topic]['ang_vel_y'], msg.angular_velocity.y)
           data[topic]['ang_vel_z'] = np.append(data[topic]['ang_vel_z'], msg.angular_velocity.z)
           data[topic]['t'] = np.append(data[topic]['t'], msg.header.stamp.to_sec())
           data[topic]['roll'] = np.append(data[topic]['roll'], r)
           data[topic]['pitch'] = np.append(data[topic]['pitch'], p)
           data[topic]['yaw'] = np.append(data[topic]['yaw'], y)
       else:
           data[topic] = {}
           data[topic]['yaw'] = np.array([y])
           data[topic]['ang_vel_x'] = np.array(msg.angular_velocity.x)
           data[topic]['ang_vel_y'] = np.array(msg.angular_velocity.y)
           data[topic]['ang_vel_z'] = np.array(msg.angular_velocity.z)
           data[topic]['t'] = np.array([msg.header.stamp.to_sec()])
           data[topic]['roll'] = np.array([r])
           data[topic]['pitch'] = np.array([p])
           data[topic]['yaw'] = np.array([y])
       return data
def update_range(data, topic, msg):
       if topic in data:
           data[topic]['z'] = np.append(data[topic]['z'], msg.range)
           data[topic]['t'] = np.append(data[topic]['t'], msg.header.stamp.to_sec())
       else:
           data[topic] = {}
           data[topic]['z'] = np.array([msg.range])
           data[topic]['t'] = np.array([msg.header.stamp.to_sec()])
       return data




if __name__ == "__main__":
    read_topic_type()
