#! /usr/bin/env python


from quad_pybullet.pybullet_master_node import pybullet_sim_node


node_name = "pybullet_sim"
robot_urdf = "/home/haoluo/catkin_ws/src/quad-sdk/quad_simulator/spirit_description/urdf/spirit.urdf"
step_rate = 1000
pybullet_pub_name = "/robot_1/state/ground_truth_pybullet"
world_urdf = "/home/haoluo/catkin_ws/src/quad-sdk/quad_simulator/gazebo_scripts/worlds/flat/flat.world"
# pybullet_sub_name = "/robot_1/control/joint_commands_pybullet"
pybullet_sub_name = "/robot_1/control/joint_command"

if __name__ == '__main__':
    new_node = pybullet_sim_node(node_name,step_rate,robot_urdf,world_urdf=world_urdf, pub_name = pybullet_pub_name,sub_name = pybullet_sub_name)
    # sensor_node = new_node.sensor_node
    new_node.run()
