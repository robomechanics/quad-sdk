#! /usr/bin/env python


from quad_pybullet.estimation_node import pybullet_estimation_node


node_name = "pybullet_sensor"
robot_urdf = "/home/haoluo/catkin_ws/src/quad-sdk/quad_simulator/spirit_description/urdf/spirit.urdf"
step_rate = 200
pybullet_pub_name = "/state/ground_truth"
pybullet_pcid = 0
pybullet_robotid = 1

if __name__ == '__main__':
    new_node = pybullet_estimation_node(node_name,pybullet_robotid,pybullet_pcid,step_rate=20,pub_name = pybullet_pub_name)
    # sensor_node = new_node.sensor_node
    new_node.run()