"""Publish a uniformly-zero grid_map terrain for the Isaac flow.

Substitute for Gazebo's mesh_to_grid_map_node when running on Isaac
without a world mesh -- gives local_planner something to plan over.

Run in the ROS2 controller-stack process (system Python 3.12), not in
the Isaac bridge process.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from grid_map_msgs.msg import GridMap, GridMapInfo
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class FlatTerrainPublisher(Node):
    """Publish a static flat grid_map at low rate."""

    def __init__(self) -> None:
        super().__init__('flat_terrain_publisher')

        self.declare_parameter('length_x', 20.0)
        self.declare_parameter('length_y', 20.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('frame_id', 'map')
        # _raw is the grid_map_filters_demo input; filtered output lands
        # at /mapping/terrain_map.
        self.declare_parameter('topic', '/mapping/terrain_map_raw')

        self.length_x = float(self.get_parameter('length_x').value)
        self.length_y = float(self.get_parameter('length_y').value)
        self.resolution = float(self.get_parameter('resolution').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        topic = str(self.get_parameter('topic').value)

        # TRANSIENT_LOCAL so late subscribers get the map; RELIABLE to
        # match local_planner's subscription.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.pub = self.create_publisher(GridMap, topic, qos)

        self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            f'Publishing flat terrain {self.length_x:.1f} x {self.length_y:.1f} m '
            f'@ {self.resolution:.3f} m to {topic}'
        )

    def _publish(self) -> None:
        msg = GridMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        info = GridMapInfo()
        info.resolution = self.resolution
        info.length_x = self.length_x
        info.length_y = self.length_y
        info.pose.position.x = 0.0
        info.pose.position.y = 0.0
        info.pose.position.z = 0.0
        info.pose.orientation.w = 1.0
        msg.info = info

        nx = int(round(self.length_x / self.resolution))
        ny = int(round(self.length_y / self.resolution))

        # Single 'z' layer of zeros; grid_map is column-major.
        msg.layers = ['z']
        msg.basic_layers = ['z']

        z_layer = Float32MultiArray()
        z_layer.layout.dim.append(
            MultiArrayDimension(label='column_index', size=ny, stride=ny * nx)
        )
        z_layer.layout.dim.append(
            MultiArrayDimension(label='row_index', size=nx, stride=nx)
        )
        z_layer.data = np.zeros(nx * ny, dtype=np.float32).tolist()
        msg.data = [z_layer]

        msg.outer_start_index = 0
        msg.inner_start_index = 0

        self.pub.publish(msg)


def main() -> None:
    """Entry point."""
    rclpy.init()
    node = FlatTerrainPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
