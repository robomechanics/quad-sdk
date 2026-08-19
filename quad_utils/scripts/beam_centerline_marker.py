#!/usr/bin/env python3
"""Publish a centreline marker for the balance beam so RViz can show it.

The Gazebo stripe added to the beam model's SDF is invisible to RViz --
RViz never loads the world SDF, it only renders the grid map published on
/mapping/terrain_map (built straight from the .ply by mesh_to_grid_map) plus
whatever marker topics are subscribed. So the line has to be published as a
visualization_msgs/Marker to appear there.

Geometry (x extent, beam half-width, eroded corridor half-width) is supplied
by mapping.py, which reads it from the same .ply the terrain map is built from
and from the live erosion radius in filter_chain.yaml. The defaults below are
only a standalone fallback -- they are overridden whenever this node is
launched normally, so a new beam width needs no edit here.

Published on a transient-local topic so RViz picks it up whenever it starts,
AND re-published on a slow timer so it still shows if the RViz display is
configured Volatile. Either QoS works.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration


class BeamCentrelineMarker(Node):
    def __init__(self):
        super().__init__('beam_centerline_marker')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/mapping/beam_centerline')
        # Geometry below is in MESH coordinates and is then transformed by
        # mesh_pose, exactly as mesh_to_grid_map places the terrain mesh --
        # otherwise the line lands at the map origin while the beam sits
        # wherever mesh_pose put it.
        #
        # All beam_world_* meshes span x in [-1.8288, 1.8288] with the narrow
        # section between x = +/-0.9144. Drawing the full length rather than
        # just the waist makes approach alignment visible too.
        self.declare_parameter('x_min', -1.8288)
        self.declare_parameter('x_max', 1.8288)
        self.declare_parameter('y_center', 0.0)
        # 2 mm above the beam top so it is not buried in the grid map
        # surface, which RViz draws as a filled mesh.
        self.declare_parameter('z', 0.002)
        self.declare_parameter('line_width', 0.005)
        self.declare_parameter('publish_period', 1.0)
        # Same [x y z roll pitch yaw] the mapping launch hands to
        # mesh_to_grid_map_node. Only translation and yaw are applied here --
        # the beam is laid flat, so roll/pitch are not meaningful for a
        # top-surface reference line.
        self.declare_parameter('mesh_pose', [0.0] * 6)
        # Fallback only -- mapping.py overrides both. The corridor bound is
        # NOT edge_half_width minus the erosion radius: erosion removes whole
        # cells and the cell centres sit half a pitch off the centreline, so
        # the 10 cm beam measures +/-0.0325, not +/-0.035. mapping.py computes
        # it; see the derivation note there and in filter_chain.yaml.
        self.declare_parameter('draw_edges', True)
        self.declare_parameter('edge_half_width', 0.05)
        self.declare_parameter('corridor_half_width', 0.035)

        self.frame_id = self.get_parameter('frame_id').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_center = self.get_parameter('y_center').value
        self.z = self.get_parameter('z').value
        self.line_width = self.get_parameter('line_width').value
        self.draw_edges = self.get_parameter('draw_edges').value
        self.edge_half_width = self.get_parameter('edge_half_width').value
        self.corridor_half_width = self.get_parameter('corridor_half_width').value

        pose = list(self.get_parameter('mesh_pose').value)
        if len(pose) != 6:
            self.get_logger().warn(
                f"mesh_pose has {len(pose)} entries, expected 6; "
                "drawing at the map origin instead.")
            pose = [0.0] * 6
        self.mesh_xyz = pose[0:3]
        self.mesh_yaw = pose[5]

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        topic = self.get_parameter('topic').value
        self.pub = self.create_publisher(MarkerArray, topic, qos)

        self.publish_markers()
        self.create_timer(
            self.get_parameter('publish_period').value, self.publish_markers)

        self.get_logger().info(
            f"Beam centreline on {topic} ({self.frame_id}): "
            f"y={self.y_center:.3f}, x=[{self.x_min:.4f}, {self.x_max:.4f}]")

    def make_line(self, marker_id, y, rgba, width_scale=1.0):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'beam'
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        # Identity orientation: an all-zero quaternion makes RViz log
        # "Uninitialized quaternion, assuming identity" on every message.
        m.pose.orientation.w = 1.0
        m.scale.x = self.line_width * width_scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.lifetime = Duration(sec=0, nanosec=0)

        c, s = math.cos(self.mesh_yaw), math.sin(self.mesh_yaw)
        for x in (self.x_min, self.x_max):
            m.points.append(Point(
                x=float(self.mesh_xyz[0] + c * x - s * y),
                y=float(self.mesh_xyz[1] + s * x + c * y),
                z=float(self.mesh_xyz[2] + self.z)))
        return m

    def publish_markers(self):
        arr = MarkerArray()
        # Centreline: opaque red.
        arr.markers.append(
            self.make_line(0, self.y_center, (1.0, 0.1, 0.1, 1.0)))

        if self.draw_edges:
            # Physical beam edges: white, thinner.
            for i, sign in enumerate((-1.0, 1.0)):
                arr.markers.append(self.make_line(
                    1 + i, self.y_center + sign * self.edge_half_width,
                    (1.0, 1.0, 1.0, 0.9), width_scale=0.6))
            # Eroded foothold corridor bound: amber, thinner.
            for i, sign in enumerate((-1.0, 1.0)):
                arr.markers.append(self.make_line(
                    3 + i, self.y_center + sign * self.corridor_half_width,
                    (1.0, 0.7, 0.0, 0.9), width_scale=0.6))

        self.pub.publish(arr)


def main():
    rclpy.init()
    node = BeamCentrelineMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
