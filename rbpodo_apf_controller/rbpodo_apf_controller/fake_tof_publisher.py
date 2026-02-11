"""Fake ToF sensor publisher for simulation.

Basic mode: publishes fixed range values (no obstacle).
Simulation mode: computes distance from sensor position/direction to virtual obstacles.
Supports multiple obstacles via /clicked_point (RViz Publish Point tool).
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger

from rbpodo_apf_controller.kdl_helper import (
    build_link3_chain,
    compute_fk,
    kdl_frame_to_pos_rot,
    rpy_to_rotation_matrix,
)

# Must match apf_node.py SENSOR_CONFIGS
SENSOR_CONFIGS = {
    "N": {"offset": np.array([0.06, 0.1484, 0.285075]), "rpy": (0.0, 0.0, 0.0)},
    "S": {"offset": np.array([-0.06, 0.1484, 0.285075]), "rpy": (0.0, 0.0, math.pi)},
    "E": {"offset": np.array([0.0, 0.0884, 0.285075]), "rpy": (0.0, 0.0, -math.pi / 2)},
    "W": {"offset": np.array([0.0, 0.2084, 0.285075]), "rpy": (0.0, 0.0, math.pi / 2)},
}

JOINT_NAMES = ["base", "shoulder", "elbow"]
MIN_RANGE = 0.02
OBSTACLE_RADIUS = 0.05


class FakeToFPublisher(Node):
    def __init__(self):
        super().__init__("fake_tof_publisher")

        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("max_range", 0.2)
        self.declare_parameter("default_range", 0.2)
        self.declare_parameter("obstacle_sim_enabled", False)
        self.declare_parameter("obstacle_position", [0.3, 0.0, 0.8])
        self.declare_parameter("tof_detect_fov", 2.96706)  # detection FOV (rad)

        self.publish_rate = self.get_parameter("publish_rate").value
        self.max_range = self.get_parameter("max_range").value
        self.default_range = self.get_parameter("default_range").value
        self.obstacle_sim_enabled = self.get_parameter("obstacle_sim_enabled").value
        self.tof_detect_fov = self.get_parameter("tof_detect_fov").value
        obs_pos = self.get_parameter("obstacle_position").value

        # KDL chain for link3
        self.chain_link3 = build_link3_chain()

        # Precompute sensor rotation matrices
        self.sensor_R_local = {}
        for name, cfg in SENSOR_CONFIGS.items():
            self.sensor_R_local[name] = rpy_to_rotation_matrix(*cfg["rpy"])

        # State
        self.q = None
        self.obstacles = []  # list of np.array([x, y, z])
        if self.obstacle_sim_enabled:
            self.obstacles.append(np.array(obs_pos))

        # Subscriptions
        self.create_subscription(JointState, "/joint_states", self._joint_states_cb, 10)
        self.create_subscription(PointStamped, "/clicked_point", self._add_obstacle_cb, 10)
        self.create_subscription(PointStamped, "/add_obstacle", self._add_obstacle_cb, 10)

        # Publishers
        self.range_pubs = {}
        for direction in SENSOR_CONFIGS:
            self.range_pubs[direction] = self.create_publisher(
                Range, f"/link3_tof_{direction}/range", 10
            )
        self.marker_pub = self.create_publisher(MarkerArray, "/obstacle_markers", 10)

        # Services
        self.create_service(Trigger, "~/clear_obstacles", self._clear_obstacles_cb)

        # Timer
        period = 1.0 / self.publish_rate
        self.create_timer(period, self._publish_cb)

        self.get_logger().info(
            f"Fake ToF publisher started (obstacle_sim={self.obstacle_sim_enabled})"
        )

    def _joint_states_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.q = np.array([name_to_pos[n] for n in JOINT_NAMES])
        except KeyError:
            pass

    def _add_obstacle_cb(self, msg):
        pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        self.obstacles.append(pos)
        self.obstacle_sim_enabled = True
        self.get_logger().info(
            f"Obstacle added at [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] "
            f"(total: {len(self.obstacles)})"
        )

    def _clear_obstacles_cb(self, request, response):
        n = len(self.obstacles)
        self.obstacles.clear()
        self.obstacle_sim_enabled = False
        response.success = True
        response.message = f"Cleared {n} obstacles"
        self.get_logger().info(response.message)
        return response

    def _publish_cb(self):
        stamp = self.get_clock().now().to_msg()

        for direction, cfg in SENSOR_CONFIGS.items():
            range_msg = Range()
            range_msg.header.stamp = stamp
            range_msg.header.frame_id = f"tof_{direction}"
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = self.tof_detect_fov
            range_msg.min_range = MIN_RANGE
            range_msg.max_range = self.max_range

            if self.obstacle_sim_enabled and self.q is not None and self.obstacles:
                range_msg.range = self._compute_range(direction, cfg)
            else:
                range_msg.range = self.default_range

            self.range_pubs[direction].publish(range_msg)

        # Publish obstacle markers
        self._publish_obstacle_markers(stamp)

    def _compute_range(self, direction, cfg):
        """Compute simulated range from sensor to nearest obstacle within FOV cone."""
        frame_link3 = compute_fk(self.chain_link3, self.q)
        p_link3, R_link3 = kdl_frame_to_pos_rot(frame_link3)

        # Sensor position in world frame
        p_sensor = p_link3 + R_link3 @ cfg["offset"]

        # Sensor look direction in world frame (+X of sensor local frame)
        R_sensor = self.sensor_R_local[direction]
        look_dir = R_link3 @ R_sensor @ np.array([1.0, 0.0, 0.0])

        half_fov = self.tof_detect_fov / 2.0
        cos_half_fov = math.cos(half_fov)
        best_range = self.max_range

        for obs_pos in self.obstacles:
            to_obs = obs_pos - p_sensor
            dist = np.linalg.norm(to_obs)
            if dist < MIN_RANGE:
                continue

            # Check if any part of obstacle sphere is within FOV cone
            cos_angle = np.dot(look_dir, to_obs) / dist
            angle_to_center = math.acos(np.clip(cos_angle, -1.0, 1.0))
            # Subtract angular radius of the obstacle sphere
            if dist > OBSTACLE_RADIUS:
                angular_radius = math.asin(OBSTACLE_RADIUS / dist)
            else:
                angular_radius = math.pi / 2.0  # inside the sphere
            if angle_to_center - angular_radius > half_fov:
                continue  # Entirely outside FOV

            # Distance to nearest surface of obstacle sphere
            surface_dist = max(dist - OBSTACLE_RADIUS, MIN_RANGE)
            if surface_dist < best_range:
                best_range = surface_dist

        return min(best_range, self.max_range)

    def _publish_obstacle_markers(self, stamp):
        """Publish red spheres for each obstacle."""
        ma = MarkerArray()

        if not self.obstacles:
            # Publish empty array to clear old markers
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "link0"
            m.ns = "obstacles"
            m.action = Marker.DELETEALL
            ma.markers.append(m)
            self.marker_pub.publish(ma)
            return

        for i, obs in enumerate(self.obstacles):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "link0"
            m.ns = "obstacles"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(obs[0])
            m.pose.position.y = float(obs[1])
            m.pose.position.z = float(obs[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = OBSTACLE_RADIUS * 2.0
            m.color.r = 1.0
            m.color.a = 0.7
            ma.markers.append(m)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FakeToFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
