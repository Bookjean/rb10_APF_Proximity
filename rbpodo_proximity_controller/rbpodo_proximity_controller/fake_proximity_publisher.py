"""Fake proximity sensor publisher for simulation.

Publishes simulated proximity distance data on the same topics as ecan_driver.
Basic mode: publishes max range (no obstacle).
Simulation mode: computes distance from sensor to virtual obstacles.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger

from rbpodo_proximity_controller.kdl_helper import (
    build_link3_chain,
    compute_fk,
    kdl_frame_to_pos_rot,
    rpy_to_rotation_matrix,
)

# Same sensor configs as proximity_controller_node
SENSOR_CONFIGS = {
    'N': {
        'offset': np.array([0.06, 0.1484, 0.285075]),
        'rpy': (0.0, 0.0, 0.0),
    },
    'S': {
        'offset': np.array([-0.06, 0.1484, 0.285075]),
        'rpy': (0.0, 0.0, math.pi),
    },
    'E': {
        'offset': np.array([0.0, 0.0884, 0.285075]),
        'rpy': (0.0, 0.0, -math.pi / 2),
    },
    'W': {
        'offset': np.array([0.0, 0.2084, 0.285075]),
        'rpy': (0.0, 0.0, math.pi / 2),
    },
}

JOINT_NAMES = ['base', 'shoulder', 'elbow']
MIN_RANGE_MM = 20.0
OBSTACLE_RADIUS = 0.05


class FakeProximityPublisher(Node):
    def __init__(self):
        super().__init__('fake_proximity_publisher')

        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('max_range', 200.0)  # mm
        self.declare_parameter('default_range', 200.0)  # mm
        self.declare_parameter('obstacle_sim_enabled', False)
        self.declare_parameter('obstacle_position', [0.3, 0.0, 0.8])
        self.declare_parameter('detect_fov', 1.7)  # rad
        self.declare_parameter('threshold_mm', 150.0)

        self.publish_rate = self.get_parameter('publish_rate').value
        self.max_range = self.get_parameter('max_range').value
        self.default_range = self.get_parameter('default_range').value
        self.obstacle_sim_enabled = self.get_parameter(
            'obstacle_sim_enabled'
        ).value
        self.detect_fov = self.get_parameter('detect_fov').value
        self.threshold_mm = self.get_parameter('threshold_mm').value
        obs_pos = self.get_parameter('obstacle_position').value

        # KDL chain for link3
        self.chain_link3 = build_link3_chain()

        # Precompute sensor rotation matrices
        self.sensor_R_local = {}
        for name, cfg in SENSOR_CONFIGS.items():
            self.sensor_R_local[name] = rpy_to_rotation_matrix(*cfg['rpy'])

        # State
        self.q = None
        self.obstacles = []
        if self.obstacle_sim_enabled:
            self.obstacles.append(np.array(obs_pos))

        # Subscriptions
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10
        )
        self.create_subscription(
            PointStamped, '/clicked_point', self._add_obstacle_cb, 10
        )
        self.create_subscription(
            PointStamped, '/add_obstacle', self._add_obstacle_cb, 10
        )

        # Publishers (same topics as ecan_driver)
        self.proximity_pub = self.create_publisher(
            Float32MultiArray, '/proximity_distance', 10
        )
        self.detection_pub = self.create_publisher(
            Int32, '/proximity_detection', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/obstacle_markers', 10
        )

        # Services
        self.create_service(
            Trigger, '~/clear_obstacles', self._clear_obstacles_cb
        )

        # Timer
        period = 1.0 / self.publish_rate
        self.create_timer(period, self._publish_cb)

        self.get_logger().info(
            f'Fake proximity publisher started '
            f'(obstacle_sim={self.obstacle_sim_enabled})'
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
            f'Obstacle added at [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] '
            f'(total: {len(self.obstacles)})'
        )

    def _clear_obstacles_cb(self, request, response):
        n = len(self.obstacles)
        self.obstacles.clear()
        self.obstacle_sim_enabled = False
        response.success = True
        response.message = f'Cleared {n} obstacles'
        self.get_logger().info(response.message)
        return response

    def _publish_cb(self):
        stamp = self.get_clock().now().to_msg()

        # Compute minimum distance across all sensors
        if (self.obstacle_sim_enabled and self.q is not None
                and self.obstacles):
            min_dist_mm = self.max_range
            for direction, cfg in SENSOR_CONFIGS.items():
                dist_mm = self._compute_range_mm(direction, cfg)
                min_dist_mm = min(min_dist_mm, dist_mm)
        else:
            min_dist_mm = self.default_range

        # Publish proximity_distance (Float32MultiArray, mm)
        dist_msg = Float32MultiArray()
        dist_msg.data = [float(min_dist_mm)]
        self.proximity_pub.publish(dist_msg)

        # Publish proximity_detection (Int32, 0 or 1)
        det_msg = Int32()
        det_msg.data = 1 if 0 < min_dist_mm < self.threshold_mm else 0
        self.detection_pub.publish(det_msg)

        # Publish obstacle markers
        self._publish_obstacle_markers(stamp)

    def _compute_range_mm(self, direction, cfg):
        """Compute simulated range (mm) from sensor to nearest obstacle."""
        frame_link3 = compute_fk(self.chain_link3, self.q)
        p_link3, R_link3 = kdl_frame_to_pos_rot(frame_link3)

        p_sensor = p_link3 + R_link3 @ cfg['offset']
        R_sensor = self.sensor_R_local[direction]
        look_dir = R_link3 @ R_sensor @ np.array([1.0, 0.0, 0.0])

        half_fov = self.detect_fov / 2.0
        max_range_m = self.max_range / 1000.0
        best_range_m = max_range_m

        for obs_pos in self.obstacles:
            to_obs = obs_pos - p_sensor
            dist = np.linalg.norm(to_obs)
            if dist < MIN_RANGE_MM / 1000.0:
                continue

            cos_angle = np.dot(look_dir, to_obs) / dist
            angle_to_center = math.acos(np.clip(cos_angle, -1.0, 1.0))
            if dist > OBSTACLE_RADIUS:
                angular_radius = math.asin(OBSTACLE_RADIUS / dist)
            else:
                angular_radius = math.pi / 2.0
            if angle_to_center - angular_radius > half_fov:
                continue

            surface_dist = max(dist - OBSTACLE_RADIUS, MIN_RANGE_MM / 1000.0)
            if surface_dist < best_range_m:
                best_range_m = surface_dist

        return min(best_range_m * 1000.0, self.max_range)

    def _publish_obstacle_markers(self, stamp):
        ma = MarkerArray()

        if not self.obstacles:
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = 'link0'
            m.ns = 'obstacles'
            m.action = Marker.DELETEALL
            ma.markers.append(m)
            self.marker_pub.publish(ma)
            return

        for i, obs in enumerate(self.obstacles):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = 'link0'
            m.ns = 'obstacles'
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
    node = FakeProximityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
