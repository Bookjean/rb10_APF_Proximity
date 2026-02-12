"""Proximity sensor-based APF controller with waypoint P2P motion.

Three operating modes:
  1. IDLE  (APF off)  - visualization only, no commands
  2. HOLD  (APF on, no trajectory) - hold anchor position, dodge obstacles
  3. EXECUTING (APF on + trajectory) - waypoint following with obstacle avoidance

Moves through multiple waypoints sequentially while avoiding obstacles
detected by a proximity sensor (ecan_driver), using Artificial Potential
Field method with sensor-point Jacobian.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
from std_srvs.srv import SetBool, Trigger

from rbpodo_proximity_controller.kdl_helper import (
    build_ee_chain,
    build_link3_chain,
    compute_fk,
    compute_jacobian,
    kdl_frame_to_pos_rot,
    rpy_to_rotation_matrix,
)

# Proximity sensor configuration on link3
# Same position/direction as ToF sensors in apf_controller
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

JOINT_NAMES = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']


class ProximityController(Node):
    def __init__(self):
        super().__init__('proximity_controller')

        # Declare parameters
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('joint_names', JOINT_NAMES)
        self.declare_parameter('q_min', [-3.14] * 6)
        self.declare_parameter('q_max', [3.14] * 6)
        self.declare_parameter('k_att', 2.0)
        self.declare_parameter('k_rep', 5.0)
        self.declare_parameter('k_q', 1.0)
        self.declare_parameter('k_hold', 2.0)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('q_limit_margin', 0.1)
        self.declare_parameter('goal_tolerance', 0.005)
        self.declare_parameter('d0', 0.15)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('r_min', 0.02)
        self.declare_parameter('ee_offset', [0.0, -0.1153, 0.0])
        self.declare_parameter('waypoint_tolerance', 0.02)
        self.declare_parameter('sensor_fov', 1.7)
        self.declare_parameter('sensor_range', 0.2)
        self.declare_parameter('sensor_marker_size', 0.1)
        self.declare_parameter('auto_execute', False)

        # Read parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.dt = self.get_parameter('dt').value
        self.joint_names = self.get_parameter('joint_names').value
        self.q_min = np.array(self.get_parameter('q_min').value)
        self.q_max = np.array(self.get_parameter('q_max').value)
        self.k_att = self.get_parameter('k_att').value
        self.k_rep = self.get_parameter('k_rep').value
        self.k_q = self.get_parameter('k_q').value
        self.k_hold = self.get_parameter('k_hold').value
        self.v_max = self.get_parameter('v_max').value
        self.q_limit_margin = self.get_parameter('q_limit_margin').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.d0 = self.get_parameter('d0').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.r_min = self.get_parameter('r_min').value
        self.waypoint_tolerance = self.get_parameter(
            'waypoint_tolerance'
        ).value
        self.sensor_fov = self.get_parameter('sensor_fov').value
        self.sensor_range = self.get_parameter('sensor_range').value
        self.sensor_marker_size = self.get_parameter(
            'sensor_marker_size'
        ).value
        self.auto_execute = self.get_parameter('auto_execute').value

        # Build KDL chains
        ee_offset = self.get_parameter('ee_offset').value
        self.chain_ee = build_ee_chain(ee_offset=ee_offset)
        self.chain_link3 = build_link3_chain()

        # Precompute sensor local rotation matrices
        self.sensor_R_local = {}
        for name, cfg in SENSOR_CONFIGS.items():
            self.sensor_R_local[name] = rpy_to_rotation_matrix(*cfg['rpy'])

        # State
        self.q = None
        self.proximity_distance = None  # raw distance in mm
        self.proximity_filtered = float('inf')  # EMA filtered, in meters
        self.proximity_ranges = {}  # dict of sensor_id -> range (meters)
        self.waypoints = []  # list of joint position arrays
        self.current_waypoint_idx = 0
        self.executing = False
        self.enabled = False
        self.q_anchor = None  # anchor joint position for hold mode
        self._auto_execute_done = False  # track if auto_execute already fired

        # Subscribers
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10
        )
        self.create_subscription(
            Float32MultiArray, '/proximity_distance',
            self._proximity_distance_cb, 10
        )
        # Subscribe to individual proximity sensor Range topics
        for i in range(1, 9):
            self.create_subscription(
                Range, f'/proximity_distance{i}',
                lambda msg, idx=i: self._proximity_range_cb(msg, idx), 10
            )
        self.create_subscription(
            Float64MultiArray, '/waypoint_manager/waypoints',
            self._waypoints_cb, 10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/position_controllers/commands', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/proximity_debug/markers', 10
        )

        # Services
        self.create_service(SetBool, '~/enable', self._enable_cb)
        self.create_service(
            Trigger, '~/execute_waypoints', self._execute_waypoints_cb
        )
        self.create_service(Trigger, '~/stop', self._stop_cb)

        # Control loop timer
        period = 1.0 / self.control_rate
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'Proximity controller initialized '
            f'(k_att={self.k_att}, k_rep={self.k_rep}, '
            f'k_hold={self.k_hold}, d0={self.d0*1000:.0f}mm)'
        )

    # --- Callbacks ---

    def _enable_cb(self, request, response):
        self.enabled = request.data
        if self.enabled:
            # Capture current position as anchor for hold mode
            if self.q is not None:
                self.q_anchor = self.q.copy()
                self.get_logger().info(
                    f'APF enabled - anchor: '
                    f'[{", ".join(f"{v:.4f}" for v in self.q_anchor)}]'
                )
            else:
                self.get_logger().warn(
                    'APF enabled but no joint state yet - '
                    'anchor will be set on first reading'
                )
        else:
            self.executing = False
            self.q_anchor = None
        state = 'enabled (HOLD mode)' if self.enabled else 'disabled'
        response.success = True
        response.message = f'Controller {state}'
        self.get_logger().info(response.message)
        return response

    def _execute_waypoints_cb(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints loaded'
            return response
        if self.q is None:
            response.success = False
            response.message = 'No joint state received yet'
            return response

        self.current_waypoint_idx = 0
        self.executing = True
        self.enabled = True
        # Keep anchor for fallback after trajectory completes
        if self.q_anchor is None:
            self.q_anchor = self.q.copy()
        response.success = True
        response.message = (
            f'Executing {len(self.waypoints)} waypoints '
            f'(APF obstacle avoidance active)'
        )
        self.get_logger().info(response.message)
        return response

    def _stop_cb(self, request, response):
        self.executing = False
        # Update anchor to current position (hold here)
        if self.q is not None:
            self.q_anchor = self.q.copy()
        response.success = True
        response.message = 'Execution stopped, switching to HOLD mode'
        self.get_logger().info(response.message)
        return response

    def _joint_states_cb(self, msg):
        if len(msg.position) < 6:
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.q = np.array(
                [name_to_pos[n] for n in self.joint_names]
            )
        except KeyError:
            pass

    def _proximity_distance_cb(self, msg):
        """Process proximity sensor distance from ecan_driver.

        msg.data contains distance values in mm.
        Convert to meters and apply EMA filter.
        """
        if not msg.data:
            return
        # Use first element as the proximity distance
        raw_mm = msg.data[0]
        raw_m = raw_mm / 1000.0  # mm to meters

        alpha = self.ema_alpha
        prev = self.proximity_filtered
        if math.isinf(prev):
            self.proximity_filtered = raw_m
        else:
            self.proximity_filtered = alpha * raw_m + (1.0 - alpha) * prev
        self.proximity_distance = raw_mm

    def _proximity_range_cb(self, msg, sensor_id):
        """Process individual proximity sensor Range message."""
        if msg.range >= 0 and msg.range <= msg.max_range:
            self.proximity_ranges[sensor_id] = msg.range

    def _waypoints_cb(self, msg):
        """Parse waypoints from waypoint_manager.

        Format: [n_waypoints, n_joints, wp0_j0, wp0_j1, ..., wp1_j0, ...]
        """
        if len(msg.data) < 2:
            return
        n_waypoints = int(msg.data[0])
        n_joints = int(msg.data[1])
        if len(msg.data) < 2 + n_waypoints * n_joints:
            return

        waypoints = []
        for i in range(n_waypoints):
            start = 2 + i * n_joints
            wp = np.array(msg.data[start:start + n_joints])
            waypoints.append(wp)
        self.waypoints = waypoints

        # Auto-execute: start trajectory when waypoints first arrive
        if (self.auto_execute and not self._auto_execute_done
                and len(waypoints) > 0 and self.q is not None):
            self._auto_execute_done = True
            self.current_waypoint_idx = 0
            self.executing = True
            self.enabled = True
            if self.q_anchor is None:
                self.q_anchor = self.q.copy()
            self.get_logger().info(
                f'Auto-executing {len(waypoints)} waypoints'
            )

    # --- Helper Methods ---

    def _compute_sensor_vis(self, q, p_link3, R_link3):
        """Compute sensor world poses for visualization and force calculation."""
        sensor_vis = []
        for direction, cfg in SENSOR_CONFIGS.items():
            R_sw = R_link3 @ self.sensor_R_local[direction]
            p_s = p_link3 + R_link3 @ cfg['offset']
            sensor_vis.append((direction, p_s, R_sw))
        return sensor_vis

    def _compute_repulsive_forces(self, q, R_link3, sensor_vis):
        """Compute repulsive joint torques from proximity sensors.

        Returns:
            tau_rep: (6,) repulsive joint torques
            rep_arrows: list of (position, force) for visualization
        """
        tau_rep = np.zeros(6)
        rep_arrows = []

        r_i = max(self.proximity_filtered, self.r_min)
        if r_i >= self.d0:
            return tau_rep, rep_arrows

        J3_full = compute_jacobian(self.chain_link3, q[:3])
        J3_v = J3_full[:3, :]
        J3_w = J3_full[3:, :]

        for direction, p_sensor, R_sensor_world in sensor_vis:
            cfg = SENSOR_CONFIGS[direction]

            # Sensor normal in world frame
            n_i = -(R_sensor_world @ np.array([1.0, 0.0, 0.0]))

            # Repulsive magnitude
            alpha_i = (
                self.k_rep * (1.0 / r_i - 1.0 / self.d0) / (r_i * r_i)
            )
            F_rep_i = alpha_i * n_i

            # Sensor point Jacobian (3x6)
            offset_world = R_link3 @ cfg['offset']
            J_si = np.zeros((3, 6))
            for j in range(3):
                J_si[:, j] = (
                    J3_v[:, j] + np.cross(J3_w[:, j], offset_world)
                )

            tau_rep += J_si.T @ F_rep_i
            rep_arrows.append((p_sensor, F_rep_i))

        return tau_rep, rep_arrows

    def _integrate(self, q, tau):
        """Convert joint torque to velocity, apply limits, integrate.

        Returns:
            q_next: (6,) next joint positions
        """
        qdot = self.k_q * tau

        # Velocity saturation
        max_qdot = np.max(np.abs(qdot))
        if max_qdot > self.v_max:
            qdot = qdot * (self.v_max / max_qdot)

        # Joint limit damping
        for i in range(6):
            dist_low = q[i] - self.q_min[i]
            dist_high = self.q_max[i] - q[i]
            margin = self.q_limit_margin
            if dist_low < margin and qdot[i] < 0:
                scale = max(dist_low / margin, 0.0)
                qdot[i] *= scale
            if dist_high < margin and qdot[i] > 0:
                scale = max(dist_high / margin, 0.0)
                qdot[i] *= scale

        # Integrate position
        q_next = q + qdot * self.dt
        q_next = np.clip(q_next, self.q_min, self.q_max)
        return q_next

    # --- Control Loop ---

    def _control_loop(self):
        if self.q is None:
            return

        q = self.q.copy()

        # FK for current EE position
        frame_ee = compute_fk(self.chain_ee, q)
        p_ee, _ = kdl_frame_to_pos_rot(frame_ee)

        # FK for link3 (sensor mount)
        frame_link3 = compute_fk(self.chain_link3, q[:3])
        p_link3, R_link3 = kdl_frame_to_pos_rot(frame_link3)

        # Sensor world poses
        sensor_vis = self._compute_sensor_vis(q, p_link3, R_link3)

        # =============================================================
        # MODE 1: APF DISABLED (IDLE) - visualization only
        # =============================================================
        if not self.enabled:
            self._publish_markers(
                p_ee, None, np.zeros(3), [], sensor_vis, q
            )
            return

        # Lazy anchor initialization
        if self.q_anchor is None:
            self.q_anchor = q.copy()
            self.get_logger().info(
                f'Anchor set (deferred): '
                f'[{", ".join(f"{v:.4f}" for v in self.q_anchor)}]'
            )

        # =============================================================
        # MODE 3: APF ON + TRAJECTORY (EXECUTING)
        # =============================================================
        if self.executing:
            # Check if all waypoints completed
            if self.current_waypoint_idx >= len(self.waypoints):
                self.executing = False
                self.q_anchor = q.copy()
                self.get_logger().info(
                    'All waypoints completed! Switching to HOLD mode.'
                )
                self._publish_cmd(q)
                self._publish_markers(
                    p_ee, None, np.zeros(3), [], sensor_vis, q
                )
                return

            # Current target waypoint (joint space)
            target_q = self.waypoints[self.current_waypoint_idx]

            # Compute target EE position via FK
            frame_target = compute_fk(self.chain_ee, target_q)
            p_target, _ = kdl_frame_to_pos_rot(frame_target)

            # Check waypoint reached (joint space tolerance)
            joint_error = np.linalg.norm(target_q - q)
            if joint_error < self.waypoint_tolerance:
                self.get_logger().info(
                    f'Waypoint {self.current_waypoint_idx} reached '
                    f'(error: {joint_error:.4f} rad)'
                )
                self.current_waypoint_idx += 1
                self._publish_cmd(q)
                self._publish_markers(
                    p_ee, p_target, np.zeros(3), [], sensor_vis, q
                )
                return

            # Attractive force toward waypoint (task-space + joint-space)
            J_ee_full = compute_jacobian(self.chain_ee, q)
            J_ee_v = J_ee_full[:3, :]
            F_att = self.k_att * (p_target - p_ee)
            q_err = target_q - q
            tau_att_joint = 0.5 * self.k_att * q_err
            tau_att = J_ee_v.T @ F_att + tau_att_joint

            # Repulsive forces
            tau_rep, rep_arrows = self._compute_repulsive_forces(
                q, R_link3, sensor_vis
            )

            # Integrate and publish
            tau = tau_att + tau_rep
            q_next = self._integrate(q, tau)
            self._publish_cmd(q_next)
            self._publish_markers(
                p_ee, p_target, F_att, rep_arrows, sensor_vis, q
            )
            return

        # =============================================================
        # MODE 2: APF ON + NO TRAJECTORY (HOLD)
        # =============================================================
        # Attractive force: joint-space pull toward anchor
        tau_att_hold = self.k_hold * (self.q_anchor - q)

        # Repulsive forces
        tau_rep, rep_arrows = self._compute_repulsive_forces(
            q, R_link3, sensor_vis
        )

        tau = tau_att_hold + tau_rep

        # Skip if at anchor with no obstacles (avoid jitter)
        if np.max(np.abs(tau)) < 1e-6:
            # Compute anchor EE position for visualization
            frame_anchor = compute_fk(self.chain_ee, self.q_anchor)
            p_anchor, _ = kdl_frame_to_pos_rot(frame_anchor)
            self._publish_markers(
                p_ee, p_anchor, np.zeros(3), rep_arrows, sensor_vis, q
            )
            return

        q_next = self._integrate(q, tau)
        self._publish_cmd(q_next)

        # Anchor EE position for visualization
        frame_anchor = compute_fk(self.chain_ee, self.q_anchor)
        p_anchor, _ = kdl_frame_to_pos_rot(frame_anchor)
        F_att_vis = self.k_hold * (p_anchor - p_ee)
        self._publish_markers(
            p_ee, p_anchor, F_att_vis, rep_arrows, sensor_vis, q
        )

    # --- Publishers ---

    def _publish_cmd(self, q):
        msg = Float64MultiArray()
        msg.data = q.tolist()
        self.cmd_pub.publish(msg)

    def _publish_markers(self, p_ee, p_target, F_att, rep_arrows,
                         sensor_vis, q):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame_id = 'link0'
        marker_id = 0
        lifetime = Duration(sec=0, nanosec=200_000_000)

        # EE sphere (green)
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.ns = 'proximity'
        m.id = marker_id
        marker_id += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(p_ee[0])
        m.pose.position.y = float(p_ee[1])
        m.pose.position.z = float(p_ee[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.08
        m.color.g = 1.0
        m.color.a = 0.9
        m.lifetime = lifetime
        ma.markers.append(m)

        # Target/Anchor sphere (yellow)
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.ns = 'proximity'
        m.id = marker_id
        marker_id += 1
        if p_target is not None:
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(p_target[0])
            m.pose.position.y = float(p_target[1])
            m.pose.position.z = float(p_target[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.a = 0.9
            m.lifetime = lifetime
        else:
            m.action = Marker.DELETE
        ma.markers.append(m)

        # F_att arrow (blue)
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.ns = 'proximity'
        m.id = marker_id
        if np.linalg.norm(F_att) > 1e-6:
            m = self._make_arrow(
                stamp, frame_id, marker_id, p_ee, F_att,
                r=0.0, g=0.3, b=1.0, scale=0.3
            )
        else:
            m.action = Marker.DELETE
        marker_id += 1
        ma.markers.append(m)

        # F_rep arrows (red)
        for p_sensor, F_rep_i in rep_arrows:
            m = self._make_arrow(
                stamp, frame_id, marker_id, p_sensor, F_rep_i,
                r=1.0, g=0.0, b=0.0, scale=0.3
            )
            marker_id += 1
            ma.markers.append(m)

        # All waypoints as spheres
        for i, wp in enumerate(self.waypoints):
            frame_wp = compute_fk(self.chain_ee, wp)
            p_wp, _ = kdl_frame_to_pos_rot(frame_wp)

            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(p_wp[0])
            m.pose.position.y = float(p_wp[1])
            m.pose.position.z = float(p_wp[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.lifetime = lifetime

            if self.executing and i == self.current_waypoint_idx:
                m.color.r = 1.0
                m.color.g = 0.5
                m.color.a = 1.0
            elif self.executing and i < self.current_waypoint_idx:
                m.color.g = 1.0
                m.color.a = 0.5
            else:
                m.color.r = 0.5
                m.color.g = 0.5
                m.color.b = 0.5
                m.color.a = 0.7
            ma.markers.append(m)

        # Waypoint index text
        for i, wp in enumerate(self.waypoints):
            frame_wp = compute_fk(self.chain_ee, wp)
            p_wp, _ = kdl_frame_to_pos_rot(frame_wp)

            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.ns = 'waypoint_labels'
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = float(p_wp[0])
            m.pose.position.y = float(p_wp[1])
            m.pose.position.z = float(p_wp[2]) + 0.06
            m.pose.orientation.w = 1.0
            m.scale.z = 0.03
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.text = f'WP{i}'
            m.lifetime = lifetime
            ma.markers.append(m)

        # Sensor position spheres (cyan) with distance info
        for idx, (direction, p_s, R_sw) in enumerate(sensor_vis):
            # Sensor sphere
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.ns = 'proximity'
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(p_s[0])
            m.pose.position.y = float(p_s[1])
            m.pose.position.z = float(p_s[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.sensor_marker_size
            m.color.r = 0.0
            m.color.g = 0.8
            m.color.b = 1.0
            m.color.a = 0.8
            m.lifetime = lifetime
            ma.markers.append(m)

            # Distance text (if available)
            sensor_id = idx + 1
            if sensor_id in self.proximity_ranges:
                dist_m = self.proximity_ranges[sensor_id]
                dist_mm = dist_m * 1000.0
                m_text = Marker()
                m_text.header.stamp = stamp
                m_text.header.frame_id = frame_id
                m_text.ns = 'proximity'
                m_text.id = marker_id
                marker_id += 1
                m_text.type = Marker.TEXT_VIEW_FACING
                m_text.action = Marker.ADD
                m_text.pose.position.x = float(p_s[0])
                m_text.pose.position.y = float(p_s[1])
                m_text.pose.position.z = float(p_s[2]) + 0.05
                m_text.pose.orientation.w = 1.0
                m_text.scale.z = 0.02
                m_text.color.r = 1.0
                m_text.color.g = 1.0
                m_text.color.b = 0.0
                m_text.color.a = 1.0
                m_text.text = f'{direction}: {dist_mm:.0f}mm'
                m_text.lifetime = lifetime
                ma.markers.append(m_text)

                # Distance arrow
                if dist_m > 0 and dist_m < 0.5:
                    look_dir = -(R_sw @ np.array([1.0, 0.0, 0.0]))
                    end_point = p_s + look_dir * dist_m
                    m_arrow = Marker()
                    m_arrow.header.stamp = stamp
                    m_arrow.header.frame_id = frame_id
                    m_arrow.ns = 'proximity'
                    m_arrow.id = marker_id
                    marker_id += 1
                    m_arrow.type = Marker.ARROW
                    m_arrow.action = Marker.ADD
                    m_arrow.points = [
                        Point(
                            x=float(p_s[0]), y=float(p_s[1]),
                            z=float(p_s[2])
                        ),
                        Point(
                            x=float(end_point[0]), y=float(end_point[1]),
                            z=float(end_point[2])
                        ),
                    ]
                    m_arrow.scale.x = 0.01
                    m_arrow.scale.y = 0.02
                    m_arrow.scale.z = 0.0
                    m_arrow.color.r = 1.0
                    m_arrow.color.g = 0.5
                    m_arrow.color.b = 0.0
                    m_arrow.color.a = 0.8
                    m_arrow.lifetime = lifetime
                    ma.markers.append(m_arrow)

        # Status text
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.ns = 'proximity'
        m.id = marker_id
        marker_id += 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 1.5
        m.pose.orientation.w = 1.0
        m.scale.z = 0.05
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.lifetime = lifetime

        if self.executing:
            status = 'EXECUTING'
        elif self.enabled:
            status = 'HOLD'
        else:
            status = 'IDLE'
        prox_mm = self.proximity_distance if self.proximity_distance else 0.0
        wp_info = (
            f'{self.current_waypoint_idx}/{len(self.waypoints)}'
            if self.executing else '-'
        )
        m.text = (
            f'Status: {status} | '
            f'WP: {wp_info} | '
            f'Prox: {prox_mm:.0f}mm'
        )
        ma.markers.append(m)

        self.marker_pub.publish(ma)

    def _make_arrow(self, stamp, frame_id, marker_id,
                    origin, force, r, g, b, scale):
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.ns = 'proximity'
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD

        f_norm = np.linalg.norm(force)
        length = min(f_norm * scale, 0.5)

        p_start = Point(
            x=float(origin[0]),
            y=float(origin[1]),
            z=float(origin[2]),
        )
        direction = force / f_norm * length
        p_end = Point(
            x=float(origin[0] + direction[0]),
            y=float(origin[1] + direction[1]),
            z=float(origin[2] + direction[2]),
        )
        m.points = [p_start, p_end]
        m.scale.x = 0.02
        m.scale.y = 0.035
        m.scale.z = 0.0
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.9
        m.lifetime = Duration(sec=0, nanosec=200_000_000)
        return m


def main(args=None):
    rclpy.init(args=args)
    node = ProximityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
