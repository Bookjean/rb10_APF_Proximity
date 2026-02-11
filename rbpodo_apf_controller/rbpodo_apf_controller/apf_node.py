"""APF (Artificial Potential Field) controller node for RB10-1300E.

Moves EE (link6) toward a goal point while avoiding obstacles detected
by 4 ToF sensors mounted on link3, using sensor-point Jacobian (B method).
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from std_srvs.srv import SetBool

from rbpodo_apf_controller.kdl_helper import (
    build_ee_chain,
    build_link3_chain,
    compute_fk,
    compute_jacobian,
    kdl_frame_to_pos_rot,
    rpy_to_rotation_matrix,
)

# Sensor local frame orientations (RPY) and offsets relative to link3
# Matches URDF xacro: link3_tof_x=0.0, link3_tof_y=0.1484,
#   link3_tof_z=0.285075, link3_tof_offset=0.06
SENSOR_CONFIGS = {
    "N": {"offset": np.array([0.06, 0.1484, 0.285075]), "rpy": (0.0, 0.0, 0.0)},
    "S": {"offset": np.array([-0.06, 0.1484, 0.285075]), "rpy": (0.0, 0.0, math.pi)},
    "E": {"offset": np.array([0.0, 0.0884, 0.285075]), "rpy": (0.0, 0.0, -math.pi / 2)},
    "W": {"offset": np.array([0.0, 0.2084, 0.285075]), "rpy": (0.0, 0.0, math.pi / 2)},
}

JOINT_NAMES = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]


class APFController(Node):
    def __init__(self):
        super().__init__("apf_controller")

        # Declare parameters
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("dt", 0.01)
        self.declare_parameter("joint_names", JOINT_NAMES)
        self.declare_parameter("q_min", [-3.14] * 6)
        self.declare_parameter("q_max", [3.14] * 6)
        self.declare_parameter("k_att", 1.0)
        self.declare_parameter("k_rep", 5.0)
        self.declare_parameter("k_q", 0.5)
        self.declare_parameter("v_max", 0.5)
        self.declare_parameter("q_limit_margin", 0.1)
        self.declare_parameter("goal_tolerance", 0.005)
        self.declare_parameter("d0", 0.5)
        self.declare_parameter("ema_alpha", 0.3)
        self.declare_parameter("r_min", 0.02)
        self.declare_parameter("tof_fov", 0.174533)  # ~10 deg
        self.declare_parameter("tof_range", 0.5)
        self.declare_parameter("tof_marker_size", 0.06)  # sensor sphere diameter (m)
        self.declare_parameter("ee_offset", [0.0, -0.1153, 0.0])

        # Read parameters
        self.control_rate = self.get_parameter("control_rate").value
        self.dt = self.get_parameter("dt").value
        self.joint_names = self.get_parameter("joint_names").value
        self.q_min = np.array(self.get_parameter("q_min").value)
        self.q_max = np.array(self.get_parameter("q_max").value)
        self.k_att = self.get_parameter("k_att").value
        self.k_rep = self.get_parameter("k_rep").value
        self.k_q = self.get_parameter("k_q").value
        self.v_max = self.get_parameter("v_max").value
        self.q_limit_margin = self.get_parameter("q_limit_margin").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.d0 = self.get_parameter("d0").value
        self.ema_alpha = self.get_parameter("ema_alpha").value
        self.r_min = self.get_parameter("r_min").value
        self.tof_fov = self.get_parameter("tof_fov").value
        self.tof_range = self.get_parameter("tof_range").value
        self.tof_marker_size = self.get_parameter("tof_marker_size").value

        # Build KDL chains
        ee_offset = self.get_parameter("ee_offset").value
        self.chain_ee = build_ee_chain(ee_offset=ee_offset)
        self.chain_link3 = build_link3_chain()

        # Precompute sensor local rotation matrices
        self.sensor_R_local = {}
        for name, cfg in SENSOR_CONFIGS.items():
            self.sensor_R_local[name] = rpy_to_rotation_matrix(*cfg["rpy"])

        # State
        self.q = None  # Current joint positions (6,)
        self.p_goal = None  # Goal position in link0 frame (3,)
        self.tof_filtered = {d: float("inf") for d in SENSOR_CONFIGS}

        # Subscribers
        self.create_subscription(JointState, "/joint_states", self._joint_states_cb, 10)
        self.create_subscription(PointStamped, "/apf_goal", self._goal_cb, 10)
        for direction in SENSOR_CONFIGS:
            self.create_subscription(
                Range,
                f"/link3_tof_{direction}/range",
                lambda msg, d=direction: self._tof_cb(msg, d),
                10,
            )

        # Publishers
        self.cmd_pub = self.create_publisher(Float64MultiArray, "/position_controllers/commands", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/apf_debug/markers", 10)

        # APF enable/disable service
        self.apf_enabled = True
        self.create_service(SetBool, "~/enable", self._enable_cb)

        # Control loop timer
        period = 1.0 / self.control_rate
        self.create_timer(period, self._control_loop)

        self.get_logger().info("APF controller initialized")

    def _enable_cb(self, request, response):
        self.apf_enabled = request.data
        if not self.apf_enabled:
            self.p_goal = None
        state = "enabled" if self.apf_enabled else "disabled (goal cleared)"
        response.success = True
        response.message = f"APF {state}"
        self.get_logger().info(f"APF {state}")
        return response

    def _joint_states_cb(self, msg):
        if len(msg.position) < 6:
            return
        # Map by joint name to handle arbitrary ordering
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.q = np.array([name_to_pos[n] for n in self.joint_names])
        except KeyError:
            pass

    def _goal_cb(self, msg):
        self.p_goal = np.array([msg.point.x, msg.point.y, msg.point.z])
        if not self.apf_enabled:
            self.apf_enabled = True
            self.get_logger().info("APF auto-enabled by new goal")
        self.get_logger().info(
            f"Goal received: [{self.p_goal[0]:.3f}, {self.p_goal[1]:.3f}, {self.p_goal[2]:.3f}]"
        )

    def _tof_cb(self, msg, direction):
        raw = msg.range
        alpha = self.ema_alpha
        prev = self.tof_filtered[direction]
        if math.isinf(prev):
            self.tof_filtered[direction] = raw
        else:
            self.tof_filtered[direction] = alpha * raw + (1.0 - alpha) * prev

    def _control_loop(self):
        # Guard: need joint states
        if self.q is None:
            return

        q = self.q.copy()

        # --- FK ---
        frame_ee = compute_fk(self.chain_ee, q)
        p_ee, _ = kdl_frame_to_pos_rot(frame_ee)

        frame_link3 = compute_fk(self.chain_link3, q[:3])
        p_link3, R_link3 = kdl_frame_to_pos_rot(frame_link3)

        # --- Precompute sensor world poses (always, for FOV viz) ---
        sensor_vis = []  # [(direction, p_sensor, R_sensor_world), ...]
        for direction, cfg in SENSOR_CONFIGS.items():
            R_sw = R_link3 @ self.sensor_R_local[direction]
            p_s = p_link3 + R_link3 @ cfg["offset"]
            sensor_vis.append((direction, p_s, R_sw))

        # --- No goal or APF disabled: publish viz only ---
        if self.p_goal is None or not self.apf_enabled:
            self._publish_markers(p_ee, self.p_goal, np.zeros(3), [], sensor_vis)
            return

        # --- Check goal reached ---
        err = np.linalg.norm(self.p_goal - p_ee)
        if err < self.goal_tolerance:
            self._publish_cmd(q)
            self._publish_markers(p_ee, self.p_goal, np.zeros(3), [], sensor_vis)
            return

        # --- Jacobians ---
        J_ee_full = compute_jacobian(self.chain_ee, q)  # (6,6)
        J_ee_v = J_ee_full[:3, :]  # linear velocity part (3,6)

        J3_full = compute_jacobian(self.chain_link3, q[:3])  # (6,3)
        J3_v = J3_full[:3, :]  # (3,3)
        J3_w = J3_full[3:, :]  # (3,3)

        # --- Attractive force ---
        F_att = self.k_att * (self.p_goal - p_ee)

        # --- Repulsive forces ---
        tau_rep = np.zeros(6)
        rep_arrows = []  # For visualization: list of (position, force_vector)

        for direction, p_sensor, R_sensor_world in sensor_vis:
            cfg = SENSOR_CONFIGS[direction]
            r_raw = self.tof_filtered[direction]
            r_i = max(r_raw, self.r_min)

            if r_i > self.d0:
                continue

            # Sensor normal in world frame: -(R_sensor_world @ [1,0,0])
            n_i = -(R_sensor_world @ np.array([1.0, 0.0, 0.0]))

            # Repulsive magnitude
            alpha_i = self.k_rep * (1.0 / r_i - 1.0 / self.d0) / (r_i * r_i)
            F_rep_i = alpha_i * n_i

            # Sensor point Jacobian (3x6, padded with zeros for joints 4-6)
            offset_world = R_link3 @ cfg["offset"]
            J_si = np.zeros((3, 6))
            for j in range(3):
                J_si[:, j] = J3_v[:, j] + np.cross(J3_w[:, j], offset_world)

            # Accumulate repulsive torque
            tau_rep += J_si.T @ F_rep_i
            rep_arrows.append((p_sensor, F_rep_i))

        # --- Total joint torque ---
        tau = J_ee_v.T @ F_att + tau_rep

        # --- Joint velocity ---
        qdot = self.k_q * tau

        # --- Velocity saturation ---
        max_qdot = np.max(np.abs(qdot))
        if max_qdot > self.v_max:
            qdot = qdot * (self.v_max / max_qdot)

        # --- Joint limit damping ---
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

        # --- Integrate position ---
        q_next = q + qdot * self.dt
        q_next = np.clip(q_next, self.q_min, self.q_max)

        # --- Publish ---
        self._publish_cmd(q_next)
        self._publish_markers(p_ee, self.p_goal, F_att, rep_arrows, sensor_vis)

    def _publish_cmd(self, q):
        msg = Float64MultiArray()
        msg.data = q.tolist()
        self.cmd_pub.publish(msg)

    def _publish_markers(self, p_ee, p_goal, F_att, rep_arrows, sensor_vis):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame_id = "link0"
        marker_id = 0
        lifetime = Duration(sec=0, nanosec=200_000_000)

        # EE sphere (green)
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.ns = "apf"
        m.id = marker_id; marker_id += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(p_ee[0]), float(p_ee[1]), float(p_ee[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.08
        m.color.g = 1.0; m.color.a = 0.9
        m.lifetime = lifetime
        ma.markers.append(m)

        # Goal sphere (yellow) — delete when no goal
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.ns = "apf"
        m.id = marker_id; marker_id += 1
        if p_goal is not None:
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(p_goal[0]), float(p_goal[1]), float(p_goal[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.r = 1.0; m.color.g = 1.0; m.color.a = 0.9
            m.lifetime = lifetime
        else:
            m.action = Marker.DELETE
        ma.markers.append(m)

        # F_att arrow (blue) from EE — delete when no force
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.ns = "apf"
        m.id = marker_id
        if np.linalg.norm(F_att) > 1e-6:
            m = self._make_arrow(stamp, frame_id, marker_id, p_ee, F_att,
                                 r=0.0, g=0.3, b=1.0, scale=0.3)
        else:
            m.action = Marker.DELETE
        marker_id += 1
        ma.markers.append(m)

        # F_rep arrows (red) from sensor positions
        for p_sensor, F_rep_i in rep_arrows:
            m = self._make_arrow(stamp, frame_id, marker_id, p_sensor, F_rep_i,
                                 r=1.0, g=0.0, b=0.0, scale=0.3)
            marker_id += 1
            ma.markers.append(m)

        # Sensor position spheres (cyan) + FOV cones
        half_angle = self.tof_fov / 2.0
        r_cone = self.tof_range * math.tan(half_angle)

        for _direction, p_s, R_sw in sensor_vis:
            # Sensor sphere
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.ns = "apf"
            m.id = marker_id; marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(p_s[0])
            m.pose.position.y = float(p_s[1])
            m.pose.position.z = float(p_s[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.tof_marker_size
            m.color.r = 0.0; m.color.g = 0.8; m.color.b = 1.0; m.color.a = 0.8
            m.lifetime = lifetime
            ma.markers.append(m)

            # FOV cone wireframe (LINE_LIST)
            d_hat = R_sw[:, 0]   # sensor look direction (local +X)
            up = R_sw[:, 1]      # local +Y
            right = R_sw[:, 2]   # local +Z
            p_far = p_s + self.tof_range * d_hat
            corners = [
                p_far + r_cone * up + r_cone * right,
                p_far + r_cone * up - r_cone * right,
                p_far - r_cone * up - r_cone * right,
                p_far - r_cone * up + r_cone * right,
            ]

            pts = []
            p0 = Point(x=float(p_s[0]), y=float(p_s[1]), z=float(p_s[2]))
            # 4 edge lines: sensor → each corner
            for c in corners:
                pts.append(p0)
                pts.append(Point(x=float(c[0]), y=float(c[1]), z=float(c[2])))
            # 4 far-end lines connecting corners
            for i in range(4):
                j = (i + 1) % 4
                pts.append(Point(x=float(corners[i][0]), y=float(corners[i][1]), z=float(corners[i][2])))
                pts.append(Point(x=float(corners[j][0]), y=float(corners[j][1]), z=float(corners[j][2])))

            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.ns = "apf"
            m.id = marker_id; marker_id += 1
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.points = pts
            m.scale.x = 0.005  # line width
            m.color.r = 0.0; m.color.g = 0.8; m.color.b = 1.0; m.color.a = 0.4
            m.lifetime = lifetime
            m.pose.orientation.w = 1.0
            ma.markers.append(m)

        self.marker_pub.publish(ma)

    def _make_arrow(self, stamp, frame_id, marker_id, origin, force, r, g, b, scale):
        """Create an arrow marker from origin in the direction of force."""
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.ns = "apf"
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD

        f_norm = np.linalg.norm(force)
        length = min(f_norm * scale, 0.5)  # Cap arrow length

        p_start = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
        direction = force / f_norm * length
        p_end = Point(
            x=float(origin[0] + direction[0]),
            y=float(origin[1] + direction[1]),
            z=float(origin[2] + direction[2]),
        )
        m.points = [p_start, p_end]
        m.scale.x = 0.02  # shaft diameter
        m.scale.y = 0.035  # head diameter
        m.scale.z = 0.0
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.9
        m.lifetime = Duration(sec=0, nanosec=200_000_000)
        return m


def main(args=None):
    rclpy.init(args=args)
    node = APFController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
