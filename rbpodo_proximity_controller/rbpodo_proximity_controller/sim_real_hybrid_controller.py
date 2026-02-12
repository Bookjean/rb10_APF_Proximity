"""Hybrid controller that can work in simulation-only or real+sim mode.

In simulation mode:
- Reads real robot's initial pose and sets simulation robot to match
- Only sends commands to simulation robot, real robot stays still
- Uses proximity sensors for APF obstacle avoidance

In real mode:
- Sends commands to both simulation and real robot
- Both robots move in sync
- Uses proximity sensors for APF obstacle avoidance
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


class SimRealHybridController(Node):
    def __init__(self):
        super().__init__('sim_real_hybrid_controller')

        # Declare parameters
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('joint_names', JOINT_NAMES)
        self.declare_parameter('q_min', [-3.14] * 6)
        self.declare_parameter('q_max', [3.14] * 6)
        self.declare_parameter('k_att', 2.0)
        self.declare_parameter('k_rep', 5.0)
        self.declare_parameter('k_q', 1.0)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('q_limit_margin', 0.1)
        self.declare_parameter('goal_tolerance', 0.005)
        self.declare_parameter('d0', 0.10)  # 100mm (10cm)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('r_min', 0.02)
        self.declare_parameter('ee_offset', [0.0, -0.1153, 0.0])
        self.declare_parameter('waypoint_tolerance', 0.02)
        self.declare_parameter('sensor_fov', 1.7)
        self.declare_parameter('sensor_range', 0.2)
        self.declare_parameter('sensor_marker_size', 0.1)
        self.declare_parameter('simulation_mode', True)  # True = sim only, False = real+sim
        self.declare_parameter('real_robot_ip', '10.0.2.7')

        # Read parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.dt = self.get_parameter('dt').value
        self.joint_names = self.get_parameter('joint_names').value
        self.q_min = np.array(self.get_parameter('q_min').value)
        self.q_max = np.array(self.get_parameter('q_max').value)
        self.k_att = self.get_parameter('k_att').value
        self.k_rep = self.get_parameter('k_rep').value
        self.k_q = self.get_parameter('k_q').value
        self.v_max = self.get_parameter('v_max').value
        self.q_limit_margin = self.get_parameter('q_limit_margin').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.d0 = self.get_parameter('d0').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.r_min = self.get_parameter('r_min').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.sensor_fov = self.get_parameter('sensor_fov').value
        self.sensor_range = self.get_parameter('sensor_range').value
        self.sensor_marker_size = self.get_parameter('sensor_marker_size').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.real_robot_ip = self.get_parameter('real_robot_ip').value

        # Build KDL chains
        ee_offset = self.get_parameter('ee_offset').value
        self.chain_ee = build_ee_chain(ee_offset=ee_offset)
        self.chain_link3 = build_link3_chain()

        # Precompute sensor local rotation matrices
        self.sensor_R_local = {}
        for name, cfg in SENSOR_CONFIGS.items():
            self.sensor_R_local[name] = rpy_to_rotation_matrix(*cfg['rpy'])

        # State
        self.q_sim = None  # Simulation robot joint positions
        self.q_real = None  # Real robot joint positions (for initial pose sync)
        self.q_real_initial = None  # Real robot initial pose (set once)
        self.proximity_ranges = {}  # dict of sensor_id -> range (meters)
        self.proximity_filtered = float('inf')  # EMA filtered, in meters
        self.waypoints = []  # list of joint position arrays
        self.current_waypoint_idx = 0
        self.executing = False
        self.enabled = False
        self.initial_pose_synced = False

        # Subscribers
        # Simulation robot joint states (from joint_state_publisher)
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10
        )
        # Real robot joint states (from rbpodo_bringup, if available)
        self.create_subscription(
            JointState, '/rbpodo/joint_states', self._real_joint_states_cb, 10
        )
        # Subscribe to proximity sensor Range topics (only sensors 1-4)
        for i in range(1, 5):
            self.create_subscription(
                Range, f'/proximity_distance{i}',
                lambda msg, idx=i: self._proximity_range_cb(msg, idx), 10
            )
        # Silent - too verbose
        self.create_subscription(
            Float64MultiArray, '/waypoint_manager/waypoints',
            self._waypoints_cb, 10
        )

        # Publishers
        # Both simulation and real robot use /position_controllers/commands
        # In simulation mode: only sim's controller_manager subscribes
        # In real mode: both sim's and real's controller_manager subscribe
        # We publish once and both receive it (if in real mode)
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
        self.create_service(SetBool, '~/set_simulation_mode', self._set_simulation_mode_cb)

        # Control loop timer
        period = 1.0 / self.control_rate
        self.create_timer(period, self._control_loop)

        mode_str = 'SIMULATION ONLY' if self.simulation_mode else 'REAL + SIMULATION'
        self.get_logger().info(f'Hybrid controller initialized - Mode: {mode_str}')
        self.get_logger().info(f'APF: d0={self.d0*1000:.1f}mm, k_rep={self.k_rep}, k_q={self.k_q}')

    def _set_simulation_mode_cb(self, request, response):
        """Toggle between simulation-only and real+sim mode."""
        self.simulation_mode = request.data
        mode_str = 'SIMULATION ONLY' if self.simulation_mode else 'REAL + SIMULATION'
        self.get_logger().info(f'Mode changed to: {mode_str}')
        response.success = True
        response.message = f'Mode set to {mode_str}'
        return response

    def _joint_states_cb(self, msg):
        """Process simulation robot joint states."""
        if len(msg.position) < 6:
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.q_sim = np.array(
                [name_to_pos[n] for n in self.joint_names]
            )
        except KeyError:
            pass

    def _real_joint_states_cb(self, msg):
        """Process real robot joint states."""
        if len(msg.position) < 6:
            return
        # Map joint_1~6 to base~wrist3
        joint_map = {
            'joint_1': 'base',
            'joint_2': 'shoulder',
            'joint_3': 'elbow',
            'joint_4': 'wrist1',
            'joint_5': 'wrist2',
            'joint_6': 'wrist3',
        }
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            mapped_positions = []
            for joint_name in self.joint_names:
                # Find corresponding joint_* name
                for j1_name, mapped_name in joint_map.items():
                    if mapped_name == joint_name and j1_name in name_to_pos:
                        mapped_positions.append(name_to_pos[j1_name])
                        break
                else:
                    return  # Missing joint
            
            self.q_real = np.array(mapped_positions)
            
            # Set initial pose once
            if not self.initial_pose_synced and self.q_real is not None:
                self.q_real_initial = self.q_real.copy()
                if self.q_sim is not None:
                    # Sync simulation to real robot's initial pose
                    self._sync_sim_to_real_initial()
                self.initial_pose_synced = True
                self.get_logger().info(
                    f'Synced simulation to real robot initial pose: {self.q_real_initial}'
                )
        except Exception as e:
            self.get_logger().warn(f'Error processing real joint states: {e}')

    def _sync_sim_to_real_initial(self):
        """Set simulation robot to real robot's initial pose."""
        if self.q_real_initial is not None:
            msg = Float64MultiArray()
            msg.data = self.q_real_initial.tolist()
            self.sim_cmd_pub.publish(msg)
            self.q_sim = self.q_real_initial.copy()

    def _proximity_range_cb(self, msg, sensor_id):
        """Process individual proximity sensor Range message."""
        # Debug: log first message only to verify topic reception (silent)
        if not hasattr(self, '_sensor_msg_count'):
            self._sensor_msg_count = {}
        if sensor_id not in self._sensor_msg_count:
            self._sensor_msg_count[sensor_id] = 0
        self._sensor_msg_count[sensor_id] += 1
        
        # Handle invalid range values
        if msg.range < 0:
            if self._sensor_msg_count[sensor_id] <= 3:
                self.get_logger().warn(f'[SENSOR {sensor_id}] Invalid range: {msg.range}')
            return
        
        # Silent - 0.0mm values are filtered out in filter update
        
        # Convert from mm to meters
        # Proximity sensors publish in mm (range is typically 0-5000mm)
        # TOF sensors publish in meters (range is typically 0-0.5m)
        # Check if range is in mm (typically > 1.0) or already in meters
        if msg.range > 1.0 or msg.max_range > 10.0:
            # Likely in mm, convert to meters
            range_m = msg.range / 1000.0
            max_range_m = msg.max_range / 1000.0
        else:
            # Already in meters (like TOF sensors)
            range_m = msg.range
            max_range_m = msg.max_range
        
        # Clamp to valid range
        if range_m < 0:
            range_m = 0.0
        if range_m > max_range_m:
            range_m = max_range_m
        
        # Debug: log first reading only
        if not hasattr(self, '_sensor_debug_count'):
            self._sensor_debug_count = {}
        if sensor_id not in self._sensor_debug_count:
            self._sensor_debug_count[sensor_id] = 0
            self.get_logger().info(
                f'[SENSOR {sensor_id}] Connected: {msg.range:.1f}mm -> {range_m*1000:.1f}mm'
            )
            self._sensor_debug_count[sensor_id] += 1
        
        # Update EMA filter for minimum distance
        if sensor_id not in self.proximity_ranges:
            self.proximity_ranges[sensor_id] = range_m
        else:
            alpha = self.ema_alpha
            prev = self.proximity_ranges[sensor_id]
            self.proximity_ranges[sensor_id] = alpha * range_m + (1.0 - alpha) * prev
        
        # Update global filtered distance (minimum across all sensors)
        # Only use valid sensor values (ignore 0.0mm - indicates sensor not active or out of range)
        if self.proximity_ranges:
            # Filter out 0.0mm values (sensors not active or out of range)
            valid_ranges = {k: v for k, v in self.proximity_ranges.items() if v > 0.001}  # > 1mm
            
            if valid_ranges:
                min_range = min(valid_ranges.values())
                prev_filtered = self.proximity_filtered
                if math.isinf(prev_filtered):
                    self.proximity_filtered = min_range
                    valid_values_str = ', '.join([f's{i}={v*1000:.1f}mm' for i, v in sorted(valid_ranges.items())])
                    self.get_logger().info(
                        f'[FILTER] Initialized: {self.proximity_filtered*1000:.1f}mm from {valid_values_str}'
                    )
                else:
                    self.proximity_filtered = self.ema_alpha * min_range + (1.0 - self.ema_alpha) * prev_filtered
            # else: No valid sensors - keep previous filtered value (already handled by valid_ranges check)
            
            # Debug: log when sensor detects obstacle (only first detection)
            if self.proximity_filtered < self.d0:
                if not hasattr(self, '_log_counter'):
                    self._log_counter = 0
                self._log_counter += 1
                # Log only first detection
                if self._log_counter == 1:
                    self.get_logger().info(
                        f'[OBSTACLE] {self.proximity_filtered*1000:.1f}mm < {self.d0*1000:.1f}mm'
                    )

    def _waypoints_cb(self, msg):
        """Parse waypoints from waypoint_manager."""
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

    def _enable_cb(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self.executing = False
        state = 'enabled' if self.enabled else 'disabled'
        response.success = True
        response.message = f'Controller {state}'
        self.get_logger().info(response.message)
        return response

    def _execute_waypoints_cb(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints loaded'
            return response
        if self.q_sim is None:
            response.success = False
            response.message = 'No joint state received yet'
            return response

        self.current_waypoint_idx = 0
        self.executing = True
        self.enabled = True
        response.success = True
        mode_str = 'simulation' if self.simulation_mode else 'real+simulation'
        response.message = (
            f'Executing {len(self.waypoints)} waypoints in {mode_str} mode '
            f'(APF obstacle avoidance active)'
        )
        self.get_logger().info(response.message)
        return response

    def _stop_cb(self, request, response):
        self.executing = False
        response.success = True
        response.message = 'Execution stopped'
        self.get_logger().info(response.message)
        return response

    def _control_loop(self):
        if self.q_sim is None:
            return

        q = self.q_sim.copy()

        # FK for current EE position
        frame_ee = compute_fk(self.chain_ee, q)
        p_ee, _ = kdl_frame_to_pos_rot(frame_ee)

        # FK for link3 (sensor mount)
        frame_link3 = compute_fk(self.chain_link3, q[:3])
        p_link3, R_link3 = kdl_frame_to_pos_rot(frame_link3)

        # Sensor world poses for visualization
        sensor_vis = []
        for direction, cfg in SENSOR_CONFIGS.items():
            R_sw = R_link3 @ self.sensor_R_local[direction]
            p_s = p_link3 + R_link3 @ cfg['offset']
            sensor_vis.append((direction, p_s, R_sw))

        # Not enabled: just publish viz
        if not self.enabled:
            if not hasattr(self, '_disabled_logged'):
                self.get_logger().warn('[STATUS] Controller is DISABLED - no motion')
                self._disabled_logged = True
            self._publish_markers(
                p_ee, None, np.zeros(3), [], sensor_vis, q
            )
            return
        
        # If not executing waypoints but enabled, still react to proximity sensors
        # (for safety - always avoid obstacles even without waypoints)
        if not self.executing:
            # No waypoints, but check if proximity sensor detects obstacle
            # If obstacle detected, apply repulsive force to move away
            # Use filtered distance, but ensure it's at least r_min
            if math.isinf(self.proximity_filtered):
                # No sensor data yet
                self._publish_markers(
                    p_ee, None, np.zeros(3), [], sensor_vis, q
                )
                return
            
            r_i = max(self.proximity_filtered, self.r_min)
            
            # Debug: always log when obstacle is detected (first few times)
            if not hasattr(self, '_obstacle_log_counter'):
                self._obstacle_log_counter = 0
            self._obstacle_log_counter += 1
            
            if r_i < self.d0:
                # Log only first detection
                if self._obstacle_log_counter == 1:
                    self.get_logger().info(
                        f'[OBSTACLE] {r_i*1000:.1f}mm < {self.d0*1000:.1f}mm - applying force'
                    )
                
                # Obstacle detected - apply repulsive force even without waypoints
                # Compute repulsive forces
                tau_rep = np.zeros(6)
                rep_arrows = []
                
                frame_link3_temp = compute_fk(self.chain_link3, q[:3])
                p_link3_temp, R_link3_temp = kdl_frame_to_pos_rot(frame_link3_temp)
                
                J3_full_temp = compute_jacobian(self.chain_link3, q[:3])
                J3_v_temp = J3_full_temp[:3, :]
                J3_w_temp = J3_full_temp[3:, :]
                
                for direction, p_sensor, R_sensor_world in sensor_vis:
                    cfg = SENSOR_CONFIGS[direction]
                    n_i = -(R_sensor_world @ np.array([1.0, 0.0, 0.0]))
                    
                    # Repulsive force calculation
                    # Avoid division by zero when r_i is very small
                    if r_i < 1e-6:
                        # Very close - use maximum repulsive force
                        alpha_i = self.k_rep * 1000.0  # Large constant force
                    elif r_i >= self.d0:
                        # Beyond influence distance - no repulsive force
                        alpha_i = 0.0
                    else:
                        # Normal repulsive force calculation
                        alpha_i = (
                            self.k_rep * (1.0 / r_i - 1.0 / self.d0) / (r_i * r_i)
                        )
                    
                    F_rep_i = alpha_i * n_i
                    
                    # Silent - too verbose
                    
                    offset_world = R_link3_temp @ cfg['offset']
                    J_si = np.zeros((3, 6))
                    for j in range(3):
                        J_si[:, j] = (
                            J3_v_temp[:, j] + np.cross(J3_w_temp[:, j], offset_world)
                        )
                    
                    tau_rep += J_si.T @ F_rep_i
                    rep_arrows.append((p_sensor, F_rep_i))
                
                # Debug: log individual sensor contributions
                if self._obstacle_log_counter <= 5:
                    self.get_logger().info(
                        f'[DEBUG] r_i={r_i*1000:.1f}mm, d0={self.d0*1000:.1f}mm, '
                        f'k_rep={self.k_rep}, tau_rep before sum: {tau_rep}'
                    )
                
                # Apply only repulsive force (no attractive force)
                tau = tau_rep
                qdot = self.k_q * tau
                
                # Silent - too verbose
                force_mag = np.linalg.norm(tau_rep)
                qdot_mag_before = np.linalg.norm(qdot)
                
                # Velocity saturation
                max_qdot = np.max(np.abs(qdot))
                if max_qdot > self.v_max:
                    qdot = qdot * (self.v_max / max_qdot)
                
                qdot_mag_after = np.linalg.norm(qdot)
                
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
                
                # Silent - too verbose
                
                # Publish
                self._publish_cmd(q_next)
                self._publish_markers(
                    p_ee, None, np.zeros(3), rep_arrows, sensor_vis, q
                )
                return
            
            # No obstacle, no waypoints - just publish viz
            self._publish_markers(
                p_ee, None, np.zeros(3), [], sensor_vis, q
            )
            return

        # Check if all waypoints completed
        if self.current_waypoint_idx >= len(self.waypoints):
            self.executing = False
            self.get_logger().info('All waypoints completed!')
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

        # --- Jacobians ---
        J_ee_full = compute_jacobian(self.chain_ee, q)  # (6,6)
        J_ee_v = J_ee_full[:3, :]  # linear velocity part (3,6)

        J3_full = compute_jacobian(self.chain_link3, q[:3])  # (6,3)
        J3_v = J3_full[:3, :]  # (3,3)
        J3_w = J3_full[3:, :]  # (3,3)

        # --- Attractive force (task space) ---
        F_att = self.k_att * (p_target - p_ee)

        # --- Attractive force (joint space, blended) ---
        q_err = target_q - q
        tau_att_joint = 0.5 * self.k_att * q_err

        # --- Repulsive forces from proximity sensor ---
        tau_rep = np.zeros(6)
        rep_arrows = []

        # Use minimum proximity distance from all sensors
        r_i = max(self.proximity_filtered, self.r_min)
        if r_i < self.d0:  # 10cm (0.1m) 이내에서 척력 발생
            # 강한 척력 적용 (10cm 이내)
            for direction, p_sensor, R_sensor_world in sensor_vis:
                cfg = SENSOR_CONFIGS[direction]

                # Sensor normal in world frame
                n_i = -(R_sensor_world @ np.array([1.0, 0.0, 0.0]))

                # Repulsive magnitude (거리가 가까울수록 강한 척력)
                # r_i가 작을수록 alpha_i가 커짐
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
            
            # Debug log when repulsive force is applied
            if np.linalg.norm(tau_rep) > 0.01:
                self.get_logger().info(
                    f'APF repulsive force active: distance={r_i*1000:.1f}mm, '
                    f'force_magnitude={np.linalg.norm(tau_rep):.3f}'
                )
            
            # Debug log when repulsive force is applied
            if np.linalg.norm(tau_rep) > 0.01:
                self.get_logger().info(
                    f'APF repulsive force active: distance={r_i*1000:.1f}mm, '
                    f'force_magnitude={np.linalg.norm(tau_rep):.3f}'
                )

        # --- Total joint torque ---
        tau = J_ee_v.T @ F_att + tau_att_joint + tau_rep

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
        self._publish_markers(
            p_ee, p_target, F_att, rep_arrows, sensor_vis, q
        )

    def _publish_cmd(self, q):
        """Publish commands to simulation and/or real robot."""
        msg = Float64MultiArray()
        msg.data = q.tolist()
        
        # No debug log for commands (too frequent)
        
        # Publish once - in simulation mode, only sim's controller_manager receives it
        # In real mode, both sim's and real's controller_manager receive it
        # (both subscribe to /position_controllers/commands)
        self.cmd_pub.publish(msg)

    def _publish_markers(self, p_ee, p_target, F_att, rep_arrows,
                         sensor_vis, q):
        """Publish visualization markers."""
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

        # Target waypoint sphere (yellow)
        if p_target is not None:
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.ns = 'proximity'
            m.id = marker_id
            marker_id += 1
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
            ma.markers.append(m)

        # F_att arrow (blue)
        if np.linalg.norm(F_att) > 1e-6:
            m = self._make_arrow(
                stamp, frame_id, marker_id, p_ee, F_att,
                r=0.0, g=0.3, b=1.0, scale=0.3
            )
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

        # Sensor position spheres with distance info
        sensor_directions = ['N', 'S', 'E', 'W']
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
            sensor_id = idx + 1  # 1-4 for N, S, E, W
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
        m.scale.x = 0.01
        m.scale.y = 0.02
        m.scale.z = 0.0
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.9
        m.lifetime = Duration(sec=0, nanosec=200_000_000)
        return m


def main(args=None):
    rclpy.init(args=args)
    node = SimRealHybridController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

