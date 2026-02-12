"""Trajectory manager node for APF controller.

Loads waypoints from YAML file and publishes them sequentially to /apf_goal.
Supports point-to-point trajectory execution with configurable wait times.
"""

import os
import yaml
import time
import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from rbpodo_apf_controller.kdl_helper import build_ee_chain, compute_fk, kdl_frame_to_pos_rot


class TrajectoryManager(Node):
    def __init__(self):
        super().__init__("trajectory_manager")
        
        # Declare parameters
        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("default_velocity", 0.5)
        self.declare_parameter("default_wait_time", 2.0)
        self.declare_parameter("goal_tolerance", 0.01)  # 1cm tolerance
        self.declare_parameter("ee_offset", [0.0, -0.1153, 0.0])
        self.declare_parameter("interactive_mode", False)  # Enable interactive menu
        
        # Read parameters
        self.trajectory_file = self.get_parameter("trajectory_file").value
        self.default_velocity = self.get_parameter("default_velocity").value
        self.default_wait_time = self.get_parameter("default_wait_time").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        ee_offset = self.get_parameter("ee_offset").value
        
        # Build KDL chain for FK
        self.chain_ee = build_ee_chain(ee_offset=ee_offset)
        self.joint_names = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
        
        # State
        self.positions = {}  # dict of position_name -> [x, y, z]
        self.trajectories = {}  # dict of trajectory_name -> [position_name, ...]
        self.current_trajectory = None
        self.current_trajectory_name = None
        self.current_waypoint_idx = 0
        self.current_goal = None  # Current goal position [x, y, z]
        self.executing = False
        self.q = None  # Current joint positions
        
        # Subscribers
        # Subscribe to both /joint_states (for rviz_sim) and /rbpodo/joint_states (for real robot)
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_states_cb, 10
        )
        self.real_joint_state_sub = self.create_subscription(
            JointState, "/rbpodo/joint_states", self._real_joint_states_cb, 10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(PointStamped, "/apf_goal", 10)
        self.status_pub = self.create_publisher(String, "/trajectory_manager/status", 10)
        
        # Services
        self.create_service(Trigger, "~/load_trajectory", self._load_trajectory_cb)
        self.create_service(Trigger, "~/execute_trajectory", self._execute_trajectory_cb)
        self.create_service(Trigger, "~/stop_trajectory", self._stop_trajectory_cb)
        self.create_service(SetBool, "~/set_trajectory_file", self._set_trajectory_file_cb)
        
        # Timer for trajectory execution
        self.timer = self.create_timer(0.1, self._trajectory_loop)
        
        # Interactive mode
        self.interactive_mode = self.get_parameter("interactive_mode").value
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Trajectory Manager initialized")
        if self.trajectory_file:
            self.get_logger().info(f"Default trajectory file: {self.trajectory_file}")
            self._load_trajectory_file(self.trajectory_file)
        else:
            self.get_logger().info("No trajectory file specified. Use service to load.")
        
        if self.interactive_mode:
            self.get_logger().info("Interactive mode enabled")
            # Start interactive menu in a separate thread
            self.interactive_thread = threading.Thread(target=self._interactive_menu, daemon=True)
            self.interactive_thread.start()
        
        self.get_logger().info("=" * 60)
    
    def _load_trajectory_file(self, yaml_file):
        """Load positions and trajectories from YAML file."""
        if not os.path.exists(yaml_file):
            self.get_logger().error(f"YAML file not found: {yaml_file}")
            return False
        
        try:
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'positions' not in data:
                self.get_logger().error("'positions' key not found in YAML file")
                return False
            
            self.positions = data['positions']
            self.get_logger().info(f"Loaded {len(self.positions)} positions from {yaml_file}")
            
            # Print positions
            for name, pos in self.positions.items():
                if isinstance(pos, list) and len(pos) >= 3:
                    self.get_logger().info(f"  - {name}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            
            # Load trajectories if available
            if 'trajectories' in data:
                self.trajectories = data['trajectories']
                self.get_logger().info(f"\nLoaded {len(self.trajectories)} trajectories:")
                for name, waypoints in self.trajectories.items():
                    self.get_logger().info(f"  - {name}: {waypoints}")
            else:
                self.trajectories = {}
                self.get_logger().info("\nNo trajectories defined in YAML")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error loading YAML file: {e}")
            return False
    
    def _load_trajectory_cb(self, request, response):
        """Service callback to load trajectory file."""
        if not self.trajectory_file:
            response.success = False
            response.message = "No trajectory file specified. Set trajectory_file parameter or use set_trajectory_file service."
            return response
        
        if self._load_trajectory_file(self.trajectory_file):
            response.success = True
            response.message = f"Trajectory file loaded: {self.trajectory_file}"
        else:
            response.success = False
            response.message = f"Failed to load trajectory file: {self.trajectory_file}"
        
        return response
    
    def _set_trajectory_file_cb(self, request, response):
        """Service callback to set trajectory file path."""
        # The file path should be in request.message or we need to modify the service
        # For now, we'll use a workaround: store path in a topic or use parameter
        response.success = False
        response.message = "Use trajectory_file parameter or load_trajectory service with file path"
        return response
    
    def _joint_states_cb(self, msg):
        """Process joint states from /joint_states (rviz_sim mode)."""
        if len(msg.position) < 6:
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.q = [name_to_pos[n] for n in self.joint_names]
        except KeyError:
            pass
    
    def _real_joint_states_cb(self, msg):
        """Process joint states from /rbpodo/joint_states (real robot mode)."""
        if len(msg.position) < 6:
            return
        # Map joint_1~6 to base~wrist3
        joint_map = {
            "joint_1": "base",
            "joint_2": "shoulder",
            "joint_3": "elbow",
            "joint_4": "wrist1",
            "joint_5": "wrist2",
            "joint_6": "wrist3",
        }
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            mapped_positions = []
            for joint_name in self.joint_names:
                rb10_name = [k for k, v in joint_map.items() if v == joint_name][0]
                mapped_positions.append(name_to_pos[rb10_name])
            self.q = mapped_positions
        except (KeyError, IndexError):
            pass
    
    def _execute_trajectory_cb(self, request, response):
        """Service callback to execute a trajectory."""
        if not self.trajectories:
            response.success = False
            response.message = "No trajectories loaded. Load trajectory file first."
            return response
        
        # Execute the first trajectory (can be extended to select by name)
        trajectory_name = list(self.trajectories.keys())[0]
        
        if trajectory_name not in self.trajectories:
            response.success = False
            response.message = f"Trajectory '{trajectory_name}' not found"
            return response
        
        if self.executing:
            response.success = False
            response.message = "Trajectory already executing. Stop current trajectory first."
            return response
        
        self.current_trajectory = self.trajectories[trajectory_name]
        self.current_trajectory_name = trajectory_name
        self.current_waypoint_idx = 0
        self.current_goal = None
        self.executing = True
        
        response.success = True
        response.message = f"Started trajectory '{trajectory_name}' with {len(self.current_trajectory)} waypoints"
        self.get_logger().info(response.message)
        
        return response
    
    def _stop_trajectory_cb(self, request, response):
        """Service callback to stop trajectory execution."""
        if not self.executing:
            response.success = False
            response.message = "No trajectory is currently executing"
            return response
        
        self.executing = False
        self.current_trajectory = None
        self.current_waypoint_idx = 0
        
        response.success = True
        response.message = "Trajectory execution stopped"
        self.get_logger().info(response.message)
        
        return response
    
    def _trajectory_loop(self):
        """Main trajectory execution loop."""
        if not self.executing or not self.current_trajectory:
            return
        
        if self.current_waypoint_idx >= len(self.current_trajectory):
            # Trajectory completed
            self.executing = False
            self.current_trajectory = None
            self.current_trajectory_name = None
            self.current_waypoint_idx = 0
            self.current_goal = None
            self.get_logger().info("Trajectory execution completed!")
            
            # Publish status
            status_msg = String()
            status_msg.data = "completed"
            self.status_pub.publish(status_msg)
            return
        
        # Get current waypoint
        position_name = self.current_trajectory[self.current_waypoint_idx]
        
        if position_name not in self.positions:
            self.get_logger().error(f"Position '{position_name}' not found in loaded positions")
            self.executing = False
            return
        
        pos = self.positions[position_name]
        
        # Check if position is valid (at least 3 values for x, y, z)
        if not isinstance(pos, list) or len(pos) < 3:
            self.get_logger().error(f"Position '{position_name}' must have at least 3 values [x, y, z]")
            self.executing = False
            return
        
        goal_pos = [float(pos[0]), float(pos[1]), float(pos[2])]
        
        # Check if we need to publish a new goal
        if self.current_goal is None or self.current_goal != goal_pos:
            # Publish new goal
            goal_msg = PointStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = "link0"
            goal_msg.point.x = goal_pos[0]
            goal_msg.point.y = goal_pos[1]
            goal_msg.point.z = goal_pos[2]
            
            self.goal_pub.publish(goal_msg)
            self.current_goal = goal_pos
            
            # Log first time only
            if self.current_waypoint_idx == 0:
                self.get_logger().info(f"Executing trajectory '{self.current_trajectory_name}': {len(self.current_trajectory)} waypoints")
            
            self.get_logger().info(
                f"[{self.current_waypoint_idx + 1}/{len(self.current_trajectory)}] "
                f"Published goal: {position_name} -> [{goal_pos[0]:.3f}, {goal_pos[1]:.3f}, {goal_pos[2]:.3f}]"
            )
            
            # Reset timer
            if not hasattr(self, '_waypoint_start_time'):
                self._waypoint_start_time = {}
            self._waypoint_start_time[self.current_waypoint_idx] = time.time()
        
        # Check if goal is reached
        goal_reached = False
        if self.q is not None and len(self.q) == 6:
            try:
                # Compute current EE position
                frame_ee = compute_fk(self.chain_ee, self.q)
                p_ee, _ = kdl_frame_to_pos_rot(frame_ee)
                
                # Check distance to goal
                error = math.sqrt(
                    (p_ee[0] - goal_pos[0])**2 +
                    (p_ee[1] - goal_pos[1])**2 +
                    (p_ee[2] - goal_pos[2])**2
                )
                
                if error < self.goal_tolerance:
                    goal_reached = True
            except Exception as e:
                # If FK fails, use timeout-based approach
                pass
        
        # Check timeout as fallback
        waypoint_key = self.current_waypoint_idx
        if waypoint_key in self._waypoint_start_time:
            elapsed = time.time() - self._waypoint_start_time[waypoint_key]
            if elapsed >= self.default_wait_time:
                goal_reached = True
        
        if goal_reached:
            self.get_logger().info(f"  → Reached waypoint {self.current_waypoint_idx + 1}: {position_name}")
            # Move to next waypoint
            self.current_waypoint_idx += 1
            self.current_goal = None  # Reset to publish next goal
            
            # Publish status
            status_msg = String()
            if self.current_waypoint_idx < len(self.current_trajectory):
                status_msg.data = f"waypoint_{self.current_waypoint_idx}/{len(self.current_trajectory)}"
            else:
                status_msg.data = "completed"
            self.status_pub.publish(status_msg)
    
    def _interactive_menu(self):
        """Interactive menu for trajectory selection and execution.
        
        Note: This menu requires stdin access. If running via ros2 launch,
        use ROS2 services instead:
        - ros2 service call /trajectory_manager/load_trajectory std_srvs/srv/Trigger
        - ros2 service call /trajectory_manager/execute_trajectory std_srvs/srv/Trigger
        - ros2 service call /trajectory_manager/stop_trajectory std_srvs/srv/Trigger
        """
        import sys
        import time
        
        # Wait a bit for ROS2 to initialize
        time.sleep(2.0)
        
        # Check if stdin is available (not available when launched via ros2 launch)
        if not sys.stdin.isatty():
            self.get_logger().warn("=" * 60)
            self.get_logger().warn("Interactive mode disabled: stdin not available")
            self.get_logger().warn("Use ROS2 services instead:")
            self.get_logger().warn("  ros2 service call /trajectory_manager/load_trajectory std_srvs/srv/Trigger")
            self.get_logger().warn("  ros2 service call /trajectory_manager/execute_trajectory std_srvs/srv/Trigger")
            self.get_logger().warn("  ros2 service call /trajectory_manager/stop_trajectory std_srvs/srv/Trigger")
            self.get_logger().warn("=" * 60)
            return
        
        while rclpy.ok():
            print("\n" + "=" * 60)
            print("Trajectory Manager - Interactive Menu")
            print("=" * 60)
            print("  1: Load trajectory file")
            print("  2: List loaded positions")
            print("  3: List loaded trajectories")
            print("  4: Execute trajectory")
            print("  5: Stop trajectory")
            print("  0: Exit menu (node continues running)")
            print("-" * 60)
            
            try:
                choice = sys.stdin.readline().strip()
                if not choice:
                    time.sleep(0.1)
                    continue
            except (EOFError, KeyboardInterrupt):
                print("\nExiting interactive menu...")
                break
            
            if choice == "0":
                print("Exiting interactive menu. Node continues running.")
                break
            
            elif choice == "1":
                # Load trajectory file
                if self.trajectory_file:
                    default_file = self.trajectory_file
                else:
                    # Try to find example file
                    try:
                        from ament_index_python.packages import get_package_share_directory
                        pkg = get_package_share_directory("rbpodo_apf_controller")
                        default_file = os.path.join(pkg, "config", "example_trajectory.yaml")
                        if not os.path.exists(default_file):
                            default_file = ""
                    except:
                        default_file = ""
                
                file_path = input(f"Enter YAML file path [{default_file}]: ").strip()
                if not file_path:
                    file_path = default_file
                
                if not file_path:
                    print("ERROR: No file path provided")
                    continue
                
                if self._load_trajectory_file(file_path):
                    self.trajectory_file = file_path
                    print(f"✓ Trajectory file loaded: {file_path}")
                else:
                    print(f"✗ Failed to load trajectory file: {file_path}")
            
            elif choice == "2":
                # List positions
                if not self.positions:
                    print("No positions loaded. Load trajectory file first (option 1).")
                else:
                    print(f"\nLoaded {len(self.positions)} positions:")
                    for name, pos in self.positions.items():
                        if isinstance(pos, list) and len(pos) >= 3:
                            print(f"  - {name}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
                        else:
                            print(f"  - {name}: {pos}")
            
            elif choice == "3":
                # List trajectories
                if not self.trajectories:
                    print("No trajectories loaded. Load trajectory file first (option 1).")
                else:
                    print(f"\nLoaded {len(self.trajectories)} trajectories:")
                    for name, waypoints in self.trajectories.items():
                        print(f"  - {name}: {waypoints}")
            
            elif choice == "4":
                # Execute trajectory
                if not self.trajectories:
                    print("ERROR: No trajectories loaded. Load trajectory file first (option 1).")
                    continue
                
                print(f"\nAvailable trajectories: {list(self.trajectories.keys())}")
                trajectory_name = input("Enter trajectory name: ").strip()
                
                if not trajectory_name:
                    # Use first trajectory as default
                    trajectory_name = list(self.trajectories.keys())[0]
                    print(f"Using default trajectory: {trajectory_name}")
                
                if trajectory_name not in self.trajectories:
                    print(f"ERROR: Trajectory '{trajectory_name}' not found")
                    continue
                
                if self.executing:
                    print("ERROR: Trajectory already executing. Stop current trajectory first (option 5).")
                    continue
                
                # Execute
                self.current_trajectory = self.trajectories[trajectory_name]
                self.current_trajectory_name = trajectory_name
                self.current_waypoint_idx = 0
                self.current_goal = None
                self.executing = True
                
                print(f"✓ Started trajectory '{trajectory_name}' with {len(self.current_trajectory)} waypoints")
            
            elif choice == "5":
                # Stop trajectory
                if not self.executing:
                    print("No trajectory is currently executing.")
                else:
                    self.executing = False
                    self.current_trajectory = None
                    self.current_trajectory_name = None
                    self.current_waypoint_idx = 0
                    self.current_goal = None
                    print("✓ Trajectory execution stopped")
            
            else:
                print("Invalid option. Please try again.")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

