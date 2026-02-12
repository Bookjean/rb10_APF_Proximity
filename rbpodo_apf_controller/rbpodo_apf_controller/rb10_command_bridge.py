"""Bridge node to convert ROS2 position commands to RB10 API calls.

For RB10 simulation mode, subscribes to /position_controllers/commands
and sends commands to RB10 robot using API.
Also publishes joint states from RB10 program simulation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import time

# Try to import RB10 API
try:
    from api.cobot import (
        ToCB,
        DisConnectToCB,
        CobotInit,
        SetProgramMode,
        PG_MODE,
        MoveJ,
        IsDataSockConnect,
        IsCommandSockConnect,
        GetCurrentSplitedJoint,
    )
    API_AVAILABLE = True
except ImportError:
    API_AVAILABLE = False
    print("[WARN] RB10 API not available. RB10 command bridge will not work.")


class RB10CommandBridge(Node):
    def __init__(self):
        super().__init__("rb10_command_bridge")
        
        # Declare parameters
        self.declare_parameter("robot_ip", "192.168.111.50")  # Changed to match robotory_rb10_ros2 default
        self.declare_parameter("simulation_mode", True)
        
        robot_ip = self.get_parameter("robot_ip").value
        simulation_mode = self.get_parameter("simulation_mode").value
        
        if not API_AVAILABLE:
            self.get_logger().error("RB10 API not available. Cannot initialize bridge.")
            return
        
        self.robot_ip = robot_ip
        self.use_simulation = simulation_mode
        
        # Initialize RB10 connection
        self.get_logger().info(f"Connecting to RB10 at {robot_ip}")
        try:
            ToCB(robot_ip)
            CobotInit()
            SetProgramMode(PG_MODE.SIMULATION if simulation_mode else PG_MODE.REAL)
            self.get_logger().info(f"Connected to RB10 (mode: {'SIMULATION' if simulation_mode else 'REAL'})")
            self.connected = True
        except Exception as e:
            self.get_logger().error(f"Failed to connect to RB10: {e}")
            self.connected = False
            return
        
        # Subscribe to position commands
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            "/position_controllers/commands",
            self._command_cb,
            10
        )
        
        # Publish joint states from RB10
        self.joint_state_pub = self.create_publisher(JointState, "/rbpodo/joint_states", 10)
        
        # Timer to publish joint states at 50 Hz
        self.create_timer(0.02, self._publish_joint_states)
        
        self.get_logger().info("RB10 Command Bridge initialized. Listening to /position_controllers/commands")
        self.get_logger().info("Publishing joint states to /rbpodo/joint_states")
    
    def _command_cb(self, msg):
        """Convert ROS2 joint position command to RB10 MoveJ."""
        if not self.connected or not API_AVAILABLE:
            return
        
        if len(msg.data) < 6:
            self.get_logger().warn(f"Received command with {len(msg.data)} joints, expected 6")
            return
        
        # Convert from radians to degrees
        joints_deg = [math.degrees(float(j)) for j in msg.data[:6]]
        
        # Send command to RB10
        try:
            if not IsDataSockConnect() or not IsCommandSockConnect():
                self.get_logger().warn("RB10 sockets not connected")
                return
            
            # Use MoveJ with default velocity and acceleration
            # Format: MoveJ(j1, j2, j3, j4, j5, j6, vel, acc)
            result = MoveJ(
                joints_deg[0], joints_deg[1], joints_deg[2],
                joints_deg[3], joints_deg[4], joints_deg[5],
                50.0,  # velocity (%)
                25.0   # acceleration (%)
            )
            
            if not result:
                self.get_logger().warn("MoveJ command failed")
        except Exception as e:
            self.get_logger().error(f"Error sending command to RB10: {e}")
    
    def _publish_joint_states(self):
        """Publish current joint states from RB10."""
        if not self.connected or not API_AVAILABLE:
            return
        
        try:
            if not IsDataSockConnect():
                return
            
            # Get current joint positions from RB10 (in degrees)
            jnts = GetCurrentSplitedJoint()
            if jnts and len(jnts) >= 6:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
                # Convert from degrees to radians
                msg.position = [math.radians(float(j)) for j in jnts[:6]]
                self.joint_state_pub.publish(msg)
        except Exception as e:
            # Silently ignore errors (RB10 might not be ready yet)
            pass
    
    def destroy_node(self):
        if API_AVAILABLE and self.connected:
            try:
                DisConnectToCB()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    if not API_AVAILABLE:
        print("[ERROR] RB10 API not available. Cannot run command bridge.")
        return
    
    node = RB10CommandBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

