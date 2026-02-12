"""Joint state mapper to convert joint_1~6 to base~wrist3 naming.

Why:
- The real robot often publishes `/joint_states` with names `joint_1`~`joint_6`.
- Our URDF expects `base`~`wrist3`.

Important:
- We publish mapped joint states to **/joint_states_mapped** to avoid mixing two
  different name conventions on `/joint_states` (which causes RViz pose "jumping").
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMapper(Node):
    def __init__(self):
        super().__init__('joint_state_mapper')
        
        # Joint name mapping: joint_1~6 -> base~wrist3
        self.joint_name_map = {
            'joint_1': 'base',
            'joint_2': 'shoulder',
            'joint_3': 'elbow',
            'joint_4': 'wrist1',
            'joint_5': 'wrist2',
            'joint_6': 'wrist3',
        }
        
        # Subscribe to raw joint states (usually joint_1~6)
        # Note: in many setups, the real robot (or servo node) publishes `/joint_states`.
        # Some bringup stacks publish `/rbpodo/joint_states`; we accept both.
        self.sub_rbpodo = self.create_subscription(
            JointState,
            '/rbpodo/joint_states',
            self.joint_state_callback,
            10
        )
        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for mapped joint_states (base~wrist3)
        self.pub = self.create_publisher(
            JointState,
            '/joint_states_mapped',
            10
        )
        
        self.get_logger().info('Joint state mapper initialized')
        self.get_logger().info('Subscribing to /rbpodo/joint_states and /joint_states (expecting joint_1~6)')
        self.get_logger().info('Publishing mapped joints to /joint_states_mapped (base~wrist3)')
    
    def joint_state_callback(self, msg):
        """Map joint names from joint_1~6 to base~wrist3."""
        # Only map messages that use the joint_1~6 convention.
        if not msg.name:
            return
        
        # Skip if already in URDF convention (base~wrist3)
        if any(n in ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3'] for n in msg.name):
            return
        
        # Additional check: only process if it has joint_1~6 names
        if not any(n in ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'] for n in msg.name):
            # Not a joint_1~6 message - ignore
            return
        
        mapped_msg = JointState()
        mapped_msg.header = msg.header
        
        # Map joint names
        mapped_names = []
        mapped_positions = []
        mapped_velocities = []
        mapped_efforts = []
        
        for i, name in enumerate(msg.name):
            if name in self.joint_name_map:
                mapped_name = self.joint_name_map[name]
                mapped_names.append(mapped_name)
                
                if i < len(msg.position):
                    mapped_positions.append(msg.position[i])
                if i < len(msg.velocity):
                    mapped_velocities.append(msg.velocity[i])
                if i < len(msg.effort):
                    mapped_efforts.append(msg.effort[i])
        
        # Only publish if we have valid mapped joints
        if mapped_names and mapped_positions:
            mapped_msg.name = mapped_names
            mapped_msg.position = mapped_positions
            if mapped_velocities:
                mapped_msg.velocity = mapped_velocities
            if mapped_efforts:
                mapped_msg.effort = mapped_efforts
            
            self.pub.publish(mapped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

