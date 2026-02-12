"""Joint state mapper to convert joint_1~6 to base~wrist3 naming.

This node subscribes to /joint_states with joint_1~6 names and
publishes to /joint_states with base~wrist3 names for compatibility
with the URDF model.
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
        
        # Subscriber for original joint_states
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for mapped joint_states
        self.pub = self.create_publisher(
            JointState,
            '/joint_states_mapped',
            10
        )
        
        self.get_logger().info('Joint state mapper initialized')
        self.get_logger().info('Mapping: joint_1~6 -> base~wrist3')
    
    def joint_state_callback(self, msg):
        """Map joint names from joint_1~6 to base~wrist3."""
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



