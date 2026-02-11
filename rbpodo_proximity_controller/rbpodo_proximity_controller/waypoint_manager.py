"""Waypoint manager node for teaching and managing robot waypoints.

Provides services to save current robot position, manage waypoint list,
and save/load waypoints to/from YAML files.
"""

import os

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, SetBool

JOINT_NAMES = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('waypoint_file', 'waypoints.yaml')
        self.declare_parameter('save_directory', '')
        self.declare_parameter('joint_names', JOINT_NAMES)
        self.declare_parameter('auto_load', True)

        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.save_directory = self.get_parameter('save_directory').value
        self.joint_names = self.get_parameter('joint_names').value
        self.auto_load = self.get_parameter('auto_load').value

        if not self.save_directory:
            self.save_directory = os.path.join(
                os.path.expanduser('~'), 'ros2_ws', 'src',
                'rbpodo_ros2', 'rbpodo_proximity_controller', 'config'
            )

        self.waypoints = []
        self.q = None

        # Subscriber
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10
        )

        # Publisher - publish waypoints for controller
        self.waypoints_pub = self.create_publisher(
            Float64MultiArray, '~/waypoints', 10
        )

        # Services
        self.create_service(
            Trigger, '~/save_current_position', self._save_current_position_cb
        )
        self.create_service(
            Trigger, '~/clear_waypoints', self._clear_waypoints_cb
        )
        self.create_service(
            Trigger, '~/delete_last_waypoint', self._delete_last_waypoint_cb
        )
        self.create_service(
            Trigger, '~/save_to_file', self._save_to_file_cb
        )
        self.create_service(
            Trigger, '~/load_from_file', self._load_from_file_cb
        )
        self.create_service(
            Trigger, '~/list_waypoints', self._list_waypoints_cb
        )

        # Periodically publish waypoints
        self.create_timer(1.0, self._publish_waypoints)

        # Auto-load waypoints from file if exists
        if self.auto_load:
            filepath = os.path.join(self.save_directory, self.waypoint_file)
            if os.path.exists(filepath):
                self._load_waypoints_from_file(filepath)
                self.get_logger().info(
                    f'Auto-loaded {len(self.waypoints)} waypoints from {filepath}'
                )

        self.get_logger().info('Waypoint manager initialized')

    def _joint_states_cb(self, msg):
        if len(msg.position) < 6:
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.q = np.array([name_to_pos[n] for n in self.joint_names])
        except KeyError:
            pass

    def _publish_waypoints(self):
        """Publish current waypoint list as flat Float64MultiArray.

        Format: [n_waypoints, n_joints, wp0_j0, wp0_j1, ..., wp1_j0, ...]
        """
        msg = Float64MultiArray()
        n_joints = 6
        data = [float(len(self.waypoints)), float(n_joints)]
        for wp in self.waypoints:
            data.extend([float(v) for v in wp['joint_positions']])
        msg.data = data
        self.waypoints_pub.publish(msg)

    def _save_current_position_cb(self, request, response):
        if self.q is None:
            response.success = False
            response.message = 'No joint state received yet'
            return response

        idx = len(self.waypoints)
        wp = {
            'name': f'waypoint_{idx}',
            'joint_positions': self.q.tolist(),
        }
        self.waypoints.append(wp)

        pos_str = ', '.join(f'{v:.4f}' for v in self.q)
        response.success = True
        response.message = (
            f'Saved waypoint_{idx}: [{pos_str}] '
            f'(total: {len(self.waypoints)})'
        )
        self.get_logger().info(response.message)
        self._publish_waypoints()
        return response

    def _clear_waypoints_cb(self, request, response):
        count = len(self.waypoints)
        self.waypoints.clear()
        response.success = True
        response.message = f'Cleared {count} waypoints'
        self.get_logger().info(response.message)
        self._publish_waypoints()
        return response

    def _delete_last_waypoint_cb(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints to delete'
            return response

        removed = self.waypoints.pop()
        response.success = True
        response.message = (
            f"Deleted {removed['name']} "
            f"(remaining: {len(self.waypoints)})"
        )
        self.get_logger().info(response.message)
        self._publish_waypoints()
        return response

    def _save_to_file_cb(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints to save'
            return response

        filepath = os.path.join(self.save_directory, self.waypoint_file)
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            data = {'waypoints': self.waypoints}
            with open(filepath, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            response.success = True
            response.message = (
                f'Saved {len(self.waypoints)} waypoints to {filepath}'
            )
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to save: {e}'
            self.get_logger().error(response.message)
        return response

    def _load_from_file_cb(self, request, response):
        filepath = os.path.join(self.save_directory, self.waypoint_file)
        try:
            self._load_waypoints_from_file(filepath)
            response.success = True
            response.message = (
                f'Loaded {len(self.waypoints)} waypoints from {filepath}'
            )
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to load: {e}'
            self.get_logger().error(response.message)
        self._publish_waypoints()
        return response

    def _load_waypoints_from_file(self, filepath):
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        if data and 'waypoints' in data:
            self.waypoints = data['waypoints']
        else:
            self.waypoints = []

    def _list_waypoints_cb(self, request, response):
        if not self.waypoints:
            response.success = True
            response.message = 'No waypoints saved'
            return response

        lines = [f'Total: {len(self.waypoints)} waypoints']
        for i, wp in enumerate(self.waypoints):
            pos_str = ', '.join(
                f'{v:.4f}' for v in wp['joint_positions']
            )
            lines.append(f"  [{i}] {wp['name']}: [{pos_str}]")
        response.success = True
        response.message = '\n'.join(lines)
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
