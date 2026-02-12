"""Disable APF on the proximity controller.

Calls /proximity_controller/enable with False, stops execution and clears anchor.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ApfOff(Node):
    def __init__(self):
        super().__init__('apf_off')
        self.client = self.create_client(
            SetBool, '/proximity_controller/enable'
        )

        self.get_logger().info('Disabling APF on proximity controller...')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'Service /proximity_controller/enable not available. '
                'Is the proximity controller running?'
            )
            self.create_timer(
                0.1, lambda: (_ for _ in ()).throw(SystemExit)
            )
            return

        req = SetBool.Request()
        req.data = False
        future = self.client.call_async(req)
        future.add_done_callback(self._done_cb)

    def _done_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'APF disabled: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        self.create_timer(
            0.5, lambda: (_ for _ in ()).throw(SystemExit)
        )


def main(args=None):
    rclpy.init(args=args)
    node = ApfOff()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
