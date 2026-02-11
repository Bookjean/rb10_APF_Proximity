"""Send the robot to home position while keeping obstacles.

Disables APF controller (clears goal), then sends home position.
Does NOT clear obstacles so they persist for the next run.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool


HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class GoHomeKeepObs(Node):
    def __init__(self):
        super().__init__("go_home_keep_obs")
        self.pub = self.create_publisher(
            Float64MultiArray, "/position_controllers/commands", 10
        )
        self.apf_client = self.create_client(SetBool, "/apf_controller/enable")

        # Disable APF (clears goal/trajectory)
        self.get_logger().info("Disabling APF controller (keeping obstacles)...")
        self._call_apf_enable(False)

        # Publish home position for 1 second at 10Hz
        self.count = 0
        self.timer = self.create_timer(0.1, self._publish)

    def _call_apf_enable(self, enable):
        if not self.apf_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("APF enable service not available, skipping")
            return
        req = SetBool.Request()
        req.data = enable
        self.apf_client.call_async(req)

    def _publish(self):
        msg = Float64MultiArray()
        msg.data = HOME_POSITION
        self.pub.publish(msg)
        self.count += 1
        if self.count >= 10:
            self.get_logger().info(f"Home position sent: {HOME_POSITION}")
            self.get_logger().info("Done. APF disabled, obstacles kept.")
            self.create_timer(0.5, lambda: (_ for _ in ()).throw(SystemExit))


def main(args=None):
    rclpy.init(args=args)
    node = GoHomeKeepObs()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
