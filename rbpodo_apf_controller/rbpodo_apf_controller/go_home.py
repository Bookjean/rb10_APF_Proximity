"""Send the robot to home position (all joints zero).

Disables APF controller (clears goal), clears obstacles, then sends home position.
APF remains disabled after completion.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger


HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class GoHome(Node):
    def __init__(self):
        super().__init__("go_home")
        self.pub = self.create_publisher(
            Float64MultiArray, "/position_controllers/commands", 10
        )
        self.apf_client = self.create_client(SetBool, "/apf_controller/enable")
        self.clear_obs_client = self.create_client(
            Trigger, "/fake_tof_publisher/clear_obstacles"
        )

        # Step 1: Disable APF (also clears goal/trajectory)
        self.get_logger().info("Disabling APF controller (clears trajectory)...")
        self._call_apf_enable(False)

        # Step 2: Clear obstacles
        self.get_logger().info("Clearing obstacles...")
        self._call_clear_obstacles()

        # Step 3: Publish home position for 1 second at 10Hz
        self.count = 0
        self.timer = self.create_timer(0.1, self._publish)

    def _call_apf_enable(self, enable):
        if not self.apf_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("APF enable service not available, skipping")
            return
        req = SetBool.Request()
        req.data = enable
        self.apf_client.call_async(req)

    def _call_clear_obstacles(self):
        if not self.clear_obs_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Clear obstacles service not available, skipping")
            return
        self.clear_obs_client.call_async(Trigger.Request())

    def _publish(self):
        msg = Float64MultiArray()
        msg.data = HOME_POSITION
        self.pub.publish(msg)
        self.count += 1
        if self.count >= 10:
            self.get_logger().info(f"Home position sent: {HOME_POSITION}")
            self.get_logger().info("Done. APF remains disabled.")
            self.create_timer(0.5, lambda: (_ for _ in ()).throw(SystemExit))


def main(args=None):
    rclpy.init(args=args)
    node = GoHome()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
