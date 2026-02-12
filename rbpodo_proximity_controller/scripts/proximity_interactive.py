#!/usr/bin/env python3
"""Interactive proximity APF controller (dataset_gen.py style).

RViz launches once at startup. All controls on the same level via keyboard.
Press 1/2 to switch sim/real anytime. APF and waypoint controls always available.

Usage:
    python3 proximity_interactive.py

Keyboard controls:
    1 : Simulation mode     2 : Real robot mode
    a : APF ON              d : APF OFF
    s : Save position       e : Execute waypoints
    w : Load YAML file      c : Clear waypoints
    x : Stop execution      t : Quit
"""

import os
import sys
import subprocess
import threading
import time

from pynput import keyboard

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

# Global state
terminal = False
controller = None
action_queue = []

CONFIG_DIR = os.path.join(
    os.path.expanduser('~'), 'ros2_ws', 'src',
    'rbpodo_ros2', 'rbpodo_proximity_controller', 'config'
)


class ProximityInteractive(Node):
    """ROS2 node for interactive APF control via keyboard."""

    def __init__(self):
        super().__init__('proximity_interactive')

        self.reentrant_group = ReentrantCallbackGroup()

        self.joint_state = None
        self.apf_enabled = False
        self.saved_count = 0

        self.enable_client = self.create_client(
            SetBool, '/proximity_controller/enable'
        )
        self.execute_client = self.create_client(
            Trigger, '/proximity_controller/execute_waypoints'
        )
        self.save_client = self.create_client(
            Trigger, '/waypoint_manager/save_current_position'
        )
        self.stop_client = self.create_client(
            Trigger, '/proximity_controller/stop'
        )
        self.load_file_client = self.create_client(
            Trigger, '/waypoint_manager/load_from_file'
        )
        self.clear_client = self.create_client(
            Trigger, '/waypoint_manager/clear_waypoints'
        )

        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10,
            callback_group=self.reentrant_group
        )

    def _joint_state_cb(self, msg):
        self.joint_state = list(msg.position)

    def _call_setbool(self, client, value, label):
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"[ERROR] {label} service not available")
            return
        req = SetBool.Request()
        req.data = value
        future = client.call_async(req)

        def cb(f):
            try:
                result = f.result()
                if value:
                    self.apf_enabled = True
                else:
                    self.apf_enabled = False
                print(f"[{label}] {result.message}")
            except Exception as e:
                print(f"[ERROR] {e}")
        future.add_done_callback(cb)

    def _call_trigger(self, client, label):
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"[ERROR] {label} service not available")
            return
        req = Trigger.Request()
        future = client.call_async(req)

        def cb(f):
            try:
                result = f.result()
                print(f"[{label}] {result.message}")
                if label == 'SAVE' and result.success:
                    self.saved_count += 1
                    print(f"  (total saved: {self.saved_count})")
                elif label == 'CLEAR':
                    self.saved_count = 0
            except Exception as e:
                print(f"[ERROR] {e}")
        future.add_done_callback(cb)

    def apf_on(self):
        self._call_setbool(self.enable_client, True, 'APF ON')

    def apf_off(self):
        self._call_setbool(self.enable_client, False, 'APF OFF')

    def execute_waypoints(self):
        self._call_trigger(self.execute_client, 'EXECUTE')

    def save_position(self):
        self._call_trigger(self.save_client, 'SAVE')

    def stop_execution(self):
        self._call_trigger(self.stop_client, 'STOP')

    def load_from_file(self):
        self._call_trigger(self.load_file_client, 'LOAD')

    def clear_waypoints(self):
        self._call_trigger(self.clear_client, 'CLEAR')


def on_press(key):
    global terminal, action_queue
    try:
        c = key.char
        if c == 't':
            terminal = True
        elif c in ('1', '2', 'a', 'd', 's', 'e', 'w', 'x', 'c'):
            action_queue.append(c)
    except AttributeError:
        pass


def switch_mode(launch_proc, log_fd, mode, robot_ip):
    """Kill current nodes, launch new mode (without RViz)."""
    if launch_proc:
        print(f"\n[MODE] Stopping current nodes...")
        launch_proc.terminate()
        try:
            launch_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            launch_proc.kill()
    if log_fd:
        log_fd.close()

    cmd = [
        'ros2', 'launch',
        'rbpodo_proximity_controller',
        'proximity_apf.launch.py',
        f'robot_mode:={mode}',
        f'robot_ip:={robot_ip}',
        'use_rviz:=false',
    ]

    mode_str = 'Simulation' if mode == 'sim' else f'Real ({robot_ip})'
    print(f"[MODE] Launching {mode_str}...")

    log_file = '/tmp/proximity_apf_launch.log'
    log_fd = open(log_file, 'w')
    launch_proc = subprocess.Popen(
        cmd, stdout=log_fd, stderr=subprocess.STDOUT
    )

    time.sleep(5.0)

    if controller.enable_client.wait_for_service(timeout_sec=10.0):
        print(f"[MODE] {mode_str} ready!\n")
    else:
        print(f"[MODE] Warning: services not available yet (check {log_file})\n")

    return launch_proc, log_fd


def handle_yaml_load():
    """Prompt for YAML file selection, then load."""
    yaml_files = []
    if os.path.isdir(CONFIG_DIR):
        yaml_files = sorted([
            f for f in os.listdir(CONFIG_DIR)
            if f.endswith('.yaml') and f != 'controller_params.yaml'
        ])

    if yaml_files:
        print("\n  Available YAML files:")
        for i, f in enumerate(yaml_files, 1):
            print(f"    {i}. {f}")

    try:
        file_input = input("  File number or path (enter to cancel): ").strip()
    except (EOFError, KeyboardInterrupt):
        return

    if not file_input:
        return

    if file_input.isdigit():
        idx = int(file_input) - 1
        if 0 <= idx < len(yaml_files):
            filepath = os.path.join(CONFIG_DIR, yaml_files[idx])
        else:
            print("  Invalid number.")
            return
    elif os.path.isabs(file_input):
        filepath = file_input
    else:
        filepath = os.path.join(CONFIG_DIR, file_input)

    if not os.path.exists(filepath):
        print(f"  Not found: {filepath}")
        return

    print(f"  Loading: {os.path.basename(filepath)}")
    try:
        subprocess.run(
            ['ros2', 'param', 'set', '/waypoint_manager',
             'waypoint_file', os.path.basename(filepath)],
            capture_output=True, timeout=5.0
        )
    except Exception:
        pass
    controller.load_from_file()


def main():
    global terminal, controller, action_queue

    # ---- RViz (launch once) ----
    try:
        pkg = get_package_share_directory('rbpodo_proximity_controller')
        rviz_config = os.path.join(pkg, 'config', 'proximity_demo.rviz')
    except Exception:
        rviz_config = None

    rviz_cmd = ['ros2', 'run', 'rviz2', 'rviz2']
    if rviz_config and os.path.exists(rviz_config):
        rviz_cmd.extend(['-d', rviz_config])

    print("Starting RViz...")
    rviz_proc = subprocess.Popen(
        rviz_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

    # ---- ROS2 node (init once) ----
    rclpy.init()
    controller = ProximityInteractive()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # ---- Keyboard listener ----
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    print()
    print("=" * 50)
    print("  RB10 Proximity APF Controller")
    print("=" * 50)
    print("  1: Simulation    2: Real robot")
    print("  a: APF ON        d: APF OFF")
    print("  s: Save pos      e: Execute waypoints")
    print("  w: Load YAML     c: Clear waypoints")
    print("  x: Stop exec     t: Quit")
    print("=" * 50)
    print()
    print("Press 1 or 2 to start.")
    print()

    launch_proc = None
    log_fd = None

    # ---- Main loop ----
    try:
        while rclpy.ok() and not terminal:
            # Process action queue
            while action_queue:
                action = action_queue.pop(0)

                if action == '1':
                        launch_proc, log_fd = switch_mode(
                            launch_proc, log_fd, 'sim', '192.168.111.50'
                        )
                elif action == '2':
                    try:
                        ip = input("  Robot IP (default: 10.0.2.7): ").strip()
                    except (EOFError, KeyboardInterrupt):
                        continue
                    launch_proc, log_fd = switch_mode(
                        launch_proc, log_fd, 'real',
                        ip if ip else '192.168.111.50'
                    )
                elif action == 'a':
                    controller.apf_on()
                elif action == 'd':
                    controller.apf_off()
                elif action == 's':
                    controller.save_position()
                elif action == 'e':
                    controller.execute_waypoints()
                elif action == 'w':
                    handle_yaml_load()
                elif action == 'x':
                    controller.stop_execution()
                elif action == 'c':
                    controller.clear_waypoints()

            # Check launch process
            if launch_proc and launch_proc.poll() is not None:
                print("[ERROR] Launch process terminated. Press 1 or 2 to restart.")
                launch_proc = None

            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    # ---- Cleanup ----
    print("\nShutting down...")
    listener.stop()
    if launch_proc:
        launch_proc.terminate()
        try:
            launch_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            launch_proc.kill()
    if log_fd:
        log_fd.close()
    rclpy.shutdown()
    executor_thread.join(timeout=2.0)
    rviz_proc.terminate()
    try:
        rviz_proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        rviz_proc.kill()
    print("Done.")


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""Interactive proximity APF controller (dataset_gen.py style).

RViz launches once at startup. All controls on the same level via keyboard.
Press 1/2 to switch sim/real anytime. APF and waypoint controls always available.

Usage:
    python3 proximity_interactive.py

Keyboard controls:
    1 : Simulation mode     2 : Real robot mode
    a : APF ON              d : APF OFF
    s : Save position       e : Execute waypoints
    w : Load YAML file      c : Clear waypoints
    x : Stop execution      t : Quit
"""

import os
import sys
import subprocess
import threading
import time

from pynput import keyboard

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

# Global state
terminal = False
controller = None
action_queue = []

CONFIG_DIR = os.path.join(
    os.path.expanduser('~'), 'ros2_ws', 'src',
    'rbpodo_ros2', 'rbpodo_proximity_controller', 'config'
)


class ProximityInteractive(Node):
    """ROS2 node for interactive APF control via keyboard."""

    def __init__(self):
        super().__init__('proximity_interactive')

        self.reentrant_group = ReentrantCallbackGroup()

        self.joint_state = None
        self.apf_enabled = False
        self.saved_count = 0

        self.enable_client = self.create_client(
            SetBool, '/proximity_controller/enable'
        )
        self.execute_client = self.create_client(
            Trigger, '/proximity_controller/execute_waypoints'
        )
        self.save_client = self.create_client(
            Trigger, '/waypoint_manager/save_current_position'
        )
        self.stop_client = self.create_client(
            Trigger, '/proximity_controller/stop'
        )
        self.load_file_client = self.create_client(
            Trigger, '/waypoint_manager/load_from_file'
        )
        self.clear_client = self.create_client(
            Trigger, '/waypoint_manager/clear_waypoints'
        )

        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10,
            callback_group=self.reentrant_group
        )

    def _joint_state_cb(self, msg):
        self.joint_state = list(msg.position)

    def _call_setbool(self, client, value, label):
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"[ERROR] {label} service not available")
            return
        req = SetBool.Request()
        req.data = value
        future = client.call_async(req)

        def cb(f):
            try:
                result = f.result()
                if value:
                    self.apf_enabled = True
                else:
                    self.apf_enabled = False
                print(f"[{label}] {result.message}")
            except Exception as e:
                print(f"[ERROR] {e}")
        future.add_done_callback(cb)

    def _call_trigger(self, client, label):
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"[ERROR] {label} service not available")
            return
        req = Trigger.Request()
        future = client.call_async(req)

        def cb(f):
            try:
                result = f.result()
                print(f"[{label}] {result.message}")
                if label == 'SAVE' and result.success:
                    self.saved_count += 1
                    print(f"  (total saved: {self.saved_count})")
                elif label == 'CLEAR':
                    self.saved_count = 0
            except Exception as e:
                print(f"[ERROR] {e}")
        future.add_done_callback(cb)

    def apf_on(self):
        self._call_setbool(self.enable_client, True, 'APF ON')

    def apf_off(self):
        self._call_setbool(self.enable_client, False, 'APF OFF')

    def execute_waypoints(self):
        self._call_trigger(self.execute_client, 'EXECUTE')

    def save_position(self):
        self._call_trigger(self.save_client, 'SAVE')

    def stop_execution(self):
        self._call_trigger(self.stop_client, 'STOP')

    def load_from_file(self):
        self._call_trigger(self.load_file_client, 'LOAD')

    def clear_waypoints(self):
        self._call_trigger(self.clear_client, 'CLEAR')


def on_press(key):
    global terminal, action_queue
    try:
        c = key.char
        if c == 't':
            terminal = True
        elif c in ('1', '2', 'a', 'd', 's', 'e', 'w', 'x', 'c'):
            action_queue.append(c)
    except AttributeError:
        pass


def switch_mode(launch_proc, log_fd, mode, robot_ip):
    """Kill current nodes, launch new mode (without RViz)."""
    if launch_proc:
        print(f"\n[MODE] Stopping current nodes...")
        launch_proc.terminate()
        try:
            launch_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            launch_proc.kill()
    if log_fd:
        log_fd.close()

    cmd = [
        'ros2', 'launch',
        'rbpodo_proximity_controller',
        'proximity_apf.launch.py',
        f'robot_mode:={mode}',
        f'robot_ip:={robot_ip}',
        'use_rviz:=false',
    ]

    mode_str = 'Simulation' if mode == 'sim' else f'Real ({robot_ip})'
    print(f"[MODE] Launching {mode_str}...")

    log_file = '/tmp/proximity_apf_launch.log'
    log_fd = open(log_file, 'w')
    launch_proc = subprocess.Popen(
        cmd, stdout=log_fd, stderr=subprocess.STDOUT
    )

    time.sleep(5.0)

    if controller.enable_client.wait_for_service(timeout_sec=10.0):
        print(f"[MODE] {mode_str} ready!\n")
    else:
        print(f"[MODE] Warning: services not available yet (check {log_file})\n")

    return launch_proc, log_fd


def handle_yaml_load():
    """Prompt for YAML file selection, then load."""
    yaml_files = []
    if os.path.isdir(CONFIG_DIR):
        yaml_files = sorted([
            f for f in os.listdir(CONFIG_DIR)
            if f.endswith('.yaml') and f != 'controller_params.yaml'
        ])

    if yaml_files:
        print("\n  Available YAML files:")
        for i, f in enumerate(yaml_files, 1):
            print(f"    {i}. {f}")

    try:
        file_input = input("  File number or path (enter to cancel): ").strip()
    except (EOFError, KeyboardInterrupt):
        return

    if not file_input:
        return

    if file_input.isdigit():
        idx = int(file_input) - 1
        if 0 <= idx < len(yaml_files):
            filepath = os.path.join(CONFIG_DIR, yaml_files[idx])
        else:
            print("  Invalid number.")
            return
    elif os.path.isabs(file_input):
        filepath = file_input
    else:
        filepath = os.path.join(CONFIG_DIR, file_input)

    if not os.path.exists(filepath):
        print(f"  Not found: {filepath}")
        return

    print(f"  Loading: {os.path.basename(filepath)}")
    try:
        subprocess.run(
            ['ros2', 'param', 'set', '/waypoint_manager',
             'waypoint_file', os.path.basename(filepath)],
            capture_output=True, timeout=5.0
        )
    except Exception:
        pass
    controller.load_from_file()


def main():
    global terminal, controller, action_queue

    # ---- RViz (launch once) ----
    try:
        pkg = get_package_share_directory('rbpodo_proximity_controller')
        rviz_config = os.path.join(pkg, 'config', 'proximity_demo.rviz')
    except Exception:
        rviz_config = None

    rviz_cmd = ['ros2', 'run', 'rviz2', 'rviz2']
    if rviz_config and os.path.exists(rviz_config):
        rviz_cmd.extend(['-d', rviz_config])

    print("Starting RViz...")
    rviz_proc = subprocess.Popen(
        rviz_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

    # ---- ROS2 node (init once) ----
    rclpy.init()
    controller = ProximityInteractive()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # ---- Keyboard listener ----
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    print()
    print("=" * 50)
    print("  RB10 Proximity APF Controller")
    print("=" * 50)
    print("  1: Simulation    2: Real robot")
    print("  a: APF ON        d: APF OFF")
    print("  s: Save pos      e: Execute waypoints")
    print("  w: Load YAML     c: Clear waypoints")
    print("  x: Stop exec     t: Quit")
    print("=" * 50)
    print()
    print("Press 1 or 2 to start.")
    print()

    launch_proc = None
    log_fd = None

    # ---- Main loop ----
    try:
        while rclpy.ok() and not terminal:
            # Process action queue
            while action_queue:
                action = action_queue.pop(0)

                if action == '1':
                    launch_proc, log_fd = switch_mode(
                        launch_proc, log_fd, 'sim', '10.0.2.7'
                    )
                elif action == '2':
                    try:
                        ip = input("  Robot IP (default: 10.0.2.7): ").strip()
                    except (EOFError, KeyboardInterrupt):
                        continue
                    launch_proc, log_fd = switch_mode(
                        launch_proc, log_fd, 'real',
                        ip if ip else '10.0.2.7'
                    )
                elif action == 'a':
                    controller.apf_on()
                elif action == 'd':
                    controller.apf_off()
                elif action == 's':
                    controller.save_position()
                elif action == 'e':
                    controller.execute_waypoints()
                elif action == 'w':
                    handle_yaml_load()
                elif action == 'x':
                    controller.stop_execution()
                elif action == 'c':
                    controller.clear_waypoints()

            # Check launch process
            if launch_proc and launch_proc.poll() is not None:
                print("[ERROR] Launch process terminated. Press 1 or 2 to restart.")
                launch_proc = None

            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    # ---- Cleanup ----
    print("\nShutting down...")
    listener.stop()
    if launch_proc:
        launch_proc.terminate()
        try:
            launch_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            launch_proc.kill()
    if log_fd:
        log_fd.close()
    rclpy.shutdown()
    executor_thread.join(timeout=2.0)
    rviz_proc.terminate()
    try:
        rviz_proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        rviz_proc.kill()
    print("Done.")


if __name__ == '__main__':
    main()
