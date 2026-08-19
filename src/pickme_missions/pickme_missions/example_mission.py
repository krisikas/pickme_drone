"""
Example mission: arm, switch to offboard, takeoff to 5m, hover for 10 seconds, land.

This runs inside the pickme-dev container. It calls services provided by
CommanderNode (running in pickme-core) and does NOT talk to PX4 directly.

Usage (inside dev container):
    colcon build --packages-select pickme_missions
    source install/setup.bash
    ros2 launch pickme_missions example.launch.py
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ExampleMission(Node):
    """Minimal mission that demonstrates calling commander services."""

    def __init__(self):
        super().__init__('example_mission')
        self.get_logger().info('Example mission node started')

        # Create service clients to commander (running in core container)
        self._arm_client = self.create_client(Trigger, '/pickme/arm')
        self._takeoff_client = self.create_client(Trigger, '/pickme/takeoff')
        self._land_client = self.create_client(Trigger, '/pickme/land')

        # Wait for commander services to be available
        self.get_logger().info('Waiting for commander services...')
        self._arm_client.wait_for_service()
        self._takeoff_client.wait_for_service()
        self._land_client.wait_for_service()
        self.get_logger().info('Commander services available!')

        # Start mission sequence
        self._timer = self.create_timer(1.0, self._run_mission)
        self._step = 0

    def _call_service(self, client, name):
        """Synchronous-style service call."""
        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'{name}: {f.result().message}')
        )

    def _run_mission(self):
        """Simple state machine: arm → takeoff → hover → land."""
        if self._step == 0:
            self.get_logger().info('Step 1: Arming...')
            self._call_service(self._arm_client, 'ARM')
            self._step += 1

        elif self._step == 3:
            self.get_logger().info('Step 2: Taking off...')
            self._call_service(self._takeoff_client, 'TAKEOFF')
            self._step += 1

        elif self._step == 13:
            self.get_logger().info('Step 3: Landing...')
            self._call_service(self._land_client, 'LAND')
            self._step += 1

        elif self._step == 20:
            self.get_logger().info('Mission complete!')
            self._timer.cancel()

        else:
            self._step += 1


def main(args=None):
    rclpy.init(args=args)
    node = ExampleMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
