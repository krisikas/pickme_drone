"""
CommanderNode — stable background service for PX4 flight control.

Provides high-level ROS 2 services for arming, takeoff, landing,
and offboard trajectory control. Translates convenient service calls
into raw PX4 uORB messages via Micro-XRCE-DDS.

Lives in pickme-core container. Does NOT restart with dev container.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleOdometry,
)
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point


# QoS profile compatible with PX4 DDS topics
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CommanderNode(Node):
    """High-level PX4 command interface running as a stable background service."""

    def __init__(self):
        super().__init__('commander_node')

        # ── State ──
        self._vehicle_status = None
        self._vehicle_odom = None
        self._offboard_setpoint_counter = 0

        # ── Publishers to PX4 ──
        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10
        )
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10
        )
        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10
        )

        # ── Subscribers from PX4 ──
        self._status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._status_cb, PX4_QOS
        )
        self._odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self._odom_cb, PX4_QOS
        )

        # ── Services ──
        self.create_service(Trigger, '/pickme/arm', self._arm_cb)
        self.create_service(Trigger, '/pickme/disarm', self._disarm_cb)
        self.create_service(Trigger, '/pickme/takeoff', self._takeoff_cb)
        self.create_service(Trigger, '/pickme/land', self._land_cb)

        # ── Offboard heartbeat timer (must publish at >= 2 Hz for PX4) ──
        self._offboard_timer = self.create_timer(0.1, self._offboard_heartbeat)

        self.get_logger().info('CommanderNode started — services: /pickme/arm, /pickme/disarm, /pickme/takeoff, /pickme/land')

    # ─────────────────────────── Callbacks ───────────────────────────

    def _status_cb(self, msg: VehicleStatus):
        self._vehicle_status = msg

    def _odom_cb(self, msg: VehicleOdometry):
        self._vehicle_odom = msg

    # ─────────────────────────── Services ────────────────────────────

    def _arm_cb(self, request, response):
        self.get_logger().info('ARM requested')
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        response.success = True
        response.message = 'Arm command sent'
        return response

    def _disarm_cb(self, request, response):
        self.get_logger().info('DISARM requested')
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        response.success = True
        response.message = 'Disarm command sent'
        return response

    def _takeoff_cb(self, request, response):
        self.get_logger().info('TAKEOFF requested')
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=1.0,  # minimum pitch
            param7=-5.0,  # altitude (NED: negative = up) — 5 meters
        )
        response.success = True
        response.message = 'Takeoff command sent (target alt: 5m)'
        return response

    def _land_cb(self, request, response):
        self.get_logger().info('LAND requested')
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        response.success = True
        response.message = 'Land command sent'
        return response

    # ─────────────────────────── Offboard ────────────────────────────

    def _offboard_heartbeat(self):
        """Publish offboard control mode at 10 Hz to keep PX4 in offboard mode.
        PX4 requires a stream of OffboardControlMode messages before and during
        offboard flight. If this stream stops, PX4 will exit offboard mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._offboard_mode_pub.publish(msg)

    # ─────────────────────── PX4 Command Helper ─────────────────────

    def _publish_vehicle_command(self, command, **params):
        """Send a VehicleCommand to PX4."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._vehicle_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
