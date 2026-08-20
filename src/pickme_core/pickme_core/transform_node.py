"""
TransformNode — stable background service for publishing transfromation.

Lives in pickme-core container. Does NOT restart with dev container.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    
)
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point



class CommanderNode(Node):
    """Transfromation publisher node"""

    def __init__(self):
        super().__init__('transform_node')

    
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
