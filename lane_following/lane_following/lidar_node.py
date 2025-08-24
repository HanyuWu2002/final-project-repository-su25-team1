import rclpy
from rclpy.node import Node

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.get_logger().info('Lidar Node started')