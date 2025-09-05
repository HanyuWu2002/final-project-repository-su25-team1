import math

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image, LaserScan
import math

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.get_logger().info('Lidar Node started')

        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def lidar_callback(self, msg):

            self.get_logger().warn(f'{len(msg.ranges)}')


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('Camera Node started')
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.camera_callback, qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def camera_callback(self, msg):
        self.get_logger().info(f'Received Image {msg}')


def main():
    rclpy.init()

    lidar_node = LidarNode()

    rclpy.spin(lidar_node)

    lidar_node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()