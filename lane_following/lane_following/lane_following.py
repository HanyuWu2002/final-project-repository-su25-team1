import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, Int32, Int32MultiArray, String
from geometry_msgs.msg import Twist
import math

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.get_logger().info('Lidar Node started')

        self.min_distance = 0.3 # meters
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.twist.linear.x = 2.0
        self.speed_publisher.publish(self.twist)

    def detect_object(self, points):
        # Loop through points
        # Detect if something is closer than the threshold
        # if it is put it in the detected
        for point in points:
            if point < self.min_distance:
                self.twist.linear.x = 0.0
                self.speed_publisher.publish(self.twist)
                self.get_logger().fatal(f'OBJECT DETECTED at point {point}')


    def lidar_callback(self, msg):
        self.detect_object(msg.ranges)

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