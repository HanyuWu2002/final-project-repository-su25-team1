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

    def count_clear(self, sector):
        """Count how many values are reasonably clear (0.5m to 3.5m)"""
        return sum(1 for r in sector if 0.5 < r < 3.5)

    def detect_object(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)

        def angle_to_index(angle_deg):
            """Convert angle in degrees to index in ranges[]"""
            angle_rad = math.radians(angle_deg)
            index = int((angle_rad - angle_min) / angle_increment)
            return max(0, min(index, num_ranges - 1))

        def get_sector(start_deg, end_deg):
            """Get cleaned range values between angles"""
            start_idx = angle_to_index(start_deg)
            end_idx = angle_to_index(end_deg)

            if end_idx >= start_idx:
                sector = ranges[start_idx:end_idx]
            else:
                sector = ranges[start_idx:] + ranges[:end_idx]

            return [r for r in sector if 0.0 < r < float('inf')]

        # Define sectors (adjust angles if needed for your setup)
        front_sector = get_sector(-30, 30)
        left_sector = get_sector(60, 120)
        right_sector = get_sector(-120, -60)

        msg_out = String()

        if front_sector and min(front_sector) < 0.5:
            clear_left = self.count_clear(left_sector)
            clear_right = self.count_clear(right_sector)

            self.get_logger().info(f"Obstacle ahead. Left: {clear_left}, Right: {clear_right}")

            if clear_left > clear_right:
                msg_out.data = "LEFT"
            else:
                msg_out.data = "RIGHT"
        else:
            msg_out.data = "No obstacle. FORWARD."

        self.get_logger().info(msg_out.data)
        # self.pub.publish(msg_out)


    def lidar_callback(self, msg):
        self.detect_object(msg)

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