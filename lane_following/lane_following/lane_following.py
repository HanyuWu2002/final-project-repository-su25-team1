import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
# from lane_following.lane_following.lidar_node import LidarNode

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.get_logger().info('Lidar Node started')
        self.subscription = self.create_subscription('sensor_msgs/msg/LaserScan', '/scan', self.lidar_callback, qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def lidar_callback(self, msg):
        self.get_logger().info(f'Received LaserScan {msg}')


def main():
    rclpy.init()

    lidarNode = LidarNode()

    rclpy.spin(lidarNode)

    lidarNode.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()