import rclpy
from rclpy.node import Node
# from lane_following.lane_following.lidar_node import LidarNode

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.get_logger().info('Lidar Node started')


def main():
    rclpy.init()

    lidarNode = LidarNode()

    rclpy.spin(lidarNode)

    lidarNode.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()