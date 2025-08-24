import rclpy
from lane_following.lidar_node import LidarNode


def main():
    rclpy.init()

    lidarNode = LidarNode()

    rclpy.spin(lidarNode)

    lidarNode.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()