from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
          Node(
            package='lane_following',
            executable='lane_following_executable',
            output='screen',
            emulate_tty=True
          ),
          # Node(
          #     package='ucsd_robocar_nav2_pkg',
          #     executable='ucsd_robocar_nav2_pkg',
          # )
    ])