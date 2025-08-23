from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
          Node(
            package='lane_following',
            executable='lane_following_executable',
            output='screen',
            emulate_tty=True
          )
    ])