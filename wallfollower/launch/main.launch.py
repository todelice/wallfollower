from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='wall_finder',
            output='screen'),
        Node(
            package='wall_follower',
            executable='wall_following',
            output='screen'),
        Node(
            package='wall_follower',
            executable='odom_recorder',
            output='screen'),
    ])
