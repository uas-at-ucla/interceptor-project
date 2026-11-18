from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="interceptor_vision",
            executable="vision_camera_node",
            output="screen",
        ),
        Node(
            package="interceptor_tracker",
            executable="tracker_node",
            output="screen",
        ),
        Node(
            package="interceptor_manager",
            executable="manager_node",
            output="screen",
        ),
    ])
