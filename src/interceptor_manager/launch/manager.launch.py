"""Start the manager node with config/manager.yaml. Does not start MAVROS or the sim."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("interceptor_manager"), "config", "manager.yaml")

    return LaunchDescription([
        # Isaac Sim doesn't publish /clock, and MAVROS stamps in wall time, so
        # all nodes use wall time to keep timestamps comparable.
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="interceptor_manager",
            executable="manager_node",
            output="screen",
            parameters=[config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
