"""Start MAVROS (PX4 config) connected to Pegasus PX4 SITL, plus the stub node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    px4_launch = os.path.join(get_package_share_directory("mavros"), "launch", "px4.launch")

    return LaunchDescription([
        # PX4 SITL's offboard MAVLink link: listens on 14540, sends to 14580.
        DeclareLaunchArgument("fcu_url", default_value="udp://:14540@127.0.0.1:14580"),
        DeclareLaunchArgument("gcs_url", default_value=""),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(px4_launch),
            launch_arguments={
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": LaunchConfiguration("gcs_url"),
            }.items(),
        ),
        Node(
            package="interceptor_mavros_stub",
            executable="mavros_stub_node",
            output="screen",
        ),
    ])
