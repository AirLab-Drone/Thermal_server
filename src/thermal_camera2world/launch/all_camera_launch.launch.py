from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("thermal_ds4025ft"),
                        "launch/thermal_ds4025ft.launch.py",
                    ),
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("thermal_ipt430m"),
                        "launch/thermal_ipt430m.launch.py",
                    ),
                ),
            ),
            Node(
                package="thermal_camera2world",
                executable="compare_thermalAlert.py",
                name="compare_thermalAlert",
                output="screen",
                namespace="compare_thermalAlert",
            ),
        ]
    )
