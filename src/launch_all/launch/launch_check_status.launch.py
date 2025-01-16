from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    launch_all_check_launch_path = os.path.join(
        get_package_share_directory("check_status"),
        "launch",
        "launch_all_check.launch.py",
    )


    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_all_check_launch_path)
            ),
        ]
    )
