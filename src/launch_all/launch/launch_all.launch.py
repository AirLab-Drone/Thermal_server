from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    launch_check_status = os.path.join(
        get_package_share_directory("launch_all"),
        "launch",
        "launch_check_status.launch.py",
    )


    launch_detect_fire = os.path.join(
        get_package_share_directory("launch_all"),
        "launch",
        "launch_detect_fire.launch.py",
    )



    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_check_status)
            ),            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_detect_fire)
            ),
        ]
    )
