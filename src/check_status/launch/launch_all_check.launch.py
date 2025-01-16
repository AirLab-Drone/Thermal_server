from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    check_drone_status = Node(
        package="check_status",
        executable="check_drone_status.py",
        name="check_drone_status",
        output="screen"
    )

    check_thermal_status = Node(
        package="check_status",
        executable="check_thermal_camera.py",
        name="check_thermal_status",
        output="screen"
    )

    check_UpS_status = Node(
        package="check_status",
        executable="check_UpS_status.py",
        name="check_UpS_status",
        output="screen"
    )

    send_fire_alert = Node(
        package="check_status",
        executable="send_fire_alert.py",
        name="send_fire_alert",
        output="screen"
    )

    return LaunchDescription(
        [
            check_drone_status,
            check_thermal_status,
            check_UpS_status,
            send_fire_alert,
        ]
    )
