from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # # Get the path to the YAML file
    # thermal_camera_params_file = os.path.join(
    #     get_package_share_directory("thermal_camera2world"),
    #     "config",
    #     "camera2world.yaml",
    # )

    return LaunchDescription(
        [
            # # Declare the path to the YAML file as a launch argument
            # DeclareLaunchArgument(
            #     "thermal_camera_params_file",
            #     default_value=thermal_camera_params_file,
            #     description="Path to the YAML file with thermal camera parameters",
            # ),
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
                package='thermal_camera2world',
                executable='compare_thermalAlert.py',
                name='compare_thermalAlert',
                output='screen',
                namespace='compare_thermalAlert',
            ),
        ]
    )
