from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    thermal_camera_params_file = os.path.join(
        get_package_share_directory("thermal_camera2world"),
        "config",
        "camera2world.yaml",
    )
    return LaunchDescription([
        # Declare the path to the YAML file as a launch argument
        DeclareLaunchArgument(
            "thermal_camera_params_file",
            default_value=thermal_camera_params_file,
            description="Path to the YAML file with thermal camera parameters",
        ),
        Node(
            package='thermal_ds4025ft',
            executable='DS4025FT_ros2_node.py',
            name='DS4025FT_ros2_node',
            namespace='thermal_DS4025FT',
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="thermal_camera2world",
                    executable="thermal_camera2world.py",
                    namespace="thermal_DS4025FT",
                    name="thermal_camera_to_world",
                    parameters=[LaunchConfiguration("thermal_camera_params_file")],
                )
            ],
        ),
    ])