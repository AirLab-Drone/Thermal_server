import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the path to the YAML file
    thermal_camera_params_file = os.path.join(
        get_package_share_directory('thermal_camera2world'), 'config', 'camera2world.yaml')

    return LaunchDescription([
        # Declare the path to the YAML file as a launch argument
        DeclareLaunchArgument(
            'thermal_camera_params_file',
            default_value=thermal_camera_params_file,
            description='Path to the YAML file with thermal camera parameters'
        ),
        # Launch the node with the specified parameters
        Node(
            package='thermal_camera2world',
            executable='test_thermal_camera2world.py',
            name='thermal_camera_to_world',
            parameters=[LaunchConfiguration('thermal_camera_params_file')]
        )
    ])