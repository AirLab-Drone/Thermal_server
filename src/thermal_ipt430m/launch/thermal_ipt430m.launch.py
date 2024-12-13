import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # 獲取lib路徑的相對路徑
    package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    lib_path = os.path.join(package_path, 'lib')

    # 設置LD_LIBRARY_PATH
    ld_library_path_current = os.environ.get('LD_LIBRARY_PATH', '')
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

        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=f'{lib_path}:{ld_library_path_current}'
        ),

        Node(
            package='thermal_ipt430m',
            executable='thermal_ipt430m_node',
            name='thermal_ipt430m',
            output='screen',
            namespace='thermal_IPT430M',
        ),

        # TimerAction(
        #     period=3.0,
        #     actions=[
        #         Node(
        #             package="thermal_camera2world",
        #             executable="thermal_camera2world.py",
        #             namespace="thermal_IPT430M",
        #             name="thermal_camera_to_world",
        #             parameters=[LaunchConfiguration("thermal_camera_params_file")],
        #         )
        #     ],
        # ),
    ])