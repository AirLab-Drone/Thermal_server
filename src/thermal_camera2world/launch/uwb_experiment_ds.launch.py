from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    thermal_ds4025ft_share = get_package_share_directory("thermal_ds4025ft")

    thermal_launch_path = os.path.join(
        thermal_ds4025ft_share,
        "launch",
        "thermal_ds4025ft.launch.py",
    )

    yaml_path = os.path.join(
        get_package_share_directory("thermal_camera2world"),
        "config",
        "camera2world.yaml",
    )


    thermal_camera2world_node = Node(
        package="thermal_camera2world",
        executable="uwb_experiment.py",
        namespace="thermal_DS4025FT",
        name="thermal_camera_to_world",
        output="screen",
        parameters = [yaml_path]
    )
    # 使用 TimerAction 包裹節點，延遲啟動
    delayed_node = TimerAction(
        period=3.0,
        actions=[thermal_camera2world_node],
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(thermal_launch_path)
            ),
            delayed_node,  # 延遲啟動的節點
        ]
    )
