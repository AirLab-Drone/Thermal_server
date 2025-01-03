from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    thermal_ipt430m_share = get_package_share_directory("thermal_ds4025ft")

    thermal_launch_path = os.path.join(
        thermal_ipt430m_share,
        "launch",
        "thermal_ds4025ft.launch.py",
    )


    # 定義 thermal_camera2world 節點
    setup_camera2world_point_node = Node(
        package="thermal_camera2world",
        executable="setup_camera2world_point.py",
        namespace="thermal_DS4025FT",
        name="setup_camera2world_point",
        output="screen",
    )

    # 使用 TimerAction 包裹節點，延遲啟動
    delayed_node = TimerAction(
        period=3.0,
        actions=[setup_camera2world_point_node],
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(thermal_launch_path)
            ),
            delayed_node,  # 延遲啟動的節點
        ]
    )
