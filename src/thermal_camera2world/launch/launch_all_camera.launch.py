from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    thermal_ipt430m_cam2world_launch_path = os.path.join(
        get_package_share_directory("thermal_camera2world"),
        "launch",
        "thermal_ipt430m_cam2world.launch.py",
    )

    thermal_ds4025ft_cam2world_launch_path = os.path.join(
        get_package_share_directory("thermal_camera2world"),
        "launch",
        "thermal_ds4025ft_cam2world.launch.py",
    )

    compare_thermalAlert = Node(
        package="thermal_camera2world",
        executable="compare_thermalAlert.py",
        name="compare_thermalAlert",
        output="screen",
    )

    # 使用 TimerAction 包裹節點，延遲啟動
    delayed_node = TimerAction(
        period=3.0,
        actions=[compare_thermalAlert],
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(thermal_ipt430m_cam2world_launch_path)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(thermal_ds4025ft_cam2world_launch_path)
            ),
            delayed_node,  # 延遲啟動的節點
        ]
    )
