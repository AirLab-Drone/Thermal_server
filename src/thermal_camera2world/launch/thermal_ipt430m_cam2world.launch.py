from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    thermal_ipt430m_share = get_package_share_directory("thermal_ipt430m")

    thermal_launch_path = os.path.join(
        thermal_ipt430m_share,
        "launch",
        "thermal_ipt430m.launch.py",
    )

    drone_yaml_path = os.path.join(
        get_package_share_directory("thermal_camera2world"),
        "config",
        "camera2world.yaml",
    )

    robot_yaml_path = os.path.join(
        get_package_share_directory("thermal_camera2world"),
        "config",
        "camera2world_robot.yaml",
    )

    thermal_camera2world_node = Node(
        package="thermal_camera2world",
        executable="thermal_camera2world.py",
        namespace="thermal_IPT430M",
        name="thermal_camera_to_world",
        output="screen",
        parameters = [drone_yaml_path]
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
