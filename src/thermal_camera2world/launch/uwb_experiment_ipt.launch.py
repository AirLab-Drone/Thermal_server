from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    thermal_ipt430m_share = get_package_share_directory("thermal_ipt430m")
    thermal_camera2world_share = get_package_share_directory("thermal_camera2world")


    thermal_launch_path = os.path.join(
        thermal_ipt430m_share,
        "launch",
        "thermal_ipt430m.launch.py",
    )


    yaml_path = os.path.join(
        thermal_camera2world_share,
        "config",
        "camera2world.yaml",
    )
    print(yaml_path)


    distortion_yaml_path = os.path.join(thermal_camera2world_share, "config", "distortion_exp.yaml")
    undistortion_yaml_path = os.path.join(thermal_camera2world_share, "config", "undistortion_exp.yaml")


    undistortion_node = Node(
        package="thermal_camera2world",
        executable="clibration_compare.py",
        namespace="thermal_IPT430M",
        name="undistortion_image",
        output="screen",
    )



    thermal_camera2world_node = Node(
        package="thermal_camera2world",
        executable="uwb_experiment.py",
        namespace="thermal_IPT430M",
        name="uwb_experiment",
        output="screen",
        parameters = [
            undistortion_yaml_path,
            {"undistortion": True},
        ]
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

            undistortion_node,

            delayed_node,  # 延遲啟動的節點
        ]
    )
