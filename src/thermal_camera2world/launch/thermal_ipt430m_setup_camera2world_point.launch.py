
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    thermal_ipt430m_share = get_package_share_directory("thermal_ipt430m")
    
    # print(f'path: {os.getcwd()}')  # path: /home/thermal-server/thermal_server
    camera2world_path = os.getcwd() + "/src/thermal_camera2world"

    drone_yaml_path = os.path.join(camera2world_path, "config", "camera2world.yaml")
    robot_yaml_path = os.path.join(camera2world_path, 'config', 'camera2world_robot.yaml')




    distortion_yaml_path = os.path.join(camera2world_path, "config", "distortion_exp.yaml")
    undistortion_yaml_path = os.path.join(camera2world_path, "config", "undistortion_exp.yaml")




    thermal_launch_path = os.path.join(
        thermal_ipt430m_share,
        "launch",
        "thermal_ipt430m.launch.py",
    )



    undistortion_node = Node(
        package="thermal_camera2world",
        executable="clibration_compare.py",
        namespace="thermal_IPT430M",
        name="undistortion_image",
        output="screen",
    )


    # 定義 thermal_camera2world 節點
    setup_camera2world_point_node = Node(
        package="thermal_camera2world",
        executable="setup_camera2world_point.py",
        namespace="thermal_IPT430M",
        name="uwb_experiment",
        parameters=[{
            'config_file':distortion_yaml_path,
            'undistortion': False,
            }],
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
            undistortion_node,
            delayed_node,  # 延遲啟動的節點
        ]
    )
