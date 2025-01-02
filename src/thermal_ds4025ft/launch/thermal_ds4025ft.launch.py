from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='thermal_ds4025ft',
            executable='DS4025FT_ros2_node.py',
            name='DS4025FT_ros2_node',
            namespace='thermal_DS4025FT',
        ),

    ])