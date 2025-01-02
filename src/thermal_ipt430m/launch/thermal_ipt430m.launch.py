import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # 獲取lib路徑的相對路徑
    package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    lib_path = os.path.join(package_path, "lib")

    # 設置LD_LIBRARY_PATH
    ld_library_path_current = os.environ.get("LD_LIBRARY_PATH", "")

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH", value=f"{lib_path}:{ld_library_path_current}"
            ),
            Node(
                package="thermal_ipt430m",
                executable="thermal_ipt430m_node",
                name="thermal_ipt430m",
                output="screen",
                namespace="thermal_IPT430M",
            ),
        ]
    )
