import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = os.path.join(
        get_package_share_directory("sonar_bringup"),
        "config",
        "pipeline.yaml",
    )

    adapter_type = LaunchConfiguration("adapter_type")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "adapter_type",
                default_value="uuvsim",
                description="Select adapter source: uuvsim or real",
            ),
            Node(
                package="sonar_adapters",
                executable="uuvsim_adapter_node",
                name="uuvsim_adapter",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(
                    PythonExpression(["'", adapter_type, "' == 'uuvsim'"])
                ),
            ),
            Node(
                package="sonar_adapters",
                executable="real_sonar_adapter_node",
                name="real_sonar_adapter",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(
                    PythonExpression(["'", adapter_type, "' == 'real'"])
                ),
            ),
            Node(
                package="sonar_perception",
                executable="yolo_infer_node",
                name="yolo_infer",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
