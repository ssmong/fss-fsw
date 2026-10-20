"""반응휠 bench 드라이버. dry_run:=false이면 YAML에 지정한 USB 장치를 연결한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from fss_common.paths import config_file


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument(
            "config_file", default_value=str(config_file("hardware", "reaction_wheel.yaml")),
        ),
        Node(
            package="fss_actuators",
            executable="reaction_wheel_driver_node",
            name="reaction_wheel_driver_node",
            parameters=[{
                "config_file": ParameterValue(LaunchConfiguration("config_file"), value_type=str),
                "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
            }],
            output="screen",
        ),
    ])
