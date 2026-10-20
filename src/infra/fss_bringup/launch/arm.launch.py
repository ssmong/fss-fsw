"""Hydra 설정으로 로봇팔 위치·속도·전류 명령 노드를 실행한다."""

from fss_common.launch_utils import node_params
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """기본은 dry_run이며 enable 서비스 전까지 움직이지 않는다."""
    return LaunchDescription(
        [
            Node(
                package="fss_arm",
                executable="arm_driver_node",
                name="arm_driver_node",
                parameters=[node_params("arm_driver_node")],
                output="screen",
            )
        ]
    )
