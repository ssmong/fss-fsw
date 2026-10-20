"""항법 서브시스템 런치 파일.

파라미터는 Hydra 트리(config/gnc/navigator/ekf.yaml)에서 생성 — SSoT.

사용법:
  ros2 launch fss_bringup navigation.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from fss_common.launch_utils import node_params


def generate_launch_description() -> LaunchDescription:
    """런치 설명을 생성한다."""

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="SIL에서 /clock 사용 여부",
    )

    ekf_node = Node(
        package="fss_navigation",
        executable="ekf_node",
        name="ekf_node",
        parameters=[
            node_params("ekf_node"),
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
    ])
