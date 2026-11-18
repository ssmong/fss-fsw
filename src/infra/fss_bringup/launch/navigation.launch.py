"""항법 서브시스템 런치 파일.

파라미터는 Hydra 트리(config/gnc/navigator/ekf.yaml)에서 생성 — SSoT.

사용법:
  ros2 launch fss_bringup navigation.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from fss_common.launch_utils import node_params, use_imu_default, vicon_delay_default


def generate_launch_description() -> LaunchDescription:
    """런치 설명을 생성한다."""

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="SIL에서 /clock 사용 여부",
    )
    return LaunchDescription([
        use_sim_time_arg,
        DeclareLaunchArgument("use_imu", default_value=use_imu_default(), choices=["true", "false"]),
        DeclareLaunchArgument("vicon_delay_ms", default_value=vicon_delay_default()),
        DeclareLaunchArgument("simulated_sensors", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=navigation_nodes),
    ])


def navigation_nodes(context):
    simulated = IfCondition(LaunchConfiguration("simulated_sensors")).evaluate(context)
    physical_noise = {"vicon_timestamp_std_ms": 0.0, "accel_model_noise_density": 0.0} if simulated else {}
    return [Node(
        package="fss_navigation",
        executable="ekf_node",
        name="ekf_node",
        parameters=[
            node_params("ekf_node"),
            physical_noise,
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "use_imu": ParameterValue(LaunchConfiguration("use_imu"), value_type=bool),
                "vicon_delay_ms": ParameterValue(LaunchConfiguration("vicon_delay_ms"), value_type=float),
            },
        ],
        output="screen",
    )]
