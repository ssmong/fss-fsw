"""센서 서브시스템 런치 파일 (실제 하드웨어 전용).

Vicon/IMU 브릿지를 실행한다.
Vicon 브릿지는 지상국 FSS_VICON의 UDP JSON을 수신한다.
파라미터는 Hydra 트리(config/hardware/*.yaml)에서 생성 — SSoT.
UDP 수신 정보(bind_ip/port/object_name)도 config/hardware/vicon.yaml에 있다.

사용법:
  ros2 launch fss_bringup sensors.launch.py
  ros2 launch fss_bringup sensors.launch.py use_realsense:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from fss_common.launch_utils import node_params
from fss_common.paths import config_file


def generate_launch_description() -> LaunchDescription:
    """런치 설명을 생성한다."""

    use_imu_arg = DeclareLaunchArgument(
        "use_imu",
        default_value="true",
        description="IMU 브릿지 노드 사용 여부",
    )
    # 기본 false: D435i는 상태 추정에 미사용 (별도 항법 테스트 시에만 켠다)
    use_realsense_arg = DeclareLaunchArgument(
        "use_realsense",
        default_value="false",
        description="RealSense D435i 사용 여부 (별도 항법 테스트용)",
    )

    # Vicon 브릿지 (UDP x/y/yaw → SpacecraftState)
    vicon_bridge = Node(
        package="fss_sensors",
        executable="vicon_bridge_node",
        name="vicon_bridge_node",
        parameters=[node_params("vicon_bridge_node")],
        output="screen",
    )

    # IMU 브릿지 (Xsens driver sensor_msgs/Imu → FSS 본체 프레임)
    imu_bridge = Node(
        package="fss_sensors",
        executable="imu_bridge_node",
        name="imu_bridge_node",
        parameters=[node_params("imu_bridge_node")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_imu")),
    )

    # RealSense D435i (별도 항법 알고리즘 테스트용, FSS 상태 추정에는 미사용)
    # 설정은 config/hardware/realsense.yaml(SSoT)을 드라이버 config_file로 로드 —
    # 인라인 인자 대신 파일 한 곳에서만 관리 (SN 고정, 프로파일 등).
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py",
            ])
        ),
        launch_arguments={
            "config_file": str(config_file("hardware", "realsense.yaml")),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_realsense")),
    )

    return LaunchDescription([
        use_imu_arg,
        use_realsense_arg,
        vicon_bridge,
        imu_bridge,
        realsense_launch,
    ])
