"""전체 런치의 SIL 구성. 팔·추력기·GNC 선택 인자를 그대로 지원한다."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """본체 시뮬레이션을 켜고 나머지 실행 옵션은 전체 런치에서 처리한다."""
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("fss_bringup"), "launch", "fss_full.launch.py"]
                    )
                ),
                launch_arguments={"use_sim": "true"}.items(),
            )
        ]
    )
