"""실제 launch 평가로 프로필·CLI 인자의 전달을 검사하고 노드 실행만 대체한다."""

import sys
from pathlib import Path
from types import ModuleType

import pytest

pytest.importorskip("launch")

from launch import Action, LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.descriptions import Executable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

BRINGUP = Path(__file__).resolve().parents[1] / "src/infra/fss_bringup"


@pytest.mark.parametrize(
    ("filename", "profile", "arguments", "expected"),
    [
        ("fss_full", "", {}, {"gnc_node": [], "valve_driver_node": []}),
        (
            "fss_full", "scheduling=rt_trial scheduling.cpu_pin=true", {},
            {"gnc_node": ["chrt", "-f", "30", "taskset", "-c", "4"],
             "valve_driver_node": ["chrt", "-f", "30", "taskset", "-c", "5"]},
        ),
        (
            "fss_full", "scheduling=rt_trial",
            {"gnc_rt_priority": "35", "actuator_rt_priority": "0"},
            {"gnc_node": ["chrt", "-f", "35"], "valve_driver_node": []},
        ),
        (
            "sim", "scheduling=rt_trial", {"gnc_rt_priority": "36"},
            {"gnc_node": ["chrt", "-f", "36"],
             "valve_driver_node": ["chrt", "-f", "30"]},
        ),
        (
            "gnc", "scheduling=rt_trial", {"rt_priority": "0", "cpu_pin": "true", "cpu_core": "2"},
            {"gnc_node": ["taskset", "-c", "2"]},
        ),
        (
            "actuators", "scheduling=rt_trial", {"rt_priority": "37"},
            {"valve_driver_node": ["chrt", "-f", "37"]},
        ),
        (
            "fss_full", "scheduling=rt_trial", {"use_thrusters": "false"},
            {"valve_driver_node": ["chrt", "-f", "30"]},
        ),
        (
            "gnc", "scheduling=rt_trial", {"gnc_backend": "python"},
            {"gnc_node": ["chrt", "-f", "30"]},
        ),
        (
            "sim", "", {"gnc_backend": "python"},
            {"gnc_node": [], "valve_driver_node": []},
        ),
    ],
)
def test_launch_scheduling_arguments(monkeypatch, filename, profile, arguments, expected):
    """하드웨어나 RT 권한 없이 Include/OpaqueFunction 및 prefix 토큰화를 실행한다."""
    captured = {}
    executables = {}

    class RecordingNode(Action):
        def __init__(self, *, name, prefix="", condition=None, **kwargs):
            super().__init__(condition=condition)
            self.name = name
            self.node_executable = kwargs["executable"]
            self.executable = Executable(cmd=["node-placeholder"], prefix=prefix)

        def execute(self, context):
            self.executable.prepare(context, self)
            assert self.name not in captured
            captured[self.name] = self.executable.final_cmd[:-1]
            executables[self.name] = self.node_executable

    modules = {
        "launch_ros": {},
        "launch_ros.actions": {"Node": RecordingNode},
        "launch_ros.parameter_descriptions": {"ParameterValue": lambda value, **kwargs: value},
        "launch_ros.substitutions": {
            "FindPackageShare": lambda package: TextSubstitution(text=str(BRINGUP)),
        },
    }
    for name, members in modules.items():
        module = ModuleType(name)
        module.__dict__.update(members)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setenv("FSS_HYDRA_OVERRIDES", profile)

    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(BRINGUP / "launch" / f"{filename}.launch.py")),
            launch_arguments={"use_sim": "true", **arguments}.items(),
        ),
    ]))
    assert service.run() == 0
    scheduled = {name: prefix for name, prefix in captured.items()
                 if name in ("gnc_node", "valve_driver_node")}
    assert scheduled == expected
    if "gnc_node" in expected:
        assert executables["gnc_node"] == (
            "gnc_node.py" if arguments.get("gnc_backend") == "python" else "gnc_node"
        )
    for node in ("ekf_node", "manager_node", "valve_driver_node"):
        if node in executables:
            assert executables[node] == node
