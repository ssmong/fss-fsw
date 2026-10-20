"""실제 권한/스케줄러 변경 없이 실행 명령 조합과 입력 거부를 검사한다."""

import shlex
import unittest
from unittest.mock import patch

from fss_common.launch_utils import execution_prefix, scheduling_defaults


class ExecutionPrefixTest(unittest.TestCase):
    def test_default_does_not_wrap_node(self):
        self.assertEqual(execution_prefix(False, "4", "0"), "")

    def test_affinity_and_fifo_can_be_selected_independently(self):
        cases = [
            (True, "0", ["taskset", "-c", "4"]),
            (False, "40", ["chrt", "-f", "40"]),
            (True, "40", ["chrt", "-f", "40", "taskset", "-c", "4"]),
        ]
        for pin, priority, expected in cases:
            with self.subTest(pin=pin, priority=priority):
                self.assertEqual(shlex.split(execution_prefix(pin, "4", priority)), expected)

    def test_priority_bounds(self):
        for priority in ("1", "99"):
            with self.subTest(priority=priority):
                self.assertEqual(
                    shlex.split(execution_prefix(False, "4", priority)),
                    ["chrt", "-f", priority],
                )

    def test_invalid_priority_is_rejected_instead_of_falling_back(self):
        for priority in ("-1", "100", "1.5", "", "fifo", "40 --other"):
            with self.subTest(priority=priority):
                with self.assertRaisesRegex(ValueError, "rt_priority"):
                    execution_prefix(False, "4", priority)

    def test_cpu_argument_stays_one_argument(self):
        for core in ("0", "10", "0,2", "1-3", "4 --other"):
            with self.subTest(core=core):
                self.assertEqual(
                    shlex.split(execution_prefix(True, core, "40")),
                    ["chrt", "-f", "40", "taskset", "-c", core],
                )


class SchedulingConfigTest(unittest.TestCase):
    def load(self, overrides=""):
        with patch.dict("os.environ", {"FSS_HYDRA_OVERRIDES": overrides}):
            return scheduling_defaults()

    def test_default_profile_preserves_normal_execution(self):
        config = self.load()
        self.assertEqual(config["cpu_pin"], "false")
        for kind in ("gnc", "actuator"):
            self.assertEqual(
                execution_prefix(False, config[f"{kind}_cpu_core"], config[f"{kind}_rt_priority"]),
                "",
            )

    def test_trial_profile_and_explicit_override_reach_execution_command(self):
        config = self.load(
            "scheduling=rt_trial scheduling.cpu_pin=true scheduling.gnc_cpu_core=2 "
            "scheduling.gnc_rt_priority=35"
        )
        self.assertEqual(config["actuator_rt_priority"], "30")
        self.assertEqual(config["actuator_cpu_core"], "5")
        self.assertEqual(config["cpu_pin"], "true")
        self.assertEqual(
            shlex.split(execution_prefix(True, config["gnc_cpu_core"], config["gnc_rt_priority"])),
            ["chrt", "-f", "35", "taskset", "-c", "2"],
        )

    def test_invalid_config_is_rejected(self):
        for override in (
            "scheduling.cpu_pin=invalid",
            "scheduling.gnc_rt_priority=true",
            "scheduling.actuator_rt_priority=100",
            "scheduling.gnc_rt_priority=1.5",
            "scheduling.gnc_cpu_core=-1",
        ):
            with self.subTest(override=override):
                with self.assertRaisesRegex(ValueError, "scheduling"):
                    self.load(override)


if __name__ == "__main__":
    unittest.main()
