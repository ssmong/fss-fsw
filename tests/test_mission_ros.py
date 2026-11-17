"""ROS bridge catalog updates with an in-memory node and no hardware."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import mission_ros
from mission_tracking import PaperTracking


class CatalogRelayTest(unittest.TestCase):
    def test_idle_timer_publishes_changes_and_method_defaults_without_status(self):
        timers, events = [], []
        node = Mock()
        node.get_clock.return_value.now.return_value.nanoseconds = 1000_000_000_000
        node.create_timer.side_effect = lambda interval, callback: timers.append((interval, callback))
        modules = {}
        declarations = {
            "fss_interfaces.msg": "SpacecraftState SystemHealth SystemMode TrajectoryReference ValveCommands ValveStatus",
            "fss_interfaces.srv": "MainValveControl SetController SetSystemMode SetTarget",
            "rcl_interfaces.msg": "Log",
            "rcl_interfaces.srv": "GetParameters",
            "geometry_msgs.msg": "PoseArray PoseStamped",
            "sensor_msgs.msg": "Imu JointState",
            "std_msgs.msg": "String",
            "std_srvs.srv": "Trigger",
        }
        for module, names in declarations.items():
            modules[module] = SimpleNamespace(**{name: type(name, (), {}) for name in names.split()})
        modules["rclpy"] = SimpleNamespace(init=Mock(), create_node=Mock(return_value=node))
        modules["rclpy.qos"] = SimpleNamespace(qos_profile_sensor_data=object())
        catalog = SimpleNamespace(plans=[], signatures={}, defaults={}, refresh=Mock(return_value=False))
        with patch.dict(sys.modules, modules), patch.multiple(
            mission_ros, create=True, PlanCatalog=lambda: catalog, load_table_plans=Mock(),
            paper_plan_elapsed=Mock(return_value=None), paper_run_files=Mock(return_value=None), PaperTracking=PaperTracking,
            ValveOutput=Mock(), ArmRegistration=Mock(),
        ):
            mission_ros.open_node(lambda kind, value: events.append((kind, value)), Mock())
            self.assertEqual(events[:2], [("plans", []), ("plan_defaults", {})])
            refresh = next(callback for _, callback in timers if callback.__name__ == "refresh_plans")
            refresh()
            self.assertEqual(len(events), 2)
            catalog.plans = [["bundle/Nominal", "A", [[0, 0], [1, 1]], [0, 1]]]
            catalog.defaults = {"Nominal": "bundle/Nominal"}
            catalog.refresh.return_value = True
            refresh()
            self.assertEqual(events[-2:], [("plans", catalog.plans), ("plan_defaults", catalog.defaults)])
            node.create_client.return_value.call_async.assert_not_called()
            import json
            status = next(call.args[2] for call in node.create_subscription.call_args_list
                          if call.args[1] == "/paper_toss/status")
            with patch.object(PaperTracking, "poll", return_value=[{"final_sample": True}]):
                events.clear()
                status(SimpleNamespace(data=json.dumps({"frame_id": "table", "plan_id": "A", "phase": "FAULT"})))
                self.assertEqual([name for name, _ in events], ["paper_trial", "tracking_error", "paper_status"])
                events.clear()
                status(SimpleNamespace(data=json.dumps({"frame_id": "table", "plan_id": "A", "phase": "PREPARING"})))
                self.assertEqual(events[0][0], "paper_status")
