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
    def test_toss_air_changes_only_valve_without_position_control_modes(self):
        for value in (True, False):
            calls, replies = [], []
            future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=True))
            actions = mission_ros.MissionActions(lambda *args: calls.append(args) or future, replies.append)
            actions.request("toss_air", value)
            actions.tick()
            self.assertEqual(calls, [("toss_air", value)])
            self.assertEqual(replies, ["에어베어링 ON" if value else "에어베어링 OFF"])

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
            modules[module] = SimpleNamespace(**{name: type(name, (), {"Request": SimpleNamespace}) for name in names.split()})
        modules["rclpy"] = SimpleNamespace(init=Mock(), create_node=Mock(return_value=node))
        modules["rclpy.qos"] = SimpleNamespace(qos_profile_sensor_data=object())
        catalog = SimpleNamespace(plans=[], signatures={}, defaults={}, refresh=Mock(return_value=False))
        with patch.dict(sys.modules, modules), patch.multiple(
            mission_ros, create=True, PlanCatalog=lambda: catalog, load_table_plans=Mock(),
            paper_plan_elapsed=Mock(return_value=None), paper_run_files=Mock(return_value=None), PaperTracking=PaperTracking,
            ValveOutput=Mock(return_value=Mock(topics=("/actuators/valve_commands", "/actuators/valve_outputs"))), ArmRegistration=Mock(),
        ):
            rejects = Mock()
            _, request = mission_ros.open_node(lambda kind, value: events.append((kind, value)), Mock(), reject=rejects)
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
            node.get_node_names.return_value = ["valve_driver_node"]
            client = node.create_client.return_value
            client.call_async.return_value = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=True))
            bench = Mock()
            bench.can_control_air.return_value = True
            with patch.multiple(mission_ros, create=True, toss_bench=bench, toss_monitor=False,
                                read_only="Self-tossing", local_service_owner=Mock(return_value=False)):
                request("toss_air", True)
                client.call_async.assert_called_once()
                self.assertIs(client.call_async.call_args.args[0].open, True)
                request.__self__.tick()
                bench.can_control_air.return_value = False
                request("toss_air", False)
                client.call_async.assert_called_once()
                rejects.assert_called_once()
