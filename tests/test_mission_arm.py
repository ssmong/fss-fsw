"""Exercise the same ROS callbacks and Qt plot items used by the desktop GUI."""

import math
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mission_arm import ViconArmOverlay, encoder_model, joint_angles, link_points, plan_arm_points
from mission_ros import ArmRegistration


def pose(x, y):
    return NS(position=NS(x=x, y=y), orientation=NS(z=0.0, w=1.0))


def header(frame, ms):
    return NS(frame_id=frame, stamp=NS(sec=10, nanosec=ms * 1_000_000))


class ArmModelTest(unittest.TestCase):
    def test_overlay_interpolates_vicon_and_uses_tracked_body_calibration(self):
        overlay = ViconArmOverlay()
        shape = (((0, 0), (1, 0), (2, 0), (3, 0)), 0)
        with patch("time.monotonic", return_value=1) as now:
            overlay.configure([0.5, 0, -math.pi, 10, 0.04, 0.15])
            overlay.add("base_link", header("base_link", 10), shape)
            overlay.add("table", header("table", 10), (1, 2, math.radians(179)))
            self.assertIsNone(overlay.table_shape())  # no extrapolation
            overlay.add("table", header("table", 30), (3, 2, math.radians(-179)))
            points, yaw = overlay.table_shape()
            self.assertAlmostEqual(points[-1][0], 4.5)
            self.assertAlmostEqual(points[-1][1], 2)
            self.assertAlmostEqual(yaw, 0)
            now.return_value += 0.2
            self.assertIsNone(overlay.table_shape())
            overlay.configure([0, 0, 0, 0, 0.02, 0.15])
            overlay.add("base_link", header("base_link", 25), shape)
            overlay.add("table", header("table", 0), (1, 2, 0))
            overlay.add("table", header("wrong", 25), (2, 2, 0))
            overlay.add("table", header("table", 50), (3, 2, 0))
            self.assertIsNone(overlay.table_shape())  # bracketing gap too large
            overlay.configure(None)
            self.assertIsNone(overlay.table_shape())

    def test_registration_queries_only_the_model_and_invalidates_on_ambiguous_source(self):
        from unittest.mock import Mock
        import json

        graph, receive, trigger = Mock(), Mock(), Mock()
        base = NS(node_name="arm_kinematics_node", node_namespace="/", endpoint_gid=b"A")
        camera = NS(node_name="vicon_bridge_node", node_namespace="/", endpoint_gid=b"B")
        cameras = [camera]
        graph.get_publishers_info_by_topic.side_effect = lambda t: cameras if "vicon" in t else [base]
        graph.get_node_names.return_value = ["arm_kinematics_node", "vicon_bridge_node"]
        reg = ArmRegistration(graph, receive, trigger)
        graph.create_client.assert_called_once_with(trigger, "/arm/kinematics_model")
        reg.poll()
        response = reg.pending
        response.done.return_value = True
        response.result.return_value = NS(success=True, message=json.dumps({
            "schema_version": 1, "configuration": {"frame_id": "table", "base_frame_id": "base_link",
            "navigation_topic": "/navigation/state", "navigation_to_base": [1, 2, 3],
            "vicon_only_alignment_ms": 12, "maximum_sample_skew_s": .04, "feedback_timeout_s": .15}}))
        reg.poll()
        receive.assert_called_with("arm_registration", [1, 2, 3, 12, .04, .15])
        cameras.append(camera)
        reg.poll()
        receive.assert_called_with("arm_registration", None)

    def test_encoder_angles_are_matched_by_name(self):
        state = NS(name=["wrist", "shoulder", "elbow"], position=[1.0, 2.0, 3.0])
        self.assertEqual(joint_angles(state), (2, 3, 1))
        state.name = ["wrist", "shoulder", "shoulder"]
        self.assertIsNone(joint_angles(state))
        state.name = ["wrist", "shoulder", "elbow"]
        state.position[1] = math.nan
        self.assertIsNone(joint_angles(state))

    def test_base_geometry_is_in_metres(self):
        points, yaw = encoder_model((math.pi / 2, -math.pi / 2, math.pi))
        self.assertAlmostEqual(points[-1][0], 0.190250 + 0.199021)
        self.assertAlmostEqual(points[-1][1], -0.000250 + 0.199021)
        self.assertAlmostEqual(yaw, math.pi)

    def test_plan_rotates_each_offset_in_its_parent_frame(self):
        offsets = [(0.10, -0.20), (0.0, -0.40), (0.03, -0.25)]
        angles = [math.pi / 2, -math.pi / 2, math.pi / 2]
        expected = [(0, 0), (0.10, -0.20), (0.50, -0.20), (0.53, -0.45), (0.60, -0.44)]
        result = plan_arm_points(angles, offsets, (0.01, -0.07))
        self.assertEqual(len(result), len(expected))
        for actual, reference in zip(result, expected):
            self.assertAlmostEqual(actual[0], reference[0])
            self.assertAlmostEqual(actual[1], reference[1])
        self.assertEqual(plan_arm_points(angles, offsets), result[:4])

    def test_plan_requires_its_own_finite_geometry(self):
        angles = (0, 0, 0)
        offsets = [(0.20, 0), (0.10, 0), (0.15, 0)]
        x, y = plan_arm_points(angles, offsets)[-1]
        self.assertAlmostEqual(x, 0.45)
        self.assertEqual(y, 0)
        self.assertIsNone(plan_arm_points(angles, None))
        self.assertIsNone(plan_arm_points((0, math.nan, 0), offsets))
        self.assertIsNone(plan_arm_points(angles, offsets[:2]))
        self.assertIsNone(plan_arm_points(angles, offsets, (0, math.inf)))

    def test_pose_arrays_require_the_requested_frame_and_four_links(self):
        msg = NS(header=NS(frame_id="base_link"), poses=[pose(0, 0)] * 4)
        self.assertIsNone(link_points(msg, "table"))
        self.assertEqual(len(link_points(msg, "base_link")[0]), 4)
        msg.poses.pop()
        self.assertIsNone(link_points(msg, "base_link"))


class ArmPlotTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import live_plot
        except ModuleNotFoundError as exc:
            if exc.name in {"PySide6", "pyqtgraph", "numpy"}:
                raise unittest.SkipTest(f"Optional desktop dependency: {exc.name}") from exc
            raise
        cls.ui = live_plot
        with patch.dict(os.environ, QT_QPA_PLATFORM="offscreen"):
            cls.app = live_plot.QApplication.instance() or live_plot.QApplication([])

    def setUp(self):
        node = self.ui.DashboardNode()
        window = self.ui.Dashboard(node)
        self.addCleanup(self.ui.QCoreApplication.sendPostedEvents, None, self.ui.QEvent.DeferredDelete)
        self.addCleanup(window.deleteLater)
        self.addCleanup(window.close)
        window._timer.stop()
        self.node, self.window = node, window

    def test_small_map_keeps_two_bubbles_visible(self):
        node, window = self.node, self.window
        window.resize(840, 520)
        window.show()
        self.app.processEvents()
        node._cb_vicon(NS(header=NS(stamp=NS(sec=1, nanosec=0)), x=2.9, y=3.9, theta=0))
        node._cb_arm_joints(NS(name=["shoulder", "elbow", "wrist"], position=[1., 2., 3.]))
        bubbles = [window._local_bubble, self.ui.VehicleBubble()]
        window._xy_plot.addItem(bubbles[1])
        for i, bubble in enumerate(bubbles):
            bubble.show_vehicle(node, f"FSS-{i}")
        self.app.processEvents()
        occupied = []
        for bubble in bubbles:
            bubble.avoid_other_bubbles(occupied)
        first, second = occupied
        self.assertFalse(first.intersects(second))
        bounds = window._xy_plot.getViewBox().sceneBoundingRect().adjusted(-1, -1, 1, 1)
        self.assertTrue(all(bounds.contains(b) for b in occupied), (bounds, occupied))

    def test_connected_arm_explains_missing_feedback_or_table_geometry(self):
        node, window = self.node, self.window
        window._update_arm()
        self.assertTrue(window._arm_notice.isHidden())
        node.request = lambda *_: None
        with patch("time.monotonic", return_value=100) as now:
            window._update_arm()
            self.assertFalse(window._arm_notice.isHidden())
            self.assertEqual(window._arm_notice.text(), "로봇팔 신호 미수신")
            node._cb_arm_base(NS(header=NS(frame_id="base_link"), poses=[pose(i, 0) for i in range(4)]))
            window._update_arm()
            self.assertEqual(window._arm_notice.text(), "로봇팔 지도 위치 미수신")
            self.assertIsNone(window._arm_links.xData)
            node._cb_arm_table(NS(header=NS(frame_id="table"), poses=[pose(i, 0) for i in range(4)]))
            window._update_arm()
            self.assertTrue(window._arm_notice.isHidden())
            now.return_value += self.ui.XY_TRAIL_SEC
            window._update_arm()
            self.assertFalse(window._arm_notice.isHidden())
            self.assertEqual(window._arm_notice.text(), "로봇팔 신호 미수신")
            node.arm_equipped = False
            window._update_arm()
            self.assertTrue(window._arm_notice.isHidden())

    def test_detail_shows_only_selected_fresh_table_tcp_and_alignment(self):
        first, window = self.node, self.window
        second = self.ui.DashboardNode()
        first.request = second.request = lambda *_: None
        tip = NS(header=header("table", 10), pose=pose(0.12345, -0.402))
        tip.pose.orientation = NS(x=0.0, y=0.0, z=math.sqrt(.5), w=math.sqrt(.5))
        with patch("time.monotonic", return_value=100) as now:
            first._cb_arm_tcp(tip)
            first._cb_paper_status(dict(phase="IDLE", grasp_guide="Move TCP (table):",
                                        launch_pose_error=[.002, -.003, .1]))
            window._update_arm()
            self.assertEqual(window._tcp_readout.text(), "테이블 파지점   X 0.123 m   Y -0.402 m   Yaw 90.0°")
            self.assertIn("-2.0 mm", window._arm_alignment.text())
            self.assertFalse(window._arm_alignment.isHidden())
            with patch.object(window, "_node", second):
                tip.header = header("base_link", 20)
                second._cb_arm_tcp(tip)
                window._update_arm()
                self.assertEqual(window._tcp_readout.text(), "테이블 파지점 · 미수신")
                self.assertTrue(window._arm_alignment.isHidden())
                self.assertEqual(window._arm_alignment.text(), "")
            tip.header = header("table", 30)
            tip.pose.orientation = NS(x=0, y=0, z=0, w=0)
            first._cb_arm_tcp(tip)
            window._update_arm()
            self.assertEqual(window._tcp_readout.text(), "테이블 파지점   X 0.123 m   Y -0.402 m")
            now.return_value += self.ui.ARM_TIMEOUT_SEC
            window._update_arm()
            self.assertEqual(window._tcp_readout.text(), "테이블 파지점 · 미수신")
            now.return_value += self.ui.SOURCE_TIMEOUT_SEC
            window._update_arm()
            self.assertTrue(window._arm_alignment.isHidden())
            self.assertEqual(window._arm_alignment.text(), "")

    def test_map_prefers_published_table_links_over_camera_overlay(self):
        node, window = self.node, self.window
        node.request = lambda *_: None
        with patch("time.monotonic", return_value=100) as now:
            body = NS(header=header("table", 10), x=1, y=2, theta=math.pi / 2)
            base = NS(header=header("base_link", 10), poses=[pose(i, 0) for i in range(4)])
            node._cb_arm_joints(NS(name=["shoulder", "elbow", "wrist"], position=[0, 0, 0]))
            node._cb_arm_registration([0, 0, 0, 0, .04, .15])
            node._cb_vicon(body)
            window._update_arm()
            self.assertIsNone(window._arm_links.xData)  # joint angles alone do not identify the model
            node._cb_arm_base(base)
            window._update_arm()
            self.assertEqual(window._arm_links.yData.tolist(), [2, 3, 4, 5])
            self.assertIsNone(window._arm_tip.xData)
            self.assertTrue(window._arm_notice.isHidden())
            window._update_arm(allow_overlay=False)
            self.assertIsNone(window._arm_links.xData)
            now.return_value += .16
            window._update_arm()
            self.assertIsNone(window._arm_links.xData)
            self.assertFalse(window._arm_notice.isHidden())
            body.header = header("table", 15)
            base.header = header("base_link", 15)
            node._cb_vicon(body)
            node._cb_arm_base(base)
            node._cb_arm_table(NS(header=header("table", 15), poses=[pose(i, 0) for i in range(4)]))
            window._update_arm()
            self.assertEqual(window._arm_links.yData.tolist(), [0, 0, 0, 0])

    def test_private_arm_uses_published_geometry(self):
        node, window = self.node, self.window
        node._cb_arm_joints(NS(name=["shoulder", "elbow", "wrist"], position=[0., 0., 0.]))
        with patch.object(self.ui, "PREFERRED_WORKSPACE", "FSS_FSW"):
            window._select_arm_cad(True)
            self.assertIsNone(window._arm_model.xData)
            self.assertTrue(window._cad_choice.isHidden())
        key = window._vehicles.tabData(window._vehicles.currentIndex())
        for workspace, reason, private_arm in (("FSS_FSW", "", False), ("FSS_FSW/runtime/cpp", "", False),
                                               ("fss-fsw", "FSS_FSW/runtime/cpp 사용 중 · 모니터링만 가능", False),
                                               ("fss-fsw", "", True)):
            link = NS(workspace=workspace, read_only=reason, private_arm=private_arm)
            with self.subTest(workspace=workspace), patch.dict(window._links, {key: link}), patch.object(self.ui, "PREFERRED_WORKSPACE", "fss-fsw"):
                window._update_arm()
                self.assertIsNone(window._arm_model.xData)
                self.assertTrue(window._cad_choice.isHidden())
                node._cb_arm_base(NS(header=NS(frame_id="base_link", stamp=NS(sec=2, nanosec=0)),
                                     poses=[pose(0, -i) for i in range(4)]))
                window._update_arm()
                self.assertEqual(window._arm_model.yData.tolist(), [0, -1, -2, -3])
                node.arm_base = None

    def test_frames_source_expiry_and_tcp_trail(self):
        node, window = self.node, self.window
        stamp = NS(sec=22, nanosec=42000)
        tip = NS(header=NS(frame_id="table", stamp=stamp), pose=pose(0, 1))
        with patch("time.monotonic", return_value=100) as now, patch.object(self.ui, "PREFERRED_WORKSPACE", "fss-fsw"):
            node._cb_arm_driver(NS(data="dry_run=True; mode=position"))
            node._cb_arm_joints(NS(name=["shoulder", "elbow", "wrist"], position=[0., 0., 1.]))
            window._update_arm()
            self.assertIn("모의", window._arm_readout.text())
            self.assertIn("57.3°", window._arm_readout.text())
            self.assertIsNone(window._arm_model.xData)
            self.assertFalse(window._local_bubble.isVisible())
            node._cb_vicon(NS(header=NS(stamp=stamp), x=0.3, y=1.2, theta=math.pi/2))
            window._update_arm()
            self.assertTrue(window._local_bubble.isVisible())
            self.assertIn("yaw 90.0°", window._local_bubble.toPlainText())
            self.assertEqual(window._local_bubble.toPlainText().split(), ["x", "0.30", "y", "1.20", "yaw", "90.0°"])
            window._select_arm_cad(True)
            self.assertEqual(len(window._arm_model.xData), 4)
            self.assertIsNone(window._arm_links.xData)
            for i in range(60):
                now.return_value = 100 + i / 10
                tip.pose.position.x = i / 10
                node._cb_arm_tcp(tip)
            links = NS(header=NS(frame_id="table", stamp=stamp), poses=[pose(i, 1) for i in range(4)])
            node._cb_arm_table(links)
            window._update_arm()
            self.assertEqual(window._arm_links.xData.tolist(), [0, 1, 2, 3])
            self.assertGreaterEqual(window._arm_trace.xData[0], 1.0)
            self.assertEqual(window._arm_tip.xData.tolist(), [5.9])
            self.assertEqual(window._gripper_link.xData.tolist(), [3, 5.9])
            tip.header.stamp = NS(sec=23, nanosec=0)
            node._cb_arm_tcp(tip)
            window._update_arm()
            self.assertEqual(window._gripper_link.xData.tolist(), [3, 5.9])
            now.return_value += 0.16
            window._update_arm()
            self.assertIsNone(window._gripper_link.xData)
            tip.header.frame_id = "base_link"
            node._cb_arm_tcp(tip)
            window._update_arm()
            self.assertEqual(window._arm_tip.xData.tolist(), [5.9])
            now.return_value += 1
            window._update_arm()
            self.assertEqual(window._arm_links.opts["pen"].color().name(), self.ui.C_IDLE)
            now.return_value += 4
            window._update_arm()
            self.assertIsNone(window._arm_trace.xData)
            self.assertIsNone(window._arm_tip.xData)
            self.assertIsNone(window._arm_links.xData)
            self.assertFalse(window._local_bubble.isVisible())
            node._cb_arm_table(links)
            window._update_arm()
            self.assertTrue(window._local_bubble.isVisible())
            self.assertEqual(window._arm_links.opts["pen"].color().name(), self.ui.C_ARM)
            node.arm_equipped = False
            window._update_arm()
            self.assertIsNone(window._arm_links.xData)
            self.assertFalse(window._tabs.isTabVisible(window._arm_tab))


if __name__ == "__main__":
    unittest.main()
