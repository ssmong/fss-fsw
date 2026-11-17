"""Exercise the same ROS callbacks and Qt plot items used by the desktop GUI."""

import math
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mission_arm import encoder_model, joint_angles, link_points


def pose(x, y):
    return NS(position=NS(x=x, y=y), orientation=NS(z=0.0, w=1.0))


class ArmModelTest(unittest.TestCase):
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

    def test_private_arm_uses_published_geometry(self):
        node, window = self.node, self.window
        node._cb_arm_joints(NS(name=["shoulder", "elbow", "wrist"], position=[0., 0., 0.]))
        with patch.object(self.ui, "PREFERRED_WORKSPACE", "FSS_FSW"):
            window._select_arm_cad(True)
            self.assertIsNone(window._arm_model.xData)
            self.assertTrue(window._cad_choice.isHidden())
        key = window._vehicles.tabData(window._vehicles.currentIndex())
        with patch.dict(window._links, {key: NS(workspace="FSS_FSW", read_only="")}), patch.object(self.ui, "PREFERRED_WORKSPACE", "fss-fsw"):
            window._update_arm()
            self.assertIsNone(window._arm_model.xData)
            self.assertTrue(window._cad_choice.isHidden())
            node._cb_arm_base(NS(header=NS(frame_id="base_link", stamp=NS(sec=2, nanosec=0)),
                                 poses=[pose(i, 0) for i in range(4)]))
            window._update_arm()
            self.assertEqual(window._arm_model.xData.tolist(), [0, 1, 2, 3])

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
