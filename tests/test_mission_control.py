"""Run the desktop dashboard offscreen with isolated, in-memory Jetson connections."""

import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


class MissionControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        scripts = str(Path(__file__).resolve().parents[1] / "scripts")
        with patch.object(sys, "path", [scripts, *sys.path]):
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
        self.window = self.ui.Dashboard(self.ui.DashboardNode())
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.window._vehicle_settings = self.ui.QSettings(
            str(Path(temp.name) / "vehicles.ini"), self.ui.QSettings.IniFormat)
        self.addCleanup(self.close_window)
        self.start = patch.object(self.ui.JetsonLink, "start")
        self.start.start()
        self.addCleanup(self.start.stop)

    def close_window(self):
        from PySide6.QtCore import QCoreApplication, QEvent

        self.window.close()
        self.window.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)

    def connect(self, host, domain):
        self.window.add_jetson(host, "operator", domain, "")
        link = self.window._links[f"{host} / {domain}"]
        link.inbox.put([("connected", None)])
        self.window._update()
        return link

    def test_bundled_korean_font_on_controls_and_map(self):
        from PySide6.QtGui import QFontDatabase, QRawFont

        fonts = (self.window._btn_estop.font(), self.window._log_text.font(),
                 self.window._xy_plot.getAxis("left").label.font(),
                 self.window._local_bubble.textItem.font())
        for font in fonts:
            raw = QRawFont.fromFont(font)
            self.assertEqual(raw.familyName(), "Pretendard Variable")
            self.assertTrue(raw.supportsCharacter(ord("팔")))
        fixed = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.assertEqual(self.window._terminal.font().family(), fixed.family())

    def test_air_bearing_shows_telemetry_without_optimistic_switch_on_click(self):
        w = self.window
        link = self.connect("first", 1)
        buttons = (w._btn_air_on, w._btn_air_off)

        def checked():
            w._update()
            return tuple(button.isChecked() for button in buttons)

        def report(name, **values):
            link.inbox.put([(name, SimpleNamespace(**values))])

        self.assertEqual(checked(), (False, False))
        report("mode", mode_name="READY")
        self.assertEqual(checked(), (False, False))
        report("valve", commanded_duty_cycles=[0] * 8, main_valve_open=False, modbus_rtt_ms=1)
        self.assertEqual(checked(), (False, True))
        self.assertTrue(w._btn_air_on.isEnabled())
        w._btn_air_on.click()
        self.assertEqual(link.commands.get_nowait()[1:], ("valve", True))
        self.assertEqual(tuple(b.isChecked() for b in buttons), (False, True))
        link.inbox.put([("reply", "에어베어링 ON")])
        self.assertEqual(checked(), (False, True))
        report("valve", commanded_duty_cycles=[0] * 8, main_valve_open=True, modbus_rtt_ms=1)
        self.assertEqual(checked(), (True, False))

        w.add_jetson("monitor", "operator", None, "", monitor_only=True)
        monitor = w._links["monitor"]
        monitor.inbox.put([("connected", SimpleNamespace(observer="b", peers=[], read_only=""))])
        self.assertEqual(checked(), (False, False))
        monitor.inbox.put([("valve", SimpleNamespace(
            commanded_duty_cycles=[0] * 8, main_valve_open=False, modbus_rtt_ms=1))])
        self.assertEqual(checked(), (False, True))
        self.assertFalse(any(b.isEnabled() for b in buttons))
        w._btn_air_on.click()
        self.assertTrue(monitor.commands.empty())
        w._vehicles.setCurrentIndex(0)
        self.assertEqual(checked(), (True, False))
        link.changed.emit("disconnected")
        self.assertEqual(checked(), (False, False))

    def test_air_bearing_only_falls_back_to_health_with_fresh_actuators(self):
        w = self.window
        link = self.connect("first", 1)
        health = dict(navigation_ok=True, navigation_degraded=False, navigation_age_sec=0.01,
                      control_loop_rate_hz=50, modbus_rtt_ms=1, warnings=[], errors=[])

        def report(opened, ready):
            link.inbox.put([("health", SimpleNamespace(
                **health, actuators_ok=ready, main_valve_open=opened))])
            w._update()
            return w._btn_air_on.isChecked(), w._btn_air_off.isChecked()

        self.assertEqual(report(False, False), (False, False))
        self.assertEqual(report(True, True), (True, False))
        link.inbox.put([("valve", SimpleNamespace(
            commanded_duty_cycles=[0] * 8, main_valve_open=False, modbus_rtt_ms=1))])
        self.assertEqual(report(True, True), (False, True))
        link.data._last_rx["valve"] -= self.ui.SOURCE_TIMEOUT_SEC + 1
        self.assertEqual(report(True, False), (False, False))
        self.assertEqual(report(False, True), (False, True))
        link.data._last_rx["health"] -= self.ui.SOURCE_TIMEOUT_SEC + 1
        w._update()
        self.assertFalse(w._btn_air_off.isChecked())
        self.assertFalse(w._btn_air_on.isChecked())

    def test_monitor_profile_blocks_commands_and_changes_mode_on_reconnect(self):
        self.window.add_jetson("observer", "operator", None, "", monitor_only=True)
        observer = self.window._links["observer"]
        observer.inbox.put([("connected", SimpleNamespace(observer="a", peers=[], read_only=""))])
        self.window._update()
        self.assertFalse(self.window._send_target_button.isEnabled())
        self.assertTrue(self.window._target_x.isEnabled())
        observer.request("mode", 5)
        self.assertTrue(observer.commands.empty())
        self.window.add_jetson("observer", "operator", None, "", monitor_only=True)
        self.assertIs(self.window._links["observer"], observer)
        self.window.add_jetson("observer", "operator", None, "")
        self.assertTrue(observer.stopped.is_set())
        self.assertFalse(self.window._links["observer"].monitor_only)

    def test_coordinates_accept_first_click_typing_offline_and_in_monitor_mode(self):
        from PySide6.QtCore import QPoint, Qt
        from PySide6.QtTest import QTest

        w = self.window
        w.show()
        self.app.processEvents()
        w._update()
        for size in ((1200, 760), (840, 520)):
            w.resize(*size)
            self.app.processEvents()
            w._target_y.setFocus()
            for field, text in ((w._target_x, "1.25"), (w._target_y, "3.456"), (w._target_yaw, "-73.5")):
                editor = field.lineEdit()
                self.assertTrue(editor.isEnabled())
                QTest.mouseClick(editor, Qt.LeftButton)
                QTest.keyClicks(editor, text)
                w._update()
                self.assertEqual(editor.text(), text)
            self.assertEqual(w._read_target(), ("PID", 1.25, 3.456, -73.5))
            self.assertFalse(w._send_target_button.isEnabled())
            self.assertLess(w._controls.height(), 110)
            controls = [w._target_x, w._target_y, w._target_yaw, w._send_target_button,
                        w._controller.button(0), w._controller.button(1), w._btn_air_on, w._btn_air_off]
            rectangles = [button.rect().translated(button.mapTo(w._controls, QPoint())) for button in controls]
            for index, rect in enumerate(rectangles):
                self.assertTrue(w._controls.rect().contains(rect))
                self.assertFalse(any(rect.intersects(other) for other in rectangles[:index]))
        w.add_jetson("observer", "operator", None, "", monitor_only=True)
        link = w._links["observer"]
        link.inbox.put([("connected", SimpleNamespace(observer="a", peers=[], read_only=""))])
        w._update()
        QTest.mouseClick(w._target_x.lineEdit(), Qt.LeftButton)
        QTest.keyClicks(w._target_x.lineEdit(), "2.345")
        self.assertEqual(w._read_target()[1], 2.345)
        w._send_target_button.click()
        self.assertTrue(link.commands.empty())

    def test_recording_follows_each_vehicle_without_sending_commands(self):
        w = self.window
        with tempfile.TemporaryDirectory() as storage:
            w._experiment_root = Path(storage)
            first = self.connect("first", 1)
            with patch.object(self.ui.QInputDialog, "getText", return_value=("trial", True)) as name:
                w._experiment_button.click()
                self.assertEqual(name.call_args.kwargs["text"], "default")
            self.assertTrue(first.data.recorder.active)
            self.assertTrue(first.data.recorder.directory.is_dir())
            self.assertEqual(w._experiment_button.text(), "실험 종료")

            w.add_jetson("second", "operator", None, "", monitor_only=True)
            second = w._links["second"]
            second.inbox.put([("connected", SimpleNamespace(observer="b", peers=[], read_only=""))])
            w._update()
            self.assertEqual(w._experiment_button.text(), "실험 시작")
            with patch.object(self.ui.QInputDialog, "getText", return_value=("second_trial", True)) as name:
                w._experiment_button.click()
                self.assertEqual(name.call_args.kwargs["text"], "trial")
            self.assertTrue(second.data.recorder.active)
            w._experiment_button.click()
            self.assertFalse(second.data.recorder.active)
            self.assertTrue(first.data.recorder.active)
            w._vehicles.setCurrentIndex(0)
            w._disconnect_button.click()
            self.assertFalse(first.data.recorder.active)
            self.assertTrue(first.commands.empty())
            self.assertTrue(second.commands.empty())

    def test_dialog_cancel_and_close_finalize_recording(self):
        w = self.window
        with tempfile.TemporaryDirectory() as storage:
            w._experiment_root = Path(storage)
            link = self.connect("first", 1)
            with patch.object(self.ui.QInputDialog, "getText", return_value=("default", False)):
                w._experiment_button.click()
            self.assertFalse(link.data.recorder.active)
            self.assertEqual(list(Path(storage).iterdir()), [])
            with patch.object(self.ui.QInputDialog, "getText", return_value=("default", True)):
                w._experiment_button.click()
            self.assertTrue(link.data.recorder.active)
            w.close()
            self.assertFalse(link.data.recorder.active)
            self.assertTrue(link.commands.empty())

    def test_vehicle_id_is_display_only_and_survives_reconnect(self):
        self.window.add_jetson("host-a", "operator", None, "", " FSS-1 ")
        first = self.window._links["host-a"]
        first.inbox.put([("connected", None)])
        self.window.add_jetson("host-b", "smrl", None, "", "FSS-1")
        second = self.window._links["host-b"]
        second.inbox.put([("connected", None)])
        self.window._update()
        self.assertEqual(self.window._vehicles.tabData(1), "host-b")
        self.assertEqual(self.window._vehicles.tabText(1), "FSS-1 (host-b) · 수신 대기")
        self.assertEqual(self.window._vehicles.tabToolTip(1), "smrl@host-b")
        self.window._send_target_button.click()
        self.assertTrue(first.commands.empty())
        self.assertFalse(second.commands.empty())
        self.assertFalse(self.window._vehicles.tabsClosable())
        self.assertFalse(self.window._vehicles.drawBase())
        self.window._disconnect_button.click()
        self.assertTrue(second.stopped.is_set())
        self.assertFalse(first.stopped.is_set())
        self.assertEqual(self.window._vehicles.tabText(0), "FSS-1 · 수신 대기")
        self.window.add_jetson("host-b", "smrl", None, "")
        self.assertEqual(self.window._links["host-b"].display_id, "FSS-1")

    def test_rename_reuses_connection_and_clearing_id_restores_address(self):
        self.window.add_jetson("host-a", "smrl", None, "", "FSS-1")
        link = self.window._links["host-a"]
        link.read_only = "실험 사용 중"
        self.window.add_jetson("host-a", "smrl", None, "", "FSS-2")
        self.assertIs(link, self.window._links["host-a"])
        self.window.add_jetson("host-a", "smrl", None, "", "")
        self.assertEqual(self.window._vehicles.tabText(0), "host-a · 연결 중")

    def test_arm_equipment_is_per_fss_and_survives_reconnect(self):
        self.window.add_jetson("host-a", "smrl", None, "", arm_equipped=False)
        first = self.window._node
        self.window.add_jetson("host-b", "smrl", None, "")
        self.assertTrue(self.window._node.arm_equipped)
        self.window._vehicles.setCurrentIndex(0)
        self.assertIs(self.window._node, first)
        self.assertFalse(first.arm_equipped)
        self.window._disconnect_button.click()
        self.window.add_jetson("host-a", "smrl", None, "")
        self.assertFalse(self.window._node.arm_equipped)
        reconnected = self.window._node
        self.window.add_jetson("host-a", "smrl", None, "", arm_equipped=True)
        self.assertIs(reconnected, self.window._node)
        self.assertTrue(reconnected.arm_equipped)

    def test_plan_auto_selection_and_manual_preview_stay_with_vehicle(self):
        first = self.connect("first", 1)
        second = self.connect("second", 2)
        key = "bundle/Nominal"
        for link in (first, second):
            # Registered body yaw turns through 180° while the path goes northeast.
            link.inbox.put([("plans", [[key, "id", [[1., 0.], [2., 3.]], [0., 4.],
                                       self.ui.np.radians([170., 190.]).tolist()]])])
        first.inbox.put([("paper_trial", ["id", 1.])])
        self.window._vehicles.setCurrentIndex(0)
        self.window._update()
        self.assertEqual(self.window._plan_curve.xData.tolist(), [1, 2])
        self.assertIsNone(self.window._xy_plot.plotItem.legend)
        ghost = self.window._ghost
        self.assertTrue(ghost.isVisible())
        self.assertEqual((ghost.x(), ghost.y()), (1.25, 0.75))
        self.assertAlmostEqual(ghost.rotation(), 175.)
        self.assertEqual(ghost.path(), self.window._ghost_outline)
        first.inbox.put([("paper_trial", ["id", 6.])])
        self.window._update()
        self.assertEqual((ghost.x(), ghost.y()), (2., 3.))
        self.assertAlmostEqual(ghost.rotation(), 190.)
        first.data._last_rx["paper_trial"] -= 10
        self.window._update()
        self.assertIsNone(self.window._plan_curve.xData)
        self.assertFalse(ghost.isVisible())
        self.window._plan_picker.setCurrentIndex(2)
        self.window._choose_plan(2)
        self.assertEqual(self.window._plan_curve.xData.tolist(), [1, 2])
        self.assertFalse(ghost.isVisible())
        self.window._vehicles.setCurrentIndex(1)
        self.assertIsNone(self.window._plan_curve.xData)
        self.window._vehicles.setCurrentIndex(0)
        self.assertEqual(self.window._plan_picker.currentData(), key)
        self.assertEqual(self.window._plan_curve.xData.tolist(), [1, 2])
        self.assertTrue(first.commands.empty())
        self.assertTrue(second.commands.empty())

    def test_commands_require_distinguishable_vehicles(self):
        first = self.connect("first", 0)
        second = self.connect("second", 0)
        for link, observer in ((first, "a"), (second, "b")):
            link.inbox.put([("connected", SimpleNamespace(observer=observer, peers=["a", "b"]))])
        self.window._update()
        self.window._send_target_button.click()
        self.assertTrue(second.commands.empty())
        self.assertFalse(self.window._send_target_button.isEnabled())
        self.assertTrue(self.window._target_x.isEnabled())
        first.peers, second.peers = ["a"], ["b"]
        self.window._update()
        self.assertTrue(self.window._send_target_button.isEnabled())

    def test_main_log_tracks_the_selected_vehicle_and_repeated_messages(self):
        first = self.connect("first", 1)
        self.connect("second", 2)
        self.window.show()
        self.app.processEvents()
        self.assertTrue(self.window._terminal.isVisible())
        self.assertEqual(self.window._tabs.indexOf(self.window._log_panel), -1)
        self.window._console_tabs.setCurrentIndex(1)
        self.assertTrue(self.window._log_panel.isVisible())
        message = SimpleNamespace(level=30, name="nav", msg="measurement <lost>")
        for _ in range(2):
            first.inbox.put([("log", message)])
            self.window._update()
        self.assertNotIn("measurement", self.window._log_text.toPlainText())
        self.window._vehicles.setCurrentIndex(0)
        self.assertTrue(self.window._terminal.isVisible())
        self.window._console_tabs.setCurrentIndex(1)
        self.assertEqual(self.window._log_text.toPlainText().count("measurement <lost>"), 2)

    def test_independent_ssh_tabs_and_log_selection_per_vehicle(self):
        first = self.connect("fss-a", 1)
        self.window._add_shell.click()
        extra = first.shells.sessions[2]
        self.assertIs(self.window._terminal.shell, extra)
        second = self.connect("fss-b", 2)
        self.assertEqual(self.window._console_tabs.count(), 2)
        self.window._console_tabs.setCurrentIndex(1)
        self.assertIsNone(self.window._terminal.shell)
        self.window._vehicles.setCurrentIndex(0)
        self.assertIs(self.window._terminal.shell, extra)
        self.window._close_shell.click()
        self.assertTrue(extra.stop.is_set())
        self.assertIs(self.window._terminal.shell, first.shells.sessions[1])
        self.assertFalse(first.shells.sessions[1].stop.is_set())
        self.window._vehicles.setCurrentIndex(1)
        self.assertEqual(self.window._console.currentWidget(), self.window._log_panel)
        self.assertFalse(second.shells.sessions[1].stop.is_set())

    def test_log_level_refilters_buffered_messages_immediately(self):
        first = self.connect("first", 1)
        for level in self.ui.LOG_LEVELS:
            first.inbox.put([("log", SimpleNamespace(level=level, name="gnc", msg=f"event-{level}"))])
        first.inbox.put([("reply_error", "target rejected"), ("reply", "target applied")])
        self.window._update()
        level_picker = self.window._log_level
        text = self.window._log_text.toPlainText
        self.assertEqual(level_picker.currentText(), "INFO")
        self.assertNotIn("event-10", text())
        self.assertIn("[INFO] gnc: event-20", text())
        level_picker.setCurrentText("DEBUG")
        self.assertIn("[DEBUG] gnc: event-10", text())
        level_picker.setCurrentText("WARNING")
        self.assertNotIn("target applied", text())
        self.assertIn("[WARNING] gnc: event-30", text())
        level_picker.setCurrentText("ERROR")
        self.assertNotIn("event-30", text())
        self.assertIn("[ERROR] target rejected", text())
        level_picker.setCurrentText("FATAL")
        self.assertNotIn("event-40", text())
        self.assertIn("[FATAL] gnc: event-50", text())
        self.connect("second", 2)
        self.assertEqual(text(), "")
        self.window._vehicles.setCurrentIndex(0)
        self.assertIn("event-50", text())

    def test_command_and_reply_stay_with_selected_vehicle(self):
        first = self.connect("first", 1)
        second = self.connect("second", 2)
        self.window._controller.button(1).click()
        self.window._target_x.setValue(1.25)
        self.window._target_y.setValue(3.5)
        self.window._target_yaw.setValue(-90)
        self.window._send_target_button.click()
        self.assertTrue(first.commands.empty())
        self.assertEqual(second.commands.get_nowait()[1:], ("target", ["lqr", 1.25, 3.5, -math.pi / 2]))

        self.window._vehicles.setCurrentIndex(0)
        self.assertEqual(self.window._read_target(), ("PID", 0.0, 0.0, 0.0))
        unavailable = "/gnc/set_target 서비스 미연결"
        second.inbox.put([("reply", unavailable)])
        self.window._update()
        self.assertEqual(self.window._command_status.text(), "")
        self.window._vehicles.setCurrentIndex(1)
        self.assertEqual(self.window._command_status.text(), unavailable)
        self.assertEqual(self.window._read_target(), ("LQR", 1.25, 3.5, -90.0))
        second.changed.emit("connection closed")
        self.window._update()
        self.assertFalse(self.window._send_target_button.isEnabled())
        self.assertFalse(self.window._btn_estop.isEnabled())
        self.window._vehicles.setCurrentIndex(0)
        self.assertTrue(self.window._send_target_button.isEnabled())

    def test_position_only_plan_has_no_heading_and_waits_for_executor_clock(self):
        link = self.connect("first", 1)
        link.inbox.put([("plans", [["bundle/old", "id", [[1., 2.], [2., 2.]], [0., 1.]]]),
                        ("paper_trial", ["id", None])])
        self.window._update()
        ghost = self.window._ghost
        self.assertFalse(ghost.isVisible())
        link.inbox.put([("paper_trial", ["id", .5])])
        self.window._update()
        self.assertTrue(ghost.isVisible())
        self.assertEqual((ghost.x(), ghost.y()), (1.5, 2.))
        self.assertEqual(ghost.path(), self.window._ghost_ring)

    def test_switch_replaces_measured_path_target_pose_and_valve_commands(self):
        first = self.connect("first", 1)
        second = self.connect("second", 2)
        header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0))
        for i, link in enumerate((first, second), start=1):
            link.inbox.put([
                ("vicon", SimpleNamespace(header=header, x=i, y=-i, theta=0)),
                ("ref", SimpleNamespace(header=header, x_ref=2*i, y_ref=i, theta_ref=-.5*i)),
                ("valve", SimpleNamespace(commanded_duty_cycles=[i / 4] * 8,
                                          main_valve_open=False, modbus_rtt_ms=0)),
            ])
        for tab, expected_x in ((1, 2), (0, 1)):
            self.window._vehicles.setCurrentIndex(tab)
            self.window._update()
            self.assertEqual(list(self.window._vicon_curve.xData), [expected_x])
            ghost = self.window._ghost
            self.assertTrue(ghost.isVisible())
            self.assertEqual((ghost.x(), ghost.y()), (2 * expected_x, expected_x))
            self.assertAlmostEqual(ghost.rotation(), self.ui.math.degrees(-.5 * expected_x))
            self.assertEqual(list(self.window._valve_cmd_bar.opts["height"]), [expected_x / 4] * 8)
        first.data._last_rx["reference"] -= 6
        self.window._update()
        self.assertFalse(ghost.isVisible())
        self.window._disconnect_jetson(0)
        self.assertIs(self.window._node, second.data)
        self.window._disconnect_jetson(0)
        self.assertIsNone(self.window._vicon_curve.xData)
        self.assertFalse(ghost.isVisible())
        self.assertFalse(self.window._valve_cmd_bar.isVisible())

    def test_stale_mode_blocks_open_and_tracking_expires_after_five_seconds(self):
        link = self.connect("first", 1)
        pose = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
            x=0.3, y=0.2, theta=0, vx=0, vy=0, omega=0, std_x=0, std_y=0, std_theta=0,
        )
        link.inbox.put([("mode", SimpleNamespace(mode_name="READY")), ("vicon", pose)])
        self.window._update()
        self.assertTrue(self.window._btn_air_on.isEnabled())
        self.assertTrue(self.window._position_dot.isVisible())

        now = self.ui.time.monotonic() + self.ui.SOURCE_TIMEOUT_SEC + 1.0
        with patch.object(self.ui.time, "monotonic", return_value=now):
            self.window._update()
            self.window._btn_air_on.click()
            self.assertTrue(link.commands.empty())
            self.assertFalse(self.window._btn_air_on.isEnabled())
            self.assertTrue(self.window._position_dot.isVisible())
            self.assertTrue(self.window._spacecraft_body.isVisible())
            self.assertEqual(list(self.window._vicon_curve.xData), [pose.x])
            self.window._btn_air_off.click()
            self.assertEqual(link.commands.get_nowait()[1:], ("valve", False))

        with patch.object(self.ui.time, "monotonic", return_value=link.data._last_rx["vicon"] + 5):
            link.inbox.put([("connected", None), ("log", SimpleNamespace(level=20, name="gnc", msg="alive"))])
            self.window._update()
            self.assertTrue(self.window._vehicles.tabText(0).endswith("Missing"))
            self.assertFalse(self.window._position_dot.isVisible())
            self.assertFalse(self.window._spacecraft_body.isVisible())
            self.assertIsNone(self.window._vicon_curve.xData)
            link.inbox.put([("vicon", pose)])
            self.window._update()
            self.assertTrue(self.window._vehicles.tabText(0).endswith("Online"))
            self.assertTrue(self.window._spacecraft_body.isVisible())

    def test_monitoring_status_uses_table_tracking_only_for_equipped_fss(self):
        with patch.object(self.ui.time, "monotonic", return_value=100):
            self.window.add_jetson("arm", "operator", None, "", monitor_only=True)
            link = self.window._links["arm"]
            link.inbox.put([("connected", None)])
            self.window._update()
            self.assertTrue(self.window._vehicles.tabText(0).endswith("수신 대기"))
        with patch.object(self.ui.time, "monotonic", return_value=105):
            self.window._update()
            self.assertTrue(self.window._vehicles.tabText(0).endswith("Missing"))
            position = SimpleNamespace(x=1.2, y=0.6)
            pose = SimpleNamespace(position=position, orientation=SimpleNamespace(z=0, w=1))
            link.data._cb_arm_table(SimpleNamespace(header=SimpleNamespace(frame_id="table"), poses=[pose]*4))
            self.assertEqual(link.data.tracking_status(), "Online")
            link.data.arm_equipped = False
            self.assertEqual(link.data.tracking_status(), "Missing")

    def test_xy_trail_expires_by_elapsed_time_without_new_messages(self):
        first = self.connect("first", 1)
        second = self.connect("second", 2)
        header = SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=0))
        for stamp, x in ((100, 1.0), (104, 1.5)):
            with patch.object(self.ui.time, "monotonic", return_value=stamp):
                for link in (first, second):
                    link.inbox.put([("vicon", SimpleNamespace(header=header, x=x, y=2.0, theta=0))])
                self.window._update()
        for stamp, expected in ((105, [1.5]), (109, [])):
            with patch.object(self.ui.time, "monotonic", return_value=stamp):
                self.window._update()
                for curve in (self.window._vicon_curve, self.window._trails["first / 1"][0]):
                    self.assertEqual([] if curve.xData is None else list(curve.xData), expected)

    def test_xy_table_keeps_exact_ranges_and_equal_meter_scales_across_resize(self):
        self.window.show()
        for width, height in ((1200, 600), (900, 830)):
            self.window.resize(width, height)
            self.app.processEvents()
            view = self.window._xy_plot.getViewBox()
            self.assertEqual(view.viewRange(), [[0, 3], [0, 4]])
            self.assertAlmostEqual(view.width() / 3, view.height() / 4)
            self.assertEqual(view.state["mouseEnabled"], [False, False])

    def test_table_perimeter_follows_table_edges_below_trajectories(self):
        border = self.window._table_perimeter
        self.assertEqual(border.rect().getRect(), (0, 0, 3, 4))
        self.assertLess(border.zValue(), self.window._vicon_curve.zValue())
        self.assertTrue(border.pen().isCosmetic())

    def test_ekf_only_does_not_draw_a_measured_pose(self):
        link = self.connect("vehicle", 1)
        link.inbox.put([("ekf", SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
            x=1, y=2, theta=1, vx=0, vy=0, omega=0, std_x=0, std_y=0, std_theta=0,
        ))])
        self.window._update()
        self.assertIsNone(self.window._vicon_curve.xData)
        self.assertFalse(self.window._spacecraft_body.isVisible())

    def test_console_grows_after_controls_and_valve_axes_fit(self):
        from PySide6.QtCore import QPoint

        w = self.window
        w.show()
        terminal_heights = []
        for width, height in ((840, 520), (1200, 760)):
            w.resize(width, height)
            self.app.processEvents()
            plot = w._valve_cmd_bar.getViewWidget()
            panel = plot.parentWidget()
            self.assertGreaterEqual(w._controls.height(), w._controls.minimumSizeHint().height())
            self.assertGreaterEqual(panel.height(), panel.minimumSizeHint().height())
            self.assertTrue(panel.rect().contains(plot.geometry()))
            self.assertLess(panel.mapTo(w, panel.rect().bottomLeft()).y(),
                            w._console_tabs.mapTo(w, QPoint()).y())
            self.assertGreaterEqual(w._console.height(), 90)
            terminal_heights.append(w._console.height())
        self.assertGreater(terminal_heights[1], terminal_heights[0])
        self.assertGreater(terminal_heights[1], w._controls.height() * 2)


if __name__ == "__main__":
    unittest.main()
