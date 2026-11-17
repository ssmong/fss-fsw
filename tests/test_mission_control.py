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

    def body_status(self, link, *, age=0, **fields):
        status = dict(observer="observer_" + link.display_id, peers=[], can_start_body=True,
                      received_at=self.ui.time.monotonic() - age)
        status.update(fields)
        link.inbox.put([("connected", SimpleNamespace(**status))])

    def test_body_start_uses_its_own_fss_even_when_another_is_selected(self):
        from PySide6.QtTest import QTest

        w = self.window
        for endpoint in ("first", "second"):
            w.add_jetson(endpoint, "smrl", None, "", f"FSS-{endpoint}")
            self.assertIsNone(w._vehicles.tabButton(w._vehicles.currentIndex(), self.ui.QTabBar.RightSide))
            self.body_status(w._links[endpoint])
        first, second = w._links.values()
        w.show()
        self.app.processEvents()
        w._update()
        buttons = [w._vehicles.tabButton(i, self.ui.QTabBar.RightSide) for i in range(2)]
        self.assertTrue(all(b.isEnabled() for b in buttons))
        self.assertEqual(buttons[0].accessibleName(), "FSS-first 본체 FSW 시작")
        self.assertTrue(first.commands.empty() and second.commands.empty())
        QTest.mouseClick(buttons[0], self.ui.Qt.LeftButton)
        self.assertEqual(w._vehicles.currentIndex(), 1)
        self.assertEqual(first.commands.get_nowait()[1:], ("start_body", True))
        self.assertTrue(second.commands.empty())
        self.assertIsNone(w._vehicles.tabButton(0, self.ui.QTabBar.RightSide))
        self.assertIs(buttons[1], w._vehicles.tabButton(1, self.ui.QTabBar.RightSide))
        self.body_status(first)  # An idle heartbeat already in transit predates the remote command.
        w._update()
        self.assertIsNone(w._vehicles.tabButton(0, self.ui.QTabBar.RightSide))
        buttons[0].click()
        self.assertTrue(first.commands.empty())
        first.inbox.put([("body_reply", "기체 소프트웨어 기동")])
        w._update()
        self.assertIsNone(w._vehicles.tabButton(0, self.ui.QTabBar.RightSide))
        self.body_status(first, can_start_body=False)
        w._update()
        self.assertIsNone(w._vehicles.tabButton(0, self.ui.QTabBar.RightSide))

    def test_body_start_rechecks_pending_status_stale_link_and_reused_endpoint(self):
        w = self.window
        first, second = self.connect("host", 0), self.connect("other", 1)
        for update in (dict(age=10), dict(read_only="모니터링"), dict(can_start_body=False),
                       dict(peers=["other_observer"])):
            self.body_status(first)
            self.body_status(second, observer="other_observer")
            w._update()
            button = w._vehicles.tabButton(0, self.ui.QTabBar.RightSide)
            self.assertTrue(button.isEnabled())
            self.body_status(first, **update)
            button.click()
            self.assertIsNone(w._vehicles.tabButton(0, self.ui.QTabBar.RightSide))
            self.assertTrue(first.commands.empty())
        self.body_status(first)
        w._update()
        button = w._vehicles.tabButton(0, self.ui.QTabBar.RightSide)
        first.inbox.put([("bridge_error", "connection lost")])
        button.click()
        self.assertTrue(first.commands.empty())
        w._disconnect_jetson(0)
        replacement = self.connect("host", 0)
        self.body_status(replacement)
        w._update()
        w._start_vehicle_body("host / 0", first)
        self.assertTrue(first.commands.empty() and replacement.commands.empty())

    def test_idle_failure_can_retry_and_missing_tracking_leaves_running_tab_compact(self):
        w = self.window
        w.show()
        w.add_jetson("host", "smrl", None, "", "FSS-" + "long" * 7)
        link = w._links["host"]
        side = self.ui.QTabBar.RightSide
        self.body_status(link, can_start_body=False)
        w._update()
        self.assertIsNone(w._vehicles.tabButton(0, side))
        compact_width = w._vehicles.tabRect(0).width()
        self.body_status(link)
        w._update()
        self.assertGreater(w._vehicles.tabRect(0).width(), compact_width + 30)
        w._vehicles.tabButton(0, side).click()
        self.assertIsNone(w._vehicles.tabButton(0, side))
        self.assertEqual(w._vehicles.tabRect(0).width(), compact_width)
        link.inbox.put([("body_error", "기동 실패")])
        w._update()
        self.assertIsNone(w._vehicles.tabButton(0, side))
        self.body_status(link)
        w._update()
        self.assertTrue(w._vehicles.tabButton(0, side).isEnabled())
        self.body_status(link, can_start_body=False)
        link.data.tracking_since -= 60
        w._update()
        self.assertTrue(w._vehicles.tabText(0).endswith("Missing"))
        self.assertIsNone(w._vehicles.tabButton(0, side))
        self.assertEqual(link.commands.qsize(), 1)

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

    def test_connect_detects_workflow_and_idle_bench_offers_both_operations(self):
        w, ui = self.window, self.ui

        def first(dialog):
            connect = dialog.findChild(ui.QDialogButtonBox).button(ui.QDialogButtonBox.Ok)
            self.assertIsNone(dialog.findChild(ui.QComboBox))
            self.assertTrue(connect.isEnabled())
            return 1

        with patch.object(ui.QDialog, "exec", first):
            w._connect_dialog()
        link = w._links["192.168.0.9"]
        self.assertEqual(link.profile, "auto")
        self.assertTrue(link.commands.empty())
        self.body_status(link, workflow="toss", toss_actions=["toss_prepare"], toss_state="대기")
        w._update()
        self.assertEqual(link.shells.selected, "log")
        self.assertIsNone(w._vehicles.tabButton(0, ui.QTabBar.RightSide))
        prepare = w._toss_buttons["toss_prepare"]
        self.assertEqual(prepare.text(), "준비")
        self.assertTrue(prepare.isEnabled())
        self.assertEqual([w._toss_method.itemText(i) for i in range(w._toss_method.count())],
                         ["Nominal", "JF_only", "Proposed", "위치 제어"])
        self.assertTrue(link.commands.empty())
        w._toss_method.setCurrentText("위치 제어")
        prepare.click()
        self.assertEqual(link.commands.get_nowait()[1:], ("start_body", True))
        self.assertTrue(link.commands.empty())
        self.assertFalse(w._toss_buttons["toss_prepare"].isEnabled())
        link.inbox.put([("body_reply", "기동")])
        self.body_status(link, workflow="control", can_start_body=False)
        w._update()
        self.assertTrue(w._toss_controls.isHidden())
        self.assertFalse(w._controls.isHidden())

    def test_toss_actions_use_selected_fss_and_require_current_permissions(self):
        w = self.window
        for address in ("first", "second"):
            w.add_jetson(address, "operator", None, "", display_id=address)
            self.body_status(w._links[address], workflow="toss", can_start_body=False,
                             toss_actions=["toss_prepare"], toss_state="준비 가능")
        a, b = w._links.values()
        w.show()
        w._update()
        self.assertTrue(w._controls.isHidden())
        self.assertEqual(w._experiment_button.text(), "기록 시작")
        self.assertFalse(w._send_target_button.isEnabled())
        self.assertIsNone(w._vehicles.tabButton(1, self.ui.QTabBar.RightSide))
        for action, value in (("start_body", True), ("mode", 3), ("toss_prepare", "unknown")):
            b.request(action, value)
        self.assertTrue(b.commands.empty())
        with tempfile.TemporaryDirectory() as storage, patch.object(self.ui.QInputDialog, "getText",
                                                                     return_value=("trial", True)):
            w._experiment_root = Path(storage)
            w._experiment_button.click()
            self.assertTrue(b.data.recorder.active)
            self.assertTrue(b.commands.empty())
            w._experiment_button.click()
        self.assertEqual(list(w._toss_buttons), ["toss_prepare", "toss_start", "toss_stop"])
        w._toss_method.setCurrentText("JF_only")
        w._toss_options.setText('label:="unterminated')
        w._toss_buttons["toss_prepare"].click()
        self.assertTrue(b.commands.empty())
        self.assertIn("추가 옵션", b.data.last_command_msg)
        options = 'tracking_mode:=velocity label:="trial B"'
        w._toss_options.setText(options)
        w._toss_buttons["toss_start"].click()
        self.assertTrue(b.commands.empty())
        w._toss_buttons["toss_prepare"].click()
        self.assertEqual(b.commands.get_nowait()[1:], ("toss_prepare", ["JF_only", "tracking_mode:=velocity", "label:=trial B"]))
        w._send_toss("toss_prepare")
        self.assertFalse(b.data.recorder.active)
        self.assertTrue(a.commands.empty() and b.commands.empty())

        w._vehicles.setCurrentIndex(0)
        self.assertEqual(w._toss_method.currentText(), "Nominal")
        self.assertEqual(w._toss_options.text(), "")
        self.body_status(a, toss_actions=["toss_start", "toss_stop"], toss_state="대기")
        a.inbox.put([("paper_status", SimpleNamespace(phase="ARMED"))])
        w._update()
        self.assertEqual(w._toss_status.text(), "ARMED")
        w._toss_buttons["toss_start"].click()
        self.assertEqual(a.commands.get_nowait()[1:], ("toss_start", True))
        self.assertTrue(b.commands.empty())
        self.assertFalse(a.data.recorder.active)
        a.inbox.put([("toss_reply", "요청 수락")])
        self.body_status(a, toss_actions=["toss_stop"], toss_state="진행 중")
        w._update()
        w._toss_buttons["toss_stop"].click()
        self.assertEqual(a.commands.get_nowait()[1:], ("toss_stop", True))

        w._vehicles.setCurrentIndex(1)
        self.assertEqual(w._toss_method.currentText(), "JF_only")
        self.assertEqual(w._toss_options.text(), options)
        self.assertNotEqual(w._toss_status.text(), "ARMED")
        w._vehicles.setCurrentIndex(0)
        a.toss_pending = False
        for fields in (dict(toss_actions=[]), dict(toss_actions=["toss_start"], read_only="외부 실행 중"),
                       dict(toss_actions=["toss_start"], peers=[b.observer]),
                       dict(toss_actions=["toss_start"], age=4)):
            self.body_status(a, **fields)
            w._send_toss("toss_start")
            self.assertTrue(a.commands.empty())
        self.assertTrue(b.commands.empty())

    def test_toss_alignment_only_shows_for_fresh_prelaunch_phase(self):
        w = self.window
        w.add_jetson("bench", "operator", None, "", monitor_only=True)
        link = w._links["bench"]
        self.body_status(link, workflow="toss", toss_actions=["toss_prepare"], toss_state="기존 실험")
        report = dict(phase="IDLE", grasp_guide="Move TCP (table):",
                      launch_pose_error=[.002, -.003, .1])
        link.inbox.put([("paper_status", SimpleNamespace(**report))])
        w.show()
        w._update()
        self.assertFalse(w._toss_buttons["toss_prepare"].isEnabled())
        self.assertIsNone(w._vehicles.tabButton(0, self.ui.QTabBar.RightSide))
        self.assertTrue(w._toss_alignment.isVisible())
        self.assertIn("-2.0 mm", w._toss_alignment.text())
        link.data._last_rx["paper_status"] -= self.ui.SOURCE_TIMEOUT_SEC + 1
        w._update()
        self.assertFalse(w._toss_alignment.isVisible())
        link.inbox.put([("paper_status", SimpleNamespace(**dict(report, phase="LAUNCH")))])
        w._update()
        self.assertFalse(w._toss_alignment.isVisible())

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

    def test_executed_registration_overrides_matching_archive_id(self):
        node = self.window._node
        current, executed = "bundle/Nominal", "bundle (실행 기록)/Nominal"
        node.receive("plans", [[current, "id", [[1.9, 0.], [1.9, 1.]], [0., 1.]],
                               [executed, "id", [[2., 0.], [2., 1.]], [0., 1.]]])
        node.receive("paper_trial", ["id", 0.5, executed])
        self.assertEqual(self.window._update_plan_and_get_target()[:2], (2.0, 0.5))
        node.receive("paper_trial", ["id", 0.5, None])
        self.assertIsNone(self.window._update_plan_and_get_target())

    def test_plan_error_graph_stays_with_vehicle_and_run(self):
        w = self.window
        first = self.connect("first", 1)
        second = self.connect("second", 2)
        w._vehicles.setCurrentIndex(0)
        sample = dict(run_id="run1", plan_key="bundle/Nominal", plan_time_s=1.,
                      error_x_m=0.03, error_y_m=0.04, error_xy_m=0.05,
                      error_yaw_rad=-math.pi / 6, error_shoulder_rad=math.pi / 3)
        first.data.receive("tracking_error", SimpleNamespace(**sample))
        first.data.plan_choice = "off"
        w._update_error_view()
        self.assertEqual(w._error_curves[2].yData.tolist(), [0.05])
        self.assertAlmostEqual(w._error_curves[3].yData[-1], -30.)
        self.assertAlmostEqual(w._error_curves[4].yData[-1], 60.)
        self.assertTrue(math.isnan(w._error_curves[5].yData[-1]))
        second.data.latest_ref = (0., 0., 0.)
        w._vehicles.setCurrentIndex(1)
        w._update_error_view()
        self.assertEqual(w._error_pages.currentIndex(), 1)
        w._choose_error_view(0)
        self.assertIsNone(w._error_curves[2].xData)
        w._vehicles.setCurrentIndex(0)
        first.data.receive("paper_status", {"frame_id": "table", "plan_id": "id", "log_directory": "/run2"})
        first.data.receive("tracking_error", {**sample, "run_id": "run2", "plan_time_s": 0.2})
        w._update_error_view()
        self.assertEqual(w._error_curves[0].xData.tolist(), [0.2])
        self.assertEqual(w._error_caption.toolTip(), "run2")

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

    def test_toss_status_changes_and_clear_are_per_vehicle(self):
        w = self.window
        first = self.connect("first", 1)
        second = self.connect("second", 2)
        w._vehicles.setCurrentIndex(0)
        node = first.data
        initial = dict(phase="WAITING_FOR_A", fss_gripper="OPEN", contact_verified=False,
                       grasp_guide="Move TCP (table): ...", launch_pose_error=[0.01, -0.002, math.pi/180])
        node.receive("paper_status", SimpleNamespace(**initial))
        self.assertIn("X -10.0 mm", node.toss_alignment)
        self.assertIn("Yaw -1.0°", node.toss_alignment)
        node.receive("paper_status", initial)
        node.receive("paper_status", {**initial, "grasp_guide": "TCP within launch grasp tolerance"})
        capture = dict(phase="CAPTURING", fss_gripper="CLOSING", contact_verified=True)
        for _ in range(4):
            node.receive("paper_status", capture)
        lines = [e.message for e in node.logs]
        for message in ("Self-tossing · 캡처 진입", "그리퍼 · CLOSING", "B 레일 접촉 확인", "시작 파지점 정렬 완료"):
            self.assertEqual(lines.count(message), 1)
        self.assertFalse(node.toss_alignment)
        second.data.append_log("다른 기체")
        with tempfile.TemporaryDirectory() as root:
            directory = node.recorder.start(root, "demo", "first")
            node.append_log("보관")
            w._clear_logs.click()
            self.assertFalse(node.logs)
            self.assertFalse(w._log_text.toPlainText())
            node.receive("paper_status", capture)
            self.assertFalse(node.logs)
            node.receive("paper_status", dict(phase="FAULT", reason="sensor timeout"))
            self.assertEqual(node.logs[-1].level, 40)
            node.recorder.stop()
            self.assertIn("보관", (directory / "logs.txt").read_text(encoding="utf8"))
        self.assertEqual(second.data.logs[-1].message, "다른 기체")

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

    def test_ghost_has_a_square_body_and_articulated_plan_arm(self):
        link = self.connect("first", 1)
        pi = self.ui.math.pi
        geometry = [[[0., -.15], [0., -.2], [0., -.2]], [0., -.1]]
        plan = ["bundle/a", "id", [[1., 2.], [1., 2.]], [0., 2.], [pi/2, pi/2],
                [[0., 0., 0.], [pi, 0., pi]], geometry]
        link.inbox.put([("plans", [plan]), ("paper_trial", ["id", 1.])])
        self.window._update()
        body, arm = self.window._ghost, self.window._ghost_arm
        half = self.ui.BODY_HALF_W
        self.assertEqual(body.path().boundingRect(), self.ui.QRectF(-half, -half, 2*half, 2*half))
        self.assertLess(body.brush().color().alpha(), 42)
        path = arm.path()
        self.assertEqual(path.elementCount(), 5)
        tip = arm.mapToParent(path.currentPosition())
        self.assertAlmostEqual(tip.x(), .4)
        self.assertAlmostEqual(tip.y(), -.05)
        on_table = body.mapToParent(tip)
        self.assertAlmostEqual(on_table.x(), 1.05)
        self.assertAlmostEqual(on_table.y(), 2.4)
        link.data.arm_equipped = False
        self.window._update()
        self.assertTrue(body.isVisible())
        self.assertTrue(arm.path().isEmpty())

    def test_live_base_target_projects_fresh_measured_arm(self):
        node = self.window._node
        pose = self.ui.math.pi / 2
        stamp = SimpleNamespace(sec=1, nanosec=0)
        frame = SimpleNamespace(frame_id="table", stamp=stamp)
        def point(x, y):
            return SimpleNamespace(position=SimpleNamespace(x=x, y=y),
                                   orientation=SimpleNamespace(z=self.ui.math.sin(pose/2), w=self.ui.math.cos(pose/2)))
        node._cb_ref(SimpleNamespace(header=frame, x_ref=2., y_ref=2., theta_ref=0.))
        node._cb_arm_table(SimpleNamespace(header=frame, poses=[point(1., 1. + v) for v in (0, .1, .2, .3)]))
        node._cb_arm_tcp(SimpleNamespace(header=frame, pose=point(1.05, 1.4)))
        self.window._update_trajectory()
        path = self.window._ghost_arm.path()
        self.assertEqual(path.elementCount(), 5)
        self.assertAlmostEqual(path.currentPosition().x(), .4)
        self.assertAlmostEqual(path.currentPosition().y(), -.05)
        self.assertIn("실측 팔 자세", self.window._ghost.toolTip())
        node._last_rx["arm_pair"] -= 5
        self.window._update_trajectory()
        self.assertTrue(self.window._ghost_arm.path().isEmpty())

    def test_switch_replaces_measured_path_target_pose_and_valve_commands(self):
        first = self.connect("first", 1)
        second = self.connect("second", 2)
        header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0))
        for i, link in enumerate((first, second), start=1):
            link.inbox.put([
                ("vicon", SimpleNamespace(header=header, x=i, y=-i, theta=0)),
                ("ref", SimpleNamespace(header=header, x_ref=2*i, y_ref=i, theta_ref=-.5*i)),
                ("valve", SimpleNamespace(commanded_duty_cycles=[0.9] * 8,
                                          main_valve_open=False, modbus_rtt_ms=0)),
                ("valve_output", SimpleNamespace(duty_cycles=[i / 4] * 8)),
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

    def test_valve_chart_distinguishes_issued_zero_from_demand_and_missing_output(self):
        link = self.connect("first", 1)
        demand = SimpleNamespace(commanded_duty_cycles=[.9] * 8, main_valve_open=False, modbus_rtt_ms=1)
        link.inbox.put([("valve", demand)])
        self.window._update()
        self.assertIn("미수신", self.window._valve_title.text())
        self.assertFalse(self.window._valve_cmd_bar.isVisible())
        link.inbox.put([("valve_output", SimpleNamespace(duty_cycles=[0] * 8)), ("valve", demand)])
        self.window._update()
        self.assertTrue(self.window._valve_cmd_bar.isVisible())
        self.assertNotIn("미수신", self.window._valve_title.text())
        self.assertEqual(list(self.window._valve_cmd_bar.opts["height"]), [0] * 8)
        link.data._last_rx["valve_output"] -= self.ui.SOURCE_TIMEOUT_SEC + 1
        link.inbox.put([("valve", demand)])
        self.window._update()
        self.assertFalse(self.window._valve_cmd_bar.isVisible())
        self.assertIn("미수신", self.window._valve_title.text())

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
