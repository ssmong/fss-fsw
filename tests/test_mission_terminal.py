"""SSH terminal routing and VT rendering without connecting to any machine."""

import os
import socket
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch


class TerminalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with patch.object(sys, "path", [str(Path(__file__).resolve().parents[1] / "scripts"), *sys.path]):
            try:
                from mission_terminal import JetsonShell, ShellSessions, TerminalView, pyte
                from PySide6.QtCore import Qt
                from PySide6.QtTest import QTest
                from PySide6.QtWidgets import QApplication
            except ModuleNotFoundError as exc:
                if exc.name == "PySide6":
                    raise unittest.SkipTest("Optional desktop dependency: PySide6") from exc
                raise
            if not pyte:
                raise unittest.SkipTest("Optional desktop dependency: pyte")
        cls.Shell, cls.View, cls.Qt, cls.QTest = JetsonShell, TerminalView, Qt, QTest
        cls.Sessions = ShellSessions
        with patch.dict(os.environ, QT_QPA_PLATFORM="offscreen"):
            cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.view = self.View()
        self.view.resize(500, 175)
        self.view.show()
        self.app.processEvents()
        self.addCleanup(self.view.close)

    def test_selected_shell_owns_input_even_during_switch(self):
        first, second = self.Shell(), self.Shell()
        for shell in (first, second):
            shell.opened.set()
            shell.status = ""
        self.view.bind(first)
        self.QTest.keyClicks(self.view, "pwd")
        self.QTest.keyClick(self.view, self.Qt.Key_Return)
        self.view.bind(second)
        self.QTest.keyClick(self.view, self.Qt.Key_C, self.Qt.ControlModifier)
        self.QTest.keyClick(self.view, self.Qt.Key_Backspace)
        self.QTest.keyClick(self.view, self.Qt.Key_Up)
        self.assertEqual(b"".join(first.input.get_nowait() for _ in range(4)), b"pwd\r")
        self.assertEqual(b"".join(second.input.get_nowait() for _ in range(3)), b"\x03\x7f\x1b[A")
        self.view.bind(None)
        self.QTest.keyClick(self.view, self.Qt.Key_Return)
        self.assertTrue(second.input.empty())

    def test_escape_sequences_unicode_split_and_screen_isolation(self):
        first, second = self.Shell(), self.Shell()
        self.view.bind(first)
        text = "오래된 값\r\x1b[2K새 값".encode()
        first.output.put(text[:-1])
        first.drain()
        first.output.put(text[-1:])
        first.drain()
        first.status = ""
        self.view.refresh()
        self.assertEqual(self.view.toPlainText().strip(), "새 값")
        self.view.bind(second)
        self.assertNotIn("새 값", self.view.toPlainText())
        self.view.bind(first)
        self.assertEqual(self.view.toPlainText().strip(), "새 값")

    def test_clipboard_paste_honors_terminal_bracket_mode(self):
        shell = self.Shell()
        shell.opened.set()
        self.view.bind(shell)
        shell.output.put(b"\x1b[?2004h")
        shell.drain()
        self.app.clipboard().setText("first\nsecond")
        self.QTest.keyClick(self.view, self.Qt.Key_V, self.Qt.ControlModifier)
        self.assertEqual(shell.input.get_nowait(), b"\x1b[200~first\rsecond\x1b[201~")

    def test_each_tab_gets_its_own_channel_and_closes_independently(self):
        class Channel:
            def __init__(self):
                self.closed = threading.Event()
                self.received = threading.Event()
                self.data = bytearray()

            def __enter__(self):
                return self

            def __exit__(self, *args):
                self.closed.set()

            def settimeout(self, timeout):
                self.timeout = timeout

            def recv(self, size):
                if self.closed.wait(self.timeout):
                    return b""
                raise socket.timeout()

            def sendall(self, payload):
                self.data.extend(payload)
                self.received.set()

        class Client:
            def invoke_shell(self, **kwargs):
                channel = Channel()
                channels.append(channel)
                return channel

        sessions = self.Sessions()
        self.addCleanup(sessions.join)
        self.addCleanup(sessions.close)
        channels = []
        sessions.attach(Client())
        first = sessions.sessions[1]
        self.assertTrue(first.opened.wait(2))
        second_key = sessions.add()
        second = sessions.sessions[second_key]
        self.assertTrue(second.opened.wait(2))
        self.assertEqual(len(channels), 2)
        self.assertFalse(any(c.data for c in channels))
        second.send(b"pwd\r")
        self.assertTrue(channels[1].received.wait(2))
        self.assertEqual(channels[1].data, b"pwd\r")
        sessions.remove(second_key)
        self.assertTrue(channels[1].closed.wait(2))
        self.assertFalse(channels[0].closed.is_set())
        first.send(b"ls\r")
        self.assertTrue(channels[0].received.wait(2))
        sessions.close()
        sessions.join()
        self.assertTrue(channels[0].closed.is_set())
        self.assertIsNone(sessions.add())


if __name__ == "__main__":
    unittest.main()
