"""Independent SSH terminals sharing a Jetson's authenticated connection."""

import queue
import socket
import threading

from PySide6.QtCore import Qt
from PySide6.QtGui import QFontDatabase, QKeySequence, QTextCursor
from PySide6.QtWidgets import QApplication, QPlainTextEdit, QTextEdit

try:
    import pyte
except ImportError:
    pyte = None


class JetsonShell:
    def __init__(self):
        self.screen = pyte.HistoryScreen(80, 16, history=500) if pyte else None
        self.stream = pyte.ByteStream(self.screen) if pyte else None
        self.output = queue.Queue(maxsize=32)
        self.input = queue.SimpleQueue()
        self.size = (80, 16)
        self.opened = threading.Event()
        self.stop = threading.Event()
        self.finished = False
        self.status = "SSH 연결 중…" if pyte else "터미널 사용에는 pyte 설치가 필요합니다."
        self.revision = 0

    def resize(self, columns, rows):
        if self.screen and (columns, rows) != self.size:
            self.size = (columns, rows)
            self.screen.resize(lines=rows, columns=columns)
            self.revision += 1

    def send(self, data):
        if self.opened.is_set() and not self.stop.is_set() and data:
            self.input.put(data)

    def drain(self):
        if not self.stream:
            return
        while True:
            try:
                self.stream.feed(self.output.get_nowait())
                self.revision += 1
            except queue.Empty:
                return

    def serve(self, client):
        stop = self.stop
        if not self.screen or stop.is_set():
            return
        try:
            size = self.size
            with client.invoke_shell(term="xterm", width=size[0], height=size[1]) as channel:
                channel.settimeout(0.05)
                self.status = ""
                self.opened.set()
                while not stop.is_set():
                    if self.size != size:
                        size = self.size
                        channel.resize_pty(width=size[0], height=size[1])
                    while not self.input.empty() and not stop.is_set():
                        channel.sendall(self.input.get_nowait())
                    try:
                        data = channel.recv(16384)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    while not stop.is_set():
                        try:
                            self.output.put(data, timeout=0.1)
                            break
                        except queue.Full:
                            continue
                self.status = "SSH 셸 종료"
        except Exception as exc:
            self.status = f"SSH 셸: {exc}"
        finally:
            self.opened.clear()
            self.finished = True


class ShellSessions:
    def __init__(self):
        self.sessions = {1: JetsonShell()}
        self.next_id = 2
        self.selected = 1
        self._lock = threading.Lock()
        self._client = None
        self._closed = False
        self._workers = set()

    def _start(self, shell):
        client = self._client

        def run():
            try:
                shell.serve(client)
            finally:
                with self._lock:
                    self._workers.discard(threading.current_thread())

        worker = threading.Thread(target=run, daemon=True)
        self._workers.add(worker)
        worker.start()

    def attach(self, client):
        with self._lock:
            if not self._closed:
                self._client = client
                for shell in self.sessions.values():
                    self._start(shell)

    def add(self):
        with self._lock:
            if self._closed:
                return
            key = self.next_id
            self.next_id += 1
            shell = self.sessions[key] = JetsonShell()
            if self._client is not None:
                self._start(shell)
            self.selected = key
            return key

    def remove(self, key):
        with self._lock:
            if key == 1 or key not in self.sessions:
                return
            self.sessions.pop(key).stop.set()
            if self.selected == key:
                self.selected = next(reversed(self.sessions))

    def close(self):
        with self._lock:
            self._closed = True
            self._client = None
            for shell in self.sessions.values():
                shell.stop.set()

    def join(self):
        # The connection owner closes the transport before waiting for PTY negotiation.
        with self._lock:
            workers = list(self._workers)
        for worker in workers:
            worker.join()


class TerminalView(QPlainTextEdit):
    """The visible widget only routes input to the currently selected session."""

    def __init__(self):
        super().__init__()
        self.shell = None
        self.last_paint = None
        self.setReadOnly(True)
        self.setAttribute(Qt.WA_InputMethodEnabled, True)
        self.setContextMenuPolicy(Qt.NoContextMenu)
        self.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setFont(QFontDatabase.systemFont(QFontDatabase.FixedFont))
        self.setStyleSheet("QPlainTextEdit { background: #fafafc; color: #1d1d1f; border: 1px solid #e0e0e0; border-radius: 8px; }")
        self.setPlaceholderText("FSS 연결 대기")
        self.setAccessibleName("선택한 FSS의 SSH 터미널")

    def bind(self, shell):
        if self.shell is shell:
            return
        self.shell = shell
        self.last_paint = None
        self.clear()
        self.setExtraSelections([])
        self._size_pty()
        self.refresh()

    def _size_pty(self):
        if self.shell is not None:
            viewport = self.viewport()
            fm = self.fontMetrics()
            margin = int(2 * self.document().documentMargin()) + 2
            self.shell.resize(max(10, (viewport.width() - margin) // max(1, fm.horizontalAdvance("M"))),
                              max(2, (viewport.height() - margin) // max(1, fm.lineSpacing())))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._size_pty()

    def refresh(self):
        shell = self.shell
        if shell is None:
            self.setPlaceholderText("FSS 연결 대기")
            return
        stamp = (shell.revision, shell.status)
        if self.last_paint == stamp:
            return
        self.last_paint = stamp
        if not shell.screen:
            self.setPlaceholderText(shell.status)
            return
        screen = shell.screen
        lines = [line.rstrip() for line in screen.display]
        x, y = screen.cursor.x, screen.cursor.y
        # Pyte uses one empty cell after a double-width glyph; QTextCursor uses UTF-16 offsets.
        before = "".join(screen.buffer[y][i].data for i in range(min(x, screen.columns)))
        if not shell.status and not screen.cursor.hidden:
            lines[y] = (lines[y] + " " * max(0, len(before) + 1 - len(lines[y])))
        output = "\n".join(lines).rstrip("\n")
        if shell.finished:
            output = output + "\n" + shell.status if output else shell.status
        self.setPlainText(output)
        if shell.status and not output.strip():
            self.clear()
        self.setPlaceholderText(shell.status)
        self.setToolTip(shell.status)
        if shell.status or screen.cursor.hidden:
            self.setExtraSelections([])
        else:
            cursor = QTextCursor(self.document().findBlockByNumber(y))
            cursor.setPosition(cursor.position() + len(before.encode("utf-16-le")) // 2)
            cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor)
            mark = QTextEdit.ExtraSelection()
            mark.cursor = cursor
            mark.format.setBackground(self.palette().color(self.palette().ColorRole.Highlight))
            self.setExtraSelections([mark])

    def wheelEvent(self, event):
        if self.shell and self.shell.screen and event.angleDelta().y():
            screen = self.shell.screen
            (screen.prev_page if event.angleDelta().y() > 0 else screen.next_page)()
            self.shell.revision += 1
            self.refresh()
        event.accept()

    def focusNextPrevChild(self, forward):
        # Tab is shell completion when typing inside a live PTY.
        if self.shell and self.shell.opened.is_set() and self.hasFocus():
            return False
        return super().focusNextPrevChild(forward)

    def inputMethodEvent(self, event):
        if self.shell and self.shell.opened.is_set():
            self.shell.send(event.commitString().encode())
            event.accept()
        else:
            super().inputMethodEvent(event)

    def keyPressEvent(self, event):
        shell = self.shell
        if shell is None or not shell.opened.is_set():
            return super().keyPressEvent(event)
        key, mods = event.key(), event.modifiers()
        ctrl, shift = bool(mods & Qt.ControlModifier), bool(mods & Qt.ShiftModifier)
        if ctrl and shift and key == Qt.Key_C:
            self.copy()
        elif event.matches(QKeySequence.Paste) or (ctrl and shift and key == Qt.Key_V):
            text = QApplication.clipboard().text().replace("\r\n", "\n").replace("\n", "\r")
            data = text.encode()
            if (2004 << 5) in shell.screen.mode:
                data = b"\x1b[200~" + data + b"\x1b[201~"
            shell.send(data)
        elif ctrl and Qt.Key_A <= key <= Qt.Key_Z:
            shell.send(bytes([key - Qt.Key_A + 1]))
        else:
            prefix = b"\x1bO" if (1 << 5) in shell.screen.mode else b"\x1b["
            keys = {Qt.Key_Return:b"\r", Qt.Key_Enter:b"\r", Qt.Key_Backspace:b"\x7f",
                    Qt.Key_Tab:b"\t", Qt.Key_Backtab:b"\x1b[Z", Qt.Key_Escape:b"\x1b",
                    Qt.Key_Up:prefix+b"A", Qt.Key_Down:prefix+b"B", Qt.Key_Right:prefix+b"C",
                    Qt.Key_Left:prefix+b"D", Qt.Key_Home:b"\x1b[H", Qt.Key_End:b"\x1b[F",
                    Qt.Key_Insert:b"\x1b[2~", Qt.Key_Delete:b"\x1b[3~",
                    Qt.Key_PageUp:b"\x1b[5~", Qt.Key_PageDown:b"\x1b[6~"}
            data = keys.get(key, event.text().encode())
            if mods & Qt.AltModifier and data:
                data = b"\x1b" + data
            shell.send(data)
        event.accept()
