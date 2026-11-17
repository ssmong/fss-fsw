import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch


class VideoTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
        from PySide6.QtWidgets import QApplication
        with patch.dict(os.environ, QT_QPA_PLATFORM="offscreen"):
            cls.app = QApplication.instance() or QApplication([])

    def spin_until(self, predicate, timeout=12):
        until = time.monotonic() + timeout
        while not predicate() and time.monotonic() < until:
            self.app.processEvents()
            time.sleep(.005)
        self.assertTrue(predicate(), "Qt media operation timed out")

    def test_renderer_fixed_grid_ghost_arm_and_five_second_history(self):
        from mission_video import TrajectoryRenderer
        r = TrajectoryRenderer()
        pose = {"pose": [1., 1., 0], "arm": [[0, 0], [.2, 0], [.4, .1]],
                "ghost": [2., 2., 0.], "ghost_arm": [[0, 0], [.2, .2]]}
        image = r.render(pose, 0)
        actual = image.pixelColor(r.screen(1, 1).toPoint())
        ghost = image.pixelColor(r.screen(2, 2).toPoint())
        self.assertGreater(ghost.red(), ghost.green() + 12)
        self.assertGreater(ghost.green(), ghost.blue() + 12)
        self.assertLess(actual.red(), 90)
        self.assertEqual((image.width(), image.height()), (640, 840))
        self.assertEqual(r.screen(3, 0).x() - r.screen(0, 0).x(), 540)
        self.assertEqual(r.screen(0, 0).y() - r.screen(0, 4).y(), 720)
        self.assertEqual(image.pixelColor(r.screen(3, 2).toPoint()).name(), "#636368")
        r.render({"pose": [1.2, 1.2, .3]}, .1)
        self.assertEqual(len(r.trail), 2)
        r.render({}, 5.1)
        self.assertFalse(r.trail)

    def test_mp4_round_trip_decodes_frames_and_duration(self):
        from mission_video import VideoRecorder, TrajectoryRenderer
        from PySide6.QtCore import QUrl
        from PySide6.QtMultimedia import QMediaPlayer, QVideoSink
        with tempfile.TemporaryDirectory() as directory:
            recorder = VideoRecorder(directory)
            self.assertFalse(recorder.error, recorder.error)
            try:
                for i in range(12):
                    recorder.capture({"pose": [1., 1., i / 10],
                                      "arm": [[0, 0], [.2, 0], [.4, .1]],
                                      "ghost": [2., 2., -.1],
                                      "ghost_arm": [[0, 0], [.2, .2]]}, i / 10)
                    self.spin_until(lambda: recorder.frames_written >= i + 1 or recorder.error)
                    self.assertFalse(recorder.error, recorder.error)
                recorder.stop()
                self.spin_until(lambda: recorder.finished)
                self.assertFalse(recorder.error, recorder.error)
                self.assertEqual(recorder.frames_written, 12)
                self.assertGreater(recorder.path.stat().st_size, 1000)
                player, sink = QMediaPlayer(), QVideoSink()
                player.setVideoSink(sink)
                frames = []
                def received(frame):
                    if frame.isValid():
                        image = frame.toImage()
                        frames.append((frame.startTime(),
                                       image.pixelColor(TrajectoryRenderer.screen(2, 2).toPoint())))
                sink.videoFrameChanged.connect(received)
                player.setSource(QUrl.fromLocalFile(str(recorder.path)))
                player.play()
                self.spin_until(lambda: player.mediaStatus() == QMediaPlayer.EndOfMedia
                                or player.error() != QMediaPlayer.NoError)
                self.assertEqual(player.error(), QMediaPlayer.NoError, player.errorString())
                self.assertEqual(len(frames), 12)
                self.assertAlmostEqual(player.duration(), 1200, delta=100)
                self.assertEqual([t for t, _ in frames], [i * 100000 for i in range(12)])
                self.assertTrue(all(color.red() > color.green() + 12
                                    and color.green() > color.blue() + 12 for _, color in frames))
                player.stop()
                player.setSource(QUrl())
                player.deleteLater()
                sink.deleteLater()
            finally:
                recorder.stop()
                self.spin_until(lambda: recorder.finished)
                recorder.deleteLater()
                self.app.processEvents()

    def test_existing_video_is_preserved_and_backpressure_is_bounded(self):
        from mission_video import VideoRecorder
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.mp4"
            path.write_bytes(b"existing recording")
            refused = VideoRecorder(directory)
            self.assertTrue(refused.finished)
            self.assertTrue(refused.error)
            refused.capture({}, 0)
            refused.stop()
            self.assertEqual(path.read_bytes(), b"existing recording")
            path.unlink()
            recorder = VideoRecorder(directory)
            self.assertFalse(recorder.error, recorder.error)
            with patch.object(recorder._input, "sendVideoFrame", return_value=False):
                for i in range(8):
                    recorder.capture({"pose": [1, 2, 0]}, i / 10)
                self.assertEqual(len(recorder._pending), 2)
                self.assertEqual(recorder.frames_dropped, 6)
            recorder.stop()
            self.spin_until(lambda: recorder.finished)
            self.assertFalse(recorder.error, recorder.error)
            self.assertEqual(recorder.frames_written, 2)
            recorder.deleteLater()
            refused.deleteLater()
            self.app.processEvents()


if __name__ == "__main__":
    unittest.main()
