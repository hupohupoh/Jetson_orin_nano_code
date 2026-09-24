from __future__ import annotations

import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock
from contextlib import redirect_stdout

import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1]
for extra in (DEPLOY_DIR, DEPLOY_DIR / "tools"):
    if str(extra) not in sys.path:
        sys.path.insert(0, str(extra))

import config  # noqa: E402
from protocol import (  # noqa: E402
    FrameDecoder,
    STATE_ENCODERS_VALID,
    STATE_FAULT,
    StatePacket,
    pack_state,
)
from read_joint_angles import (  # noqa: E402
    CaptureSession,
    KeypointStore,
    StreamLogger,
    keypoint_from,
    parse_args,
    run_prompt_loop,
)


def make_state(sequence: int, joint_position: np.ndarray, flags: int = STATE_ENCODERS_VALID):
    return StatePacket(
        sequence=sequence,
        timestamp_us=sequence * 5000,
        joint_position=np.asarray(joint_position, dtype=np.float32),
        joint_velocity=np.zeros(config.NUM_JOINTS, dtype=np.float32),
        accel_m_s2=np.array([0.0, 0.0, 9.81], dtype=np.float32),
        gyro_rad_s=np.zeros(3, dtype=np.float32),
        orientation_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        status_flags=flags,
    )


class FakeLink:
    """Serves protocol-encoded packets on demand and records sent frames."""

    def __init__(self, values: np.ndarray, limit: int = 4000) -> None:
        self._values = np.asarray(values, dtype=np.float32)
        self._limit = limit
        self._sequence = 0
        self._lock = threading.Lock()
        self.sent: list[tuple] = []

    def _next(self) -> StatePacket:
        source = make_state(self._sequence + 1, self._values)
        # Round-trip through the real wire format so the bytes are the same ones
        # the STM32 would send.
        decoder = FrameDecoder()
        decoded = list(decoder.feed(pack_state(source)))
        assert decoder.crc_errors == 0 and len(decoded) == 1
        with self._lock:
            self._sequence += 1
        return decoded[0]

    def wait_for_state(self, timeout_s: float = 3.0) -> StatePacket:
        return self._next()

    def get_latest_state(self, max_age_s: float = 0.05) -> StatePacket:
        if self._sequence >= self._limit:
            raise TimeoutError("playback exhausted")
        return self._next()

    def send_command(self, timestamp_us, joint_target, kp_scale, kd_scale, command_flags):
        self.sent.append((joint_target, kp_scale, kd_scale, command_flags))

    def close(self) -> None:
        pass


def feed_inputs(lines: list[str]):
    """An input_fn that plays back lines, then raises EOFError to stop the loop."""
    remaining = list(lines)

    def _input(_prompt: str = "") -> str:
        if not remaining:
            raise EOFError
        return remaining.pop(0)

    return _input


class KeypointStoreTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "keypoints.json"
        self.store = KeypointStore(self.path, {"port": "/dev/ttyACM0"})

    def tearDown(self):
        self._tmp.cleanup()

    def test_rewrites_the_whole_file_after_each_capture(self):
        self.store.add({"name": "a"})
        self.store.add({"name": "b"})
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        self.assertEqual([point["name"] for point in payload["keypoints"]], ["a", "b"])
        self.assertEqual(payload["port"], "/dev/ttyACM0")
        self.assertEqual(len(self.store), 2)

    def test_leaves_no_temporary_file_behind(self):
        self.store.add({"name": "a"})
        self.assertFalse((self.path.parent / (self.path.name + ".tmp")).exists())

    def test_save_creates_an_empty_file_for_a_session_with_no_captures(self):
        self.store.save()
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        self.assertEqual(payload["keypoints"], [])


class CaptureSessionTests(unittest.TestCase):
    def test_snapshot_averages_a_stable_window(self):
        session = CaptureSession(FakeLink(config.Q_DEFAULT))
        snapshot = session.snapshot(samples=4, tolerance_rad=0.01, timeout_s=2.0)
        self.assertTrue(snapshot.is_stable(0.01))
        self.assertEqual(snapshot.frames, 4)
        np.testing.assert_allclose(snapshot.angle_rad, config.Q_DEFAULT, atol=1e-5)

    def test_snapshot_reports_an_unstable_pose_instead_of_failing(self):
        class DriftingLink(FakeLink):
            """A link whose value moves every frame, so it can never settle."""

            def _next(self):
                with self._lock:
                    self._sequence += 1
                    sequence = self._sequence
                return make_state(
                    sequence, np.full(config.NUM_JOINTS, sequence * 0.5, dtype=np.float32)
                )

        session = CaptureSession(DriftingLink(np.zeros(config.NUM_JOINTS, dtype=np.float32)))
        snapshot = session.snapshot(samples=6, tolerance_rad=1e-3, timeout_s=0.05)
        self.assertFalse(snapshot.is_stable(1e-3))
        self.assertGreater(snapshot.max_spread_rad, 1e-3)

    def test_a_fault_flag_aborts_the_capture(self):
        link = FakeLink(config.Q_DEFAULT)
        poisoned = make_state(1, config.Q_DEFAULT, flags=STATE_FAULT)
        session = CaptureSession(link)
        with mock.patch.object(link, "get_latest_state", return_value=poisoned):
            with self.assertRaises(RuntimeError):
                session.snapshot(samples=2, tolerance_rad=0.1, timeout_s=0.2)

    def test_initialize_rejects_a_missing_encoder_flag(self):
        link = FakeLink(config.Q_DEFAULT)
        session = CaptureSession(link)
        with mock.patch.object(
            link, "wait_for_state", return_value=make_state(1, config.Q_DEFAULT, flags=0)
        ):
            with self.assertRaises(RuntimeError):
                session.wait_for_first(timeout_s=0.2)


class StreamLoggerTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "stream.csv"

    def tearDown(self):
        self._tmp.cleanup()

    def _run_logger(self, log_hz: float, seconds: float = 0.08, keepalive: bool = True):
        link = FakeLink(config.Q_DEFAULT)
        logger = StreamLogger(link, self.path, log_hz, keepalive)
        logger.start()
        time.sleep(seconds)
        logger.close()
        return link, logger

    def test_writes_a_header_and_rows_in_policy_order(self):
        _, logger = self._run_logger(log_hz=2000)
        with self.path.open(encoding="utf-8", newline="") as handle:
            rows = list(csv.reader(handle))
        expected = ["host_time_iso", "elapsed_s", "state_sequence"] + [
            f"{name}_rad" for name in config.JOINT_NAMES
        ]
        self.assertEqual(rows[0], expected)
        self.assertEqual(len(rows[0]), 3 + config.NUM_JOINTS)
        self.assertGreater(logger.rows, 0)
        self.assertEqual(len(rows) - 1, logger.rows)
        np.testing.assert_allclose(
            [float(value) for value in rows[1][3:]], config.Q_DEFAULT, atol=1e-5
        )

    def test_log_hz_zero_writes_no_rows(self):
        _, logger = self._run_logger(log_hz=0.0)
        self.assertEqual(logger.rows, 0)
        with self.path.open(encoding="utf-8", newline="") as handle:
            self.assertEqual(len(list(csv.reader(handle))), 1)  # header only

    def test_keepalive_never_requests_motor_enable(self):
        link, _ = self._run_logger(log_hz=0.0, seconds=0.15)
        self.assertGreater(len(link.sent), 0, "expected at least one keep-alive frame")
        for _, kp_scale, kd_scale, command_flags in link.sent:
            self.assertEqual(command_flags, 0)
            self.assertEqual((kp_scale, kd_scale), (0.0, 0.0))

    def test_no_keepalive_sends_nothing(self):
        link, _ = self._run_logger(log_hz=200, seconds=0.05, keepalive=False)
        self.assertEqual(link.sent, [])


class PromptLoopTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.store = KeypointStore(Path(self._tmp.name) / "keypoints.json", {})
        self.args = parse_args(["--samples", "3", "--stable-tol-deg", "1.0", "--timeout", "1"])

    def tearDown(self):
        self._tmp.cleanup()

    def _run(self, lines: list[str]):
        session = CaptureSession(FakeLink(config.Q_DEFAULT))
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            code = run_prompt_loop(session, self.store, self.args, feed_inputs(lines))
        return code, buffer.getvalue()

    def test_enter_saves_an_auto_named_keypoint(self):
        code, output = self._run([""])
        self.assertEqual(code, 0)
        self.assertEqual(len(self.store), 1)
        self.assertEqual(self.store.keypoints[0]["name"], "keypoint_000")
        self.assertTrue(self.store.keypoints[0]["stable"])
        self.assertIn("saved 'keypoint_000'", output)

    def test_a_typed_name_labels_the_keypoint(self):
        self._run(["crouch"])
        self.assertEqual(self.store.keypoints[0]["name"], "crouch")

    def test_auto_names_do_not_collide_across_captures(self):
        self._run(["", "", ""])
        self.assertEqual(
            [point["name"] for point in self.store.keypoints],
            ["keypoint_000", "keypoint_001", "keypoint_002"],
        )

    def test_peek_shows_angles_without_saving(self):
        _, output = self._run(["p", "peek", "?"])
        self.assertEqual(len(self.store), 0)
        self.assertIn(config.JOINT_NAMES[0], output)
        self.assertNotIn("saved", output)

    def test_quit_exits_without_saving(self):
        code, _ = self._run(["q", ""])
        self.assertEqual(code, 0)
        self.assertEqual(len(self.store), 0)


class KeypointPayloadTests(unittest.TestCase):
    def test_keypoint_carries_both_units_and_provenance(self):
        session = CaptureSession(FakeLink(config.Q_DEFAULT))
        snapshot = session.snapshot(samples=3, tolerance_rad=1.0, timeout_s=1.0)
        keypoint = keypoint_from(snapshot, 0, "start", 1.0)
        self.assertEqual(keypoint["name"], "start")
        self.assertEqual(len(keypoint["joint_position_rad"]), config.NUM_JOINTS)
        np.testing.assert_allclose(
            keypoint["joint_position_deg"],
            np.rad2deg(keypoint["joint_position_rad"]),
            atol=1e-9,
        )
        self.assertIn("state_sequence", keypoint)
        self.assertIn("max_spread_deg", keypoint)


if __name__ == "__main__":
    unittest.main()
