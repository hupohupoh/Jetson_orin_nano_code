from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1]
for extra in (DEPLOY_DIR, DEPLOY_DIR / "tools"):
    if str(extra) not in sys.path:
        sys.path.insert(0, str(extra))

import config  # noqa: E402
import joint_angle_gui as gui  # noqa: E402
from read_joint_angles import (  # noqa: E402
    CaptureSession,
    KeypointStore,
    Snapshot,
    StreamLogger,
    keypoint_from,
)


# Deliberately a *Python* float, exactly as production builds it. `Snapshot.
# is_stable` returns numpy's bool when handed a numpy float, and numpy bools are
# not JSON serializable -- see KeypointSchemaTests.test_tolerance_must_be_a_plain_float.
TOLERANCE_RAD = float(np.deg2rad(0.5))


def make_snapshot(values, spread_rad=0.001, frames=100, elapsed_s=0.5):
    # float(...) matters: a numpy float here makes Snapshot.is_stable return a
    # numpy bool, which json.dumps refuses.
    return Snapshot(
        angle_rad=np.asarray(values, dtype=np.float64),
        max_spread_rad=float(spread_rad),
        frames=frames,
        elapsed_s=elapsed_s,
        flags=0x08,
        sequence=4242,
    )


class FakeSession:
    """Stands in for CaptureSession so the worker can be driven deterministically."""

    def __init__(self, snapshot=None, error=None, delay_s=0.0):
        self._snapshot = snapshot
        self._error = error
        self._delay_s = delay_s

    def snapshot(self, samples, tolerance_rad, timeout_s):
        if self._delay_s:
            time.sleep(self._delay_s)
        if self._error is not None:
            raise self._error
        return self._snapshot


class RowTests(unittest.TestCase):
    def test_rows_render_in_policy_order_with_both_units(self):
        rows = gui.build_rows(config.Q_DEFAULT)
        self.assertEqual(len(rows), config.NUM_JOINTS)
        self.assertEqual([row[0] for row in rows], list(range(config.NUM_JOINTS)))
        self.assertEqual([row[1] for row in rows], list(config.JOINT_NAMES))
        for index, name, radians, degrees in rows:
            self.assertEqual(name, config.JOINT_NAMES[index])
            self.assertAlmostEqual(radians, float(config.Q_DEFAULT[index]), places=6)
            self.assertAlmostEqual(degrees, np.rad2deg(float(config.Q_DEFAULT[index])), places=4)

    def test_rows_reject_a_wrong_length(self):
        with self.assertRaises(ValueError):
            gui.build_rows(np.zeros(config.NUM_JOINTS - 1))


class DescribeCaptureTests(unittest.TestCase):
    def test_a_settled_capture_is_described_as_stable(self):
        snapshot = make_snapshot(config.Q_DEFAULT, spread_rad=np.deg2rad(0.08))
        text, stable = gui.describe_capture(snapshot, "crouch", 0, TOLERANCE_RAD)
        self.assertTrue(stable)
        self.assertIn("crouch", text)
        self.assertIn("#0", text)
        self.assertIn("stable", text)

    def test_a_moving_pose_is_flagged_for_retake(self):
        snapshot = make_snapshot(config.Q_DEFAULT, spread_rad=np.deg2rad(3.0))
        text, stable = gui.describe_capture(snapshot, "crouch", 1, TOLERANCE_RAD)
        self.assertFalse(stable)
        self.assertIn("NOT STABLE", text)


class CaptureWorkerTests(unittest.TestCase):
    def test_capture_runs_off_thread_and_reports_its_result(self):
        snapshot = make_snapshot(config.Q_DEFAULT)
        worker = gui.CaptureWorker(
            FakeSession(snapshot=snapshot, delay_s=0.25), samples=10,
            tolerance_rad=0.5, timeout_s=1.0,
        )
        started = time.monotonic()
        worker.start()
        # start() must return at once, and the result must not be there yet --
        # that is what keeps the window responsive during a slow capture.
        self.assertLess(time.monotonic() - started, 0.1)
        self.assertIsNone(worker.poll())

        result = self._await(worker)
        self.assertEqual(result[0], "ok")
        self.assertIs(result[1], snapshot)

    def test_worker_surfaces_errors_instead_of_swallowing_them(self):
        worker = gui.CaptureWorker(
            FakeSession(error=TimeoutError("no STM32 state packet arrived")),
            samples=10, tolerance_rad=0.5, timeout_s=1.0,
        )
        worker.start()
        kind, payload = self._await(worker)
        self.assertEqual(kind, "error")
        self.assertIn("TimeoutError", payload)
        self.assertIn("no STM32 state packet", payload)

    def test_poll_returns_none_while_the_worker_is_still_running(self):
        worker = gui.CaptureWorker(
            FakeSession(snapshot=make_snapshot(config.Q_DEFAULT), delay_s=0.2),
            samples=10, tolerance_rad=0.5, timeout_s=1.0,
        )
        worker.start()
        self.assertIsNone(worker.poll())
        self._await(worker)

    @staticmethod
    def _await(worker, timeout_s=5.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            result = worker.poll()
            if result is not None:
                return result
            time.sleep(0.005)
        raise AssertionError("capture worker produced no result in time")


class SimulatedLinkTests(unittest.TestCase):
    def test_simulated_link_emits_finite_angles_near_the_default_pose(self):
        link = gui.SimulatedLink()
        for _ in range(20):
            state = link.get_latest_state()
            self.assertTrue(np.isfinite(state.joint_position).all())
            self.assertEqual(state.joint_position.shape, (config.NUM_JOINTS,))
            # Drift plus noise must stay close enough to the default pose that
            # the joints remain inside their configured ranges.
            self.assertLess(
                float(np.max(np.abs(state.joint_position - config.Q_DEFAULT))),
                gui.SimulatedLink.DRIFT_RAD + 6 * gui.SimulatedLink.NOISE_RAD,
            )

    def test_simulated_link_advances_its_sequence(self):
        link = gui.SimulatedLink()
        sequences = [link.get_latest_state().sequence for _ in range(5)]
        self.assertEqual(sequences, sorted(set(sequences)))
        self.assertEqual(len(set(sequences)), 5)

    def test_stream_logger_over_the_simulated_link_never_enables_motors(self):
        """The whole --simulate path must keep the enable bit clear."""
        with tempfile.TemporaryDirectory() as directory:
            link = gui.SimulatedLink()
            logger = StreamLogger(link, Path(directory) / "stream.csv", 200.0, True)
            logger.start()
            time.sleep(0.15)
            logger.close()

        self.assertGreater(len(link.sent), 0, "expected at least one keep-alive frame")
        for _, _, kp_scale, kd_scale, command_flags in link.sent:
            self.assertEqual(command_flags, 0)
            self.assertEqual((kp_scale, kd_scale), (0.0, 0.0))


class SimulatedCapturePathTests(unittest.TestCase):
    def test_a_keypoint_can_be_captured_with_no_hardware(self):
        """The GUI's whole read path, exercised against the simulated link."""
        link = gui.SimulatedLink()
        session = CaptureSession(link)
        session.wait_for_first(timeout_s=1.0)
        snapshot = session.snapshot(
            samples=20, tolerance_rad=float(np.deg2rad(0.5)), timeout_s=2.0
        )

        self.assertEqual(snapshot.frames, 20)
        self.assertTrue(snapshot.is_stable(float(np.deg2rad(0.5))))
        self.assertTrue(np.isfinite(snapshot.angle_rad).all())


class KeypointSchemaTests(unittest.TestCase):
    """Lock the on-disk schema so the GUI and the CLI tool cannot drift apart."""

    EXPECTED_META = {"host_time_iso", "port", "joint_names", "stable_tol_deg",
                     "samples", "keypoints"}
    EXPECTED_KEYPOINT = {
        "index", "name", "host_time_iso", "frames", "elapsed_s", "max_spread_deg",
        "stable", "state_sequence", "state_flags", "joint_position_rad",
        "joint_position_deg",
    }

    def test_keypoint_entry_has_exactly_the_documented_fields(self):
        snapshot = make_snapshot(config.Q_DEFAULT)
        keypoint = keypoint_from(snapshot, 0, "crouch", TOLERANCE_RAD)
        self.assertEqual(set(keypoint), self.EXPECTED_KEYPOINT)
        self.assertEqual(len(keypoint["joint_position_rad"]), config.NUM_JOINTS)
        self.assertEqual(len(keypoint["joint_position_deg"]), config.NUM_JOINTS)
        np.testing.assert_allclose(
            keypoint["joint_position_deg"],
            np.rad2deg(keypoint["joint_position_rad"]),
            atol=1e-9,
        )

    def test_keypoint_payload_is_json_serializable(self):
        """Guards the numpy-bool trap.

        ``Snapshot.is_stable`` returns ``np.bool_`` when either operand is a numpy
        float, and ``json.dumps`` refuses those. Production avoids it by casting
        the tolerance with ``float(...)``; this pins the requirement.
        """
        keypoint = keypoint_from(make_snapshot(config.Q_DEFAULT), 0, "crouch",
                                 TOLERANCE_RAD)
        # np.bool_ is not a subclass of bool, so this catches a regression.
        self.assertIsInstance(keypoint["stable"], bool)
        json.dumps(keypoint)  # must not raise

    def test_a_gui_written_session_file_has_the_cli_layout(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "keypoints.json"
            store = KeypointStore(
                path,
                {
                    "host_time_iso": "2026-09-24T14:30:12.001+08:00",
                    "port": "/dev/ttyACM0",
                    "joint_names": list(config.JOINT_NAMES),
                    "stable_tol_deg": 0.5,
                    "samples": 100,
                },
            )
            store.add(keypoint_from(make_snapshot(config.Q_DEFAULT), 0, "crouch",
                                    TOLERANCE_RAD))
            store.save()
            payload = json.loads(path.read_text(encoding="utf-8"))

        self.assertEqual(set(payload), self.EXPECTED_META)
        self.assertEqual(payload["joint_names"], list(config.JOINT_NAMES))
        self.assertEqual(len(payload["keypoints"]), 1)
        self.assertEqual(payload["keypoints"][0]["name"], "crouch")


class ArgumentTests(unittest.TestCase):
    def test_defaults_match_the_command_line_tool(self):
        args = gui.parse_args([])
        self.assertEqual(args.port, "/dev/ttyACM0")
        self.assertEqual(args.baud, 921600)
        self.assertFalse(args.simulate)
        self.assertEqual(args.samples, 100)
        self.assertEqual(args.stable_tol_deg, 0.5)
        self.assertEqual(args.timeout, 3.0)
        self.assertEqual(args.log_hz, 20.0)

    def test_simulate_flag_is_opt_in(self):
        self.assertTrue(gui.parse_args(["--simulate"]).simulate)



class StatusBannerTests(unittest.TestCase):
    """Green must mean exactly one thing: motors off, safe to touch."""

    @staticmethod
    def telemetry(**overrides):
        from types import SimpleNamespace

        base = dict(
            mode="disabled", enabled=False, fault=None, motors_reported_on=False,
            telemetry_ok=True, clipped=False, segment_index=0, segment_count=0,
            segment_phase=0.0, source_label="", state_flags=8, state_sequence=1,
        )
        base.update(overrides)
        return SimpleNamespace(**base)

    def test_no_port_is_grey(self):
        text, colour = gui.describe_status(telemetry=None, stale=True, linked=False)
        self.assertIn("no port", text)
        self.assertEqual(colour, gui.GREY)

    def test_stale_telemetry_is_never_green(self):
        _text, colour = gui.describe_status(
            telemetry=self.telemetry(), stale=True, linked=True
        )
        self.assertEqual(colour, gui.GREY)

    def test_disabled_and_connected_is_green(self):
        text, colour = gui.describe_status(
            telemetry=self.telemetry(), stale=False, linked=True
        )
        self.assertEqual(colour, gui.GREEN)
        self.assertIn("safe to touch", text)

    def test_armed_and_holding_is_amber_not_green(self):
        text, colour = gui.describe_status(
            telemetry=self.telemetry(mode="holding", enabled=True,
                                     motors_reported_on=True),
            stale=False, linked=True,
        )
        self.assertEqual(colour, gui.AMBER)
        self.assertIn("MOTORS ON", text)

    def test_moving_is_red(self):
        text, colour = gui.describe_status(
            telemetry=self.telemetry(mode="moving", enabled=True,
                                     motors_reported_on=True, segment_count=3,
                                     source_label="keypoint crouch"),
            stale=False, linked=True,
        )
        self.assertEqual(colour, gui.RED)
        self.assertIn("keypoint crouch", text)

    def test_armed_but_unconfirmed_is_amber(self):
        """The firmware has not reported the motors on, so do not claim it has."""
        text, colour = gui.describe_status(
            telemetry=self.telemetry(enabled=True, motors_reported_on=False),
            stale=False, linked=True,
        )
        self.assertEqual(colour, gui.AMBER)
        self.assertIn("has not reported", text)

    def test_fault_and_estop_outrank_everything_and_are_red(self):
        for overrides, needle in (
            (dict(fault="FaultError: boom", enabled=True, motors_reported_on=True), "FAULT"),
            (dict(mode="estop", enabled=True, motors_reported_on=True), "EMERGENCY STOP"),
        ):
            with self.subTest(overrides=overrides):
                text, colour = gui.describe_status(
                    telemetry=self.telemetry(**overrides), stale=False, linked=True
                )
                self.assertEqual(colour, gui.RED)
                self.assertIn(needle, text)


class SimulatedLinkMotionTests(unittest.TestCase):
    """The simulator has to close the loop, or the safety chain is untestable here."""

    def test_pose_integrates_toward_the_commanded_target(self):
        import joint_motion

        link = gui.SimulatedLink()
        goal = config.Q_DEFAULT.copy()
        goal[3] += 0.4
        before = float(link.pose[3])
        for _ in range(20):
            link.send_command(0, goal, 1.0, 1.0, 1)  # COMMAND_ENABLE
        self.assertGreater(float(link.pose[3]), before)
        self.assertLessEqual(float(link.pose[3]), float(goal[3]) + 1e-9,
                             "the simulated joint must not overshoot past the target")

    def test_a_blocked_joint_does_not_move(self):
        link = gui.SimulatedLink()
        link.blocked = True
        goal = config.Q_DEFAULT.copy()
        goal[3] += 0.4
        before = link.pose.copy()
        for _ in range(20):
            link.send_command(0, goal, 1.0, 1.0, 1)
        np.testing.assert_allclose(link.pose, before)

    def test_motors_report_on_only_after_arming(self):
        import time as _time

        link = gui.SimulatedLink()
        state = link.get_latest_state()
        self.assertEqual(state.status_flags & 0x01, 0, "reported motors on while disabled")
        link.send_command(0, config.Q_DEFAULT, 1.0, 1.0, 1)
        _time.sleep(gui.SimulatedLink.MOTOR_REPORT_DELAY_S + 0.05)
        self.assertTrue(link.get_latest_state().status_flags & 0x01)

    def test_estop_frame_reports_the_motors_off_again(self):
        import time as _time

        link = gui.SimulatedLink()
        link.send_command(0, config.Q_DEFAULT, 1.0, 1.0, 1)
        _time.sleep(gui.SimulatedLink.MOTOR_REPORT_DELAY_S + 0.05)
        self.assertTrue(link.get_latest_state().status_flags & 0x01)
        link.send_command(0, config.Q_DEFAULT, 0.0, 0.0, 2)  # COMMAND_ESTOP
        self.assertFalse(link.get_latest_state().status_flags & 0x01)

    def test_the_motion_loop_runs_against_the_simulator(self):
        """End-to-end with no hardware: the whole fixed-policy path."""
        import joint_motion

        link = gui.SimulatedLink()
        loop = joint_motion.ControlLoop(link, clock=lambda: 0.0, sleep=lambda _s: None)
        self.assertTrue(loop._seed_from_measured(timeout_s=0.2))
        loop.set_enable(True)
        goal = config.Q_DEFAULT.copy()
        goal[3] += 0.3
        loop.request_move(goal)
        for tick in range(200):
            try:
                loop._tick(tick * joint_motion.CONTROL_DT)
            except joint_motion.FaultError as exc:
                self.fail(f"the simulator tripped a fault: {exc}")
            if loop.snapshot().mode == "holding":
                break
        self.assertEqual(loop.snapshot().mode, "holding")
        self.assertGreater(float(link.pose[3]), float(config.Q_DEFAULT[3]))

if __name__ == "__main__":
    unittest.main()
