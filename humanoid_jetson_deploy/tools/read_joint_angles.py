#!/usr/bin/env python3
"""Record STM32 joint angles: a continuous stream plus named key points.

Before the first state, this tool sends disabled probe frames unless
``--no-keepalive`` is selected. Every frame it sends carries
``command_flags=0`` and zero gains, so you can position the robot by hand,
then press Enter to capture the pose.

Two files are written per session, in joint order ``config.JOINT_NAMES`` (the
order the policy model is loaded with):

``stream.csv``
    Every reading, continuously, at ``--log-hz``. Recorded by a background
    thread, so it keeps running while the program waits for your Enter.

``keypoints.json``
    One entry per capture. Press Enter to save the current pose; the capture
    averages about half a second of frames and reports how much the pose moved.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import sys
import threading
import time
from typing import TYPE_CHECKING

import numpy as np

DEPLOY_DIR = Path(__file__).resolve().parents[1]
if str(DEPLOY_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOY_DIR))

import config  # noqa: E402
from motor_test_common import monotonic_us  # noqa: E402
from protocol import STATE_ENCODERS_VALID, STATE_FAULT  # noqa: E402

if TYPE_CHECKING:
    from serial_link import SerialLink


KEEPALIVE_HZ = 50.0
QUIT_COMMANDS = ("q", "quit", "exit")
PEEK_COMMANDS = ("p", "peek", "?")


def timestamp() -> str:
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")


def iso_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def validate_state(state, *, context: str = "state") -> None:
    """Reject a packet the firmware flags as faulty or not yet trustworthy."""
    if state.status_flags & STATE_FAULT:
        raise RuntimeError(f"STM32 reports a motor fault: flags=0x{state.status_flags:08X}")
    if not state.status_flags & STATE_ENCODERS_VALID:
        raise RuntimeError(
            f"STM32 encoder-valid flag is missing on {context}: "
            f"flags=0x{state.status_flags:08X}"
        )
    if not np.isfinite(state.joint_position).all():
        raise RuntimeError(f"STM32 joint position contains NaN or Inf on {context}")


@dataclass(frozen=True)
class Snapshot:
    """Averaged joint angles plus the evidence needed to judge the capture."""

    angle_rad: np.ndarray
    max_spread_rad: float
    frames: int
    elapsed_s: float
    flags: int
    sequence: int

    def is_stable(self, tolerance_rad: float) -> bool:
        return self.max_spread_rad <= tolerance_rad


class StreamLogger:
    """Background thread: keep-alive frames plus the continuous reading CSV.

    This runs independently of the capture prompt so that the stream keeps
    recording while the program waits for you to press Enter, and so the STM32
    command watchdog stays fed during that wait.
    """

    def __init__(self, link: "SerialLink", path: Path, log_hz: float, keepalive: bool) -> None:
        self.link = link
        self.path = path
        self.log_hz = log_hz
        self.keepalive = keepalive
        self.rows = 0
        self._file = None
        self._writer = None
        self._last_state = None
        self._last_sequence: int | None = None
        self._next_log = time.monotonic()
        self._next_keepalive = time.monotonic()
        self._start_monotonic = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="joint-angle-stream", daemon=True
        )

    def start(self) -> None:
        import csv

        self._file = self.path.open("w", encoding="utf-8", newline="", buffering=1)
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            ["host_time_iso", "elapsed_s", "state_sequence"]
            + [f"{name}_rad" for name in config.JOINT_NAMES]
        )
        self._start_monotonic = time.monotonic()
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            now = time.monotonic()
            # Echo the measured pose with enable=0 and zero gains; the enable bit
            # stays clear, so the motors cannot move.
            if self.keepalive and self._last_state is not None and now >= self._next_keepalive:
                try:
                    self.link.send_command(
                        monotonic_us(), self._last_state.joint_position, 0.0, 0.0, 0
                    )
                except Exception:  # noqa: BLE001 - a dropped keep-alive is not fatal
                    pass
                self._next_keepalive = max(now, self._next_keepalive) + 1.0 / KEEPALIVE_HZ

            try:
                state = self.link.get_latest_state(max_age_s=0.5)
            except TimeoutError:
                time.sleep(0.01)
                continue
            if state.sequence == self._last_sequence:
                time.sleep(0.002)
                continue
            self._last_sequence = state.sequence
            self._last_state = state

            if self.log_hz <= 0.0 or time.monotonic() < self._next_log:
                continue
            self._writer.writerow(
                [
                    iso_now(),
                    f"{time.monotonic() - self._start_monotonic:.6f}",
                    int(state.sequence),
                    *[f"{value:.8f}" for value in state.joint_position],
                ]
            )
            self.rows += 1
            self._next_log = max(time.monotonic(), self._next_log) + 1.0 / self.log_hz

    def get_latest(self):
        """The newest packet seen by the stream thread, or ``None``."""
        return self._last_state

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=0.5)
        if self._file is not None and not self._file.closed:
            self._file.close()


class KeypointStore:
    """Every captured key point, held in one JSON file.

    The file is rewritten atomically after each capture so that quitting or
    crashing mid-session never loses the key points already taken.
    """

    def __init__(self, path: Path, meta: dict) -> None:
        self.path = path
        self.meta = meta
        self.keypoints: list[dict] = []

    def add(self, keypoint: dict) -> None:
        self.keypoints.append(keypoint)
        self.save()

    def save(self) -> None:
        payload = {**self.meta, "keypoints": self.keypoints}
        temporary = self.path.with_name(self.path.name + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        os.replace(temporary, self.path)

    def __len__(self) -> int:
        return len(self.keypoints)


class CaptureSession:
    """Read joint angles for capture, independent of the stream thread."""

    def __init__(self, link: "SerialLink") -> None:
        self.link = link
        self._last_sequence: int | None = None

    def latest(self):
        """The current packet, or ``None`` if nothing fresh has arrived."""
        try:
            state = self.link.get_latest_state(max_age_s=0.5)
        except TimeoutError:
            return None
        return state

    def wait_for_first(self, timeout_s: float = 5.0, send_probe: bool = True):
        if not send_probe:
            state = self.link.wait_for_state(timeout_s=timeout_s)
            validate_state(state)
            return state
        deadline = time.monotonic() + timeout_s
        disabled_target = np.zeros(config.NUM_JOINTS, dtype=np.float32)
        while time.monotonic() < deadline:
            started = time.monotonic()
            self.link.send_command(monotonic_us(), disabled_target, 0.0, 0.0, 0)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                state = self.link.wait_for_state(timeout_s=min(1.0 / KEEPALIVE_HZ, remaining))
            except TimeoutError:
                time.sleep(max(0.0, min(1.0 / KEEPALIVE_HZ - (time.monotonic() - started),
                                        deadline - time.monotonic())))
                continue
            validate_state(state)
            return state
        raise TimeoutError("No valid STM32 state packet received")

    def snapshot(self, samples: int, tolerance_rad: float, timeout_s: float) -> Snapshot:
        """Average the newest ``samples`` frames, stopping early once they agree.

        The rolling window only holds consecutive frames, so a pose that is still
        moving keeps the loop running instead of polluting the average with the
        moment you were still moving.
        """
        window: deque[np.ndarray] = deque(maxlen=samples)
        start = time.monotonic()
        deadline = start + timeout_s
        flags = 0
        sequence = 0

        while time.monotonic() < deadline:
            try:
                state = self.link.get_latest_state(max_age_s=0.5)
            except TimeoutError:
                time.sleep(0.002)
                continue
            if state.sequence == self._last_sequence:
                time.sleep(0.002)
                continue
            self._last_sequence = state.sequence
            validate_state(state, context="capture")
            window.append(np.asarray(state.joint_position, dtype=np.float64))
            flags, sequence = state.status_flags, state.sequence
            if len(window) < samples:
                continue
            if float(np.max(np.ptp(np.asarray(window), axis=0))) <= tolerance_rad:
                break

        if not window:
            raise TimeoutError(f"no STM32 state packet arrived within {timeout_s:.1f} s")

        stacked = np.asarray(window)
        return Snapshot(
            angle_rad=stacked.mean(axis=0),
            max_spread_rad=float(np.max(np.ptp(stacked, axis=0))),
            frames=len(window),
            elapsed_s=time.monotonic() - start,
            flags=flags,
            sequence=sequence,
        )


def format_angles(values: np.ndarray, indent: str = "      ") -> str:
    """One compact line of ``name=value`` pairs, for the live readout."""
    return "  ".join(
        f"{name}={value:+.3f}"
        for name, value in zip(config.JOINT_NAMES, np.asarray(values, dtype=np.float64))
    )


def format_array(values: np.ndarray, indent: str = "    ", per_line: int = 6) -> str:
    """A numpy literal with the same six-per-line layout as config.py."""
    chunks = [
        ", ".join(f"{value:+.6f}" for value in values[start : start + per_line])
        for start in range(0, len(values), per_line)
    ]
    body = ",\n".join(f"{indent}{chunk}" for chunk in chunks)
    return f"np.array([\n{body},\n], dtype=np.float32)"


def keypoint_from(snapshot: Snapshot, index: int, name: str, tolerance_rad: float) -> dict:
    return {
        "index": index,
        "name": name,
        "host_time_iso": iso_now(),
        "frames": snapshot.frames,
        "elapsed_s": round(snapshot.elapsed_s, 6),
        "max_spread_deg": float(np.rad2deg(snapshot.max_spread_rad)),
        "stable": snapshot.is_stable(tolerance_rad),
        "state_sequence": snapshot.sequence,
        "state_flags": f"0x{snapshot.flags:08X}",
        "joint_position_rad": [float(value) for value in snapshot.angle_rad],
        "joint_position_deg": [float(np.rad2deg(value)) for value in snapshot.angle_rad],
    }


def capture_keypoint(
    session: CaptureSession,
    store: KeypointStore,
    args: argparse.Namespace,
    name: str | None,
) -> bool:
    """Take one stable snapshot and append it to the store."""
    tolerance_rad = float(np.deg2rad(args.stable_tol_deg))
    try:
        snapshot = session.snapshot(
            samples=args.samples, tolerance_rad=tolerance_rad, timeout_s=args.timeout
        )
    except TimeoutError as exc:
        print(f"  capture failed: {exc}")
        return False

    index = len(store)
    label = name.strip() if name and name.strip() else f"keypoint_{index:03d}"
    store.add(keypoint_from(snapshot, index, label, tolerance_rad))

    stability = "stable" if snapshot.is_stable(tolerance_rad) else "NOT STABLE"
    print(
        f"  saved '{label}' #{index}: {snapshot.frames} frames in "
        f"{snapshot.elapsed_s:.2f}s, spread "
        f"{np.rad2deg(snapshot.max_spread_rad):.3f} deg ({stability})"
    )
    print(f"  {format_angles(snapshot.angle_rad)}")
    if not snapshot.is_stable(tolerance_rad):
        print("  warning: the pose was still moving; hold the robot still and retake it")
    return True


def print_peek(session: CaptureSession) -> None:
    state = session.latest()
    if state is None:
        print("  no fresh STM32 state packet")
        return
    try:
        validate_state(state, context="peek")
    except RuntimeError as exc:
        print(f"  {exc}")
        return
    print(f"  {format_angles(state.joint_position)}")
    print(f"  state_sequence={state.sequence} flags=0x{state.status_flags:08X}")


def run_prompt_loop(
    session: CaptureSession,
    store: KeypointStore,
    args: argparse.Namespace,
    input_fn=None,
) -> int:
    """Prompt for names; each Enter captures and stores a key point.

    ``input_fn`` defaults to the builtin ``input``, resolved at call time rather
    than bound as a default argument so tests can substitute it.
    """
    if input_fn is None:
        input_fn = input
    print()
    print("Press Enter to save the current pose as a key point.")
    print("  <name>  save it under that name")
    print("  p       show the current angles without saving")
    print("  q       finish and close the files")
    print()

    while True:
        try:
            answer = input_fn(f"[{len(store)} saved] name (Enter=auto) > ")
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        command = answer.strip()
        lowered = command.lower()
        if lowered in QUIT_COMMANDS:
            return 0
        if lowered in PEEK_COMMANDS:
            print_peek(session)
            continue
        capture_keypoint(session, store, args, command or None)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument(
        "--baud",
        type=int,
        default=921600,
        help="Line coding only; native USB CDC ignores the physical baud rate",
    )
    parser.add_argument(
        "--out-dir", default="logs/joint_angles", help="Parent directory for sessions"
    )
    parser.add_argument(
        "--session",
        default=None,
        help="Session folder name; defaults to a timestamp",
    )
    parser.add_argument("--samples", type=int, default=100, help="Frames per capture")
    parser.add_argument(
        "--stable-tol-deg",
        type=float,
        default=0.5,
        help="Stop a capture once every joint moves less than this across the window",
    )
    parser.add_argument("--timeout", type=float, default=3.0, help="Capture time limit")
    parser.add_argument(
        "--log-hz",
        type=float,
        default=20.0,
        help="Continuous stream.csv rate; 0 disables the stream log",
    )
    parser.add_argument(
        "--no-keepalive",
        action="store_true",
        help="Send no frames at all; every keep-alive already has the enable bit clear",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.samples < 1:
        raise SystemExit("--samples must be at least 1")
    if args.stable_tol_deg <= 0.0:
        raise SystemExit("--stable-tol-deg must be positive")
    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be positive")
    if args.log_hz < 0.0:
        raise SystemExit("--log-hz cannot be negative")

    # Keep --help and the offline tests usable without pyserial; the hardware
    # dependency is only needed once a serial device is actually opened.
    from serial_link import SerialLink

    session_dir = Path(args.out_dir).expanduser() / (args.session or timestamp())
    session_dir.mkdir(parents=True, exist_ok=True)
    stream_path = session_dir / "stream.csv"
    keypoints_path = session_dir / "keypoints.json"

    print(f"Opening {args.port} (line coding {args.baud})")
    print("Motors stay disabled: every frame this tool sends has command_flags=0")

    link = SerialLink(args.port, args.baud)
    stream = StreamLogger(link, stream_path, args.log_hz, not args.no_keepalive)
    store = KeypointStore(
        keypoints_path,
        {
            "host_time_iso": iso_now(),
            "port": args.port,
            "joint_names": list(config.JOINT_NAMES),
            "stable_tol_deg": args.stable_tol_deg,
            "samples": args.samples,
        },
    )
    session = CaptureSession(link)

    try:
        first = session.wait_for_first(timeout_s=5.0, send_probe=not args.no_keepalive)
        print(f"First state packet: sequence={first.sequence} flags=0x{first.status_flags:08X}")
        stream.start()
        store.save()  # create the file up front, even if nothing is captured
        print(f"Session directory: {session_dir}")
        if args.log_hz > 0.0:
            print(f"Continuous stream: {stream_path.name} at {args.log_hz:g} Hz")
        print(f"Key points:        {keypoints_path.name}")

        run_prompt_loop(session, store, args)
    except Exception as exc:  # noqa: BLE001 - surface firmware faults as one line
        print(f"FAIL: {exc}")
        return 1
    finally:
        stream.close()
        link.close()

    print()
    print(f"Session: {session_dir}")
    print(f"  {len(store)} key points -> {keypoints_path.name}")
    if args.log_hz > 0.0:
        print(f"  {stream.rows} stream rows -> {stream_path.name}")

    if store.keypoints:
        print()
        print("Last key point, policy-order array (rad), ready to paste:")
        print(format_array(np.asarray(store.keypoints[-1]["joint_position_rad"])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
