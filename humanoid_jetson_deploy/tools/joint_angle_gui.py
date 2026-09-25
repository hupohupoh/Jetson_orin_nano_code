#!/usr/bin/env python3
"""Tkinter window for recording STM32 joint angles and driving fixed poses.

Two jobs, on two tabs:

* **Record** -- position the robot by hand and capture key points. Motors stay
  off here.
* **Fixed policy** -- type target angles or load a key point file, and the robot
  moves there. This *does* drive motors, behind a master enable switch and an
  always-visible emergency stop.

The learned policy is a separate program: ``main.py``. The two never run at
once, because pyserial does not lock the device and a second opener silently
corrupts the frame stream instead of failing.

It reuses ``CaptureSession``, ``KeypointStore`` and ``keypoint_from`` from
``read_joint_angles.py``, so the ``keypoints.json`` it writes is field-for-field
identical to that tool's output and can be loaded by either one.

Threading rules -- these are load-bearing, do not relax them:

1. The Tk main thread owns every widget. No other thread may touch one.
2. ``joint_motion.ControlLoop`` is the only thread that writes to the serial
   port. It replaced the read-only ``StreamLogger`` in this window because the
   two must never overlap: one echoes the measured pose with zero gains while
   the other commands a trajectory, and the STM32 would see an alternating
   enable/disable stream and chatter.
3. A capture runs on a short-lived worker thread, because ``snapshot()`` blocks
   for up to ``--timeout``. Results come back through a queue and are applied by
   the Tk thread, so the key point store has exactly one writer.
4. The display reads ``link.get_latest_state()`` directly rather than anything a
   writer published, so it keeps showing encoders whichever component commands.

Run with ``--simulate`` to exercise the whole window with no robot attached. The
simulated link closes the control loop, so the safety chain and the stall
detector can be driven without hardware.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import queue
import sys
import threading
import time

import numpy as np

TOOLS_DIR = Path(__file__).resolve().parent
DEPLOY_DIR = TOOLS_DIR.parent
for extra in (DEPLOY_DIR, TOOLS_DIR):
    if str(extra) not in sys.path:
        sys.path.insert(0, str(extra))

import config  # noqa: E402
from protocol import (  # noqa: E402
    COMMAND_ENABLE,
    COMMAND_ESTOP,
    STATE_ENCODERS_VALID,
    STATE_MOTORS_ENABLED,
    StatePacket,
)
from read_joint_angles import (  # noqa: E402
    CaptureSession,
    KeypointStore,
    iso_now,
    keypoint_from,
    timestamp,
)


REFRESH_MS = 100
CAPTURE_POLL_MS = 50
# A packet older than this makes the window report "no telemetry".
TELEMETRY_GRACE_S = 1.0


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
        "--simulate",
        action="store_true",
        help="Run against a synthetic link instead of hardware; use this to "
        "confirm the window displays before connecting the robot",
    )
    parser.add_argument(
        "--out-dir", default="logs/joint_angles", help="Parent directory for sessions"
    )
    parser.add_argument(
        "--session", default=None, help="Session folder name; defaults to a timestamp"
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


def build_rows(values_rad) -> list[tuple[int, str, float, float]]:
    """Rows for the table, in policy joint order: index, name, radians, degrees."""
    values = np.asarray(values_rad, dtype=np.float64)
    if values.shape != (config.NUM_JOINTS,):
        raise ValueError(
            f"expected {config.NUM_JOINTS} joint values, got {values.shape}"
        )
    return [
        (index, name, float(values[index]), float(np.rad2deg(values[index])))
        for index, name in enumerate(config.JOINT_NAMES)
    ]


def describe_capture(snapshot, label: str, index: int, tolerance_rad: float) -> tuple[str, bool]:
    """Return ``(text, is_stable)`` for the last-capture panel."""
    stable = snapshot.is_stable(tolerance_rad)
    text = (
        f"{label}  #{index}    {snapshot.frames} frames / {snapshot.elapsed_s:.2f}s"
        f"    spread {np.rad2deg(snapshot.max_spread_rad):.3f} deg"
        f"    {'stable' if stable else 'NOT STABLE - retake this pose'}"
    )
    return text, stable


GREEN, AMBER, RED, GREY = "#006000", "#b06000", "#b00000", "#444444"


def describe_status(*, telemetry, stale: bool, linked: bool) -> tuple[str, str]:
    """Return ``(text, colour)`` for the banner.

    Armed-ness is derived only from the firmware's own status bits plus packet
    freshness -- never from what this window asked for, and never from log text.
    ``main.py`` prints ``MOTORS ENABLED`` from its argument, before it has even
    opened the port, so that string proves nothing.

    Green means exactly one thing: the motors are off and it is safe to touch
    the robot. Armed states are amber and red so that colour alone never reads
    as "safe".
    """
    if not linked:
        return "no port - not connected", GREY
    if telemetry is None or stale:
        return "linking - no telemetry from the STM32", GREY
    if telemetry.fault:
        return f"FAULT: {telemetry.fault} - motors commanded off", RED
    if telemetry.mode == "estop":
        return "EMERGENCY STOP latched - motors commanded off", RED
    if telemetry.enabled and telemetry.motors_reported_on:
        if telemetry.mode == "moving" and telemetry.segment_count:
            remaining = 1.0 - telemetry.segment_phase
            text = (
                f"MOTORS ON - moving to '{telemetry.source_label}' "
                f"(step {telemetry.segment_index + 1}/{telemetry.segment_count})"
            )
            if telemetry.clipped:
                text += " - pushing against the safety window"
            return text, RED
        return "MOTORS ON - holding, hands clear", AMBER
    if telemetry.enabled:
        return "armed, but the firmware has not reported the motors on", AMBER
    return "connected - motors OFF (safe to touch)", GREEN


class SimulatedLink:
    """A synthetic STM32 stand-in for ``--simulate``.

    Implements the ``SerialLink`` surface the recorder and the motion loop use,
    so the window, the capture path, the safety chain and both output files are
    exercised exactly as they would be with hardware attached.

    The pose integrates toward the last enabled target with a first-order lag.
    That matters beyond realism: without it the measured-position window and the
    stall detector cannot be driven at all without a robot, so the detector whose
    entire job is catching a silently stalled joint would never run outside the
    unit tests. Set ``blocked`` to model a joint that will not move.
    """

    DRIFT_RAD = 0.02
    NOISE_RAD = 0.0005
    PERIOD_S = 60.0
    #: Fraction of the remaining gap the simulated joint closes per command.
    FOLLOW_GAIN = 0.35
    #: How long after arming the simulated motors take to report mode 2, which
    #: is what STATE_MOTORS_ENABLED reflects on the real firmware.
    MOTOR_REPORT_DELAY_S = 0.10

    def __init__(self, seed: int = 20260924) -> None:
        self.sent: list[tuple] = []
        self.pose = np.asarray(config.Q_DEFAULT, dtype=np.float64).copy()
        self.blocked = False
        self._rng = np.random.default_rng(seed)
        self._sequence = 0
        self._lock = threading.Lock()
        self._start = time.monotonic()
        self._enabled_since: float | None = None

    def _next(self) -> StatePacket:
        elapsed = time.monotonic() - self._start
        drift = self.DRIFT_RAD * math.sin(2.0 * math.pi * elapsed / self.PERIOD_S)
        with self._lock:
            self._sequence += 1
            sequence = self._sequence
            base = self.pose.copy()
            reported = (
                self._enabled_since is not None
                and time.monotonic() - self._enabled_since >= self.MOTOR_REPORT_DELAY_S
            )
        noise = self.NOISE_RAD * self._rng.standard_normal(config.NUM_JOINTS)
        flags = STATE_ENCODERS_VALID
        if reported:
            flags |= STATE_MOTORS_ENABLED
        return StatePacket(
            sequence=sequence,
            timestamp_us=(time.monotonic_ns() // 1000) & 0xFFFFFFFF,
            joint_position=(base + drift + noise).astype(np.float32),
            joint_velocity=np.zeros(config.NUM_JOINTS, dtype=np.float32),
            accel_m_s2=np.array([0.0, 0.0, 9.81], dtype=np.float32),
            gyro_rad_s=np.zeros(3, dtype=np.float32),
            orientation_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            status_flags=flags,
        )

    def get_latest_state(self, max_age_s: float = 0.05) -> StatePacket:
        return self._next()

    def wait_for_state(self, timeout_s: float = 3.0) -> StatePacket:
        return self._next()

    def send_command(self, timestamp_us, joint_target, kp_scale, kd_scale,
                     command_flags, lock_timeout_s=None) -> bool:
        target = np.asarray(joint_target, dtype=np.float64).copy()
        self.sent.append(
            (timestamp_us, target.copy(), kp_scale, kd_scale, command_flags)
        )
        live = bool(command_flags & COMMAND_ENABLE) and not (
            command_flags & COMMAND_ESTOP
        )
        with self._lock:
            if live:
                if self._enabled_since is None:
                    self._enabled_since = time.monotonic()
                if not self.blocked:
                    self.pose = self.pose + self.FOLLOW_GAIN * (target - self.pose)
            else:
                self._enabled_since = None
        return True

    def close(self) -> None:
        pass


class CaptureWorker:
    """Run one capture off the Tk thread so the window stays responsive."""

    def __init__(self, session: CaptureSession, samples: int, tolerance_rad: float,
                 timeout_s: float) -> None:
        self._session = session
        self._samples = samples
        self._tolerance_rad = tolerance_rad
        self._timeout_s = timeout_s
        self._results: queue.Queue = queue.Queue(maxsize=1)
        self._thread = threading.Thread(
            target=self._run, name="joint-angle-capture", daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        try:
            snapshot = self._session.snapshot(
                samples=self._samples,
                tolerance_rad=self._tolerance_rad,
                timeout_s=self._timeout_s,
            )
        except Exception as exc:  # noqa: BLE001 - the GUI must show any failure
            self._results.put(("error", f"{type(exc).__name__}: {exc}"))
        else:
            self._results.put(("ok", snapshot))

    def poll(self):
        """Return ``None`` while running, else ``("ok", snapshot)`` or ``("error", text)``."""
        try:
            return self._results.get_nowait()
        except queue.Empty:
            return None

    def is_alive(self) -> bool:
        return self._thread.is_alive()


class JointAngleApp:
    """The window.

    Threading rules, which are load-bearing:

    * The Tk main thread owns every widget. No other thread may touch one.
    * ``ControlLoop`` is the only thread that writes to the serial port. It
      replaced the read-only ``StreamLogger`` because the two must never
      overlap: one echoes the measured pose with zero gains while the other
      commands a trajectory, and the STM32 would see an alternating
      enable/disable stream and chatter.
    * A capture runs on a short-lived worker thread, because ``snapshot()``
      blocks. Results come back through a queue and are applied by the Tk
      thread, so the store has one writer.
    * The display reads ``link.get_latest_state()`` directly rather than
      anything the writer published, so it keeps showing encoders no matter
      which component is currently commanding.
    """

    def __init__(self, root, link, loop, store, session, session_dir, args) -> None:
        self.root = root
        self.link = link
        self.loop = loop
        self.store = store
        self.session = session
        self.session_dir = session_dir
        self.args = args
        self.tolerance_rad = float(np.deg2rad(args.stable_tol_deg))
        self._worker: CaptureWorker | None = None
        self._last_packet_at = 0.0
        self._last_sequence: int | None = None
        self._pending_label: str | None = None
        self._closed = False
        self._refresh_job: str | None = None
        self._keypoints: list = []
        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._refresh()

    # ---------------------------------------------------------------- widgets

    def _build_widgets(self) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.root.title("Joint angle tool")
        self.root.minsize(760, 700)

        container = ttk.Frame(self.root, padding=10)
        container.pack(fill="both", expand=True)
        container.columnconfigure(0, weight=1)
        container.rowconfigure(1, weight=1)

        self.banner = ttk.Label(container, text="starting...", anchor="w",
                                font=("TkDefaultFont", 11, "bold"))
        self.banner.grid(row=0, column=0, sticky="ew", pady=(0, 6))

        self.notebook = ttk.Notebook(container)
        self.notebook.grid(row=1, column=0, sticky="nsew")

        record = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(record, text="Record")
        self._build_record_tab(record, ttk)

        motion = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(motion, text="Fixed policy")
        self._build_motion_tab(motion, ttk)

        self.last_capture = ttk.Label(container, text="last capture: none yet", anchor="w")
        self.last_capture.grid(row=2, column=0, sticky="ew", pady=(6, 0))

        self.footer = ttk.Label(container, text="", anchor="w", justify="left")
        self.footer.grid(row=3, column=0, sticky="ew", pady=(6, 0))

        buttons = ttk.Frame(container)
        buttons.grid(row=4, column=0, sticky="ew", pady=(10, 0))
        self.estop_hint = ttk.Label(buttons, text="", anchor="w", justify="left")
        self.estop_hint.pack(side="left", fill="x", expand=True)
        self.estop_button = tk.Button(
            buttons, text="EMERGENCY STOP", command=self._on_estop,
            background="#c00000", foreground="white", activebackground="#ff2020",
            font=("TkDefaultFont", 12, "bold"), height=2, width=22,
        )
        self.estop_button.pack(side="right")
        # Reachable without the mouse: on a bench the hand is not on the trackpad.
        for sequence in ("<Escape>", "<space>"):
            self.root.bind_all(sequence, self._on_estop_key)

    def _build_record_tab(self, parent, ttk) -> None:
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)
        ttk.Label(
            parent,
            text="Motors stay off here. Position the robot by hand, then capture.",
            anchor="w", foreground="#555555",
        ).grid(row=0, column=0, sticky="ew", pady=(0, 6))

        columns = ("index", "joint", "rad", "deg")
        self.tree = ttk.Treeview(parent, columns=columns, show="headings", height=12)
        for column, heading, width, anchor in (
            ("index", "#", 40, "e"),
            ("joint", "joint", 230, "w"),
            ("rad", "rad", 120, "e"),
            ("deg", "deg", 100, "e"),
        ):
            self.tree.heading(column, text=heading)
            self.tree.column(column, width=width, anchor=anchor,
                             stretch=column == "joint")
        for index, name in enumerate(config.JOINT_NAMES):
            self.tree.insert("", "end", iid=str(index), values=(index, name, "", ""))
        self.tree.grid(row=1, column=0, sticky="nsew")

        row = ttk.Frame(parent)
        row.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        self.capture_button = ttk.Button(row, text="Capture key point",
                                         command=self._on_capture)
        self.capture_button.pack(side="left")
        self.capture_status = ttk.Label(row, text="", anchor="w")
        self.capture_status.pack(side="left", padx=12)

    def _build_motion_tab(self, parent, ttk) -> None:
        import tkinter as tk

        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

        ttk.Label(
            parent,
            text="Type target angles in degrees, or load a key point file.",
            anchor="w", foreground="#555555",
        ).grid(row=0, column=0, sticky="ew", pady=(0, 6))

        angles = ttk.LabelFrame(parent, text="target angles (degrees)", padding=8)
        angles.grid(row=1, column=0, sticky="nsew")
        for column, heading in enumerate(("joint", "current", "target")):
            ttk.Label(angles, text=heading).grid(row=0, column=column, sticky="w", padx=4)
        self.target_entries: dict = {}
        self.current_labels: dict = {}
        for index, name in enumerate(config.JOINT_NAMES):
            ttk.Label(angles, text=name).grid(row=index + 1, column=0, sticky="w", padx=4)
            current = ttk.Label(angles, text="-", width=10, anchor="e")
            current.grid(row=index + 1, column=1, sticky="e", padx=4)
            self.current_labels[index] = current
            variable = tk.StringVar(value="")
            entry = ttk.Entry(angles, textvariable=variable, width=10, justify="right")
            entry.grid(row=index + 1, column=2, sticky="e", padx=4)
            self.target_entries[index] = variable

        controls = ttk.Frame(parent)
        controls.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(controls, text="Use current pose",
                   command=self._fill_targets_from_current).pack(side="left")
        ttk.Button(controls, text="Move to these angles",
                   command=self._on_move_to_typed).pack(side="left", padx=6)

        keypoints = ttk.LabelFrame(parent, text="key points", padding=8)
        keypoints.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        keypoints.columnconfigure(0, weight=1)
        self.keypoint_list = tk.Listbox(keypoints, height=6, exportselection=False)
        self.keypoint_list.grid(row=0, column=0, columnspan=3, sticky="ew")
        self.keypoint_status = ttk.Label(keypoints, text="no file loaded", anchor="w")
        self.keypoint_status.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(4, 4))
        ttk.Button(keypoints, text="Load file...",
                   command=self._on_load_keypoints).grid(row=2, column=0, sticky="w")
        ttk.Button(keypoints, text="Move to selected",
                   command=self._on_move_to_keypoint).grid(row=2, column=1, padx=6)
        ttk.Button(keypoints, text="Play all in order",
                   command=self._on_play_sequence).grid(row=2, column=2)

        arm = ttk.Frame(parent)
        arm.grid(row=4, column=0, sticky="ew", pady=(8, 0))
        self.arm_button = ttk.Button(arm, text="Enable motors", command=self._on_toggle_arm)
        self.arm_button.pack(side="left")
        self.arm_status = ttk.Label(arm, text="motors disabled", anchor="w")
        self.arm_status.pack(side="left", padx=12)

    # ---------------------------------------------------------------- refresh

    def _read_measured(self):
        """Latest policy-frame pose, or ``(None, None)``.

        Reads the link rather than anything the writer published, so the display
        keeps working whichever component is currently commanding.
        """
        try:
            state = self.link.get_latest_state(max_age_s=TELEMETRY_GRACE_S)
        except TimeoutError:
            return None, None
        return state, np.asarray(
            config.motor_to_policy_position(state.joint_position), dtype=np.float64
        )

    def _refresh(self) -> None:
        # The window can be destroyed between two scheduled calls; rescheduling
        # after that raises a Tcl error from the after queue.
        if self._closed:
            return
        state, measured = self._read_measured()
        now = time.monotonic()
        if state is not None and state.sequence != self._last_sequence:
            self._last_sequence = state.sequence
            self._last_packet_at = now
        stale = self._last_packet_at == 0.0 or now - self._last_packet_at > TELEMETRY_GRACE_S

        if measured is not None:
            for index, _name, radians, degrees in build_rows(measured):
                self.tree.set(str(index), "rad", f"{radians:+.6f}")
                self.tree.set(str(index), "deg", f"{degrees:+.3f}")
                self.current_labels[index].config(text=f"{degrees:+.2f}")
        else:
            for index in range(config.NUM_JOINTS):
                self.tree.set(str(index), "rad", "")
                self.tree.set(str(index), "deg", "")
                self.current_labels[index].config(text="-")

        telemetry = self.loop.snapshot()
        text, colour = describe_status(telemetry=telemetry, stale=stale, linked=True)
        self.banner.config(text=text, foreground=colour)
        self._refresh_arm_button(telemetry)
        self.footer.config(text=self._footer_text(telemetry, state, stale))
        self._refresh_job = self.root.after(REFRESH_MS, self._refresh)

    def _refresh_arm_button(self, telemetry) -> None:
        armed = telemetry.enabled
        self.arm_button.config(text="Disable motors" if armed else "Enable motors")
        if telemetry.fault or telemetry.mode == "estop":
            self.arm_button.config(state="normal", text="Clear stop")
            self.arm_status.config(
                text="press Clear stop, then Enable", foreground=RED
            )
        else:
            self.arm_button.config(state="normal")
            self.arm_status.config(
                text="motors enabled - disarming drops the robot"
                if armed else "motors disabled",
                foreground=AMBER if armed else "#333333",
            )
        # The red button's meaning is stated next to it rather than by
        # relabelling it: under stress an operator reads position and colour,
        # not text, so a button whose label moves is a trap.
        self.estop_hint.config(
            text=("Stop the motion loop and de-energize the motors."
                  if armed else "Nothing is moving. This is already safe.")
        )

    def _footer_text(self, telemetry, state, stale) -> str:
        port = "simulated link" if self.args.simulate else self.args.port
        decoder = getattr(self.link, "decoder", None)
        crc = getattr(decoder, "crc_errors", 0)
        return "\n".join(
            [
                f"port: {port}   telemetry: {'stale' if stale else 'ok'}"
                f"   crc errors: {crc}   state_seq: {getattr(state, 'sequence', '-')}",
                f"mode: {telemetry.mode}   flags: 0x{telemetry.state_flags:08X}"
                f"   key points saved: {len(self.store)}",
                f"session: {self.session_dir}",
            ]
        )

    # ---------------------------------------------------------------- capture

    def _on_capture(self) -> None:
        from tkinter import simpledialog

        name = simpledialog.askstring(
            "Capture key point", "Name (leave empty to auto-number):", parent=self.root
        )
        if name is None:
            return
        self._pending_label = name.strip() or None
        self.capture_button.config(state="disabled")
        self.capture_status.config(text="capturing...", foreground="#333333")
        self._worker = CaptureWorker(
            self.session, self.args.samples, self.tolerance_rad, self.args.timeout
        )
        self._worker.start()
        self.root.after(CAPTURE_POLL_MS, self._poll_capture)

    def _poll_capture(self) -> None:
        if self._worker is None:
            return
        result = self._worker.poll()
        if result is None:
            if self._worker.is_alive():
                self.root.after(CAPTURE_POLL_MS, self._poll_capture)
                return
            result = ("error", "capture thread stopped without a result")
        self._worker = None
        self.capture_button.config(state="normal")
        kind, payload = result
        if kind == "error":
            self._pending_label = None
            self.capture_status.config(text=f"capture failed: {payload}", foreground=RED)
            return
        self._store_keypoint(payload)

    def _store_keypoint(self, snapshot) -> None:
        """Build and persist the key point on the Tk thread (the only writer)."""
        index = len(self.store)
        label = self._pending_label or f"keypoint_{index:03d}"
        self._pending_label = None
        keypoint = keypoint_from(snapshot, index, label, self.tolerance_rad)
        # A key point taken while armed records a commanded pose, not a
        # hand-posed one. Say so, so nobody later plays it back as "hand-posed".
        keypoint["motors_enabled"] = bool(self.loop.snapshot().enabled)
        self.store.add(keypoint)
        text, stable = describe_capture(snapshot, label, index, self.tolerance_rad)
        self.last_capture.config(text=text, foreground=GREEN if stable else RED)
        self.capture_status.config(
            text=f"saved '{label}'" if stable else f"saved '{label}' - pose was moving",
            foreground=GREEN if stable else AMBER,
        )

    # ---------------------------------------------------------------- motion

    def _fill_targets_from_current(self) -> None:
        _state, measured = self._read_measured()
        if measured is None:
            self._set_motion_status("no telemetry to copy", RED)
            return
        for index, value in enumerate(measured):
            self.target_entries[index].set(f"{np.rad2deg(value):+.2f}")

    def _typed_targets(self):
        """Parse the entries, refusing anything outside the limits.

        Refusing rather than clamping is deliberate. Silently clamping would let
        the operator believe the robot went to 0.9 rad when it went to 0.45.
        """
        values = np.zeros(config.NUM_JOINTS, dtype=np.float64)
        for index, name in enumerate(config.JOINT_NAMES):
            text = self.target_entries[index].get().strip()
            if not text:
                self._set_motion_status(f"{name}: enter a value first", RED)
                return None
            try:
                values[index] = np.deg2rad(float(text))
            except ValueError:
                self._set_motion_status(f"{name}: {text!r} is not a number", RED)
                return None
        low = config.Q_LOWER + config.JOINT_LIMIT_MARGIN_RAD
        high = config.Q_UPPER - config.JOINT_LIMIT_MARGIN_RAD
        outside = (values < low) | (values > high)
        if np.any(outside):
            names = [n for n, hit in zip(config.JOINT_NAMES, outside) if hit]
            self._set_motion_status(
                f"refused - outside the joint limits: {', '.join(names)}", RED
            )
            return None
        return values

    def _on_move_to_typed(self) -> None:
        targets = self._typed_targets()
        if targets is None:
            return
        self._request_move(targets, "typed angles")

    def _on_load_keypoints(self) -> None:
        from tkinter import filedialog, messagebox

        import joint_motion

        path = filedialog.askopenfilename(
            title="Open a keypoints.json",
            filetypes=[("Key point files", "*.json")],
            initialdir=str(self.session_dir),
        )
        if not path:
            return
        try:
            self._keypoints = joint_motion.load_keypoints(path)
        except joint_motion.KeypointFileError as exc:
            messagebox.showerror("Cannot use that key point file", str(exc))
            return
        self.keypoint_list.delete(0, "end")
        for keypoint in self._keypoints:
            flag = "" if keypoint.stable else "   [NOT STABLE when recorded]"
            self.keypoint_list.insert("end", f"{keypoint.index}  {keypoint.name}{flag}")
        unstable = [k.name for k in self._keypoints if not k.stable]
        note = f"; {len(unstable)} were not stable when recorded" if unstable else ""
        self.keypoint_status.config(
            text=f"{len(self._keypoints)} key points from {Path(path).name}{note}",
            foreground=AMBER if unstable else "#333333",
        )

    def _selected_keypoint(self):
        selection = self.keypoint_list.curselection()
        if not selection or not self._keypoints:
            return None
        return self._keypoints[selection[0]]

    def _on_move_to_keypoint(self) -> None:
        keypoint = self._selected_keypoint()
        if keypoint is None:
            self._set_motion_status("select a key point first", RED)
            return
        self._request_move(keypoint.rad, f"keypoint {keypoint.name}")

    def _on_play_sequence(self) -> None:
        if not self._keypoints:
            self._set_motion_status("load a key point file first", RED)
            return
        import joint_motion

        segments = [
            joint_motion.Segment(destination_policy_rad=keypoint.rad, duration_s=0.0)
            for keypoint in self._keypoints
        ]
        self.loop.request_trajectory(segments, label=f"{len(segments)} key points")
        self._set_motion_status(
            f"playing {len(self._keypoints)} key points in order; "
            "it holds the last one and does not loop",
            "#333333",
        )

    def _request_move(self, targets, label: str) -> None:
        import joint_motion

        _state, measured = self._read_measured()
        if measured is None:
            self._set_motion_status("no telemetry; refusing to move", RED)
            return
        preview = joint_motion.preview_move(targets, measured)
        if not preview.is_inside_limits:
            self._set_motion_status("refused - a target is outside the joint limits", RED)
            return
        if self.loop.snapshot().enabled and not self._confirm(
            preview.describe(), "Move", "MOVE"
        ):
            return
        self.loop.request_move(targets, label=label)
        self._set_motion_status(preview.describe(), "#333333")

    def _confirm(self, detail: str, title: str, word: str) -> bool:
        """Require a typed word, not a button.

        The repo's own precedent is motor_test_common.require_confirmation
        demanding a typed string. A button would also be defeated by a
        double-click landing on it.
        """
        from tkinter import simpledialog

        answer = simpledialog.askstring(
            title,
            f"{detail}\n\nThe robot will move. Keep hands clear and the physical\n"
            f"emergency stop within reach.\n\nType {word} to proceed:",
            parent=self.root,
        )
        return (answer or "").strip().upper() == word

    def _set_motion_status(self, text: str, colour: str) -> None:
        self.capture_status.config(text=text, foreground=colour)

    # ---------------------------------------------------------------- arming

    def _on_toggle_arm(self) -> None:
        telemetry = self.loop.snapshot()
        if telemetry.fault or telemetry.mode == "estop":
            self.loop.reset_fault()
            self._set_motion_status("stop cleared; press Enable to arm", "#333333")
            return
        if telemetry.enabled:
            # Disarming de-energizes; it does not hold the pose.
            self.loop.set_enable(False)
            self._set_motion_status(
                "motors disabled - a standing robot will go limp", AMBER
            )
            return
        if not (config.CALIBRATION_CONFIRMED and config.IMU_CALIBRATION_CONFIRMED):
            self._set_motion_status(
                "refused - confirm the calibrations in config.py first", RED
            )
            return
        if not self._confirm(
            "Enabling drives every motor toward the commanded pose.",
            "Enable motors", "ENABLE",
        ):
            return
        self.loop.set_enable(True)
        self._set_motion_status("motors enabled", AMBER)

    def _on_estop(self) -> None:
        self.loop.set_enable(False)
        self.loop.emergency_stop("operator")
        self._set_motion_status("EMERGENCY STOP - motors commanded off", RED)

    def _on_estop_key(self, _event) -> None:
        self._on_estop()

    # ---------------------------------------------------------------- shutdown

    def close(self) -> None:
        """Stop the loop and release the port. Idempotent."""
        if self._closed:
            return
        self._closed = True
        # Cancel the pending refresh before the widgets go away, or Tk raises
        # "invalid command name ..._refresh" when it runs the stale after-script.
        if self._refresh_job is not None:
            try:
                self.root.after_cancel(self._refresh_job)
            except Exception:  # noqa: BLE001 - the interpreter may already be gone
                pass
            self._refresh_job = None
        try:
            self.loop.shutdown(timeout_s=1.5)
        finally:
            try:
                self.root.destroy()
            except Exception:  # noqa: BLE001 - the window may already be gone
                pass


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.samples < 1:
        raise SystemExit("--samples must be at least 1")
    if args.stable_tol_deg <= 0.0:
        raise SystemExit("--stable-tol-deg must be positive")
    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be positive")

    import tkinter as tk
    from tkinter import messagebox, ttk

    import joint_motion

    root = tk.Tk()
    try:
        style = ttk.Style()
        if "clam" in style.theme_names():
            style.theme_use("clam")
    except Exception:  # noqa: BLE001 - theming is cosmetic
        pass

    if args.simulate:
        link = SimulatedLink()
    else:
        try:
            link = joint_motion.open_link(args.port, args.baud)
        except Exception as exc:  # noqa: BLE001 - a bad port must not crash the app
            messagebox.showerror(
                "Cannot open the serial port",
                f"{type(exc).__name__}: {exc}\n\nPort: {args.port}\n\n"
                "Check with:  ls /dev/ttyACM*\n"
                "Permissions: sudo usermod -aG dialout $USER  (then log out and back in)\n"
                "Another program may hold it: main.py, read_joint_angles.py, "
                "hold_standing_pose.py.\n\n"
                "Or run with --simulate to try the window with no hardware.",
            )
            root.destroy()
            return 1

    session_dir = Path(args.out_dir).expanduser() / (args.session or timestamp())
    session_dir.mkdir(parents=True, exist_ok=True)
    store = KeypointStore(
        session_dir / "keypoints.json",
        {
            "host_time_iso": iso_now(),
            "port": args.port,
            "joint_names": list(config.JOINT_NAMES),
            "stable_tol_deg": args.stable_tol_deg,
            "samples": args.samples,
        },
    )
    session = CaptureSession(link)
    loop = joint_motion.ControlLoop(
        link, log_path=session_dir / "commands.csv", log_hz=args.log_hz
    )

    try:
        first = session.wait_for_first(timeout_s=5.0, send_probe=not args.no_keepalive)
        print(f"First state packet: sequence={first.sequence} "
              f"flags=0x{first.status_flags:08X}")
        store.save()
        loop.start()
        print(f"Session directory: {session_dir}")
        print("Motors stay disabled until you enable them in the window.")
    except Exception as exc:  # noqa: BLE001
        messagebox.showerror(
            "No STM32 state packet",
            f"{type(exc).__name__}: {exc}\n\nPort: {args.port}\n"
            "The STM32 must be powered and sending before the window can start.\n\n"
            "Or run with --simulate to try the window with no hardware.",
        )
        link.close()
        root.destroy()
        return 1

    app = JointAngleApp(root, link, loop, store, session, session_dir, args)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # Idempotent, and the only place that guarantees the log is flushed even
        # if the mainloop returns for a reason other than a normal close.
        app.close()
        print(f"Session: {session_dir}")
        print(f"  {len(store)} key points -> keypoints.json")
        print(f"  {loop._log_rows_written()} command rows -> commands.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
