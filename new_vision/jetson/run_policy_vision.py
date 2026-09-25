#!/usr/bin/env python3
"""New line detector -> steering PID -> UDP connector -> walking ONNX policy.

Run this entry point for policy walking, not run_robot.py's V2 serial output.
Only the policy process owns the STM32 serial device.

A detected shape card is reported in the connector's ``qr`` field, 1..6, for
``--card-hold-ms``. Acting on it is the receiver's business and is not decided
here; bar crossing is still not signalled.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import time

from camera_config import load as load_camera
from policy_bridge import ConnectorClient, SteeringController


def parse_args():
    camera = load_camera()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=int(os.getenv("CAM_IDX", camera["index"])))
    parser.add_argument("--width", type=int, default=camera["width"])
    parser.add_argument("--height", type=int, default=camera["height"])
    parser.add_argument("--camera-height-cm", type=float,
                        default=float(os.getenv("CAM_HEIGHT_CM", camera["mount_height_cm"])))
    parser.add_argument("--camera-pitch-deg", type=float,
                        default=float(os.getenv("CAM_PITCH_DEG", camera["pitch_deg"])))
    parser.add_argument("--camera-vfov-deg", type=float,
                        default=float(os.getenv("CAM_VFOV_DEG", camera["vfov_deg"])))
    parser.add_argument("--connector-host", default="127.0.0.1")
    parser.add_argument("--connector-port", type=int, default=5006)
    parser.add_argument("--vx", type=float, default=0.4,
                        help="Forward speed with valid detection, m/s")
    parser.add_argument("--max-wz", type=float, default=0.5,
                        help="Yaw-rate limit, rad/s (0..0.5)")
    parser.add_argument("--steer-full-scale-cm", type=float, default=10.0,
                        help="Cross-track error (cm) producing max-wz; smaller means stronger steering")
    parser.add_argument("--yaw-sign", type=int, choices=(-1, 1), default=1,
                        help="Image steering to policy yaw; flip it if the robot turns the wrong way")
    parser.add_argument("--step-len-cm", type=float, default=float(os.getenv("STEP_LEN_CM", "8")))
    parser.add_argument("--preview-gain", type=float,
                        default=float(os.getenv("PREVIEW_GAIN", "0")),
                        help="Heading feedforward: steps of predicted drift to steer out. "
                             "0 (default) leaves heading feedback to the detector's angle term, "
                             "which is 7x weaker but carries far less of the angle bias")
    parser.add_argument("--lost-hold-s", type=float, default=0.2,
                        help="Hold the last command this long after line loss before stopping")
    parser.add_argument("--deriv-pole", type=float,
                        default=float(os.getenv("JETSON_PID_D_FILTER", "0.78")),
                        help="IIR pole on the D term; higher is smoother, 0 disables the filter")
    parser.add_argument("--bias-cm", type=float,
                        default=float(os.getenv("STEER_BIAS_CM", "5")),
                        help="Standing trim added to fused_err_cm on curves, shifting where "
                             "the loop settles to cancel a one-sided lateral offset. Set it "
                             "to the err the log shows standing in the curve")
    parser.add_argument("--bias-gate-px", type=float,
                        default=float(os.getenv("STEER_BIAS_GATE_PX", "12")),
                        help="abs(curve_px) at which --bias-cm is fully applied; it fades "
                             "to zero by curve_px 0 so straights are untouched")
    parser.add_argument("--no-shape-detect", action="store_true",
                        help="Skip geometric card detection entirely; qr stays -1")
    parser.add_argument("--card-hold-ms", type=float,
                        default=float(os.getenv("CARD_HOLD_MS", "3000")),
                        help="How long qr keeps reporting a detected card. 3000 matches the "
                             "rules' action window; acting on it is the receiver's business. "
                             "Keep it under ShapeDetector cooldown_ms so cards cannot re-fire")
    parser.add_argument("--shape-every", type=int,
                        default=max(1, int(os.getenv("SHAPE_EVERY", "6"))),
                        help="Run card detection every N frames. Measured at 1280x720: "
                             "31 ms with no card in view, 95-122 ms with one, against "
                             "29 ms for the line detector alone. 6 keeps the loop near "
                             "29 Hz clear and 23 Hz while a card is visible")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=0.0,
                        help="0 runs until Ctrl+C")
    args = parser.parse_args()
    if not 1 <= args.connector_port <= 65535:
        parser.error("connector-port must be between 1 and 65535")
    if args.width <= 0 or args.height <= 0:
        parser.error("camera dimensions must be positive")
    if (not all(math.isfinite(v) for v in (args.camera_height_cm, args.camera_pitch_deg,
                                         args.camera_vfov_deg, args.max_seconds))
            or args.camera_height_cm <= 0 or not 0 < args.camera_vfov_deg < 180
            or not 0 < args.camera_pitch_deg < 90 or args.max_seconds < 0):
        parser.error("invalid camera geometry or max-seconds")
    if not math.isfinite(args.card_hold_ms) or args.card_hold_ms < 0:
        parser.error("card-hold-ms must be finite and nonnegative")
    if args.shape_every < 1:
        parser.error("shape-every must be at least 1")
    return args


def main():
    args = parse_args()
    # Reuse the dual-mode PID defaults/environment overrides of run_robot.py.
    def gains(mode, defaults):
        return tuple(float(os.getenv(f"JETSON_PID_{mode}_{name}", str(value)))
                     for name, value in zip(("KP", "KI", "KD"), defaults))

    controller = SteeringController(
        vx=args.vx, max_wz=args.max_wz, steer_full_scale_cm=args.steer_full_scale_cm,
        yaw_sign=args.yaw_sign, step_len_cm=args.step_len_cm, preview_gain=args.preview_gain,
        straight_gains=gains("STRAIGHT", (0.83, 0.004, 0.095)),
        curve_gains=gains("CURVE", (0.83, 0.006, 0.16)),
        integral_limit=float(os.getenv("JETSON_PID_I_CLAMP", "60")),
        lost_hold_s=args.lost_hold_s, deriv_pole=args.deriv_pole,
        bias_cm=args.bias_cm, bias_gate_px=args.bias_gate_px,
    )
    # Lazy imports keep --help and controller tests usable without a camera stack.
    import cv2
    from line_detector_v1_warp import LineDetector
    from utils import open_camera, show_debug_windows

    shape = shape_names = None
    if not args.no_shape_detect:
        from shape_detector import ShapeDetector
        # Same confirmation/cooldown run_robot.py uses, so a card triggers once.
        shape = ShapeDetector(stable_frames=3, cooldown_ms=3200, debug=False)
        shape_names = {number: name for name, number in shape.action_map.items()}

    stopped = False

    def stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    client = ConnectorClient(args.connector_host, args.connector_port)
    cap = None
    try:
        cap = open_camera(args.camera, args.width, args.height)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera {args.camera}")
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height
        detector = LineDetector(width, height, cam_height_cm=args.camera_height_cm,
                                cam_pitch_deg=args.camera_pitch_deg,
                                cam_vfov_deg=args.camera_vfov_deg)
        print(f"Camera {args.camera}: {width}x{height}; UDP -> "
              f"{args.connector_host}:{args.connector_port}; vx={args.vx} m/s; "
              f"max_wz={args.max_wz} rad/s; yaw_sign={args.yaw_sign}", flush=True)
        start = previous = time.monotonic()
        last_log = -math.inf
        frames = 0
        card_action = -1
        card_until = 0.0
        while not stopped:
            now = time.monotonic()
            if args.max_seconds > 0 and now - start >= args.max_seconds:
                break
            ok, frame = cap.read()
            if not ok:
                controller.reset()
                client.publish(0.0, 0.0, card_action)
                if now - last_log >= 0.5:
                    print("[vision -> connector] camera read failed; vx=0 wz=0", flush=True)
                    last_log = now
                time.sleep(0.02)
                continue
            _, _, confidence, visualization, debug = detector.process(frame)
            processed = time.monotonic()
            frames += 1
            if shape is not None and frames % args.shape_every == 0:
                action, _ = shape.update(
                    frame, lane_offset_cm=float(debug.get("base_err_cm", 0.0)))
                if action is not None:
                    card_action = action
                    card_until = processed + args.card_hold_ms / 1000.0
                    print(f"[shape] qr={action} ({shape_names.get(action, '?')}) "
                          f"held {args.card_hold_ms:.0f} ms", flush=True)
            if card_action != -1 and processed >= card_until:
                card_action = -1
            vx, wz = controller.command(debug, confidence, processed - previous)
            previous = processed
            client.publish(vx, wz, card_action)
            if processed - last_log >= 0.5:
                print(f"[vision -> connector] vx={vx:+.3f} m/s wz={wz:+.3f} rad/s "
                      f"steer={controller.last_steer:+.2f}cm "
                      f"err={debug.get('fused_err_cm', 0.0):+.1f}cm "
                      f"err_eff={controller.last_err_eff:+.1f}cm "
                      f"ang={debug.get('angle_err_deg', 0.0):+.1f}deg "
                      f"curve={int(bool(debug.get('curve_mode', False)))} "
                      f"curve_px={debug.get('curve_px', 0.0):+.0f}(thr18) "
                      f"far_px={debug.get('far_err_px', 0.0):+.0f} "
                      f"conf={confidence:.3f} lost={debug.get('lost_frames', '?')} "
                      f"qr={card_action}", flush=True)
                last_log = processed
            if not args.headless:
                cv2.putText(frame, f"vx={vx:+.3f} wz={wz:+.3f} Q=quit", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("Policy vision", frame)
                show_debug_windows(debug, visualization)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        client.close()
        if cap is not None:
            cap.release()
        if not args.headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
