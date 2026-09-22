"""图卡触发离线评估 — 统计触发次数，用于验证误触发拦截。

跑一段视频或相机，报告：触发了几次、每次是什么形状、走了哪条检测路径。
空场地跑出 0 次 = 拦截有效；有图卡跑出 1 次 = 功能正常。

用法:
    python scripts/evaluate_shape_trigger.py --video run.mp4
    python scripts/evaluate_shape_trigger.py --cam 1 --seconds 20
    python scripts/evaluate_shape_trigger.py --video run.mp4 --dump-frames out/

环境变量 SHAPE_TRIGGER_DIST_CM 可覆盖触发距离（默认 40cm），便于现场调参回放。
"""
import argparse
import os
import sys
import time

import cv2

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "jetson"))
from shape_detector import ShapeDetector  # noqa: E402

SHAPE_NAME = {1: "circle/举左手", 2: "pentagon/举右手", 3: "square/抬左腿",
              4: "diamond/抬右腿", 5: "cross/举双手", 6: "triangle/摇头"}


def main():
    if sys.platform == "win32" and hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--video", help="视频文件路径")
    src.add_argument("--cam", type=int, help="相机索引")
    ap.add_argument("--seconds", type=float, default=20.0, help="相机模式的时长")
    ap.add_argument("--every", type=int, default=3,
                    help="每 N 帧检测一次（对齐 run_robot 的隔帧调用，默认3）")
    ap.add_argument("--lane-err-cm", type=float, default=None,
                    help="机身相对赛道中心的偏差cm；不传按画面中央算")
    ap.add_argument("--stable-frames", type=int, default=3)
    ap.add_argument("--cooldown-ms", type=int, default=3200)
    ap.add_argument("--dump-frames", default=None,
                    help="触发时把该帧存到该目录")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video if args.video else args.cam)
    if not cap.isOpened():
        print(f"[ERROR] 打不开 {args.video or ('相机 ' + str(args.cam))}")
        sys.exit(1)

    sd = ShapeDetector(stable_frames=args.stable_frames,
                       cooldown_ms=args.cooldown_ms, debug=False)
    print(f"触发距离 {sd.cfg['trigger_dist_cm']:.0f}cm  "
          f"→ 框宽 ≥{sd.cfg['trigger_box_w']:.0f}px  中心y ≥{sd.cfg['trigger_y']:.0f}  "
          f"赛道半宽 17.5cm")

    if args.dump_frames:
        os.makedirs(args.dump_frames, exist_ok=True)

    t0 = time.time()
    n = trig = 0
    verified = fallback = near_pass = 0
    shapes = {}
    triggers = []

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        if args.cam is not None and (time.time() - t0) > args.seconds:
            break
        n += 1
        if n % args.every:
            continue

        action, dbg = sd.update(frame, lane_offset_cm=args.lane_err_cm)
        if dbg:
            if dbg.get("fallback"):
                fallback += 1
            elif dbg.get("card_found"):
                verified += 1
            if dbg.get("gate_near"):
                near_pass += 1
            s = dbg.get("shape")
            if s:
                shapes[s] = shapes.get(s, 0) + 1

        if action is not None:
            trig += 1
            qw = dbg.get("quad_work")
            box_w = cy = -1
            if qw is not None:
                xs, ys = qw[:, 0], qw[:, 1]
                box_w = float(xs.max() - xs.min())
                cy = (float(ys.min()) + float(ys.max())) * 0.5
            triggers.append((n, action))
            print(f"  >>> 帧 {n:5d}  action={action} "
                  f"({SHAPE_NAME.get(action, '?')})  "
                  f"框宽={box_w:.0f}  中心y={cy:.0f}")
            if args.dump_frames:
                cv2.imwrite(os.path.join(args.dump_frames, f"trigger_{n:05d}.png"),
                            frame)

    cap.release()
    dt = time.time() - t0

    print()
    print(f"处理帧数 {n}（每 {args.every} 帧检测一次，检测 {n // args.every} 次）"
          f"  用时 {dt:.1f}s")
    print(f"触发次数 {trig}")
    print(f"检测路径: verified {verified} 次 / fallback {fallback} 次"
          f"（fallback 不给触发权）")
    print(f"过近距闸门 {near_pass} 次")
    print(f"分类形状统计 {shapes}")
    print()
    if trig == 0:
        print("结论: 零触发 —— 空场地场景下拦截有效。")
        print("      若画面里确实有图卡，说明门槛偏严，检查 SHAPE_TRIGGER_DIST_CM。")
    else:
        print(f"结论: 触发 {trig} 次 —— 对照画面确认每张图卡只触发一次"
              f"（重复触发说明闩锁失效）。")


if __name__ == "__main__":
    main()
