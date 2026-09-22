#!/usr/bin/env python3
"""Motor-side protocol monitor for the V2 P1 frame stream.

Two input modes:
  --file <path>   Replay a .bin capture written by the Webots FileSink.
  --udp <port>    Listen on a UDP port (Webots UdpSink default port: 56565).

Outputs both per-frame decode lines and an aggregate summary on EOF / Ctrl-C:
  - Total frames, frames per type, CRC errors, dropped bytes
  - Sequence gaps (mod 256)
  - Mean / p95 inter-frame interval per type
  - Safety state per PDF section 7 (SAFE_STOP/LOST_RECOVERY)

Examples:
  python tools/motor_listener.py --file generated/uart_dump.bin
  python tools/motor_listener.py --udp 56565 --print-every 10
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from typing import Dict, List

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "openmv"))

import protocol_v2 as p2  # type: ignore  # noqa: E402


def percentile(values: List[float], p: float) -> float:
	if not values:
		return float("nan")
	s = sorted(values)
	idx = int(round((len(s) - 1) * p))
	return s[max(0, min(len(s) - 1, idx))]


def msg_name(t: int) -> str:
	return {
		p2.MSG_HEARTBEAT: "HB",
		p2.MSG_LINE_CTRL: "LINE_CTRL",
		p2.MSG_QR_EVENT: "QR_EVENT",
		p2.MSG_OBSTACLE_EVENT: "OBSTACLE",
		p2.MSG_MODE_SWITCH_REQ: "MODE_SW",
		p2.MSG_ACK: "ACK",
		p2.MSG_ROBOT_STATE: "ROBOT_STATE",
		p2.MSG_ESTOP: "ESTOP",
	}.get(t, "0x%02X" % t)


class Stats:
	def __init__(self):
		self.total = 0
		self.by_type: Dict[int, int] = {}
		# seq counter is shared across all msg_types in this implementation,
		# so we track gaps globally (per-type tracking would always misreport).
		self.last_global_seq = None
		self.global_seq_gaps = 0
		self.intervals: Dict[int, List[float]] = {}
		self.last_ts: Dict[int, float] = {}
		self.last_ctrl_wall = None
		self.last_hb_wall = None
		self.safety_events = 0

	def observe(self, frame: p2.Frame, wall_ms: float):
		self.total += 1
		t = frame.msg_type
		self.by_type[t] = self.by_type.get(t, 0) + 1

		if self.last_global_seq is not None:
			gap = (frame.seq - self.last_global_seq) & 0xFF
			if gap != 1:
				self.global_seq_gaps += 1
		self.last_global_seq = frame.seq

		# inter-frame interval per type
		if t in self.last_ts:
			self.intervals.setdefault(t, []).append(wall_ms - self.last_ts[t])
		self.last_ts[t] = wall_ms

		if t == p2.MSG_LINE_CTRL:
			self.last_ctrl_wall = wall_ms
		elif t == p2.MSG_HEARTBEAT:
			self.last_hb_wall = wall_ms

	def safety_check(self, now_ms: float, safe_stop_ms: float = 300.0) -> bool:
		oldest = None
		for last in (self.last_ctrl_wall, self.last_hb_wall):
			if last is None:
				continue
			oldest = last if oldest is None else max(oldest, last)
		if oldest is None:
			return False
		return (now_ms - oldest) > safe_stop_ms


def print_frame(frame: p2.Frame, idx: int, every: int) -> None:
	if every > 0 and (idx % every) != 0:
		return
	t = frame.msg_type
	tag = msg_name(t)
	common = "#%05d %-12s seq=%3d ts=%10dms flags=0x%02X len=%d" % (
		idx, tag, frame.seq, frame.ts_ms, frame.flags, len(frame.payload),
	)
	if t == p2.MSG_LINE_CTRL:
		d = p2.parse_line_ctrl(frame.payload) or {}
		print("%s | mode=%s conf=%3s lost=%s route=%s ex=%5smm ang=%6scdeg" % (
			common, d.get("mode"), d.get("conf"), d.get("lost"), d.get("route"),
			d.get("ex_mm"), d.get("ang_cdeg"),
		))
	elif t == p2.MSG_HEARTBEAT:
		d = p2.parse_heartbeat(frame.payload) or {}
		print("%s | mode=%s" % (common, d.get("mode")))
	elif t == p2.MSG_ACK:
		d = p2.parse_ack(frame.payload) or {}
		print("%s | ack_seq=%s ack_msg=0x%02X code=%s" % (
			common, d.get("ack_seq"), int(d.get("ack_msg_type", 0)) & 0xFF, d.get("code"),
		))
	else:
		print("%s | payload=%s" % (common, frame.payload.hex()))


def summary(parser: p2.StreamParser, stats: Stats) -> None:
	print("\n========= protocol monitor summary =========")
	print("frames_ok       :", parser.frames_ok)
	print("crc_errors      :", parser.crc_errors)
	print("discarded_bytes :", parser.discarded_bytes)
	print("total observed  :", stats.total)
	print("global_seq_gaps :", stats.global_seq_gaps,
		  "(0 expected; seq counter is shared across msg types)")
	print("by msg_type     :")
	for t, n in sorted(stats.by_type.items()):
		print("  %-12s : %d" % (msg_name(t), n))
	print("inter-frame interval (ms): mean / p95 / max")
	for t, ivs in stats.intervals.items():
		if not ivs:
			continue
		mean = sum(ivs) / len(ivs)
		print("  %-12s : %.1f / %.1f / %.1f  (n=%d)" % (
			msg_name(t), mean, percentile(ivs, 0.95), max(ivs), len(ivs),
		))


def run_file(path: str, every: int, chunk: int = 1024) -> int:
	if not os.path.isfile(path):
		print(f"[ERROR] no such file: {path}")
		return 2
	parser = p2.StreamParser()
	stats = Stats()
	idx = 0
	with open(path, "rb") as f:
		while True:
			data = f.read(chunk)
			if not data:
				break
			frames = parser.feed(data)
			for fr in frames:
				idx += 1
				print_frame(fr, idx, every)
				stats.observe(fr, fr.ts_ms)
	summary(parser, stats)
	return 0


def run_udp(port: int, every: int, host: str = "0.0.0.0") -> int:
	parser = p2.StreamParser()
	stats = Stats()
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.bind((host, port))
	sock.settimeout(1.0)
	print(f"[INFO] listening UDP on {host}:{port}, Ctrl-C to stop")
	idx = 0
	t0 = time.time() * 1000.0
	try:
		while True:
			try:
				data, _addr = sock.recvfrom(2048)
			except socket.timeout:
				now = time.time() * 1000.0 - t0
				if stats.safety_check(now):
					print(f"[SAFETY] >300ms without LINE_CTRL/HB at t={now:.0f}ms")
					stats.safety_events += 1
				continue
			now = time.time() * 1000.0 - t0
			frames = parser.feed(data)
			for fr in frames:
				idx += 1
				print_frame(fr, idx, every)
				stats.observe(fr, now)
	except KeyboardInterrupt:
		print("\n[INFO] stopped by user")
	finally:
		sock.close()
	summary(parser, stats)
	print("safety_events   :", stats.safety_events)
	return 0


def main() -> int:
	ap = argparse.ArgumentParser()
	g = ap.add_mutually_exclusive_group(required=True)
	g.add_argument("--file", help="Read frames from a binary capture file")
	g.add_argument("--udp", type=int, help="Listen on a UDP port for live frames")
	ap.add_argument("--print-every", type=int, default=1,
					help="Print every Nth frame (set 0 to mute per-frame output)")
	args = ap.parse_args()

	if args.file:
		return run_file(args.file, args.print_every)
	return run_udp(args.udp, args.print_every)


if __name__ == "__main__":
	sys.exit(main())
