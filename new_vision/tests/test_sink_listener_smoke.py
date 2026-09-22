"""End-to-end smoke test: sink writes -> listener parses.

Run: python tests/test_sink_listener_smoke.py
"""

from __future__ import annotations

import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "openmv"))
sys.path.insert(0, os.path.join(ROOT, "Webots", "controllers", "line_follow_transfer"))

import protocol_v2 as p2  # type: ignore  # noqa: E402
import uart_sink  # type: ignore  # noqa: E402


class TestFileSinkRoundtrip(unittest.TestCase):
	def test_write_then_parse(self):
		tmp = tempfile.NamedTemporaryFile(suffix=".bin", delete=False)
		tmp.close()
		try:
			sink = uart_sink.FileSink(tmp.name)
			proto = p2.VisionProtocolV2()
			# Emit 30 LINE_CTRL + HEARTBEAT pairs
			for i in range(30):
				sink.write(proto.build_heartbeat(mode_u8=1))
				sink.write(proto.build_line_ctrl(
					mode_u8=1, conf_u8=70 + (i % 5),
					lost_u8=0, route_u8=(i % 4) + 1,
					ex_mm_i16=-10 + i, ang_cdeg_i16=300 - i,
				))
			sink.close()

			parser = p2.StreamParser()
			with open(tmp.name, "rb") as f:
				frames = parser.feed(f.read())

			self.assertEqual(parser.crc_errors, 0)
			self.assertEqual(parser.discarded_bytes, 0)
			self.assertEqual(len(frames), 60)
			hb = [f for f in frames if f.msg_type == p2.MSG_HEARTBEAT]
			ctrl = [f for f in frames if f.msg_type == p2.MSG_LINE_CTRL]
			self.assertEqual(len(hb), 30)
			self.assertEqual(len(ctrl), 30)

			d = p2.parse_line_ctrl(ctrl[0].payload)
			self.assertEqual(d["conf"], 70)
			self.assertEqual(d["ex_mm"], -10)
			self.assertEqual(d["ang_cdeg"], 300)
			self.assertEqual(d["route"], 1)

			d2 = p2.parse_line_ctrl(ctrl[5].payload)
			self.assertEqual(d2["route"], (5 % 4) + 1)
		finally:
			os.unlink(tmp.name)


class TestMultiSink(unittest.TestCase):
	def test_failures_isolated(self):
		class _Boom:
			def write(self, data):
				raise RuntimeError("boom")
			def close(self):
				return

		tmp = tempfile.NamedTemporaryFile(suffix=".bin", delete=False)
		tmp.close()
		try:
			fs = uart_sink.FileSink(tmp.name)
			ms = uart_sink.MultiSink([_Boom(), fs, _Boom()])
			ms.write(b"hello")
			ms.close()
			with open(tmp.name, "rb") as f:
				self.assertEqual(f.read(), b"hello")
		finally:
			os.unlink(tmp.name)


class TestNullSink(unittest.TestCase):
	def test_unknown_transport_yields_null(self):
		s = uart_sink.build_sink_from_config("nonsense")
		s.write(b"x")  # must not raise
		s.close()


if __name__ == "__main__":
	unittest.main(verbosity=2)
