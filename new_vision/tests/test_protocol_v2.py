"""Protocol V2 unit tests.

Run from repo root:
    python -m pytest tests/test_protocol_v2.py -v
or simply:
    python tests/test_protocol_v2.py
"""

from __future__ import annotations

import os
import struct
import sys
import unittest

# Allow running from repo root without installing.
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "openmv"))

import protocol_v2 as p2  # type: ignore  # noqa: E402


# Reference example from docs/motor_protocol_v2_p1.md §10:
#   mode=1, conf=70, lost=0, route=2, ex=-10mm, ang=300cdeg, v=0, w=0
#   payload (hex): 01 46 00 02 F6 FF 2C 01 00 00 00 00
PDF_EXAMPLE_PAYLOAD = bytes.fromhex("01 46 00 02 F6 FF 2C 01 00 00 00 00".replace(" ", ""))


class TestCRC(unittest.TestCase):
	def test_crc_known_vector(self):
		# CCITT-FALSE("123456789") = 0x29B1
		self.assertEqual(p2._crc16_ccitt_false(b"123456789"), 0x29B1)

	def test_crc_empty(self):
		self.assertEqual(p2._crc16_ccitt_false(b""), 0xFFFF)


class TestLineCtrlPayload(unittest.TestCase):
	def test_pdf_example_payload_bytes(self):
		# Build payload directly via struct to confirm bit layout.
		payload = struct.pack(
			"<BBBBhhhh",
			1, 70, 0, 2, -10, 300, 0, 0,
		)
		self.assertEqual(payload, PDF_EXAMPLE_PAYLOAD)

	def test_build_line_ctrl_payload_matches_pdf(self):
		proto = p2.VisionProtocolV2()
		frame = proto.build_line_ctrl(
			mode_u8=1, conf_u8=70, lost_u8=0, route_u8=2,
			ex_mm_i16=-10, ang_cdeg_i16=300,
		)
		# Frame layout: SOF1 SOF2 VER MSG FLAGS SEQ TS_MS(4) LEN PAYLOAD CRC(2)
		# Payload starts at index 11, length = 12.
		self.assertEqual(len(frame), 2 + 9 + 12 + 2)
		self.assertEqual(frame[0], p2.SOF1)
		self.assertEqual(frame[1], p2.SOF2)
		self.assertEqual(frame[2], 0x02)               # VER
		self.assertEqual(frame[3], p2.MSG_LINE_CTRL)   # MSG
		self.assertEqual(frame[10], 12)                # LEN
		self.assertEqual(bytes(frame[11:23]), PDF_EXAMPLE_PAYLOAD)


class TestRoundtrip(unittest.TestCase):
	def test_line_ctrl_roundtrip(self):
		proto = p2.VisionProtocolV2()
		frame = proto.build_line_ctrl(
			mode_u8=1, conf_u8=70, lost_u8=0, route_u8=2,
			ex_mm_i16=-10, ang_cdeg_i16=300,
		)
		parser = p2.StreamParser()
		frames = parser.feed(frame)
		self.assertEqual(len(frames), 1)
		f = frames[0]
		self.assertEqual(f.msg_type, p2.MSG_LINE_CTRL)
		decoded = p2.parse_line_ctrl(f.payload)
		self.assertEqual(decoded["mode"], 1)
		self.assertEqual(decoded["conf"], 70)
		self.assertEqual(decoded["lost"], 0)
		self.assertEqual(decoded["route"], 2)
		self.assertEqual(decoded["ex_mm"], -10)
		self.assertEqual(decoded["ang_cdeg"], 300)
		self.assertEqual(parser.crc_errors, 0)

	def test_heartbeat_roundtrip(self):
		proto = p2.VisionProtocolV2()
		frame = proto.build_heartbeat(mode_u8=1)
		parser = p2.StreamParser()
		frames = parser.feed(frame)
		self.assertEqual(len(frames), 1)
		self.assertEqual(frames[0].msg_type, p2.MSG_HEARTBEAT)
		self.assertEqual(p2.parse_heartbeat(frames[0].payload)["mode"], 1)

	def test_ack_roundtrip(self):
		proto = p2.VisionProtocolV2()
		frame = proto.build_ack(orig_seq=42, orig_msg_type=p2.MSG_QR_EVENT, code=0)
		parser = p2.StreamParser()
		frames = parser.feed(frame)
		self.assertEqual(len(frames), 1)
		f = frames[0]
		self.assertTrue(f.is_ack())
		self.assertEqual(f.msg_type, p2.MSG_ACK)
		decoded = p2.parse_ack(f.payload)
		self.assertEqual(decoded["ack_seq"], 42)
		self.assertEqual(decoded["ack_msg_type"], p2.MSG_QR_EVENT)
		self.assertEqual(decoded["code"], 0)

	def test_event_roundtrip_qr(self):
		proto = p2.VisionProtocolV2()
		frame = proto.build_qr_event(action_id=3)
		parser = p2.StreamParser()
		frames = parser.feed(frame)
		self.assertEqual(len(frames), 1)
		f = frames[0]
		self.assertEqual(f.msg_type, p2.MSG_QR_EVENT)
		self.assertTrue(f.needs_ack())
		self.assertEqual(f.payload[0], 3)


class TestStreamParser(unittest.TestCase):
	def test_chunked_feed(self):
		proto = p2.VisionProtocolV2()
		f1 = proto.build_line_ctrl(1, 70, 0, 2, -10, 300)
		f2 = proto.build_heartbeat(1)
		stream = f1 + f2
		parser = p2.StreamParser()
		got = []
		# Feed byte-by-byte.
		for i in range(len(stream)):
			got.extend(parser.feed(stream[i:i + 1]))
		self.assertEqual(len(got), 2)
		self.assertEqual(got[0].msg_type, p2.MSG_LINE_CTRL)
		self.assertEqual(got[1].msg_type, p2.MSG_HEARTBEAT)

	def test_garbage_prefix(self):
		proto = p2.VisionProtocolV2()
		frame = proto.build_heartbeat(1)
		parser = p2.StreamParser()
		junk = b"\x00\x01\x02\xFF\x55\x12random"
		frames = parser.feed(junk + frame)
		self.assertEqual(len(frames), 1)
		self.assertEqual(frames[0].msg_type, p2.MSG_HEARTBEAT)
		self.assertGreater(parser.discarded_bytes, 0)

	def test_crc_corruption_detected(self):
		proto = p2.VisionProtocolV2()
		frame = bytearray(proto.build_heartbeat(1))
		# Flip one bit in the payload.
		frame[11] ^= 0x01
		parser = p2.StreamParser()
		frames = parser.feed(bytes(frame))
		self.assertEqual(frames, [])
		self.assertEqual(parser.crc_errors, 1)

	def test_partial_then_complete(self):
		proto = p2.VisionProtocolV2()
		frame = proto.build_line_ctrl(1, 70, 0, 2, -10, 300)
		parser = p2.StreamParser()
		frames = parser.feed(frame[:8])
		self.assertEqual(frames, [])
		frames = parser.feed(frame[8:])
		self.assertEqual(len(frames), 1)


class TestPendingAcks(unittest.TestCase):
	def test_send_then_ack_clears(self):
		sent = []
		pending = p2.PendingAcks(sent.append, timeout_ms=80, retry_max=3)
		pending.send(b"frame", seq=10, msg_type=p2.MSG_QR_EVENT)
		self.assertEqual(pending.pending_count(), 1)
		self.assertTrue(pending.on_ack(10, p2.MSG_QR_EVENT))
		self.assertEqual(pending.pending_count(), 0)

	def test_retry_then_drop(self):
		sent = []
		pending = p2.PendingAcks(sent.append, timeout_ms=0, retry_max=2)
		pending.send(b"frame", seq=5, msg_type=p2.MSG_QR_EVENT)
		# 1st tick -> retry 1
		pending.tick()
		# 2nd tick -> retry 2
		pending.tick()
		# 3rd tick -> exceeded retry_max -> drop
		pending.tick()
		self.assertEqual(pending.pending_count(), 0)
		self.assertEqual(pending.dropped, 1)
		# Initial send + 2 retries = 3 total
		self.assertEqual(len(sent), 3)


class TestSafetyMonitor(unittest.TestCase):
	def test_assess_receive_states(self):
		mon = p2.SafetyMonitor(safe_stop_ms=300, lost_recovery_ms=800)
		now = p2._ticks_ms()
		self.assertEqual(mon.assess_receive(None), p2.SafetyMonitor.SAFE_STOP)
		self.assertEqual(mon.assess_receive(now), p2.SafetyMonitor.SAFE_OK)
		self.assertEqual(mon.assess_receive(now - 350), p2.SafetyMonitor.SAFE_STOP)
		self.assertEqual(mon.assess_receive(now - 900), p2.SafetyMonitor.LOST_RECOVERY)


if __name__ == "__main__":
	unittest.main(verbosity=2)
