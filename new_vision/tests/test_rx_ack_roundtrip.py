"""End-to-end test: vision sends event -> motor parses -> motor sends ACK ->
vision parses ACK and clears the PendingAcks queue.

Run: python tests/test_rx_ack_roundtrip.py
"""

from __future__ import annotations

import os
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "openmv"))

import protocol_v2 as p2  # type: ignore  # noqa: E402


class FakeUart:
	"""Minimal pair of UARTs with byte-level FIFO; mimics what main1 + main board would have."""

	def __init__(self):
		self._tx = bytearray()
		self.peer = None

	def write(self, data):
		assert self.peer is not None
		self.peer._tx.extend(bytes(data))

	def any(self):
		return len(self._tx)

	def read(self, n):
		if n <= 0 or not self._tx:
			return b""
		out = bytes(self._tx[:n])
		del self._tx[:n]
		return out


def make_pair():
	a, b = FakeUart(), FakeUart()
	a.peer = b
	b.peer = a
	return a, b


class TestVisionMotorRoundtrip(unittest.TestCase):
	def test_qr_event_acked(self):
		vision_uart, motor_uart = make_pair()

		# Vision side
		v_proto = p2.VisionProtocolV2()
		v_pending = p2.PendingAcks(vision_uart.write, timeout_ms=80, retry_max=3)
		v_parser = p2.StreamParser()

		# Motor side
		m_proto = p2.VisionProtocolV2()
		m_parser = p2.StreamParser()

		# Vision emits a QR event (action_id=3)
		frame = v_proto.build_qr_event(action_id=3)
		v_pending.send(frame, seq=frame[5], msg_type=p2.MSG_QR_EVENT)
		self.assertEqual(v_pending.pending_count(), 1)

		# Motor pulls bytes
		got = m_parser.feed(motor_uart.read(motor_uart.any()))
		self.assertEqual(len(got), 1)
		self.assertEqual(got[0].msg_type, p2.MSG_QR_EVENT)
		self.assertTrue(got[0].needs_ack())
		self.assertEqual(got[0].payload[0], 3)

		# Motor emits ACK
		ack = m_proto.build_ack(orig_seq=got[0].seq, orig_msg_type=p2.MSG_QR_EVENT, code=0)
		motor_uart.write(ack)

		# Vision receives ACK, clears pending
		got_v = v_parser.feed(vision_uart.read(vision_uart.any()))
		self.assertEqual(len(got_v), 1)
		self.assertEqual(got_v[0].msg_type, p2.MSG_ACK)
		self.assertTrue(got_v[0].is_ack())
		decoded = p2.parse_ack(got_v[0].payload)
		removed = v_pending.on_ack(decoded["ack_seq"], decoded["ack_msg_type"])
		self.assertTrue(removed)
		self.assertEqual(v_pending.pending_count(), 0)

	def test_event_retries_until_dropped(self):
		vision_uart, _motor = make_pair()
		v_proto = p2.VisionProtocolV2()
		v_pending = p2.PendingAcks(vision_uart.write, timeout_ms=0, retry_max=3)
		frame = v_proto.build_obstacle_event(present_u8=1, distance_cm_u8=12)
		v_pending.send(frame, seq=frame[5], msg_type=p2.MSG_OBSTACLE_EVENT)
		# Motor never ACKs -> three retries then drop
		v_pending.tick()  # retry 1
		v_pending.tick()  # retry 2
		v_pending.tick()  # retry 3
		v_pending.tick()  # exceeds, dropped
		self.assertEqual(v_pending.pending_count(), 0)
		self.assertEqual(v_pending.dropped, 1)


if __name__ == "__main__":
	unittest.main(verbosity=2)
