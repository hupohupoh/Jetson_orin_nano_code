"""Pluggable byte sinks for the simulated UART output.

In the Webots simulation we do not actually drive a serial port; instead we
mirror the bytes that real OpenMV would write into pluggable sinks so that:

- FileSink: produces a `.bin` capture for offline parsing / regression tests.
- UdpSink: pushes packets to a localhost UDP port so that motor_listener.py
  (or any other monitor) can consume them in real time.
- MultiSink: write to several sinks at once.

The interface is intentionally minimal: `write(bytes_like)` + `close()`.
This matches `pyserial.Serial.write` so the same controller code can later be
pointed at a real serial port without changes.
"""

from __future__ import annotations

import os
import socket
from typing import Iterable, Optional


class _BaseSink:
	def write(self, data) -> None:
		raise NotImplementedError

	def close(self) -> None:
		pass


class FileSink(_BaseSink):
	"""Append raw bytes to a binary file. Auto-creates parent directories."""

	def __init__(self, path: str):
		self.path = path
		parent = os.path.dirname(path)
		if parent:
			os.makedirs(parent, exist_ok=True)
		self._fh = open(path, "ab", buffering=0)
		self.bytes_written = 0

	def write(self, data) -> None:
		if not data:
			return
		self._fh.write(bytes(data))
		self.bytes_written += len(data)

	def close(self) -> None:
		try:
			self._fh.close()
		except Exception:
			pass


class UdpSink(_BaseSink):
	"""Send each `write()` as a single UDP datagram to (host, port).

	We send one datagram per controller call (typically one frame per call),
	so that a listener can rely on datagram boundaries == frame boundaries.
	If the caller writes multiple frames in one call, they will arrive bundled
	in a single datagram, which the listener's StreamParser still handles.
	"""

	def __init__(self, host: str = "127.0.0.1", port: int = 56565):
		self.host = host
		self.port = int(port)
		self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.bytes_written = 0
		self.send_errors = 0

	def write(self, data) -> None:
		if not data:
			return
		try:
			self._sock.sendto(bytes(data), (self.host, self.port))
			self.bytes_written += len(data)
		except Exception:
			self.send_errors += 1

	def close(self) -> None:
		try:
			self._sock.close()
		except Exception:
			pass


class MultiSink(_BaseSink):
	"""Fan out to several sinks. Failures in one sink do not affect others."""

	def __init__(self, sinks: Iterable[_BaseSink]):
		self._sinks = list(sinks)

	def write(self, data) -> None:
		for s in self._sinks:
			try:
				s.write(data)
			except Exception:
				pass

	def close(self) -> None:
		for s in self._sinks:
			try:
				s.close()
			except Exception:
				pass


class NullSink(_BaseSink):
	def write(self, data) -> None:
		return


def build_sink_from_config(
	transport: str,
	file_path: Optional[str] = None,
	udp_host: str = "127.0.0.1",
	udp_port: int = 56565,
) -> _BaseSink:
	"""Factory used by the controller.

	transport in {"none", "file", "udp", "both"}. Falls back to NullSink for
	unknown values so that the controller never crashes on configuration typos.
	"""
	t = (transport or "none").strip().lower()
	if t == "file":
		if not file_path:
			return NullSink()
		return FileSink(file_path)
	if t == "udp":
		return UdpSink(udp_host, udp_port)
	if t == "both":
		sinks = []
		if file_path:
			sinks.append(FileSink(file_path))
		sinks.append(UdpSink(udp_host, udp_port))
		return MultiSink(sinks)
	return NullSink()
