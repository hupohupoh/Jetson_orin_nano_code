#!/usr/bin/env python3
"""Keep the two protocol_v2.py copies in sync.

The protocol module deliberately lives in two places:
- openmv/protocol_v2.py        (master, runs on OpenMV)
- Webots/controllers/line_follow_transfer/protocol_v2.py  (mirror, runs on PC)

OpenMV does not always cope with symlinks on a flash filesystem, so we keep
two byte-identical copies and use this tool to verify or refresh them.

Usage:
  python tools/sync_protocol.py --check    # exit 0 if equal, 1 otherwise
  python tools/sync_protocol.py --apply    # copy master -> mirror
"""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MASTER = os.path.join(REPO_ROOT, "openmv", "protocol_v2.py")
MIRROR = os.path.join(REPO_ROOT, "Webots", "controllers", "line_follow_transfer", "protocol_v2.py")


def sha256(path: str) -> str:
	with open(path, "rb") as f:
		return hashlib.sha256(f.read()).hexdigest()


def main() -> int:
	ap = argparse.ArgumentParser()
	g = ap.add_mutually_exclusive_group(required=True)
	g.add_argument("--check", action="store_true", help="Verify master and mirror are identical")
	g.add_argument("--apply", action="store_true", help="Overwrite mirror with master")
	args = ap.parse_args()

	if not os.path.isfile(MASTER):
		print(f"[ERROR] master missing: {MASTER}")
		return 2

	if args.apply:
		os.makedirs(os.path.dirname(MIRROR), exist_ok=True)
		shutil.copyfile(MASTER, MIRROR)
		print(f"[OK] copied {os.path.relpath(MASTER, REPO_ROOT)} -> {os.path.relpath(MIRROR, REPO_ROOT)}")
		return 0

	# --check
	if not os.path.isfile(MIRROR):
		print(f"[FAIL] mirror missing: {MIRROR}")
		return 1
	h_master = sha256(MASTER)
	h_mirror = sha256(MIRROR)
	if h_master == h_mirror:
		print(f"[OK] in sync (sha256={h_master[:12]}...)")
		return 0
	print("[FAIL] master and mirror differ")
	print(f"  master sha256: {h_master}")
	print(f"  mirror sha256: {h_mirror}")
	print("Run: python tools/sync_protocol.py --apply")
	return 1


if __name__ == "__main__":
	sys.exit(main())
