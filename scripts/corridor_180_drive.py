#!/usr/bin/env python3
"""Vision-guided hops: scan → estimate short/long straight → drive that far → stop → re-scan.

Mandatory before every forward segment:
  A) Center RoArm (clear camera FOV)
  B) CV scan: corridor metrics + wall risk + range class (none/short/medium/long)
  C) Drive only the estimated duration (or skip if none)
  D) Stop, re-center arm, re-scan for next hop feedback
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
import urllib.parse
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

OUT = Path("/home/ws/beast-image/corridor_180")
OUT.mkdir(parents=True, exist_ok=True)
LOG = OUT / "run_log_hops.txt"

TURN_Z = 0.35
TURN_Z_FINE = 0.16
DRIVE_X = 0.08
DRIVE_PERIOD = 0.40
MIN_SPIN_S = 9.0
MAX_SPIN_S = 16.0
MAX_ALIGN_S = 10.0
ALIGN_ERR = 0.12

# Range → drive time at DRIVE_X≈0.08 (approx; no odometry)
RANGE_DRIVE_S = {
    "none": 0.0,
    "short": 3.0,
    "medium": 7.0,
    "long": 12.0,
}
RANGE_DRIVE_M_APPROX = {  # rough operator feedback only
    "none": 0.0,
    "short": 0.35,
    "medium": 0.85,
    "long": 1.5,
}

MIN_SCORE_DRIVE = 0.48
MAX_CENTER_ERR_DRIVE = 0.20
WALL_RISK_STOP = 0.45
MIN_BOTH_SIDES = 0.22
MAX_WALL_FILL = 0.58


def log(msg: str) -> None:
    line = msg if msg.endswith("\n") else msg + "\n"
    sys.stdout.write(line)
    sys.stdout.flush()
    with open(LOG, "a", encoding="utf-8") as fh:
        fh.write(line)


def flask_cmd(payload: dict, timeout: float = 6.0) -> dict:
    cmd = "base -c " + json.dumps(payload, separators=(",", ":"))
    data = urllib.parse.urlencode({"command": cmd}).encode("utf-8")
    req = urllib.request.Request(
        "http://127.0.0.1:5000/send_command", data=data, method="POST"
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def stop() -> None:
    for p in ({"T": 13, "X": 0, "Z": 0}, {"T": 1, "L": 0, "R": 0}):
        try:
            flask_cmd(p)
        except Exception:
            pass


def ensure_direct() -> None:
    data = json.dumps({"mode": "direct"}).encode("utf-8")
    req = urllib.request.Request(
        "http://127.0.0.1:5000/api/control_mode",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=5) as resp:
        st = json.loads(resp.read().decode("utf-8"))
    log(f"control_mode={st.get('control_mode')} serial={st.get('serial_open')}")


def center_roarm() -> dict:
    log("=== CENTER ROARM ===")
    out: Dict[str, Any] = {}
    try:
        r1 = flask_cmd({"T": 144, "E": 60, "Z": 24, "R": 0})
        out["t144"] = r1.get("status") or r1
        time.sleep(1.0)
        r2 = flask_cmd({"T": 100})
        out["t100"] = r2.get("status") or r2
        time.sleep(2.0)
        req = urllib.request.Request("http://127.0.0.1:5000/api/status")
        with urllib.request.urlopen(req, timeout=5) as resp:
            st = json.loads(resp.read().decode("utf-8"))
        out["joints"] = (st.get("roarm") or {}).get("last_joints")
        log(f"RoArm joints={out['joints']}")
    except Exception as e:
        out["error"] = str(e)
        log(f"RoArm center WARN: {e}")
    return out
