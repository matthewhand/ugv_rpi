#!/usr/bin/env python3
"""Safe RoArm + base motion: arm is part of the robot footprint.

Rules:
  1) Camera near-field check BEFORE any arm sweep or base drive.
  2) Close obstacles → arm stays TUCKED (compact), no wide base/shoulder swings.
  3) Arm scan uses envelope limited by clearance class (tiny / medium / free).
  4) Base never drives with arm extended; re-tuck then re-check FOV.
  5) Base hop only when clear + arm tucked.

Usage:
  ./ugv-env/bin/python scripts/safe_arm_base.py --demo
  ./ugv-env/bin/python scripts/safe_arm_base.py --scan-only
  ./ugv-env/bin/python scripts/safe_arm_base.py --hop
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.parse
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

OUT = Path("/home/ws/beast-image/demo_scan_move")
OUT.mkdir(parents=True, exist_ok=True)
LOG = OUT / "safe_arm_base_log.txt"

# Postures (radians). Stock "home" is inverted-L and sticks out — bad near furniture.
# travel_tuck: lean shoulder back + fold elbow in (compact footprint for driving).
# scan_ready: slightly more open for look-ahead without full reach.
try:
    from roarm_ctrl import POSES as _ROARM_POSES

    TRAVEL_TUCK = dict(_ROARM_POSES["travel_tuck"])
    SCAN_READY = dict(_ROARM_POSES["scan_ready"])
    STOCK_HOME = dict(_ROARM_POSES["home"])
except Exception:
    # Deep crouch: strong shoulder lean-back + deep elbow fold (low CG default stance)
    TRAVEL_TUCK = {"base": 0.0, "shoulder": -0.62, "elbow": 0.88, "hand": 3.05}
    SCAN_READY = {"base": 0.0, "shoulder": -0.28, "elbow": 1.15, "hand": 3.05}
    STOCK_HOME = {"base": 0.0, "shoulder": 0.0, "elbow": 1.5708, "hand": 3.1416}

TUCK = dict(TRAVEL_TUCK)  # default stow for nav = travel_tuck (not inverted L)
HOME_UI = {"T": 144, "E": 60, "Z": 24, "R": 0}  # stock UI home only if explicitly needed

# Scan envelopes by clearance (max |base| yaw, shoulder/elbow delta from *tuck*)
# Sweeps are relative to travel_tuck so we never slam into the stock inverted-L reach.
ENVELOPES = {
    "blocked": {"base": 0.0, "shoulder": 0.0, "elbow_delta": 0.0},
    "tight": {"base": 0.10, "shoulder": 0.06, "elbow_delta": 0.06},
    "open": {"base": 0.28, "shoulder": 0.10, "elbow_delta": 0.12},
}

DRIVE_X = 0.07
DRIVE_PERIOD = 0.40


def log(msg: str) -> None:
    line = msg if msg.endswith("\n") else msg + "\n"
    sys.stdout.write(line)
    sys.stdout.flush()
    with open(LOG, "a", encoding="utf-8") as fh:
        fh.write(line)


def flask_cmd(payload: dict, timeout: float = 8.0) -> dict:
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
    log(f"control_mode={st.get('control_mode')}")


def arm_set(base: float, shoulder: float, elbow: float, hand: float = 3.05, settle: float = 1.3) -> None:
    flask_cmd(
        {
            "T": 102,
            "base": float(base),
            "shoulder": float(shoulder),
            "elbow": float(elbow),
            "hand": float(hand),
            "spd": 0,
            "acc": 10,
        }
    )
    time.sleep(settle)


def arm_tuck(settle: float = 1.6) -> None:
    """Travel tuck: lean back + elbow folded — NOT stock inverted-L (T:100).

    Stock T:100 / inverted L reaches forward and hits nearby furniture (e.g. drying rack).
    """
    log(
        "ARM TRAVEL_TUCK lean_back shoulder={shoulder} elbow_fold={elbow}".format(
            **TUCK
        )
    )
    # Direct joints only — avoid T:100 which forces inverted L then we fight it
    arm_set(
        TUCK["base"],
        TUCK["shoulder"],
        TUCK["elbow"],
        TUCK["hand"],
        settle=settle,
    )


def arm_scan_ready(settle: float = 1.2) -> None:
    """Slightly more open than travel tuck for look-ahead camera."""
    log("ARM SCAN_READY")
    arm_set(
        SCAN_READY["base"],
        SCAN_READY["shoulder"],
        SCAN_READY["elbow"],
        SCAN_READY["hand"],
        settle=settle,
    )


class Cam:
    def __init__(self) -> None:
        self.cap = None
        self.open()

    def open(self) -> None:
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        for idx in (0, 1, 2):
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                cap.release()
                continue
            time.sleep(0.12)
            ok, f = cap.read()
            if ok and f is not None:
                self.cap = cap
                log(f"camera {idx} {f.shape}")
                return
            cap.release()
        raise RuntimeError("no camera")

    def grab(self, n: int = 4):
        frame = None
        for _ in range(n):
            ok, frame = self.cap.read()
            if not ok:
                frame = None
                time.sleep(0.03)
        if frame is None:
            self.open()
            ok, frame = self.cap.read()
        if frame is None:
            raise RuntimeError("no frame")
        return frame

    def release(self) -> None:
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass


def save(frame, name: str, note: str = "") -> Path:
    img = frame.copy()
    if note:
        cv2.putText(img, note[:95], (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 180), 1)
    p = OUT / name
    cv2.imwrite(str(p), img, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    return p


def near_field_clearance(frame) -> dict:
    """Estimate how close obstacles are to the *robot body+arm* volume.

    Uses full FOV (arm can swing into sides) with emphasis on near band.
    clearance: blocked | tight | open
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Near band: lower 55% — obstacles that body/arm would hit first
    near = gray[int(h * 0.35): int(h * 0.95), :]
    near_e = cv2.Canny(near, 40, 120)
    # Center near (straight ahead of chassis)
    c0, c1 = int(w * 0.25), int(w * 0.75)
    cnear = near[:, c0:c1]
    cnear_e = near_e[:, c0:c1]
    # Side near (arm swing volume)
    left = near[:, : int(w * 0.30)]
    right = near[:, int(w * 0.70) :]
    left_e = near_e[:, : int(w * 0.30)]
    right_e = near_e[:, int(w * 0.70) :]

    def dens(e):
        return float(np.mean(e > 0))

    center_edge = dens(cnear_e)
    left_edge = dens(left_e)
    right_edge = dens(right_e)
    center_std = float(np.std(cnear))
    center_mean = float(np.mean(cnear))

    # Large dark/metallic structures (racks) produce high edge density close up
    # Flat close wall: high mean, low std
    close_wall = center_mean > 150 and center_std < 32 and center_edge < 0.035
    close_structure = center_edge > 0.06  # rack / furniture filling near FOV
    side_left_close = left_edge > 0.07
    side_right_close = right_edge > 0.07

    # Floor openness in bottom center for driving
    floor = gray[int(h * 0.60): int(h * 0.95), c0:c1]
    floor_e = cv2.Canny(floor, 35, 100)
    floor_open = float(np.clip(1.0 - dens(floor_e) * 10.0, 0, 1))

    col = np.mean(gray[int(h * 0.55): int(h * 0.95)].astype(np.float32), axis=0) - 2.5 * np.mean(
        cv2.Canny(gray[int(h * 0.55): int(h * 0.95)], 35, 100).astype(np.float32), axis=0
    )
    k = max(5, w // 18) | 1
    col_s = cv2.GaussianBlur(col.reshape(1, -1), (k, 1), 0).ravel()
    best_x = int(np.argmax(col_s))
    free_err = (best_x - w / 2) / (w / 2)

    if close_structure or close_wall:
        clearance = "blocked"
        arm_safe = False
        drive_ok = False
        reason = "obstacle in body/arm near field (rack/wall/furniture)"
    elif center_edge > 0.04 or side_left_close or side_right_close:
        clearance = "tight"
        arm_safe = True  # tiny motions only
        drive_ok = floor_open >= 0.45 and abs(free_err) < 0.35 and not close_structure
        reason = "tight space — limited arm envelope, careful base only"
    else:
        clearance = "open"
        arm_safe = True
        drive_ok = floor_open >= 0.40 and abs(free_err) < 0.40
        reason = "near field relatively open"

    # Drive range
    if not drive_ok:
        rng, drive_s = "none", 0.0
    elif clearance == "tight" or floor_open < 0.55:
        rng, drive_s = "short", 2.2
    else:
        rng, drive_s = "short", 3.0  # still short by default

    return {
        "clearance": clearance,
        "arm_safe_to_sweep": arm_safe and clearance != "blocked",
        "drive_ok": drive_ok,
        "range": rng,
        "drive_s": drive_s,
        "reason": reason,
        "center_edge": round(center_edge, 4),
        "left_edge": round(left_edge, 4),
        "right_edge": round(right_edge, 4),
        "floor_open": round(floor_open, 3),
        "free_err": round(float(np.clip(free_err, -1.5, 1.5)), 3),
        "best_x_frac": round(best_x / float(w), 3),
        "close_wall": close_wall,
        "close_structure": close_structure,
        "side_left_close": side_left_close,
        "side_right_close": side_right_close,
    }


def annotate(frame, m: dict, note: str = "") -> np.ndarray:
    img = frame.copy()
    h, w = img.shape[:2]
    # body+arm near zone
    cv2.rectangle(img, (0, int(h * 0.35)), (w - 1, h - 1), (0, 140, 255), 1)
    cv2.rectangle(img, (int(w * 0.25), int(h * 0.35)), (int(w * 0.75), h - 1), (0, 255, 255), 1)
    bx = int(m.get("best_x_frac", 0.5) * w)
    cv2.line(img, (bx, int(h * 0.55)), (bx, h), (0, 255, 0), 2)
    color = {"blocked": (0, 0, 255), "tight": (0, 165, 255), "open": (0, 255, 0)}.get(
        m.get("clearance", "tight"), (200, 200, 200)
    )
    cv2.putText(
        img,
        f"{note} {m.get('clearance')} arm_sweep={m.get('arm_safe_to_sweep')} "
        f"drive={m.get('range')}~{m.get('drive_s')}s",
        (8, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        1,
    )
    cv2.putText(
        img,
        m.get("reason", "")[:90],
        (8, 46),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (220, 220, 220),
        1,
    )
    return img


def safe_arm_scan(cam: Cam) -> dict:
    """Scan only within clearance-limited envelope. Always start/end tucked."""
    log("=== SAFE ARM SCAN (envelope limited by near-field) ===")
    arm_tuck()
    frame = cam.grab(5)
    m0 = near_field_clearance(frame)
    save(annotate(frame, m0, "TUCK_PRE"), "safe_00_tuck.jpg")
    log(f"pre-scan clearance={m0['clearance']}: {m0['reason']}")

    env = ENVELOPES[m0["clearance"]]
    views: List[dict] = []

    if m0["clearance"] == "blocked":
        log("SKIP arm sweep — obstacle in arm/body volume (would hit like the drying rack)")
        return {"clearance": m0, "views": views, "skipped_sweep": True}

    # Do not swing into a side that looks closed
    allow_left = not m0.get("side_left_close")
    allow_right = not m0.get("side_right_close")
    bmax = env["base"]
    smax = env["shoulder"]
    ed = env["elbow_delta"]
    elbow0 = TUCK["elbow"]

    # Deltas applied on top of travel_tuck (keep overall compact)
    tb, ts, te = TUCK["base"], TUCK["shoulder"], TUCK["elbow"]
    poses: List[Tuple[str, float, float, float]] = [
        ("tuck_center", tb, ts, te),
    ]
    if allow_left and bmax > 0:
        poses.append(("peek_left", tb - bmax, ts, te))
    if allow_right and bmax > 0:
        poses.append(("peek_right", tb + bmax, ts, te))
    if ed > 0 and m0["clearance"] == "open":
        # slight un-fold for look, still far shorter than inverted L
        # Slight un-fold from deep crouch for a peek — still far below inverted-L
        poses.append(("peek_ahead", tb, ts + smax, min(te + ed, 1.25)))

    for name, b, s, e in poses:
        log(f"  arm → {name} base={b:.2f} shoulder={s:.2f} elbow={e:.2f} [env={m0['clearance']}]")
        arm_set(b, s, e, TUCK["hand"], settle=1.2)
        frame = cam.grab(4)
        m = near_field_clearance(frame)
        p = save(annotate(frame, m, f"SCAN_{name}"), f"safe_scan_{name}.jpg")
        views.append({"pose": name, "metrics": m, "still": str(p)})
        # If any pose suddenly sees blocked center, abort further sweep and tuck
        if m["clearance"] == "blocked" or m.get("close_structure"):
            log(f"  abort sweep at {name} — near structure detected")
            break

    arm_tuck()
    frame = cam.grab(5)
    m_end = near_field_clearance(frame)
    save(annotate(frame, m_end, "TUCK_POST"), "safe_01_tuck_post.jpg")
    return {"clearance": m_end, "views": views, "skipped_sweep": False, "pre": m0}


def safe_base_hop(cam: Cam) -> dict:
    """Drive only with arm tucked and near-field clear."""
    log("=== SAFE BASE HOP (arm must be tucked) ===")
    arm_tuck()
    frame = cam.grab(6)
    m = near_field_clearance(frame)
    save(annotate(frame, m, "PRE_DRIVE"), "safe_02_pre_drive.jpg")
    log(f"pre-drive: {m['clearance']} drive_ok={m['drive_ok']} {m['reason']}")

    if not m["drive_ok"] or m["drive_s"] <= 0:
        # small yaw only if not blocked (don't shove into rack)
        if m["clearance"] == "blocked":
            log("NO base motion — blocked near field")
            return {"drove": False, "reason": m}
        log("yaw only toward free floor")
        for _ in range(6):
            err = m["free_err"]
            z = float(np.clip(-err * 0.45, -0.18, 0.18))
            if abs(z) < 0.08:
                z = -0.10 if err > 0 else 0.10
            flask_cmd({"T": 13, "X": 0.0, "Z": z})
            time.sleep(0.35)
            frame = cam.grab(2)
            m = near_field_clearance(frame)
            if m["drive_ok"]:
                break
        stop()
        arm_tuck(1.2)
        frame = cam.grab(5)
        m = near_field_clearance(frame)
        save(annotate(frame, m, "AFTER_YAW"), "safe_02b_after_yaw.jpg")
        if not m["drive_ok"]:
            log("still not clear — no forward")
            return {"drove": False, "reason": m}

    drive_s = min(float(m["drive_s"]), 2.8)
    log(f"forward {drive_s}s X={DRIVE_X} arm=TUCKED")
    t0 = time.time()
    last = 0.0
    n = 0
    while time.time() - t0 < drive_s:
        frame = cam.grab(2)
        m = near_field_clearance(frame)
        if m["clearance"] == "blocked" or m.get("close_structure") or m["floor_open"] < 0.25:
            log(f"ABORT drive: {m['reason']}")
            stop()
            save(annotate(frame, m, "ABORT"), "safe_03_abort.jpg")
            break
        z = float(np.clip(-m["free_err"] * 0.25, -0.10, 0.10))
        now = time.time()
        if now - last >= DRIVE_PERIOD:
            flask_cmd({"T": 13, "X": DRIVE_X, "Z": z})
            last = now
            n += 1
            log(f"  base #{n} clr={m['clearance']} floor={m['floor_open']}")
    stop()
    arm_tuck(1.2)
    frame = cam.grab(5)
    m_end = near_field_clearance(frame)
    save(annotate(frame, m_end, "END"), "safe_04_end.jpg")
    return {"drove": n > 0, "cmds": n, "budget_s": drive_s, "end": m_end}


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true", help="scan then hop")
    ap.add_argument("--scan-only", action="store_true")
    ap.add_argument("--hop", action="store_true", help="tuck + base hop only")
    args = ap.parse_args(argv)
    if not any([args.demo, args.scan_only, args.hop]):
        args.demo = True

    LOG.write_text("")
    log(f"UTC {datetime.now(timezone.utc).isoformat()}")
    log("SAFE ARM+BASE: arm is part of robot footprint")
    ensure_direct()
    stop()
    cam = Cam()
    results: Dict[str, Any] = {"utc": datetime.now(timezone.utc).isoformat(), "ok": False}
    try:
        if args.scan_only or args.demo:
            results["scan"] = safe_arm_scan(cam)
            # strip nothing large
        if args.hop or args.demo:
            results["base"] = safe_base_hop(cam)
        results["ok"] = True
        results["summary"] = {
            "scan_clearance": (results.get("scan") or {}).get("clearance", {}).get("clearance"),
            "sweep_skipped": (results.get("scan") or {}).get("skipped_sweep"),
            "base_drove": (results.get("base") or {}).get("drove"),
        }
        log(f"SUMMARY {json.dumps(results['summary'])}")
        print(json.dumps(results["summary"], indent=2))
        return 0
    except Exception as e:
        import traceback

        log(f"EXCEPTION {e}\n{traceback.format_exc()}")
        results["error"] = str(e)
        return 1
    finally:
        stop()
        try:
            arm_tuck(1.0)
        except Exception:
            pass
        stop()
        cam.release()
        (OUT / "safe_arm_base_results.json").write_text(
            json.dumps(results, indent=2, default=str), encoding="utf-8"
        )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        stop()
