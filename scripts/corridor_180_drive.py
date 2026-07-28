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


class Cam:
    def __init__(self) -> None:
        self.cap = None
        self.idx = None
        self.open()

    def open(self) -> None:
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = None
        for idx in (0, 1, 2, 3):
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                cap.release()
                continue
            time.sleep(0.15)
            ok, f = cap.read()
            if ok and f is not None:
                self.cap = cap
                self.idx = idx
                log(f"camera index={idx} {f.shape}")
                return
            cap.release()
        raise RuntimeError("no camera")

    def grab(self, n: int = 3):
        for attempt in range(3):
            frame = None
            for _ in range(n):
                if self.cap is None:
                    break
                ok, frame = self.cap.read()
                if not ok:
                    frame = None
                    time.sleep(0.03)
            if frame is not None:
                return frame
            log(f"camera re-open attempt {attempt+1}")
            time.sleep(0.4)
            try:
                self.open()
            except Exception as e:
                log(f"reopen fail: {e}")
        raise RuntimeError("camera frame fail")

    def release(self) -> None:
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass


def save(frame, name: str, note: str = "") -> Path:
    img = frame.copy()
    if note:
        cv2.putText(img, note[:95], (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 180), 1)
    p = OUT / name
    cv2.imwrite(str(p), img, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    return p


def wall_risk(frame) -> dict:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    roi = gray[int(h * 0.25): int(h * 0.85), int(w * 0.15): int(w * 0.85)]
    mean, std = float(np.mean(roi)), float(np.std(roi))
    edges = cv2.Canny(roi, 50, 120)
    edge_frac = float(np.mean(edges > 0))
    lap = float(cv2.Laplacian(roi, cv2.CV_64F).var())
    risk = 0.0
    reasons: List[str] = []
    if mean > 140 and std < 35 and edge_frac < 0.04:
        risk += 0.50
        reasons.append("close_wall_texture")
    if mean > 165 and std < 28:
        risk += 0.35
        reasons.append("bright_flat")
    if edge_frac < 0.015 and mean > 120:
        risk += 0.30
        reasons.append("low_edges")
    if lap < 50 and mean > 130:
        risk += 0.25
        reasons.append("low_laplacian")
    cx0, cx1 = int(w * 0.35), int(w * 0.65)
    center = gray[int(h * 0.2): int(h * 0.7), cx0:cx1]
    cstd = float(np.std(center))
    cmean = float(np.mean(center))
    if cstd < 28 and cmean > 130:
        risk += 0.35
        reasons.append("center_wall_fill")
    return {
        "risk": min(1.0, risk),
        "mean": round(mean, 1),
        "std": round(std, 1),
        "edge_frac": round(edge_frac, 4),
        "laplacian": round(lap, 1),
        "center_std": round(cstd, 1),
        "center_mean": round(cmean, 1),
        "reasons": reasons,
    }


def corridor_metrics(frame) -> dict:
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    y0, y1 = int(h * 0.25), int(h * 0.75)
    band = gray[y0:y1, :]
    edges = cv2.Canny(band, 50, 130)
    mid = w // 2
    left_e = float(np.mean(edges[:, :mid]))
    right_e = float(np.mean(edges[:, mid:]))
    bal = (right_e - left_e) / (right_e + left_e + 1e-6)

    yf0, yf1 = int(h * 0.45), int(h * 0.92)
    floor = gray[yf0:yf1, :]
    floor_e = cv2.Canny(floor, 40, 120)
    col_score = np.mean(floor.astype(np.float32), axis=0) - 2.5 * np.mean(
        floor_e.astype(np.float32), axis=0
    )
    k = max(5, w // 20) | 1
    col_s = cv2.GaussianBlur(col_score.reshape(1, -1), (k, 1), 0).ravel()
    best_x = int(np.argmax(col_s))
    free_err = (best_x - mid) / (w / 2.0)

    center_band = col_s[int(w * 0.35): int(w * 0.65)]
    side_max = max(
        float(np.max(col_s[: int(w * 0.25)])),
        float(np.max(col_s[int(w * 0.75):])),
    )
    center_max = float(np.max(center_band))
    aisle_centered = float(np.clip((center_max - side_max) / 40.0 + 0.5, 0, 1))

    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, 40, minLineLength=int(h * 0.12), maxLineGap=20
    )
    vx_list = []
    if lines is not None:
        for x1, y1_, x2, y2_ in lines[:, 0]:
            if abs(x2 - x1) < 3:
                continue
            slope = (y2_ - y1_) / float(x2 - x1)
            if abs(slope) < 0.15:
                continue
            x_at_top = x1 - y1_ / slope
            if -w < x_at_top < 2 * w:
                vx_list.append(x_at_top)
    if vx_list:
        vx = float(np.median(vx_list))
        vp_err = (vx - mid) / (w / 2.0)
    else:
        vx = float(mid)
        vp_err = 0.0

    center_err = float(
        np.clip(
            0.50 * free_err
            + 0.30 * float(np.clip(vp_err, -1.5, 1.5))
            + 0.20 * bal,
            -1.5,
            1.5,
        )
    )
    both_sides = min(left_e, right_e) / (max(left_e, right_e) + 1e-6)
    open_norm = float(np.clip((col_s[mid] - np.percentile(col_s, 30)) / 40.0, 0, 1))
    center_good = float(np.clip(1.0 - abs(center_err) / 0.55, 0, 1))
    score = (
        0.35 * center_good
        + 0.25 * both_sides
        + 0.20 * open_norm
        + 0.20 * aisle_centered
    )
    mid_roi = edges[:, int(w * 0.3): int(w * 0.7)]
    wall_fill = 1.0 - float(np.clip(np.mean(mid_roi) / 40.0, 0, 1))

    # Far-field openness (upper center): how free the "distance" looks
    far = gray[int(h * 0.12): int(h * 0.42), int(w * 0.30): int(w * 0.70)]
    far_e = cv2.Canny(far, 40, 110)
    far_edge = float(np.mean(far_e > 0))
    far_std = float(np.std(far))
    # Open hall ahead often has moderate edges (door frames) not a flat wall; flat wall = low edge + mid brightness
    far_mean = float(np.mean(far))
    if far_mean > 145 and far_edge < 0.025 and far_std < 30:
        far_open = 0.15  # wall close-ish
    elif far_edge < 0.015 and far_mean > 130:
        far_open = 0.25
    else:
        # more structure / depth cues → longer clear path
        far_open = float(np.clip(0.35 + far_edge * 8.0 + min(far_std, 50) / 80.0, 0, 1))

    # Depth proxy: free-path score decay from near (bottom) to far (top) in center columns
    cx0, cx1 = int(w * 0.38), int(w * 0.62)
    strip = gray[:, cx0:cx1]
    strip_e = cv2.Canny(strip, 40, 110)
    rows = strip.shape[0]
    bands = []
    for i in range(4):
        a, b = int(rows * (0.2 + i * 0.18)), int(rows * (0.35 + i * 0.18))
        a = max(0, min(rows - 1, a))
        b = max(a + 1, min(rows, b))
        e = float(np.mean(strip_e[a:b] > 0))
        mu = float(np.mean(strip[a:b]))
        # open if not edge-dense wall
        bands.append(1.0 - float(np.clip(e * 12.0, 0, 0.85)) if mu < 200 else 0.3)
    # bands[0] nearer bottom-mid, bands[-1] higher/farther
    depth_clear = float(np.mean(bands[1:]))  # mid-to-far

    return {
        "center_err": round(center_err, 4),
        "free_err": round(free_err, 4),
        "vp_err": round(float(np.clip(vp_err, -2, 2)), 4),
        "score": round(score, 4),
        "both_sides": round(both_sides, 4),
        "aisle_centered": round(aisle_centered, 4),
        "wall_fill": round(wall_fill, 4),
        "far_open": round(far_open, 4),
        "depth_clear": round(depth_clear, 4),
        "far_edge": round(far_edge, 4),
        "far_mean": round(far_mean, 1),
        "best_x": best_x,
        "vx": round(vx, 1),
        "best_x_frac": round(best_x / float(w), 3),
    }


def clear_to_drive(m: dict, wr: dict) -> Tuple[bool, List[str]]:
    reasons: List[str] = []
    if wr["risk"] >= WALL_RISK_STOP:
        reasons.append(f"wall_risk={wr['risk']}:{wr['reasons']}")
    if m["score"] < MIN_SCORE_DRIVE:
        reasons.append(f"score={m['score']}<{MIN_SCORE_DRIVE}")
    if abs(m["center_err"]) > MAX_CENTER_ERR_DRIVE:
        reasons.append(f"center_err={m['center_err']}")
    if m["both_sides"] < MIN_BOTH_SIDES:
        reasons.append(f"both_sides={m['both_sides']}")
    if m["wall_fill"] > MAX_WALL_FILL and m["aisle_centered"] < 0.45:
        reasons.append(f"wall_fill={m['wall_fill']}")
    if not (0.30 <= m["best_x_frac"] <= 0.70):
        reasons.append(f"best_x_off_center={m['best_x_frac']}")
    if m.get("aisle_centered", 0) < 0.38:
        reasons.append(f"aisle_not_centered={m.get('aisle_centered')}")
    return (len(reasons) == 0, reasons)


def estimate_straight_range(m: dict, wr: dict) -> dict:
    """Classify how far we can safely go straight before the next CV scan.

    Returns range_class none|short|medium|long plus drive_s and human message.
    """
    ok, reasons = clear_to_drive(m, wr)
    if not ok:
        return {
            "range_class": "none",
            "drive_s": 0.0,
            "approx_m": 0.0,
            "confidence": 0.9,
            "can_go_straight": False,
            "message": "Do not go straight — CV blocked: " + "; ".join(reasons),
            "reasons": reasons,
            "scores": {
                "corridor": m.get("score"),
                "far_open": m.get("far_open"),
                "depth_clear": m.get("depth_clear"),
                "wall_risk": wr.get("risk"),
            },
        }

    # Composite depth score in [0,1]
    depth = (
        0.35 * float(m.get("far_open", 0))
        + 0.30 * float(m.get("depth_clear", 0))
        + 0.20 * float(m.get("score", 0))
        + 0.15 * (1.0 - float(m.get("wall_fill", 1)))
    )
    depth = float(np.clip(depth, 0, 1))
    # Near obstacles / high wall risk shorten
    if wr["risk"] >= 0.30:
        depth *= 0.55
    if m["wall_fill"] > 0.50:
        depth *= 0.70
    if abs(m["center_err"]) > 0.12:
        depth *= 0.85  # slightly off-center → shorter hop

    if depth < 0.38:
        cls = "short"
    elif depth < 0.62:
        cls = "medium"
    else:
        cls = "long"

    conf = float(np.clip(0.45 + 0.55 * depth, 0.4, 0.95))
    drive_s = RANGE_DRIVE_S[cls]
    approx_m = RANGE_DRIVE_M_APPROX[cls]
    msg = (
        f"Straight ahead looks {cls.upper()} "
        f"(~{drive_s:.0f}s / ~{approx_m:.1f}m at X={DRIVE_X}). "
        f"depth={depth:.2f} far_open={m.get('far_open')} wall_fill={m.get('wall_fill')}"
    )
    return {
        "range_class": cls,
        "drive_s": drive_s,
        "approx_m": approx_m,
        "confidence": round(conf, 3),
        "depth_score": round(depth, 4),
        "can_go_straight": True,
        "message": msg,
        "reasons": [],
        "scores": {
            "corridor": m.get("score"),
            "far_open": m.get("far_open"),
            "depth_clear": m.get("depth_clear"),
            "wall_fill": m.get("wall_fill"),
            "wall_risk": wr.get("risk"),
            "center_err": m.get("center_err"),
        },
    }


def annotate(frame, m: dict, extra: str = "", wr: Optional[dict] = None, rng: Optional[dict] = None):
    img = frame.copy()
    h, w = img.shape[:2]
    mid = w // 2
    cv2.line(img, (mid, 0), (mid, h), (80, 80, 80), 1)
    cv2.rectangle(
        img,
        (int(w * 0.35), int(h * 0.2)),
        (int(w * 0.65), int(h * 0.85)),
        (60, 60, 120),
        1,
    )
    bx = int(m.get("best_x", mid))
    cv2.line(img, (bx, int(h * 0.45)), (bx, h), (0, 255, 0), 2)
    vx = int(np.clip(m.get("vx", mid), 0, w - 1))
    cv2.circle(img, (vx, int(h * 0.3)), 6, (0, 165, 255), 2)
    t1 = f"err={m.get('center_err')} sc={m.get('score')} far={m.get('far_open')} {extra}"
    cv2.putText(img, t1[:95], (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 200), 1)
    y = 44
    if wr:
        cv2.putText(
            img,
            f"wall_risk={wr.get('risk')} {wr.get('reasons')}"[:95],
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 180, 255),
            1,
        )
        y += 20
    if rng:
        col = (0, 255, 0) if rng.get("can_go_straight") else (0, 0, 255)
        cv2.putText(
            img,
            f"RANGE={rng.get('range_class')} ~{rng.get('drive_s')}s conf={rng.get('confidence')}"[:95],
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            col,
            1,
        )
    return img


def hist_corr(a, b) -> float:
    ha = cv2.calcHist([a], [0, 1], None, [32, 32], [0, 256, 0, 256])
    hb = cv2.calcHist([b], [0, 1], None, [32, 32], [0, 256, 0, 256])
    cv2.normalize(ha, ha)
    cv2.normalize(hb, hb)
    return float(cv2.compareHist(ha, hb, cv2.HISTCMP_CORREL))


def gray_mad(a, b) -> float:
    ga = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY).astype(np.float32)
    gb = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY).astype(np.float32)
    if ga.shape != gb.shape:
        return -1.0
    return float(np.mean(np.abs(ga - gb)))


def vision_scan(cam: Cam, tag: str) -> dict:
    """Center is caller's job; grab frames and return full scan + range estimate."""
    frame = cam.grab(6)
    m = corridor_metrics(frame)
    wr = wall_risk(frame)
    ok, fail = clear_to_drive(m, wr)
    rng = estimate_straight_range(m, wr)
    path = save(annotate(frame, m, tag, wr, rng), f"{tag}.jpg")
    log(f"SCAN [{tag}] clear={ok} range={rng['range_class']} — {rng['message']}")
    if fail:
        log(f"  block reasons: {fail}")
    return {
        "tag": tag,
        "still": str(path),
        "corridor": m,
        "wall": wr,
        "clear_to_drive": ok,
        "fail_reasons": fail,
        "range": rng,
        "frame": frame,  # not serialized
    }


def drive_segment(cam: Cam, drive_s: float, hop_id: int) -> dict:
    """Drive forward for drive_s seconds with light centering; stop on wall risk."""
    if drive_s <= 0:
        return {"n_cmd": 0, "wall_stop": False, "duration_s": 0.0}
    log(f"=== DRIVE hop={hop_id} budget={drive_s:.1f}s X={DRIVE_X} ===")
    t0 = time.time()
    last = 0.0
    n = 0
    wall_stop = False
    while time.time() - t0 < drive_s:
        frame = cam.grab(2)
        m = corridor_metrics(frame)
        wr = wall_risk(frame)
        if wr["risk"] >= WALL_RISK_STOP or (
            m["wall_fill"] > 0.68 and abs(m["center_err"]) > 0.30
        ):
            stop()
            wall_stop = True
            save(annotate(frame, m, f"HOP{hop_id}_WALL", wr), f"hop{hop_id}_wall.jpg")
            log(f"WALL STOP mid-hop {wr}")
            break
        err = m["center_err"]
        z = float(np.clip(-err * 0.28, -0.12, 0.12))
        now = time.time()
        if now - last >= DRIVE_PERIOD:
            flask_cmd({"T": 13, "X": DRIVE_X, "Z": z})
            last = now
            n += 1
            if n % 3 == 1:
                log(f"  hop{hop_id} #{n} err={err:.3f} sc={m['score']} risk={wr['risk']}")
    stop()
    dur = time.time() - t0
    log(f"DRIVE hop={hop_id} done cmds={n} wall_stop={wall_stop} t={dur:.1f}s")
    return {"n_cmd": n, "wall_stop": wall_stop, "duration_s": round(dur, 2)}


def scan_only_mode() -> int:
    ensure_direct()
    stop()
    center_roarm()
    cam = Cam()
    try:
        scan = vision_scan(cam, "scan_only")
        # drop frame for json
        out = {k: v for k, v in scan.items() if k != "frame"}
        (OUT / "last_scan.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
        log(json.dumps(out["range"], indent=2))
        print(json.dumps({"range": out["range"], "clear": out["clear_to_drive"], "still": out["still"]}, indent=2))
        return 0 if out["clear_to_drive"] else 2
    finally:
        stop()
        cam.release()


def hop_loop(max_hops: int = 3, do_turn: bool = False) -> int:
    LOG.write_text("", encoding="utf-8")
    log(f"UTC {datetime.now(timezone.utc).isoformat()}")
    log(f"TASK hop_loop max_hops={max_hops} do_turn={do_turn}")
    ensure_direct()
    stop()
    cam = Cam()
    results: Dict[str, Any] = {
        "utc": datetime.now(timezone.utc).isoformat(),
        "hops": [],
        "ok": False,
    }
    try:
        # Optional initial 180 not default — user may already face desired way
        if do_turn:
            center_roarm()
            # reuse simplified turn from prior script logic
            start = cam.grab(5)
            t0 = time.time()
            last = 0.0
            stable = 0
            while time.time() - t0 < MAX_SPIN_S:
                now = time.time()
                if now - last >= 0.35:
                    flask_cmd({"T": 13, "X": 0.0, "Z": TURN_Z})
                    last = now
                frame = cam.grab(2)
                m = corridor_metrics(frame)
                wr = wall_risk(frame)
                sim = hist_corr(start, frame)
                mad = gray_mad(start, frame)
                elapsed = now - t0
                if elapsed >= MIN_SPIN_S:
                    loose = (
                        m["score"] >= 0.42
                        and abs(m["center_err"]) < 0.22
                        and wr["risk"] < WALL_RISK_STOP
                        and 0.28 <= m["best_x_frac"] <= 0.72
                    )
                    if loose and sim < 0.55 and mad > 25:
                        stable += 1
                    else:
                        stable = max(0, stable - 1)
                    if stable >= 3:
                        stop()
                        log(f"TURN done t={elapsed:.1f}")
                        break
            stop()
            # yaw fine align
            t1 = time.time()
            last = 0.0
            while time.time() - t1 < MAX_ALIGN_S:
                frame = cam.grab(2)
                m = corridor_metrics(frame)
                err = m["center_err"]
                if abs(err) < ALIGN_ERR and m["score"] >= 0.42:
                    stop()
                    break
                z = float(np.clip(-err * 0.5, -TURN_Z_FINE, TURN_Z_FINE))
                if abs(z) < 0.07:
                    z = 0.07 if z >= 0 else -0.07
                now = time.time()
                if now - last >= 0.30:
                    flask_cmd({"T": 13, "X": 0.0, "Z": z})
                    last = now
            stop()

        for hop in range(1, max_hops + 1):
            log(f"\n######## HOP {hop}/{max_hops} ########")
            # A) arm center  B) CV scan + range feedback
            center_roarm()
            time.sleep(0.3)
            scan = vision_scan(cam, f"hop{hop}_pre")
            rng = scan["range"]
            hop_rec = {
                "hop": hop,
                "pre_scan": {k: v for k, v in scan.items() if k != "frame"},
            }
            print("\n" + "=" * 60)
            print(f"HOP {hop} CV FEEDBACK: {rng['message']}")
            print(f"  class={rng['range_class']}  drive_s={rng['drive_s']}  conf={rng['confidence']}")
            print("=" * 60 + "\n")

            if not rng["can_go_straight"] or rng["drive_s"] <= 0:
                hop_rec["drove"] = False
                hop_rec["note"] = "blocked — no forward motion"
                results["hops"].append(hop_rec)
                # post-scan still useful for operator
                center_roarm()
                post = vision_scan(cam, f"hop{hop}_blocked")
                hop_rec["post_scan"] = {k: v for k, v in post.items() if k != "frame"}
                results["ok"] = hop > 1  # partial ok if earlier hops worked
                break

            # C) drive only estimated distance
            drive = drive_segment(cam, float(rng["drive_s"]), hop)
            hop_rec["drive"] = drive
            hop_rec["drove"] = drive["n_cmd"] > 0

            # D) stop already done; re-center + re-scan
            center_roarm()
            time.sleep(0.3)
            post = vision_scan(cam, f"hop{hop}_post")
            hop_rec["post_scan"] = {k: v for k, v in post.items() if k != "frame"}
            results["hops"].append(hop_rec)

            # If post scan says none or wall stop, halt hop loop
            if drive.get("wall_stop") or not post["range"]["can_go_straight"]:
                log(f"Stopping hop loop after hop {hop} (wall or post-scan blocked)")
                break
            # If post range is still open, continue to next hop
            log(f"Post-hop range={post['range']['range_class']}: {post['range']['message']}")

        results["ok"] = any(h.get("drove") for h in results["hops"])
        results["summary"] = {
            "hops_done": len(results["hops"]),
            "ranges": [
                (h.get("pre_scan") or {}).get("range", {}).get("range_class")
                for h in results["hops"]
            ],
            "last_post_range": (
                (results["hops"][-1].get("post_scan") or {}).get("range")
                if results["hops"]
                else None
            ),
        }
        log(f"SUMMARY {json.dumps(results['summary'], indent=2)}")
        return 0 if results["ok"] else 2
    except Exception as e:
        log(f"EXCEPTION {e}\n{traceback.format_exc()}")
        results["error"] = str(e)
        return 1
    finally:
        stop()
        try:
            cam.release()
        except Exception:
            pass
        # strip non-serializable
        (OUT / "results_hops.json").write_text(
            json.dumps(results, indent=2, default=str), encoding="utf-8"
        )
        log(f"wrote {OUT / 'results_hops.json'}")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="CV range-gated corridor hops")
    ap.add_argument("--scan-only", action="store_true", help="Center arm + one CV scan, no drive")
    ap.add_argument("--hops", type=int, default=2, help="Max scan→drive→rescan cycles")
    ap.add_argument("--turn180", action="store_true", help="Do vision 180 before first hop")
    args = ap.parse_args(argv)
    if args.scan_only:
        return scan_only_mode()
    return hop_loop(max_hops=max(1, args.hops), do_turn=bool(args.turn180))


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        stop()
