"""USB-serial driver for Waveshare RoArm-M2 (standalone ESP32 on CP2102).

This is separate from base_ctrl.BaseController (Beast chassis on ttyAMA0).
Confirmed dialect on this build: T:100/102/105/114/121/210 @ 115200, RTS/DTR off.
"""
from __future__ import annotations

import glob
import json
import math
import os
import threading
import time
from typing import Any, Dict, Optional, Tuple

import serial
from serial import SerialException

# Default by-id path for this Beast's CP2102 (overridable via config / env).
DEFAULT_BY_ID_GLOB = (
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_*-if00-port0"
)
DEFAULT_BAUD = 115200

# Safe joint limits (radians) for UI-driven motion — within proven test ranges.
_BASE_LIM = (-1.2, 1.2)
_SHOULDER_LIM = (-0.9, 0.9)
# Elbow: smaller = more folded (forearm toward upper arm) → lower CG when stowed
_ELBOW_LIM = (0.85, 2.2)
_HAND_LIM = (1.8, 3.2)

# Stock Waveshare "init" — inverted L, arm reaches forward (long footprint, high CG).
_HOME = {"base": 0.0, "shoulder": 0.0, "elbow": 1.5708, "hand": 3.1416}

# Named postures for mobile use. Arm is part of the robot footprint:
#   home         — stock inverted L (workspace ready, sticks out, high CG)
#   travel_tuck  — DEFAULT STANCE: lean shoulder back + deep elbow fold
#                  (mass over chassis, lower CG, better tip resistance)
#   scan_ready   — slightly open for look-ahead without full reach
#   elbow_in     — fold forearm only (mild compact)
POSES = {
    "home": dict(_HOME),
    # Desired default while rolling: deep crouch, not inverted L.
    # Push shoulder + elbow hard so camera/EE mass sits low over the chassis.
    "travel_tuck": {
        "base": 0.0,
        "shoulder": -0.62,  # strong lean-back (shoulder extension toward body)
        "elbow": 0.88,  # deep elbow fold (near limit) — lowers CG
        "hand": 3.05,
    },
    "scan_ready": {
        "base": 0.0,
        "shoulder": -0.28,
        "elbow": 1.15,  # half-open from deep tuck for a peek
        "hand": 3.05,
    },
    "elbow_in": {
        "base": 0.0,
        "shoulder": -0.20,
        "elbow": 0.95,
        "hand": 3.05,
    },
    # Alias used by nav scripts
    "tuck": None,  # filled below → travel_tuck
}
POSES["tuck"] = dict(POSES["travel_tuck"])


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def resolve_port(configured: Optional[str] = None) -> Optional[str]:
    """Return a usable device path or None."""
    env = os.environ.get("ROARM_SERIAL") or os.environ.get("ROARM_PORT")
    for cand in (configured, env):
        if cand and os.path.exists(cand):
            return cand
    matches = sorted(glob.glob(DEFAULT_BY_ID_GLOB))
    if matches:
        return matches[0]
    # Last resort: first ttyUSB* that is not clearly something else
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    return usb[0] if usb else None


def e_z_r_to_joints(
    e: float,
    z: float,
    r: float,
    *,
    default_e: float = 60.0,
    default_z: float = 24.0,
    default_r: float = 0.0,
    hand: Optional[float] = None,
) -> Dict[str, float]:
    """Map stock UI T:144 E/Z/R workspace knobs to RoArm joint radians.

    Stock web stick (module_type=1):
      armR = -inputX/7   (≈ degrees of base yaw)
      armZ = f(stick Y)  (height-ish, default 24)
      armE = scroll      (reach/extension, default 60, range ~60..450)

    USB RoArm has no T:144 IK — approximate with linear maps around home.
    
    Args:
        hand: Optional gripper angle in radians. If None, preserves home position.
              Callers should pass current hand from _last_joints to avoid closing grip.
    """
    # Base yaw: R is already degrees-ish from the UI
    base = _clamp(math.radians(float(r) - float(default_r)), *_BASE_LIM)

    # Height → shoulder (positive Z above default → shoulder up a bit)
    z_delta = float(z) - float(default_z)
    shoulder = _clamp(-z_delta * 0.012, *_SHOULDER_LIM)

    # Extension → elbow (default E → ~π/2 home; more E → more extended / smaller angle)
    e_span = max(1.0, 450.0 - float(default_e))
    e_norm = _clamp((float(e) - float(default_e)) / e_span, 0.0, 1.0)
    elbow = _clamp(1.5708 - e_norm * 0.55, *_ELBOW_LIM)

    # Preserve current hand position if provided, else default to home (closed)
    hand_rad = float(hand) if hand is not None else _HOME["hand"]
    return {
        "base": base,
        "shoulder": shoulder,
        "elbow": elbow,
        "hand": hand_rad,
    }


# Official Waveshare RoArm-M2-S link lengths (EEMode=0 clamp).
# Source: RoArm-M2_example/RoArm-M2_config.h in waveshareteam/roarm_m2
# https://github.com/waveshareteam/roarm_m2
ARM_L1_MM = 126.06  # base column (visual; firmware lastZ starts at the shoulder)
ARM_L2A_MM = 236.82  # upper-arm vertical at home
ARM_L2B_MM = 30.00  # shoulder→elbow forward offset at home
ARM_L3A_MM = 280.15  # forearm
ARM_L3B_MM = 1.73
ARM_L2_MM = math.hypot(ARM_L2A_MM, ARM_L2B_MM)
ARM_L3_MM = math.hypot(ARM_L3A_MM, ARM_L3B_MM)
ARM_T2_RAD = math.atan2(ARM_L2B_MM, ARM_L2A_MM)

# Firmware T:104 example / init pose in millimetres (z from shoulder, L1 omitted).
ARM_INIT_X_MM = ARM_L3A_MM + ARM_L2B_MM  # 310.15
ARM_INIT_Z_MM = ARM_L2A_MM - ARM_L3B_MM  # 235.09 (IK seed; EEMode=0 FK z = L2A)

HAND_CLOSED_RAD = 3.1416
HAND_OPEN_RAD = 1.08


def _wrap_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def planar_ik(
    r_mm: float,
    z_mm: float,
    *,
    seed_shoulder: float = 0.0,
    seed_elbow: float = 1.5708,
    enforce_limits: bool = True,
) -> Optional[Dict[str, Any]]:
    """Closed-form 2-link planar IK — exact inverse of `forward_kinematics`.

    Planar frame is from the shoulder pivot (X forward, Z up), millimetres;
    base yaw is independent (rotate afterwards). Matches firmware FK:
        pos = L2·e(π/2−(shoulder+t2)) + L3·e(π/2−(elbow+shoulder))

    Returns {"shoulder": rad, "elbow": rad, "clamped": bool} or None when the
    target is outside the physical annulus |L2−L3|..L2+L3. Branch (elbow-up /
    elbow-down) is chosen for continuity with the seed pose so small stick /
    slider steps never flip the arm through the workspace.
    """
    l2 = ARM_L2_MM
    l3 = ARM_L3_MM
    r = float(r_mm)
    z = float(z_mm)
    d = math.hypot(r, z)
    d_max = l2 + l3 - 0.5  # 0.5 mm margin avoids acos domain noise at full reach
    d_min = abs(l2 - l3) + 0.5
    if d > d_max or d < d_min:
        return None

    beta = math.atan2(z, r)
    cos_gamma = _clamp((l2 * l2 + d * d - l3 * l3) / (2.0 * l2 * d), -1.0, 1.0)
    gamma = math.acos(cos_gamma)

    def _solve(u: float) -> Dict[str, Any]:
        # Exact forearm direction: passes from the elbow through the target.
        w = math.atan2(z - l2 * math.sin(u), r - l2 * math.cos(u))
        delta = _wrap_pi(w - u)
        elbow_raw = ARM_T2_RAD - delta
        shoulder_raw = (math.pi / 2.0) - ARM_T2_RAD - u
        sh = _clamp(shoulder_raw, *_SHOULDER_LIM)
        el = _clamp(elbow_raw, *_ELBOW_LIM)
        if abs(sh - shoulder_raw) > 1e-9 or abs(el - elbow_raw) > 1e-9:
            # Per-joint clamping is NOT a constrained projection: keeping the
            # other joint at its raw value can land far from the target (e.g.
            # a lift jog from travel_tuck lunged ~60 mm forward in reach).
            # Coordinate descent, each step closed-form optimal:
            #   elbow fixed → linkage is rigid about the shoulder pivot, best
            #   shoulder aims the EE at the target bearing;
            #   shoulder fixed → best elbow points the forearm at the target.
            # Both 1-D subproblems are unimodal over the limit intervals
            # (< π wide), so interval clamping stays optimal per step.
            beta_t = math.atan2(z, r)
            for _ in range(3):
                v_r = l2 * math.sin(ARM_T2_RAD) + l3 * math.sin(el)
                v_z = l2 * math.cos(ARM_T2_RAD) + l3 * math.cos(el)
                sh = _clamp(_wrap_pi(math.atan2(v_z, v_r) - beta_t), *_SHOULDER_LIM)
                u_i = (math.pi / 2.0) - (sh + ARM_T2_RAD)
                w_i = math.atan2(z - l2 * math.sin(u_i), r - l2 * math.cos(u_i))
                el = _clamp(ARM_T2_RAD - _wrap_pi(w_i - u_i), *_ELBOW_LIM)
        # Honest post-clamp error so a limit-bound branch can lose to a
        # branch that actually reaches the target.
        th2 = (math.pi / 2.0) - (sh + ARM_T2_RAD)
        th3 = (math.pi / 2.0) - (el + sh)
        px = l2 * math.cos(th2) + l3 * math.cos(th3)
        pz = l2 * math.sin(th2) + l3 * math.sin(th3)
        err = math.hypot(px - r, pz - z)
        cont = abs(sh - seed_shoulder) + abs(el - seed_elbow)
        cl = abs(sh - shoulder_raw) > 1e-9 or abs(el - elbow_raw) > 1e-9
        return {
            "shoulder": sh,
            "elbow": el,
            "clamped": cl,
            "_err": err,
            "_cont": cont,
        }

    a = _solve(beta + gamma)
    b = _solve(beta - gamma)
    # FK error dominates; continuity breaks ties between exact solutions so
    # small stick / slider steps never flip the arm through the workspace.
    best = min((a, b), key=lambda s: (round(s["_err"], 6), s["_cont"]))
    shoulder, elbow = best["shoulder"], best["elbow"]
    clamped = best["clamped"]

    return {"shoulder": shoulder, "elbow": elbow, "clamped": clamped}


def relative_move(
    dr_mm: float = 0.0,
    dz_mm: float = 0.0,
    dyaw_deg: float = 0.0,
    joints: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    """Coordinated EE jog: reach ±Δ keeping height, lift ±Δ keeping reach, yaw ±Δ.

    Seeds IK from the current pose (branch continuity), clamps targets that fall
    outside the physical workspace / safe joint limits, and reports what was
    actually achieved (`achieved_r_mm` / `achieved_z_mm`) plus a `clamped` flag
    so UIs can stay honest about partial moves. Hand is always preserved.
    """
    cur = dict(POSES["travel_tuck"] if joints is None else joints)
    # Missing elbow/hand must default to home, not 0.0 rad: hand is passed
    # straight to the servo (0.0 is past fully-open) and elbow 0.0 is outside
    # the safe box.
    defaults = {"base": 0.0, "shoulder": 0.0, "elbow": _HOME["elbow"], "hand": _HOME["hand"]}
    for key in ("base", "shoulder", "elbow", "hand"):
        val = cur.get(key)
        cur[key] = defaults[key] if val is None else float(val)

    fk_cur = forward_kinematics(cur["base"], cur["shoulder"], cur["elbow"])
    # Signed radial reach from the shoulder axis (tuck leans BACKWARD: x<0);
    # must keep the sign or a zero-delta jog would mirror the pose.
    r0 = fk_cur["r"] * 1000.0
    z0 = fk_cur["z"] * 1000.0

    if abs(float(dr_mm)) < 1e-9 and abs(float(dz_mm)) < 1e-9:
        # Yaw is base-only by contract: never re-solve shoulder/elbow, or a
        # pure yaw jog from a raw (console-commanded) pose outside the safe
        # box would snap the planar joints to the box mid-turn.
        new_base = _clamp(cur["base"] + math.radians(float(dyaw_deg)), *_BASE_LIM)
        yaw_clamped = abs(new_base - (cur["base"] + math.radians(float(dyaw_deg)))) > 1e-9
        return {
            "ok": True,
            "joints": {
                "base": new_base,
                "shoulder": cur["shoulder"],
                "elbow": cur["elbow"],
                "hand": cur["hand"],
            },
            "clamped": yaw_clamped,
            "target_r_mm": r0,
            "target_z_mm": z0,
            "achieved_r_mm": r0,
            "achieved_z_mm": z0,
        }

    r_t = r0 + float(dr_mm)
    z_t = z0 + float(dz_mm)

    sol = planar_ik(r_t, z_t, seed_shoulder=cur["shoulder"], seed_elbow=cur["elbow"])
    clamped = False
    if sol is None:
        # Outside the annulus: walk from the current pose toward the target and
        # stop at the last still-reachable point (keeps the other axis as close
        # as possible instead of snapping radially).
        l2 = ARM_L2_MM
        l3 = ARM_L3_MM
        d_max = l2 + l3 - 6.0
        d_min = abs(l2 - l3) + 6.0

        def _inside(px: float, pz: float) -> bool:
            dd = math.hypot(px, pz)
            return d_min <= dd <= d_max

        dir_x, dir_z = r_t - r0, z_t - z0
        norm = math.hypot(dir_x, dir_z)
        if norm < 1e-6:
            dir_x, dir_z = r_t, z_t
            norm = math.hypot(r_t, z_t)
            best = _clamp(norm, d_min, d_max)
            r_t, z_t = r_t * best / max(norm, 1e-6), z_t * best / max(norm, 1e-6)
        elif _inside(r0, z0):
            t_lo, t_hi = 0.0, 1.0
            for _ in range(40):
                mid = (t_lo + t_hi) / 2.0
                if _inside(r0 + dir_x * mid, z0 + dir_z * mid):
                    t_lo = mid
                else:
                    t_hi = mid
            r_t, z_t = r0 + dir_x * t_lo, z0 + dir_z * t_lo
        else:
            # Radial projection must divide by the TARGET magnitude, not the
            # delta magnitude (`norm`), or the scaled point flies outside the
            # workspace and the jog fails instead of clamping.
            t_norm = math.hypot(r_t, z_t)
            scale = _clamp(t_norm, d_min, d_max) / max(t_norm, 1e-6)
            r_t, z_t = r_t * scale, z_t * scale
        clamped = True
        sol = planar_ik(r_t, z_t, seed_shoulder=cur["shoulder"], seed_elbow=cur["elbow"])
        if sol is None:
            return {
                "ok": False,
                "error": "target outside workspace even after clamp",
                "joints": dict(cur),
                "clamped": True,
            }
    elif sol.get("clamped"):
        clamped = True

    new_base = _clamp(cur["base"] + math.radians(float(dyaw_deg)), *_BASE_LIM)
    if abs(new_base - (cur["base"] + math.radians(float(dyaw_deg)))) > 1e-9:
        clamped = True

    joints_out = {
        "base": new_base,
        "shoulder": sol["shoulder"],
        "elbow": sol["elbow"],
        "hand": cur["hand"],
    }
    fk_new = forward_kinematics(new_base, sol["shoulder"], sol["elbow"])
    return {
        "ok": True,
        "joints": joints_out,
        "clamped": clamped or bool(sol.get("clamped")),
        "target_r_mm": r_t,
        "target_z_mm": z_t,
        "achieved_r_mm": fk_new["r"] * 1000.0,
        "achieved_z_mm": fk_new["z"] * 1000.0,
    }


def kinematics_public() -> Dict[str, float]:
    """Link lengths for the shared 3D twin (millimetres)."""
    return {
        "l1_mm": ARM_L1_MM,
        "l2a_mm": ARM_L2A_MM,
        "l2b_mm": ARM_L2B_MM,
        "l3a_mm": ARM_L3A_MM,
        "l3b_mm": ARM_L3B_MM,
        "l2_mm": ARM_L2_MM,
        "l3_mm": ARM_L3_MM,
        "init_x_mm": ARM_INIT_X_MM,
        "init_z_mm": ARM_INIT_Z_MM,
    }


def forward_kinematics(
    base: float,
    shoulder: float,
    elbow: float,
    hand: Optional[float] = None,
) -> Dict[str, float]:
    """RoArm-M2 EEMode=0 FK matching firmware `RoArmM2_computePosbyJointRad`.

    polarToCartesian(l2, π/2 − (shoulder + t2rad))
    polarToCartesian(l3, π/2 − (elbow + shoulder))
    then yaw `base` in the XY plane (X+ forward, Y+ left, Z+ up).

    Returns metres. `z` is firmware lastZ (from the shoulder). `z_world` includes L1.
    """
    l2 = ARM_L2_MM / 1000.0
    l3 = ARM_L3_MM / 1000.0
    l1 = ARM_L1_MM / 1000.0
    th2 = (math.pi / 2.0) - (float(shoulder) + ARM_T2_RAD)
    a_out = l2 * math.cos(th2)
    b_out = l2 * math.sin(th2)
    th3 = (math.pi / 2.0) - (float(elbow) + float(shoulder))
    c_out = l3 * math.cos(th3)
    d_out = l3 * math.sin(th3)
    r_ee = a_out + c_out
    z_ee = b_out + d_out
    base_r = float(base)
    return {
        "x": r_ee * math.cos(base_r),
        "y": r_ee * math.sin(base_r),
        "z": z_ee,
        "z_world": l1 + z_ee,
        "r": r_ee,
        "elbow_r": a_out,
        "elbow_z": b_out,
        "hand": float(hand) if hand is not None else _HOME["hand"],
    }


class RoArmController:
    """Thread-safe USB JSON serial to the RoArm ESP32."""

    def __init__(
        self,
        port: Optional[str] = None,
        baud: int = DEFAULT_BAUD,
        *,
        auto_open: bool = True,
    ) -> None:
        self.port_cfg = port
        self.baud = int(baud or DEFAULT_BAUD)
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.RLock()
        self._last_error: Optional[str] = None
        self._last_joints = dict(_HOME)
        self._connected_port: Optional[str] = None
        if auto_open:
            self.open()

    # ---- lifecycle -------------------------------------------------
    def open(self) -> bool:
        with self._lock:
            if self._ser is not None and getattr(self._ser, "is_open", False):
                return True
            path = resolve_port(self.port_cfg)
            if not path:
                self._last_error = "no RoArm USB serial device"
                self._ser = None
                self._connected_port = None
                return False
            try:
                ser = serial.Serial()
                ser.port = path
                ser.baudrate = self.baud
                ser.timeout = 0.2
                ser.write_timeout = 0.5
                ser.dsrdtr = False
                ser.rtscts = False
                ser.open()
                try:
                    ser.dtr = False
                    ser.rts = False
                except Exception:
                    pass
                time.sleep(0.12)
                # Drain boot noise if the open caused a reset
                t0 = time.time()
                last = time.time()
                while time.time() - t0 < 2.5:
                    n = ser.in_waiting
                    if n:
                        ser.read(n)
                        last = time.time()
                    elif time.time() - last > 0.45 and time.time() - t0 > 0.6:
                        break
                    time.sleep(0.04)
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                self._ser = ser
                self._connected_port = path
                self._last_error = None
                return True
            except Exception as e:
                self._last_error = str(e)
                self._ser = None
                self._connected_port = None
                return False

    def close(self) -> None:
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
            self._ser = None
            self._connected_port = None

    def ensure_open(self) -> bool:
        with self._lock:
            if self._ser is not None and getattr(self._ser, "is_open", False):
                # Device may have vanished after tip-over
                if self._connected_port and not os.path.exists(self._connected_port):
                    self.close()
                else:
                    return True
            return self.open()

    @property
    def connected(self) -> bool:
        with self._lock:
            return bool(self._ser and getattr(self._ser, "is_open", False))

    def status(self) -> Dict[str, Any]:
        return {
            "connected": self.connected,
            "port": self._connected_port,
            "baud": self.baud,
            "last_error": self._last_error,
            "last_joints": dict(self._last_joints),
        }

    # ---- low-level IO ----------------------------------------------
    def send_json(self, payload: dict, read_s: float = 0.15) -> Tuple[bool, str]:
        if not self.ensure_open():
            return False, self._last_error or "not connected"
        raw = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
        with self._lock:
            ser = self._ser
            if ser is None:
                return False, "not connected"
            try:
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                ser.write(raw)
                ser.flush()
                time.sleep(read_s)
                resp = b""
                t0 = time.time()
                while time.time() - t0 < max(0.05, read_s):
                    n = ser.in_waiting
                    if n:
                        resp += ser.read(n)
                        time.sleep(0.03)
                        if ser.in_waiting:
                            resp += ser.read(ser.in_waiting)
                        break
                    time.sleep(0.02)
                return True, resp.decode("utf-8", errors="replace")
            except (OSError, SerialException, termios_err()) as e:
                self._last_error = str(e)
                try:
                    self.close()
                except Exception:
                    pass
                return False, str(e)

    # ---- high-level ------------------------------------------------
    def set_joints(
        self,
        base: float,
        shoulder: float,
        elbow: float,
        hand: float,
        *,
        spd: float = 0,
        acc: float = 12,
    ) -> Tuple[bool, str]:
        payload = {
            "T": 102,
            "base": float(base),
            "shoulder": float(shoulder),
            "elbow": float(elbow),
            "hand": float(hand),
            "spd": spd,
            "acc": acc,
        }
        ok, text = self.send_json(payload, read_s=0.12)
        if ok:
            self._last_joints = {
                "base": float(base),
                "shoulder": float(shoulder),
                "elbow": float(elbow),
                "hand": float(hand),
            }
        return ok, text

    def set_from_e_z_r(
        self,
        e: float,
        z: float,
        r: float,
        *,
        default_e: float = 60.0,
        default_z: float = 24.0,
        default_r: float = 0.0,
        spd: float = 0,
        acc: float = 12,
    ) -> Tuple[bool, str, Dict[str, float]]:
        # Preserve current hand position to avoid closing grip on stick/height moves
        current_hand = self._last_joints.get("hand", _HOME["hand"])
        j = e_z_r_to_joints(
            e, z, r, default_e=default_e, default_z=default_z, default_r=default_r,
            hand=current_hand
        )
        ok, text = self.set_joints(
            j["base"], j["shoulder"], j["elbow"], j["hand"], spd=spd, acc=acc
        )
        return ok, text, j

    def set_joint_deg(
        self, joint: int, angle_deg: float, spd: int = 35, acc: int = 12
    ) -> Tuple[bool, str]:
        return self.send_json(
            {"T": 121, "joint": int(joint), "angle": float(angle_deg), "spd": spd, "acc": acc},
            read_s=0.2,
        )

    def home(self) -> Tuple[bool, str]:
        """Stock firmware init pose (inverted L). Prefer pose('travel_tuck') for driving."""
        ok, text = self.send_json({"T": 100}, read_s=0.4)
        if ok:
            self._last_joints = dict(_HOME)
        return ok, text

    def pose(self, name: str = "travel_tuck", *, spd: float = 0, acc: float = 10) -> Tuple[bool, str]:
        """Move to a named posture (home / travel_tuck / scan_ready / elbow_in / tuck)."""
        key = (name or "travel_tuck").strip().lower()
        if key not in POSES or POSES[key] is None:
            return False, f"unknown pose {name!r}; choose from {sorted(k for k,v in POSES.items() if v)}"
        j = POSES[key]
        return self.set_joints(
            j["base"], j["shoulder"], j["elbow"], j["hand"], spd=spd, acc=acc
        )

    def travel_tuck(self) -> Tuple[bool, str]:
        """Compact lean-back + elbow-fold — default stow while the base drives."""
        return self.pose("travel_tuck")

    def torque(self, on: bool = True) -> Tuple[bool, str]:
        return self.send_json({"T": 210, "cmd": 1 if on else 0}, read_s=0.15)

    def feedback(self) -> Tuple[bool, str]:
        return self.send_json({"T": 105}, read_s=0.35)

    def led(self, level: int = 0) -> Tuple[bool, str]:
        return self.send_json({"T": 114, "led": int(level)}, read_s=0.1)


def termios_err():
    """OSError covers termios.error I/O failures on dead USB."""
    return OSError


# Process-wide singleton. Flask and tools must use these helpers — do not
# keep a second handle in app.py (stop would leave this one open/stale).
_roarm: Optional[RoArmController] = None
_roarm_lock = threading.Lock()


def current_roarm() -> Optional[RoArmController]:
    """Return the live controller if one was started, without opening USB."""
    return _roarm


def get_roarm(
    port: Optional[str] = None,
    baud: int = DEFAULT_BAUD,
    *,
    enabled: bool = True,
) -> Optional[RoArmController]:
    """Return shared RoArmController, or None if disabled."""
    global _roarm
    if not enabled:
        return None
    with _roarm_lock:
        if _roarm is None:
            _roarm = RoArmController(port=port, baud=baud, auto_open=True)
        return _roarm


def shutdown_roarm() -> None:
    global _roarm
    with _roarm_lock:
        arm = _roarm
        _roarm = None
    if arm is None:
        return
    try:
        arm.close()
    except Exception:
        pass
