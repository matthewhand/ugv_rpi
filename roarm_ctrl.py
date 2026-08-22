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


# Process-wide singleton (lazy). App sets this via get_roarm().
_roarm: Optional[RoArmController] = None
_roarm_lock = threading.Lock()


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
        if _roarm is not None:
            try:
                _roarm.close()
            except Exception:
                pass
            _roarm = None
