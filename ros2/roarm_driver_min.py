#!/usr/bin/env python3
"""Minimal Waveshare RoArm-M2 USB driver for ROS 2 Humble.

Independent of chassis UART (ttyAMA0). Opens the RoArm's own ESP32 on CP2102
and bridges:

  /ugv/roarm/joint_command (sensor_msgs/JointState) → USB JSON T:102
  USB T:105 feedback (best-effort) → /ugv/roarm/joint_states

Joint names (fixed order):
  roarm_base, roarm_shoulder, roarm_elbow, roarm_hand

Env:
  ROARM_SERIAL / ROARM_PORT  device path (default: auto by-id CP2102 / ttyUSB*)
  ROARM_BAUD                 baud (default 115200)
  ROARM_FEEDBACK_HZ          feedback poll rate (0 = disable, default 2)
  ROARM_REQUIRE_SERIAL       if 1/true, exit when serial missing (default 0 = soft fail + retry)

Runs inside dudulrx0601/ugv_rpi_ros_humble (rclpy + pyserial) or any Humble env.
Must not open base UART. Fail soft when USB unplugged so chassis stack stays up.
"""
from __future__ import annotations

import glob
import json
import os
import sys
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
    from serial import SerialException
except ImportError as e:  # pragma: no cover
    print("pyserial required:", e, file=sys.stderr)
    raise

JOINT_NAMES = [
    "roarm_base",
    "roarm_shoulder",
    "roarm_elbow",
    "roarm_hand",
]

DEFAULT_BY_ID_GLOB = (
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_*-if00-port0"
)
_HOME = (0.0, 0.0, 1.5708, 3.1416)


def _env_flag(name: str, default: bool = False) -> bool:
    raw = (os.environ.get(name) or "").strip().lower()
    if not raw:
        return default
    return raw in ("1", "true", "yes", "on")


def resolve_port(configured: Optional[str] = None) -> Optional[str]:
    env = os.environ.get("ROARM_SERIAL") or os.environ.get("ROARM_PORT")
    for cand in (configured, env):
        if cand and os.path.exists(cand):
            return cand
    matches = sorted(glob.glob(DEFAULT_BY_ID_GLOB))
    if matches:
        return matches[0]
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    return usb[0] if usb else None


def _positions_from_msg(msg: JointState) -> Optional[Tuple[float, float, float, float]]:
    names = list(msg.name) if msg.name else []
    pos = list(msg.position) if msg.position else []
    if not pos:
        return None
    if not names:
        if len(pos) >= 4:
            return float(pos[0]), float(pos[1]), float(pos[2]), float(pos[3])
        return None
    out: List[Optional[float]] = [None, None, None, None]
    for i, n in enumerate(names):
        if i >= len(pos):
            break
        key = (n or "").strip().lower()
        # Accept full names and short aliases
        if key in ("roarm_base", "base"):
            out[0] = float(pos[i])
        elif key in ("roarm_shoulder", "shoulder"):
            out[1] = float(pos[i])
        elif key in ("roarm_elbow", "elbow"):
            out[2] = float(pos[i])
        elif key in ("roarm_hand", "hand", "gripper"):
            out[3] = float(pos[i])
    # Positional fallback if names unknown but 4 values present
    if all(v is None for v in out) and len(pos) >= 4:
        return float(pos[0]), float(pos[1]), float(pos[2]), float(pos[3])
    if any(v is None for v in out):
        # Fill missing from last home defaults
        filled = [
            out[i] if out[i] is not None else _HOME[i] for i in range(4)
        ]
        if all(v is None for v in out):
            return None
        return float(filled[0]), float(filled[1]), float(filled[2]), float(filled[3])
    return float(out[0]), float(out[1]), float(out[2]), float(out[3])  # type: ignore[arg-type]


class RoArmDriverMin(Node):
    def __init__(self) -> None:
        super().__init__("roarm_driver")
        self._ser: Optional[serial.Serial] = None
        self._port: Optional[str] = None
        self._baud = int(os.environ.get("ROARM_BAUD") or "115200")
        self._last_joints = list(_HOME)
        self._last_open_attempt = 0.0
        self._open_interval = 2.0

        cmd_topic = (
            os.environ.get("UGV_ROARM_JOINT_CMD_TOPIC") or "/ugv/roarm/joint_command"
        ).strip()
        st_topic = (
            os.environ.get("UGV_ROARM_JOINT_STATES_TOPIC") or "/ugv/roarm/joint_states"
        ).strip()

        # Use absolute topic names so Flask/rosbridge (/ugv/roarm/...) match
        st_abs = st_topic if st_topic.startswith("/") else f"/{st_topic}"
        cmd_abs = cmd_topic if cmd_topic.startswith("/") else f"/{cmd_topic}"
        self._pub = self.create_publisher(JointState, st_abs, 10)
        self.create_subscription(JointState, cmd_abs, self._cmd_cb, 10)

        fb_hz = float(os.environ.get("ROARM_FEEDBACK_HZ") or "2")
        if fb_hz > 0:
            self.create_timer(1.0 / fb_hz, self._feedback_tick)
        self.create_timer(1.0, self._reconnect_tick)

        ok = self._open_serial()
        self.get_logger().info(
            f"roarm_driver_min ready cmd={cmd_topic} states={st_topic} "
            f"port={self._port or 'none'} open={ok} baud={self._baud}"
        )

    def _open_serial(self) -> bool:
        if self._ser is not None and getattr(self._ser, "is_open", False):
            if self._port and not os.path.exists(self._port):
                self._close_serial()
            else:
                return True
        path = resolve_port()
        if not path:
            self.get_logger().warning(
                "RoArm serial not found (set ROARM_SERIAL or plug CP2102)",
                throttle_duration_sec=10.0,
            )
            return False
        try:
            ser = serial.Serial()
            ser.port = path
            ser.baudrate = self._baud
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
            # Drain boot noise
            t0 = time.time()
            last = time.time()
            while time.time() - t0 < 2.0:
                n = ser.in_waiting
                if n:
                    ser.read(n)
                    last = time.time()
                elif time.time() - last > 0.4 and time.time() - t0 > 0.5:
                    break
                time.sleep(0.04)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            self._ser = ser
            self._port = path
            self.get_logger().info(f"Opened RoArm serial {path} @ {self._baud}")
            return True
        except Exception as e:
            self._ser = None
            self._port = None
            self.get_logger().warning(
                f"RoArm serial open failed ({path}): {e}",
                throttle_duration_sec=5.0,
            )
            return False

    def _close_serial(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def _reconnect_tick(self) -> None:
        if self._ser is not None and getattr(self._ser, "is_open", False):
            return
        now = time.time()
        if now - self._last_open_attempt < self._open_interval:
            return
        self._last_open_attempt = now
        self._open_serial()

    def _write_json(self, payload: dict, read_s: float = 0.1) -> Tuple[bool, str]:
        if not self._open_serial():
            return False, "not connected"
        raw = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
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
                    break
                time.sleep(0.02)
            return True, resp.decode("utf-8", errors="replace")
        except (OSError, SerialException) as e:
            self.get_logger().error(f"serial write failed: {e}")
            self._close_serial()
            return False, str(e)

    def _cmd_cb(self, msg: JointState) -> None:
        joints = _positions_from_msg(msg)
        if joints is None:
            return
        base, shoulder, elbow, hand = joints
        ok, _ = self._write_json(
            {
                "T": 102,
                "base": base,
                "shoulder": shoulder,
                "elbow": elbow,
                "hand": hand,
                "spd": 0,
                "acc": 12,
            },
            read_s=0.08,
        )
        if ok:
            self._last_joints = [base, shoulder, elbow, hand]
            self._publish_states(self._last_joints)

    def _publish_states(self, positions: List[float]) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = "roarm_base_link"
        js.name = list(JOINT_NAMES)
        js.position = [float(p) for p in positions]
        self._pub.publish(js)

    def _feedback_tick(self) -> None:
        if self._ser is None or not getattr(self._ser, "is_open", False):
            # Still publish last commanded as soft state so tools see something
            self._publish_states(self._last_joints)
            return
        ok, text = self._write_json({"T": 105}, read_s=0.25)
        if not ok:
            self._publish_states(self._last_joints)
            return
        # Best-effort parse; firmware response shapes vary — fall back to last cmd
        parsed = self._parse_feedback(text)
        if parsed is not None:
            self._last_joints = list(parsed)
        self._publish_states(self._last_joints)

    @staticmethod
    def _parse_feedback(text: str) -> Optional[List[float]]:
        if not text or not text.strip():
            return None
        for line in text.splitlines():
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                obj = json.loads(line)
            except Exception:
                continue
            # Common RoArm shapes
            for keys in (
                ("base", "shoulder", "elbow", "hand"),
                ("b", "s", "e", "h"),
            ):
                if all(k in obj for k in keys):
                    try:
                        return [float(obj[k]) for k in keys]
                    except (TypeError, ValueError):
                        pass
            if "name" in obj and "position" in obj:
                try:
                    dummy = JointState()
                    dummy.name = list(obj["name"])
                    dummy.position = [float(x) for x in obj["position"]]
                    got = _positions_from_msg(dummy)
                    if got:
                        return list(got)
                except Exception:
                    pass
        return None

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()


def main() -> None:
    require = _env_flag("ROARM_REQUIRE_SERIAL", False)
    rclpy.init()
    try:
        if require and not resolve_port():
            print("[roarm_driver_min] ROARM_REQUIRE_SERIAL set and no device; exiting")
            rclpy.shutdown()
            raise SystemExit(1)
        node = RoArmDriverMin()
    except Exception as e:
        print(f"[roarm_driver_min] init failed: {e}")
        try:
            rclpy.shutdown()
        except Exception:
            pass
        raise SystemExit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # ExternalShutdownException etc. — exit cleanly for container restart policy
        print(f"[roarm_driver_min] spin ended: {e}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
