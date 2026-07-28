#!/usr/bin/env python3
"""Host-side RoArm bridge via rosbridge (no rclpy required).

Use when the ROS stack runs in Docker (rosbridge :9090) but the arm USB
should be owned on the host (Flask released USB, or exclusive driver mode).

  rosbridge /ugv/roarm/joint_command → host USB T:102
  optional T:105 / last cmd → /ugv/roarm/joint_states

Requires: websocket-client, pyserial (ugv-env has both). Does NOT need rclpy.

Usage:
  cd ~/ugv_rpi
  ./ugv-env/bin/python ros2/roarm_bridge_host.py

Env:
  ROSBRIDGE_URL=ws://127.0.0.1:9090
  ROARM_SERIAL, ROARM_BAUD
  UGV_ROARM_JOINT_CMD_TOPIC, UGV_ROARM_JOINT_STATES_TOPIC
  ROARM_FEEDBACK_HZ (default 2; 0 disables)
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    import websocket  # websocket-client
except ImportError:
    print("websocket-client required", file=sys.stderr)
    raise SystemExit(1)

import roarm_ctrl  # noqa: E402

JOINT_NAMES = ["roarm_base", "roarm_shoulder", "roarm_elbow", "roarm_hand"]


def _topic_cmd() -> str:
    return (os.environ.get("UGV_ROARM_JOINT_CMD_TOPIC") or "/ugv/roarm/joint_command").strip()


def _topic_states() -> str:
    return (os.environ.get("UGV_ROARM_JOINT_STATES_TOPIC") or "/ugv/roarm/joint_states").strip()


def _rosbridge_url() -> str:
    return (os.environ.get("ROSBRIDGE_URL") or "ws://127.0.0.1:9090").strip()


def _positions_from_msg(msg: dict) -> Optional[List[float]]:
    names = list(msg.get("name") or [])
    pos = list(msg.get("position") or [])
    if not pos:
        return None
    if not names and len(pos) >= 4:
        return [float(pos[0]), float(pos[1]), float(pos[2]), float(pos[3])]
    out: List[Optional[float]] = [None, None, None, None]
    aliases = {
        "roarm_base": 0,
        "base": 0,
        "roarm_shoulder": 1,
        "shoulder": 1,
        "roarm_elbow": 2,
        "elbow": 2,
        "roarm_hand": 3,
        "hand": 3,
        "gripper": 3,
    }
    for i, n in enumerate(names):
        if i >= len(pos):
            break
        idx = aliases.get((n or "").strip().lower())
        if idx is not None:
            out[idx] = float(pos[i])
    if all(v is None for v in out) and len(pos) >= 4:
        return [float(x) for x in pos[:4]]
    home = [0.0, 0.0, 1.5708, 3.1416]
    if any(v is not None for v in out):
        return [out[i] if out[i] is not None else home[i] for i in range(4)]  # type: ignore[misc]
    return None


class RoArmHostBridge:
    def __init__(self) -> None:
        self.url = _rosbridge_url()
        self.cmd_topic = _topic_cmd()
        self.st_topic = _topic_states()
        self._ws = None
        self._id = 0
        self._lock = threading.Lock()
        self._last_joints = [0.0, 0.0, 1.5708, 3.1416]
        port = (os.environ.get("ROARM_SERIAL") or os.environ.get("ROARM_PORT") or "").strip() or None
        baud = int(os.environ.get("ROARM_BAUD") or roarm_ctrl.DEFAULT_BAUD)
        self.arm = roarm_ctrl.RoArmController(port=port, baud=baud, auto_open=True)
        self._running = True

    def _next_id(self) -> str:
        self._id += 1
        return str(self._id)

    def _send(self, payload: dict) -> None:
        if self._ws is None:
            raise RuntimeError("not connected")
        self._ws.send(json.dumps(payload))

    def connect(self) -> None:
        self._ws = websocket.create_connection(self.url, timeout=5.0)
        # Advertise states publisher
        self._send(
            {
                "op": "advertise",
                "id": f"advertise:{self.st_topic}:{self._next_id()}",
                "topic": self.st_topic,
                "type": "sensor_msgs/msg/JointState",
            }
        )
        self._send(
            {
                "op": "subscribe",
                "id": f"subscribe:{self.cmd_topic}:{self._next_id()}",
                "topic": self.cmd_topic,
                "type": "sensor_msgs/msg/JointState",
            }
        )
        print(
            f"[roarm_bridge_host] connected {self.url} "
            f"cmd={self.cmd_topic} states={self.st_topic} "
            f"usb={self.arm.status()}",
            flush=True,
        )

    def publish_states(self, positions: List[float]) -> None:
        msg = {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "roarm_base_link"},
            "name": list(JOINT_NAMES),
            "position": [float(p) for p in positions],
            "velocity": [],
            "effort": [],
        }
        with self._lock:
            self._send(
                {
                    "op": "publish",
                    "id": f"publish:{self.st_topic}:{self._next_id()}",
                    "topic": self.st_topic,
                    "msg": msg,
                }
            )

    def handle_command(self, msg: dict) -> None:
        joints = _positions_from_msg(msg)
        if joints is None:
            return
        ok, text = self.arm.set_joints(
            joints[0], joints[1], joints[2], joints[3], spd=0, acc=12
        )
        if ok:
            self._last_joints = joints
            try:
                self.publish_states(joints)
            except Exception as e:
                print(f"[roarm_bridge_host] state pub failed: {e}", flush=True)
        else:
            print(f"[roarm_bridge_host] USB write failed: {text}", flush=True)

    def _recv_loop(self) -> None:
        assert self._ws is not None
        while self._running:
            try:
                raw = self._ws.recv()
            except Exception as e:
                print(f"[roarm_bridge_host] recv error: {e}", flush=True)
                break
            if not raw:
                break
            try:
                data = json.loads(raw)
            except Exception:
                continue
            if data.get("op") == "publish" and data.get("topic") in (
                self.cmd_topic,
                self.cmd_topic.lstrip("/"),
            ):
                self.handle_command(data.get("msg") or {})

    def _feedback_loop(self) -> None:
        hz = float(os.environ.get("ROARM_FEEDBACK_HZ") or "2")
        if hz <= 0:
            return
        period = 1.0 / hz
        while self._running:
            time.sleep(period)
            try:
                if self.arm.connected:
                    self.arm.feedback()
                self.publish_states(self._last_joints)
            except Exception:
                pass

    def run(self) -> None:
        backoff = 1.0
        while self._running:
            try:
                self.connect()
                backoff = 1.0
                t_fb = threading.Thread(target=self._feedback_loop, daemon=True)
                t_fb.start()
                self._recv_loop()
            except Exception as e:
                print(f"[roarm_bridge_host] connect/run: {e}", flush=True)
            try:
                if self._ws:
                    self._ws.close()
            except Exception:
                pass
            self._ws = None
            if not self._running:
                break
            time.sleep(backoff)
            backoff = min(15.0, backoff * 1.5)

    def stop(self) -> None:
        self._running = False
        try:
            if self._ws:
                self._ws.close()
        except Exception:
            pass
        try:
            self.arm.close()
        except Exception:
            pass


def main() -> int:
    bridge = RoArmHostBridge()
    try:
        bridge.run()
    except KeyboardInterrupt:
        bridge.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
