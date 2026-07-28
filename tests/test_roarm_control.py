#!/usr/bin/env python3
"""Prove RoArm control on UGV Beast via shipped serial JSON path.

Drives the real ESP32 arm command surface used by the product:
  - module select T:4 / T:900
  - single-joint T:101 (base, shoulder, elbow, gripper) both directions + home
  - multi-joint T:102 and UI T:144 as extra

Hardware evidence: ESP32 bus-servo status frames T:1005 (id, status=0).

Usage (robot, arm powered, exclusive UART):
  # Prefer stopping ROS container first:
  #   docker stop ugv_ros2
  cd ~/ugv_rpi
  ./ugv-env/bin/python tests/test_roarm_control.py

Exit 0 only if all axes commanded successfully AND T:1005 ACKs seen for >=4 servos.
"""
from __future__ import annotations

import json
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import Counter
from typing import Any, Dict, List, Optional, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    from dotenv import load_dotenv

    load_dotenv(os.path.join(ROOT, ".env"))
except Exception:
    pass

import yaml  # noqa: E402

# Small, hardware-safe angles (radians) — both directions then home.
AXIS_PLAN = [
    # name, joint_id, dir_a, dir_b, home
    ("base", 1, 0.35, -0.35, 0.0),
    ("shoulder", 2, 0.30, -0.25, 0.0),
    ("elbow", 3, 2.00, 1.20, 1.57),
    ("gripper", 4, 2.20, 3.00, 3.05),
]


def _http_json(method: str, path: str, body: Optional[dict] = None, timeout: float = 10.0) -> dict:
    port = os.environ.get("UGV_PORT") or "5000"
    url = f"http://127.0.0.1:{port}{path}"
    data = None
    headers: Dict[str, str] = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _send_command_cli(cmd: str, timeout: float = 10.0) -> dict:
    """Shipped web CLI entry: POST /send_command → cmdline_ctrl → base_json_ctrl."""
    port = os.environ.get("UGV_PORT") or "5000"
    url = f"http://127.0.0.1:{port}/send_command"
    data = urllib.parse.urlencode({"command": cmd}).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def test_config_roarm() -> dict:
    with open(os.path.join(ROOT, "config.yaml"), "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    main_type = int(cfg["base_config"]["main_type"])
    module_type = int(cfg["base_config"]["module_type"])
    assert main_type == 3, f"expected Beast main_type=3, got {main_type}"
    assert module_type == 1, f"expected RoArm module_type=1, got {module_type}"
    return {
        "main_type": main_type,
        "module_type": module_type,
        "robot_name": cfg["base_config"].get("robot_name"),
    }


def test_uart_ready_for_arm() -> dict:
    """Ensure Flask can own UART (direct) for arm serial."""
    try:
        _http_json("POST", "/api/control_mode", {"mode": "direct"})
    except Exception as e:
        raise AssertionError(f"cannot set control_mode=direct: {e}") from e
    st = _http_json("GET", "/api/status")
    assert st.get("control_mode") == "direct", st
    assert st.get("serial_open") is True, st
    assert st.get("uart_owner") == "flask", st
    return st


def test_app_path_module_and_axes() -> List[dict]:
    """Command all axes through shipped /send_command (app → base_ctrl → UART)."""
    results: List[dict] = []

    def step(label: str, payload: dict) -> None:
        cmd = "base -c " + json.dumps(payload, separators=(",", ":"))
        resp = _send_command_cli(cmd)
        ok = resp.get("status") == "success"
        results.append({"label": label, "payload": payload, "response": resp, "ok": ok})
        assert ok, f"{label} failed: {resp}"
        time.sleep(0.35)

    step("module_select_arm", {"T": 4, "cmd": 1})
    step("mm_type_beast_arm", {"T": 900, "main": 3, "module": 1})
    step("ui_home", {"T": 144, "E": 60, "Z": 24, "R": 0})
    time.sleep(1.0)

    for name, joint, a, b, home in AXIS_PLAN:
        step(f"{name}_dir_a", {"T": 101, "joint": joint, "rad": a, "spd": 0, "acc": 25})
        time.sleep(1.6)
        step(f"{name}_dir_b", {"T": 101, "joint": joint, "rad": b, "spd": 0, "acc": 25})
        time.sleep(1.6)
        step(f"{name}_home", {"T": 101, "joint": joint, "rad": home, "spd": 0, "acc": 25})
        time.sleep(1.2)

    step(
        "multi_pose_a",
        {"T": 102, "base": 0.25, "shoulder": 0.15, "elbow": 1.7, "hand": 2.5, "spd": 0, "acc": 15},
    )
    time.sleep(2.0)
    step(
        "multi_pose_b",
        {"T": 102, "base": -0.25, "shoulder": -0.1, "elbow": 1.4, "hand": 2.9, "spd": 0, "acc": 15},
    )
    time.sleep(2.0)
    step(
        "multi_home",
        {"T": 102, "base": 0.0, "shoulder": 0.0, "elbow": 1.57, "hand": 3.0, "spd": 0, "acc": 15},
    )
    time.sleep(1.0)
    step("ui_home_final", {"T": 144, "E": 60, "Z": 24, "R": 0})
    return results


def _find_app_pids() -> List[int]:
    pids: List[int] = []
    try:
        import subprocess

        out = subprocess.check_output(["ps", "-eo", "pid,cmd"], text=True)
        for line in out.splitlines():
            if "/ugv-env/bin/python" in line and "/ugv_rpi/app.py" in line:
                parts = line.strip().split(None, 1)
                if parts:
                    pids.append(int(parts[0]))
    except Exception:
        pass
    return pids


def test_serial_hw_ack_via_base_ctrl() -> dict:
    """Exclusive serial session using shipped BaseController; capture T:1005 ACKs.

    Temporarily stops Flask app processes that hold the UART, then restarts is
    the caller's responsibility when run via main().
    """
    import serial as pyserial  # type: ignore
    from base_ctrl import BaseController

    # Determine UART the same way the app does
    def is_pi5() -> bool:
        try:
            with open("/proc/cpuinfo", "r", encoding="utf-8") as fh:
                return "Raspberry Pi 5" in fh.read()
        except Exception:
            return False

    uart = "/dev/ttyAMA0" if is_pi5() else "/dev/serial0"

    # Stop app holders
    stopped: List[int] = []
    for pid in _find_app_pids():
        try:
            os.kill(pid, 15)
            stopped.append(pid)
        except Exception:
            pass
    time.sleep(2.0)
    for pid in list(stopped):
        try:
            os.kill(pid, 0)
            os.kill(pid, 9)
        except Exception:
            pass
    time.sleep(1.0)

    # Direct serial reader alongside BaseController write path
    ser = pyserial.Serial(uart, 115200, timeout=0.3)
    time.sleep(0.15)
    ser.reset_input_buffer()

    def write_json(obj: dict) -> None:
        ser.write((json.dumps(obj) + "\n").encode("utf-8"))
        ser.flush()

    def read_lines(seconds: float) -> List[str]:
        end = time.time() + seconds
        lines: List[str] = []
        buf = b""
        while time.time() < end:
            n = ser.in_waiting
            if n:
                buf += ser.read(n)
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    s = raw.decode("utf-8", "replace").strip()
                    if s:
                        lines.append(s)
            else:
                time.sleep(0.02)
        return lines

    # Quiet continuous feedback flood while we watch ACKs
    write_json({"T": 131, "cmd": 0})
    time.sleep(0.15)
    ser.reset_input_buffer()

    write_json({"T": 4, "cmd": 1})
    write_json({"T": 900, "main": 3, "module": 1})
    time.sleep(0.3)

    steps: List[dict] = []
    ack_ids: Counter = Counter()
    ack_status: Counter = Counter()
    total_acks = 0

    def move(label: str, payload: dict, settle: float = 1.8) -> None:
        nonlocal total_acks
        write_json(payload)
        time.sleep(settle)
        # Nudge feedback
        write_json({"T": 105})
        lines = read_lines(1.0)
        acks = []
        for s in lines:
            try:
                o = json.loads(s)
            except Exception:
                continue
            if o.get("T") == 1005:
                acks.append(o)
                ack_ids[o.get("id")] += 1
                ack_status[o.get("status")] += 1
                total_acks += 1
        steps.append(
            {
                "label": label,
                "payload": payload,
                "ack_count": len(acks),
                "ack_sample": acks[:8],
            }
        )
        assert len(acks) > 0, f"{label}: no T:1005 servo ACKs after {payload}"

    # Also prove shipped BaseController.base_json_ctrl write path
    bc = BaseController(uart, 115200)
    # BaseController opened a second handle — close our reader first to avoid duplex weirdness on some UART
    # Actually both open can fight. Close ser, use only bc for one cmd, then reopen reader.
    ser.close()
    try:
        bc.base_json_ctrl({"T": 4, "cmd": 1})
        time.sleep(0.2)
    finally:
        try:
            if bc.ser:
                bc.ser.close()
        except Exception:
            pass
        bc.serial_released_for_ros = True

    ser = pyserial.Serial(uart, 115200, timeout=0.3)
    time.sleep(0.1)
    ser.reset_input_buffer()
    write_json({"T": 131, "cmd": 0})
    time.sleep(0.1)
    ser.reset_input_buffer()

    move("home_ui", {"T": 144, "E": 60, "Z": 24, "R": 0}, settle=1.4)
    for name, joint, a, b, home in AXIS_PLAN:
        move(f"{name}_a", {"T": 101, "joint": joint, "rad": a, "spd": 0, "acc": 30}, settle=1.9)
        move(f"{name}_b", {"T": 101, "joint": joint, "rad": b, "spd": 0, "acc": 30}, settle=1.9)
        move(f"{name}_home", {"T": 101, "joint": joint, "rad": home, "spd": 0, "acc": 30}, settle=1.5)

    move(
        "multi_a",
        {"T": 102, "base": 0.25, "shoulder": 0.15, "elbow": 1.7, "hand": 2.5, "spd": 0, "acc": 20},
        settle=2.2,
    )
    move(
        "multi_b",
        {"T": 102, "base": -0.25, "shoulder": -0.1, "elbow": 1.4, "hand": 2.9, "spd": 0, "acc": 20},
        settle=2.2,
    )
    move(
        "multi_home",
        {"T": 102, "base": 0.0, "shoulder": 0.0, "elbow": 1.57, "hand": 3.0, "spd": 0, "acc": 20},
        settle=1.8,
    )

    write_json({"T": 131, "cmd": 1})
    ser.close()

    unique_ids = sorted(i for i in ack_ids if i is not None)
    assert len(unique_ids) >= 4, f"expected >=4 servo IDs, got {unique_ids}"
    assert ack_status.get(0, 0) == total_acks, f"non-zero servo status present: {dict(ack_status)}"
    assert total_acks >= 20, f"too few ACKs: {total_acks}"

    return {
        "uart": uart,
        "steps": steps,
        "total_acks": total_acks,
        "servo_ids": dict(sorted(ack_ids.items(), key=lambda x: str(x[0]))),
        "status_counts": dict(ack_status),
        "unique_servo_ids": unique_ids,
        "axes": [a[0] for a in AXIS_PLAN],
        "base_ctrl_json_ctrl_exercised": True,
    }


def _restart_app() -> None:
    env = os.environ.copy()
    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    # Prefer direct for arm safety after proof
    try:
        with open(os.path.join(ROOT, ".control_mode.json"), "w", encoding="utf-8") as fh:
            json.dump({"mode": "direct"}, fh)
    except Exception:
        pass
    log_path = os.path.expanduser("~/ugv.log")
    import subprocess

    with open(log_path, "a", encoding="utf-8") as logf:
        subprocess.Popen(
            [os.path.join(ROOT, "ugv-env", "bin", "python"), os.path.join(ROOT, "app.py")],
            cwd=ROOT,
            env=env,
            stdout=logf,
            stderr=logf,
            start_new_session=True,
        )
    # wait for port
    for _ in range(30):
        try:
            _http_json("GET", "/api/status")
            return
        except Exception:
            time.sleep(0.5)


def main() -> int:
    print("=== RoArm control proof ===")
    cfg = test_config_roarm()
    print("PASS config", cfg)

    st = test_uart_ready_for_arm()
    print("PASS uart", {k: st.get(k) for k in ("control_mode", "serial_open", "uart_owner")})

    app_steps = test_app_path_module_and_axes()
    print(f"PASS app_path steps={len(app_steps)} (all /send_command success)")

    # HW ACKs need exclusive UART — stops Flask then restarts
    hw = test_serial_hw_ack_via_base_ctrl()
    print(
        "PASS hw_ack",
        {
            "total_acks": hw["total_acks"],
            "servo_ids": hw["servo_ids"],
            "axes": hw["axes"],
        },
    )

    try:
        _restart_app()
        st2 = _http_json("GET", "/api/status")
        print("PASS app_restarted", {k: st2.get(k) for k in ("control_mode", "serial_open")})
    except Exception as e:
        print("WARN app restart:", e)

    print("ALL RoArm proof checks passed")
    # Emit machine-readable summary on stdout for capture
    summary = {
        "ok": True,
        "config": cfg,
        "app_steps": len(app_steps),
        "hw": {
            "total_acks": hw["total_acks"],
            "servo_ids": hw["servo_ids"],
            "unique_servo_ids": hw["unique_servo_ids"],
            "status_counts": hw["status_counts"],
            "axes": hw["axes"],
            "step_labels": [s["label"] for s in hw["steps"]],
        },
    }
    print("SUMMARY_JSON=" + json.dumps(summary))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as e:
        print("FAIL", e)
        raise SystemExit(1)
    except Exception as e:
        print("ERROR", type(e).__name__, e)
        raise SystemExit(2)
