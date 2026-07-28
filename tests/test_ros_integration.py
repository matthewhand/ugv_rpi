#!/usr/bin/env python3
"""Gating tests for ROS 2 dual-path integration (shipped code paths).

Run on the robot with ugv_ros2 up and Flask app running:
  cd ~/ugv_rpi && ./ugv-env/bin/python -m pytest tests/test_ros_integration.py -v
  # or without pytest:
  ./ugv-env/bin/python tests/test_ros_integration.py
"""
from __future__ import annotations

import json
import os
import sys
import urllib.error
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    from dotenv import load_dotenv

    load_dotenv(os.path.join(ROOT, ".env"))
except Exception:
    pass

import ros_motion  # noqa: E402  — shipped module under test


def _http_json(method: str, path: str, body: dict | None = None, timeout: float = 10.0):
    url = f"http://127.0.0.1:{os.environ.get('UGV_PORT', '5000')}{path}"
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def test_rosbridge_status_ok():
    st = ros_motion.rosbridge_status()
    assert st.get("ok") is True, st
    assert "9090" in (st.get("url") or ""), st


def test_publish_cmd_vel_near_zero():
    r = ros_motion.publish_cmd_vel(0.0, 0.0)
    assert r.get("ok") is True, r
    assert r.get("topic") in ("/cmd_vel", "cmd_vel") or str(r.get("topic", "")).endswith(
        "cmd_vel"
    ), r


def test_ros_drive_async_stop_zero():
    r = ros_motion.ros_drive(0.0, 0.0, duration_ms=50)
    assert r.get("ok") is True, r


def test_api_status_ros2_uart_released():
    # Ensure mode is ros2 via shipped API
    mode = _http_json("POST", "/api/control_mode", {"mode": "ros2"})
    assert mode.get("control_mode") == "ros2" or mode.get("success") is True, mode
    st = _http_json("GET", "/api/status")
    assert st.get("control_mode") == "ros2", st
    assert st.get("uart_owner") == "ros2", st
    assert st.get("serial_open") is False, st


def test_stack_restart_and_rosbridge_recovers():
    r = _http_json("POST", "/api/stack_restart", {})
    assert r.get("success") is True, r
    # poll rosbridge via shipped helper
    import time

    ok = False
    last = None
    for _ in range(20):
        last = ros_motion.rosbridge_status()
        if last.get("ok"):
            ok = True
            break
        time.sleep(1)
    assert ok, last


def test_direct_reclaim_then_back_to_ros2():
    direct = _http_json("POST", "/api/control_mode", {"mode": "direct"})
    assert direct.get("control_mode") == "direct", direct
    assert direct.get("uart_owner") == "flask", direct
    assert direct.get("serial_open") is True, direct
    # Return UART to ROS and restart stack so bringup re-opens serial
    ros = _http_json("POST", "/api/control_mode", {"mode": "ros2"})
    assert ros.get("control_mode") == "ros2", ros
    assert ros.get("serial_open") is False, ros
    rr = _http_json("POST", "/api/stack_restart", {})
    assert rr.get("success") is True, rr
    import time

    for _ in range(20):
        if ros_motion.rosbridge_status().get("ok"):
            break
        time.sleep(1)
    assert ros_motion.rosbridge_status().get("ok") is True


def test_config_beast_identity():
    import yaml

    with open(os.path.join(ROOT, "config.yaml"), "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert int(cfg["base_config"]["main_type"]) == 3
    assert int(cfg["base_config"]["module_type"]) == 1
    assert "Beast" in str(cfg["base_config"]["robot_name"])


def test_compose_and_driver_artifacts_exist():
    for rel in (
        "ros2/docker-compose.yml",
        "ros2/entrypoint.sh",
        "ros2/ugv_driver_min.py",
        "ros2/roarm_driver_min.py",
        "ros2/roarm_bridge_host.py",
        "ros2/start_roarm_driver.sh",
        "ros_motion.py",
    ):
        path = os.path.join(ROOT, rel)
        assert os.path.isfile(path), path


def main() -> int:
    tests = [
        test_compose_and_driver_artifacts_exist,
        test_config_beast_identity,
        test_rosbridge_status_ok,
        test_publish_cmd_vel_near_zero,
        test_ros_drive_async_stop_zero,
        test_api_status_ros2_uart_released,
        test_stack_restart_and_rosbridge_recovers,
        test_direct_reclaim_then_back_to_ros2,
    ]
    failed = 0
    for t in tests:
        name = t.__name__
        try:
            t()
            print(f"PASS {name}")
        except Exception as e:
            failed += 1
            print(f"FAIL {name}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
