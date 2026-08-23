#!/usr/bin/env python3
"""Offline dual-HW gating tests (no exclusive hardware required).

Proves hangar/loadout gates:
  - rover+ptz does not want RoArm
  - attachment=roarm2 wants RoArm + T:144 map / poses exist
  - Seek look-around source gates USB RoArm before PTZ
  - chassis driver preference favors ugv_driver_min without requiring ugv_ws
"""
from __future__ import annotations

import ast
import os
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import loadout  # noqa: E402
import roarm_ctrl  # noqa: E402
from ros_motion import (  # noqa: E402
    parse_chassis_driver_pids,
    prefer_ugv_driver_min,
)


class TestHangarRoArmGate(unittest.TestCase):
    def test_rover_ptz_does_not_want_roarm(self):
        self.assertFalse(loadout.wants_roarm({"base": "rover", "attachment": "ptz"}))
        payload = loadout.public_payload(
            {"base": "rover", "attachment": "ptz"},
            roarm_started=False,
        )
        self.assertFalse(payload["roarm_started"])

    def test_roarm2_wants_but_started_is_explicit(self):
        self.assertTrue(loadout.wants_roarm({"attachment": "roarm2"}))
        self.assertFalse(
            loadout.public_payload({"attachment": "roarm2"}, roarm_started=False)[
                "roarm_started"
            ]
        )
        self.assertTrue(
            loadout.public_payload({"attachment": "roarm2"}, roarm_started=True)[
                "roarm_started"
            ]
        )


class TestRoArmPostures(unittest.TestCase):
    def test_travel_tuck_lower_than_home(self):
        home = roarm_ctrl.POSES["home"]
        tuck = roarm_ctrl.POSES["travel_tuck"]
        self.assertLess(tuck["elbow"], home["elbow"])
        self.assertLess(tuck["shoulder"], 0.0)
        self.assertIn("tuck", roarm_ctrl.POSES)

    def test_e_z_r_maps_to_joints(self):
        j = roarm_ctrl.e_z_r_to_joints(60, 24, 0)
        self.assertIn("base", j)
        self.assertIn("elbow", j)
        self.assertAlmostEqual(j["base"], 0.0, places=5)


class TestSeekRoArmGateInSource(unittest.TestCase):
    def test_seek_look_deg_has_usb_branch(self):
        path = os.path.join(ROOT, "app.py")
        src = open(path, encoding="utf-8").read()
        idx_fn = src.find("def _seek_look_deg")
        self.assertGreater(idx_fn, 0)
        body = src[idx_fn : idx_fn + 3500]
        self.assertIn("arm_usb_enabled()", body)
        self.assertIn("roarm_look", body)
        self.assertIn("skipped_ptz", body)
        self.assertIn("send_gimbal_command", body)
        self.assertLess(body.find("arm_usb_enabled()"), body.find("send_gimbal_command"))

    def test_seek_look_deg_ast_contains_early_return_on_roarm(self):
        path = os.path.join(ROOT, "app.py")
        tree = ast.parse(open(path, encoding="utf-8").read())
        fn = None
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name == "_seek_look_deg":
                fn = node
                break
        self.assertIsNotNone(fn, "_seek_look_deg not found")
        returns = [n for n in ast.walk(fn) if isinstance(n, ast.Return)]
        self.assertGreaterEqual(len(returns), 2, "expected RoArm early return + PTZ return")

    def test_app_uses_roarm_ctrl_singleton(self):
        src = Path(ROOT, "app.py").read_text(encoding="utf-8")
        self.assertNotIn("\n_roarm = None\n", src)
        stop = src[src.find("def _stop_roarm") : src.find("def _stop_roarm") + 900]
        self.assertIn("shutdown_roarm", stop)
        self.assertIn("current_roarm", stop)
        start = src[src.find("def _start_roarm") : src.find("def _start_roarm") + 1400]
        self.assertIn("roarm_ctrl.get_roarm", start)
        self.assertIn("current_roarm", start)


class TestDriverMinPrefer(unittest.TestCase):
    def test_prefer_helper_orders_driver_min(self):
        self.assertEqual(
            prefer_ugv_driver_min("auto", driver_min_available=True, bringup_available=True),
            "ugv_driver_min",
        )
        self.assertEqual(
            prefer_ugv_driver_min("auto", driver_min_available=False, bringup_available=True),
            "ugv_bringup",
        )
        self.assertEqual(prefer_ugv_driver_min("ugv_bringup", driver_min_available=True), "ugv_bringup")
        self.assertEqual(prefer_ugv_driver_min("auto"), "none")

    def test_parse_chassis_driver_pids_includes_driver_min(self):
        ps = (
            "  11 bash -lc pgrep -af ugv_bringup\n"
            "  42 python3 /tmp/ugv_driver_min.py\n"
            "  55 /opt/ros/humble/lib/ugv_bringup/ugv_bringup --ros-args\n"
            "  99 python3 /opt/ugv_ros2/roarm_driver_min.py\n"
        )
        # bringup PIDs first, then ugv_driver_min (roarm_driver_min excluded)
        self.assertEqual(parse_chassis_driver_pids(ps), [55, 42])

    def test_app_probe_prefers_driver_min_source(self):
        src = Path(ROOT, "app.py").read_text(encoding="utf-8")
        self.assertIn("def _probe_chassis_driver_kind", src)
        idx = src.find("def _probe_chassis_driver_kind")
        body = src[idx : idx + 1800]
        self.assertIn("ugv_driver_min", body)
        self.assertIn("never require ugv_ws", body.lower() or body)
        # Soft check: ugv_ws setup is optional fallback only
        self.assertIn("ugv_bringup", body)

    def test_ugv_driver_min_file_present(self):
        self.assertTrue((ROOT / "ros2" / "ugv_driver_min.py").is_file())


class TestAppArmUsbGate(unittest.TestCase):
    def test_arm_usb_enabled_uses_loadout(self):
        src = Path(ROOT, "app.py").read_text(encoding="utf-8")
        idx = src.find("def arm_usb_enabled")
        self.assertGreater(idx, 0)
        body = src[idx : idx + 400]
        self.assertIn("wants_roarm", body)
        self.assertIn("_loadout_store", body)

    def test_route_json_has_t144_branch(self):
        src = Path(ROOT, "app.py").read_text(encoding="utf-8")
        idx = src.find("def _route_json_command")
        body = src[idx : idx + 6000]
        self.assertIn("arm_ui_types", body)
        self.assertIn("_route_arm_ui_cmd", body)
        self.assertIn("roarm_raw_types", body)


if __name__ == "__main__":
    unittest.main()
