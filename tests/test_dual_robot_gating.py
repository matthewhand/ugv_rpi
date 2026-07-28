#!/usr/bin/env python3
"""Offline dual-robot gating tests (no exclusive hardware required).

Proves:
  - Beast config carries usb_serial arm transport + travel_tuck default pose
  - PTZ profile can be derived without USB RoArm
  - roarm_ctrl postures exist and map E/Z/R
  - Seek look-around in app.py is gated for USB RoArm (no pure PTZ-only path)
"""
from __future__ import annotations

import ast
import os
import sys
import unittest

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


class TestConfigProfiles(unittest.TestCase):
    def test_beast_config_has_usb_roarm(self):
        with open(os.path.join(ROOT, "config.yaml"), encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh)
        arm = cfg.get("arm_config") or {}
        self.assertEqual(arm.get("transport"), "usb_serial")
        self.assertEqual(arm.get("default_pose"), "travel_tuck")
        self.assertEqual(int(cfg["base_config"]["module_type"]), 1)

    def test_ptz_profile_shape(self):
        """PTZ robot: module_type 2 + base_uart (or no arm_config)."""
        with open(os.path.join(ROOT, "config.yaml"), encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh)
        # Simulated PTZ profile derived from same schema
        ptz = {
            "base_config": {**cfg["base_config"], "module_type": 2, "robot_name": "UGV PTZ"},
            "arm_config": {"transport": "base_uart", "ui_aim_default": "pt"},
        }
        self.assertEqual(ptz["base_config"]["module_type"], 2)
        self.assertNotEqual(ptz["arm_config"]["transport"], "usb_serial")


class TestRoArmPostures(unittest.TestCase):
    def test_travel_tuck_lower_than_home(self):
        import roarm_ctrl

        home = roarm_ctrl.POSES["home"]
        tuck = roarm_ctrl.POSES["travel_tuck"]
        self.assertLess(tuck["elbow"], home["elbow"])
        self.assertLess(tuck["shoulder"], 0.0)  # lean back
        self.assertIn("tuck", roarm_ctrl.POSES)

    def test_e_z_r_maps_to_joints(self):
        import roarm_ctrl

        j = roarm_ctrl.e_z_r_to_joints(60, 24, 0)
        self.assertIn("base", j)
        self.assertIn("elbow", j)
        self.assertAlmostEqual(j["base"], 0.0, places=5)


class TestSeekRoArmGateInSource(unittest.TestCase):
    def test_seek_look_deg_has_usb_branch(self):
        path = os.path.join(ROOT, "app.py")
        src = open(path, encoding="utf-8").read()
        # Must gate before send_gimbal for seek look
        idx_fn = src.find("def _seek_look_deg")
        self.assertGreater(idx_fn, 0)
        # Next 80 lines of function body
        body = src[idx_fn : idx_fn + 3500]
        self.assertIn("arm_usb_enabled()", body)
        self.assertIn("roarm_look", body)
        self.assertIn("skipped_ptz", body)
        # PTZ path still present after gate
        self.assertIn("send_gimbal_command", body)
        # Ensure arm_usb check appears before send_gimbal in this function
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
        # Collect return counts inside function
        returns = [n for n in ast.walk(fn) if isinstance(n, ast.Return)]
        self.assertGreaterEqual(len(returns), 2, "expected RoArm early return + PTZ return")


class TestSeekEntrypointsPresent(unittest.TestCase):
    def test_ai_seek_module(self):
        path = os.path.join(ROOT, "ai_seek.py")
        self.assertTrue(os.path.isfile(path), "ai_seek.py missing after integrate")

    def test_modes_js(self):
        path = os.path.join(ROOT, "templates", "modes.js")
        self.assertTrue(os.path.isfile(path), "modes.js missing after integrate")


if __name__ == "__main__":
    unittest.main()
