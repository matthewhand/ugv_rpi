#!/usr/bin/env python3
"""Unit tests for chassis/attachment loadout (no Flask, no cameras, no UART)."""
from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import loadout  # noqa: E402


class TestNormalize(unittest.TestCase):
    def test_default(self):
        lo = loadout.normalize_loadout(None)
        self.assertEqual(lo["base"], "rover")
        self.assertEqual(lo["attachment"], "ptz")
        self.assertFalse(lo["use_lidar"])
        self.assertEqual(lo["camera_prefer"], "auto")

    def test_unknown_falls_back(self):
        lo = loadout.normalize_loadout({"base": "hovercraft", "attachment": "laser"})
        self.assertEqual(lo["base"], "rover")
        self.assertEqual(lo["attachment"], "ptz")

    def test_bool_coercion(self):
        self.assertTrue(loadout.normalize_loadout({"use_lidar": "yes"})["use_lidar"])
        self.assertFalse(loadout.normalize_loadout({"use_lidar": "off"})["use_lidar"])

    def test_known_combo(self):
        lo = loadout.normalize_loadout(
            {"base": "beast", "attachment": "roarm2", "use_lidar": True, "camera_prefer": "usb"}
        )
        self.assertEqual(lo["base"], "beast")
        self.assertEqual(lo["attachment"], "roarm2")
        self.assertTrue(lo["use_lidar"])
        self.assertEqual(lo["camera_prefer"], "usb")


class TestWaveshareTypes(unittest.TestCase):
    def test_rover_ptz_types(self):
        types = loadout.effective_types({"base": "rover", "attachment": "ptz"})
        self.assertEqual(types["main_type"], 2)
        self.assertEqual(types["module_type"], 2)
        self.assertEqual(types["drive"], "wheels")
        self.assertFalse(types["arm_wanted"])

    def test_beast_roarm_types_wanted(self):
        types = loadout.effective_types({"base": "beast", "attachment": "roarm2"})
        self.assertEqual(types["main_type"], 3)
        self.assertEqual(types["module_type"], 1)
        self.assertEqual(types["drive"], "tracks")
        self.assertTrue(types["arm_wanted"])
        self.assertTrue(types["arm_wired"])

    def test_wants_roarm_gate(self):
        self.assertFalse(loadout.wants_roarm({"base": "rover", "attachment": "ptz"}))
        self.assertFalse(loadout.wants_roarm({"base": "beast", "attachment": "ptz"}))
        self.assertTrue(loadout.wants_roarm({"base": "beast", "attachment": "roarm2"}))
        self.assertTrue(loadout.wants_roarm({"base": "rover", "attachment": "roarm2"}))

    def test_none_attachment(self):
        types = loadout.effective_types({"base": "rover", "attachment": "none"})
        self.assertEqual(types["module_type"], 0)

    def test_from_config_roundtrip(self):
        cfg = {
            "base_config": {
                "main_type": 3,
                "module_type": 1,
                "use_lidar": False,
                "drive_linear_sign": 1,
            }
        }
        lo = loadout.loadout_from_config(cfg)
        self.assertEqual(lo["base"], "beast")
        self.assertEqual(lo["attachment"], "roarm2")


class TestApplyDoesNotTouchDriveSigns(unittest.TestCase):
    def test_preserves_drive_signs(self):
        cfg = {
            "base_config": {
                "main_type": 2,
                "module_type": 2,
                "use_lidar": False,
                "drive_linear_sign": -1,
                "drive_angular_sign": 1,
                "robot_name": "UGV Rover",
            }
        }
        loadout.apply_loadout_to_config(
            cfg, {"base": "beast", "attachment": "none", "use_lidar": True}
        )
        self.assertEqual(cfg["base_config"]["main_type"], 3)
        self.assertEqual(cfg["base_config"]["module_type"], 0)
        self.assertTrue(cfg["base_config"]["use_lidar"])
        self.assertEqual(cfg["base_config"]["drive_linear_sign"], -1)
        self.assertEqual(cfg["base_config"]["drive_angular_sign"], 1)
        self.assertEqual(cfg["base_config"]["robot_name"], "UGV Beast")


class TestCameraStrategy(unittest.TestCase):
    def test_rover_auto_is_csi(self):
        self.assertEqual(loadout.camera_strategy({"base": "rover"}), "csi")

    def test_beast_auto_is_usb(self):
        self.assertEqual(loadout.camera_strategy({"base": "beast"}), "usb")

    def test_explicit_overrides(self):
        self.assertEqual(
            loadout.camera_strategy({"base": "rover", "camera_prefer": "usb"}), "usb"
        )
        self.assertEqual(
            loadout.camera_strategy({"base": "beast", "camera_prefer": "csi"}), "csi"
        )


class TestPublicPayload(unittest.TestCase):
    def test_ptz_roarm_not_started(self):
        payload = loadout.public_payload({"base": "rover", "attachment": "ptz"})
        self.assertFalse(payload["roarm_started"])
        self.assertFalse(loadout.wants_roarm(payload["loadout"]))

    def test_roarm2_started_flag_honored(self):
        payload = loadout.public_payload(
            {"base": "beast", "attachment": "roarm2"},
            roarm_started=True,
        )
        self.assertTrue(payload["roarm_started"])
        self.assertEqual(payload["arm_status"], "started")

    def test_roarm2_wanted_but_not_started(self):
        payload = loadout.public_payload(
            {"base": "beast", "attachment": "roarm2"},
            roarm_started=False,
        )
        self.assertFalse(payload["roarm_started"])
        self.assertEqual(payload["arm_status"], "wanted")

    def test_drive_signs_are_read_only_from_cfg(self):
        cfg = {"base_config": {"drive_linear_sign": -1, "drive_angular_sign": 1}}
        payload = loadout.public_payload({"base": "rover", "attachment": "ptz"}, cfg)
        self.assertEqual(payload["drive_linear_sign"], -1.0)
        self.assertEqual(payload["drive_angular_sign"], 1.0)


class TestLoadoutStore(unittest.TestCase):
    def test_persist_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = loadout.LoadoutStore(tmp)
            saved = store.set({"base": "beast", "attachment": "roarm2", "use_lidar": False})
            self.assertEqual(saved["base"], "beast")
            self.assertTrue(os.path.isfile(os.path.join(tmp, ".loadout.json")))
            again = loadout.LoadoutStore(tmp)
            loaded = again.load()
            self.assertEqual(loaded["base"], "beast")
            self.assertEqual(loaded["attachment"], "roarm2")
            with open(os.path.join(tmp, ".loadout.json"), encoding="utf-8") as fh:
                disk = json.load(fh)
            self.assertEqual(disk["base"], "beast")

    def test_missing_file_falls_back_to_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = loadout.LoadoutStore(tmp)
            loaded = store.load(
                fallback_from_config={
                    "base_config": {"main_type": 2, "module_type": 2, "use_lidar": False}
                }
            )
            self.assertEqual(loaded["base"], "rover")
            self.assertEqual(loaded["attachment"], "ptz")

    def test_set_does_not_start_hardware(self):
        """LoadoutStore itself never opens RoArm — app.py owns that gate."""
        with tempfile.TemporaryDirectory() as tmp:
            store = loadout.LoadoutStore(tmp)
            payload = store.set({"attachment": "roarm2"})
            pub = store.public(roarm_started=False)
            self.assertEqual(payload["attachment"], "roarm2")
            self.assertFalse(pub["roarm_started"])
            self.assertTrue(loadout.wants_roarm(payload))


class TestShippedYamlExamples(unittest.TestCase):
    """Example profile files are documentation; live config.yaml stays rover."""

    def _load_yaml(self, name):
        import yaml

        with open(os.path.join(ROOT, name), encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    def test_live_config_not_beast_defaults(self):
        cfg = self._load_yaml("config.yaml")
        self.assertEqual(int(cfg["base_config"]["main_type"]), 2)
        self.assertEqual(int(cfg["base_config"]["drive_linear_sign"]), -1)
        self.assertFalse(bool(cfg["base_config"].get("use_lidar")))

    def test_example_rover_yaml(self):
        cfg = self._load_yaml("config.rover.yaml")
        lo = loadout.loadout_from_config(cfg)
        self.assertEqual(lo["base"], "rover")
        self.assertEqual(lo["attachment"], "ptz")
        self.assertFalse(lo["use_lidar"])
        self.assertEqual(int(cfg["base_config"]["drive_linear_sign"]), -1)
        self.assertEqual(loadout.camera_strategy(lo), "csi")

    def test_example_beast_yaml(self):
        cfg = self._load_yaml("config.beast.yaml")
        lo = loadout.loadout_from_config(cfg)
        self.assertEqual(lo["base"], "beast")
        self.assertEqual(lo["attachment"], "roarm2")
        self.assertFalse(lo["use_lidar"])
        self.assertEqual(int(cfg["base_config"]["drive_linear_sign"]), 1)
        self.assertEqual(loadout.camera_strategy(lo), "usb")
        self.assertTrue(loadout.wants_roarm(lo))


class TestSourceGates(unittest.TestCase):
    """Hangar gates: camera live apply + RoArm only on roarm2."""

    def test_api_loadout_applies_camera_and_syncs_roarm(self):
        src = Path(ROOT, "app.py").read_text(encoding="utf-8")
        idx = src.find("def api_loadout")
        self.assertGreater(idx, 0)
        body = src[idx : idx + 3500]
        self.assertIn("_loadout_store.set", body)
        self.assertIn("apply_camera_prefer", body)
        self.assertIn("_sync_roarm_to_loadout", body)
        self.assertNotIn("open(thisPath + '/config.yaml', \"w\")", body)

    def test_cv_ctrl_has_live_camera_apply(self):
        src = Path(ROOT, "cv_ctrl.py").read_text(encoding="utf-8")
        self.assertIn("def apply_camera_prefer", src)
        self.assertIn("def _open_usb_camera", src)
        self.assertIn("camera_strategy", src)

    def test_roarm_ctrl_present_with_poses_and_t144(self):
        self.assertTrue(os.path.isfile(os.path.join(ROOT, "roarm_ctrl.py")))
        import roarm_ctrl

        self.assertIn("travel_tuck", roarm_ctrl.POSES)
        self.assertTrue(callable(roarm_ctrl.e_z_r_to_joints))
        joints = roarm_ctrl.e_z_r_to_joints(60, 24, 0)
        self.assertIn("base", joints)
        self.assertTrue(callable(roarm_ctrl.current_roarm))
        self.assertTrue(callable(roarm_ctrl.shutdown_roarm))
        self.assertIsNone(roarm_ctrl.current_roarm())

    def test_gitignore_runtime_loadout(self):
        text = Path(ROOT, ".gitignore").read_text(encoding="utf-8")
        self.assertIn(".loadout.json", text)
        self.assertIn("*.bak*", text)
        self.assertIn(".env.*", text)


class TestCameraPreferApplyUnit(unittest.TestCase):
    def test_cv_ctrl_exposes_live_prefer_helpers(self):
        # Avoid importing cv_ctrl (mediapipe/matplotlib stack). Assert the API exists in source.
        src = Path(ROOT, "cv_ctrl.py").read_text(encoding="utf-8")
        self.assertIn("def get_camera_first", src)
        self.assertIn("def set_camera_first", src)
        self.assertIn("def apply_camera_prefer", src)
        self.assertIn("self._close_cameras()", src)


if __name__ == "__main__":
    unittest.main()
