#!/usr/bin/env python3
"""Unit tests for chassis/attachment loadout (no Flask, no cameras, no UART)."""
from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

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
        self.assertFalse(types["arm_offline"])

    def test_beast_roarm_types_offline(self):
        types = loadout.effective_types({"base": "beast", "attachment": "roarm2"})
        self.assertEqual(types["main_type"], 3)
        self.assertEqual(types["module_type"], 1)
        self.assertEqual(types["drive"], "tracks")
        self.assertTrue(types["arm_offline"])
        self.assertEqual(types["arm_message"], loadout.ARM_OFFLINE_MESSAGE)
        self.assertFalse(types["arm_wired"])

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

    def test_does_not_add_arm_config(self):
        cfg = {"base_config": {"drive_linear_sign": -1}}
        loadout.apply_loadout_to_config(cfg, {"base": "beast", "attachment": "roarm2"})
        self.assertNotIn("arm_config", cfg)


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
    def test_roarm_not_started(self):
        payload = loadout.public_payload({"base": "beast", "attachment": "roarm2"})
        self.assertFalse(payload["roarm_started"])
        self.assertEqual(payload["arm_status"], "offline")
        self.assertEqual(payload["arm_message"], loadout.ARM_OFFLINE_MESSAGE)
        self.assertIn("bases", payload)
        self.assertIn("attachments", payload)

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

    def test_set_does_not_import_roarm(self):
        self.assertFalse(os.path.isfile(os.path.join(ROOT, "roarm_ctrl.py")))
        with tempfile.TemporaryDirectory() as tmp:
            store = loadout.LoadoutStore(tmp)
            payload = store.set({"attachment": "roarm2"})
            pub = store.public()
            self.assertEqual(payload["attachment"], "roarm2")
            self.assertFalse(pub["roarm_started"])
            self.assertNotIn("roarm_ctrl", sys.modules)


class TestShippedYamlExamples(unittest.TestCase):
    """Example profile files are documentation; live config.yaml stays rover."""

    def _load_yaml(self, name):
        import yaml

        with open(os.path.join(ROOT, name), encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    def test_live_config_not_beast_defaults(self):
        cfg = self._load_yaml("config.yaml")
        # This tree is the rover. Do not ship beast drive sign / main_type as live.
        self.assertEqual(int(cfg["base_config"]["main_type"]), 2)
        self.assertEqual(int(cfg["base_config"]["drive_linear_sign"]), -1)
        self.assertFalse(bool(cfg["base_config"].get("use_lidar")))
        self.assertNotIn("arm_config", cfg)

    def test_example_rover_yaml(self):
        cfg = self._load_yaml("config.rover.yaml")
        lo = loadout.loadout_from_config(cfg)
        self.assertEqual(lo["base"], "rover")
        self.assertEqual(lo["attachment"], "ptz")
        self.assertFalse(lo["use_lidar"])
        self.assertEqual(int(cfg["base_config"]["drive_linear_sign"]), -1)
        self.assertNotIn("arm_config", cfg)
        self.assertEqual(loadout.camera_strategy(lo), "csi")

    def test_example_beast_yaml(self):
        cfg = self._load_yaml("config.beast.yaml")
        lo = loadout.loadout_from_config(cfg)
        self.assertEqual(lo["base"], "beast")
        self.assertEqual(lo["attachment"], "roarm2")
        self.assertFalse(lo["use_lidar"])
        self.assertEqual(int(cfg["base_config"]["drive_linear_sign"]), 1)
        self.assertNotIn("arm_config", cfg)
        self.assertEqual(loadout.camera_strategy(lo), "usb")
        # Example file may comment arm_config; parsed YAML must not enable it.
        text = Path(ROOT, "config.beast.yaml").read_text(encoding="utf-8")
        self.assertIn("not wired", text.lower())
        self.assertIn("# arm_config:", text)


class TestSourceGates(unittest.TestCase):
    """Keep RoArm / drive-sign side effects out of this consolidate slice."""

    def test_api_loadout_does_not_start_arm_or_write_yaml(self):
        src = Path(ROOT, "app.py").read_text(encoding="utf-8")
        idx = src.find("def api_loadout")
        self.assertGreater(idx, 0)
        body = src[idx : idx + 2000]
        self.assertNotIn("set_version(", body)
        self.assertNotIn("roarm_ctrl", body)
        self.assertNotIn("open(thisPath + '/config.yaml', \"w\")", body)
        self.assertIn("_loadout_store.set", body)
        self.assertIn("roarm_started", body)

    def test_cv_ctrl_has_gated_usb_rediscovery(self):
        src = Path(ROOT, "cv_ctrl.py").read_text(encoding="utf-8")
        self.assertIn("def _open_usb_camera", src)
        self.assertIn("camera_strategy", src)
        self.assertIn("usb camera read failed after reopen", src)
        self.assertNotIn("import roarm", src)

    def test_gitignore_runtime_loadout(self):
        text = Path(ROOT, ".gitignore").read_text(encoding="utf-8")
        self.assertIn(".loadout.json", text)


if __name__ == "__main__":
    unittest.main()
