#!/usr/bin/env python3
"""Unit tests for Hailo loadout gate + feature stubs (no device / no HEF)."""
from __future__ import annotations

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
import hailo_features  # noqa: E402
from hailo_features import mqtt_topics  # noqa: E402
from hailo_features.runner import tick  # noqa: E402


class TestHailoNormalize(unittest.TestCase):
    def test_default_hailo_block(self):
        lo = loadout.normalize_loadout(None)
        self.assertIn("hailo", lo)
        self.assertEqual(lo["hailo"]["enabled"], "auto")
        self.assertTrue(lo["hailo"]["pose"])
        self.assertEqual(lo["hailo"]["device"], "/dev/hailo0")

    def test_enabled_coercion(self):
        self.assertEqual(loadout.normalize_hailo({"enabled": True})["enabled"], "true")
        self.assertEqual(loadout.normalize_hailo({"enabled": "off"})["enabled"], "false")
        self.assertEqual(loadout.normalize_hailo({"enabled": "AUTO"})["enabled"], "auto")

    def test_wanted_false(self):
        self.assertFalse(
            loadout.hailo_wanted({"hailo": {"enabled": "false"}}, device_exists=True)
        )

    def test_wanted_true_ignores_missing_device(self):
        self.assertTrue(
            loadout.hailo_wanted({"hailo": {"enabled": "true"}}, device_exists=False)
        )

    def test_wanted_auto(self):
        self.assertTrue(
            loadout.hailo_wanted({"hailo": {"enabled": "auto"}}, device_exists=True)
        )
        self.assertFalse(
            loadout.hailo_wanted({"hailo": {"enabled": "auto"}}, device_exists=False)
        )

    def test_store_deep_merge_hailo(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = loadout.LoadoutStore(tmp)
            store.load()
            store.set({"hailo": {"enabled": "true", "pose": False}})
            lo = store.get()
            self.assertEqual(lo["hailo"]["enabled"], "true")
            self.assertFalse(lo["hailo"]["pose"])
            self.assertTrue(lo["hailo"]["hand"])  # preserved default


class TestHailoStubs(unittest.TestCase):
    def test_tick_skipped_when_disabled(self):
        out = tick({"hailo": {"enabled": "false"}}, frame=None)
        self.assertTrue(out["skipped"])
        self.assertEqual(out["reason"], "disabled_or_no_device")
        self.assertEqual(out["results"], {})

    def test_tick_runs_stubs_when_forced(self):
        out = tick({"hailo": {"enabled": "true", "pose": True, "hand": True, "face": True}})
        self.assertFalse(out["skipped"])
        self.assertIn("pose", out["results"])
        # No HEF on typical host: skipped. With HEF + matching hailortcli: ok probe.
        pose = out["results"]["pose"]
        self.assertTrue(pose.get("skipped") or pose.get("ok"))
        self.assertIn("hand", out["results"])
        self.assertIn("face", out["results"])
        self.assertIn("pose", out["mqtt_topics"])

    def test_mqtt_topics(self):
        self.assertEqual(mqtt_topics.topic("pose"), "ugv/hailo/pose")
        self.assertEqual(mqtt_topics.topic("status", "robot/x"), "robot/x/status")

    def test_status_payload(self):
        with mock.patch("hailo_features.detect.device_present", return_value=True):
            st = hailo_features.status_payload({"hailo": {"enabled": "auto"}})
        self.assertTrue(st["active"])
        self.assertTrue(st["device_present"])


if __name__ == "__main__":
    unittest.main()