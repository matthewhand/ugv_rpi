#!/usr/bin/env python3
"""USB capture settings (no hardware required)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv_ctrl  # noqa: E402


class UsbCaptureSettings(unittest.TestCase):
    def test_empty_defaults_5mp_mjpg(self):
        w, h, fps, fourcc = cv_ctrl.usb_capture_settings({})
        self.assertEqual(w, 2592)
        self.assertEqual(h, 1944)
        self.assertEqual(fourcc, 'MJPG')
        self.assertGreater(fps, 0)

    def test_reads_config_vga(self):
        w, h, fps, fourcc = cv_ctrl.usb_capture_settings({
            'default_res_w': 640,
            'default_res_h': 480,
            'fourcc': 'mjpg',
            'default_fps': 15,
        })
        self.assertEqual((w, h, int(fps), fourcc), (640, 480, 15, 'MJPG'))

    def test_bad_fourcc_falls_back_mjpg(self):
        _, _, _, fourcc = cv_ctrl.usb_capture_settings({'fourcc': 'JPG'})
        self.assertEqual(fourcc, 'MJPG')

    def test_fourcc_to_str_mjpg(self):
        packed = ord('M') | (ord('J') << 8) | (ord('P') << 16) | (ord('G') << 24)
        self.assertEqual(cv_ctrl.fourcc_to_str(packed), 'MJPG')


if __name__ == '__main__':
    unittest.main()
