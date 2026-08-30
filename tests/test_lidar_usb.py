#!/usr/bin/env python3
"""USB lidar port selection and LD19 frame parse (no hardware required)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import base_ctrl  # noqa: E402


LIDAR_BY_ID = (
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)
ROARM_BY_ID = (
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_ABC-if00-port0"
)


def _glob_map(mapping):
    def globber(pat):
        if pat in mapping:
            return list(mapping[pat])
        for key, vals in mapping.items():
            if key.endswith("*") and pat.startswith(key[:-1]):
                # not used
                pass
        if pat == "/dev/ttyACM*":
            return list(mapping.get("/dev/ttyACM*", []))
        if pat == "/dev/ttyUSB*":
            return list(mapping.get("/dev/ttyUSB*", []))
        if "CP2102N" in pat:
            return list(mapping.get("roarm", []))
        if "CP2102_USB_to_UART" in pat:
            return list(mapping.get("lidar_by_id", []))
        return []

    return globber


class TestLidarPortCandidates(unittest.TestCase):
    def test_prefers_cp2102_not_cp2102n(self):
        globber = _glob_map(
            {
                "/dev/ttyACM*": [],
                "/dev/ttyUSB*": ["/dev/ttyUSB0", "/dev/ttyUSB1"],
                "lidar_by_id": [LIDAR_BY_ID],
                "roarm": [ROARM_BY_ID],
            }
        )
        real = {
            LIDAR_BY_ID: "/dev/ttyUSB0",
            ROARM_BY_ID: "/dev/ttyUSB1",
            "/dev/ttyUSB0": "/dev/ttyUSB0",
            "/dev/ttyUSB1": "/dev/ttyUSB1",
        }
        exists = lambda p: p in real or p in real.values()
        ports = base_ctrl.lidar_port_candidates(
            env=None,
            globber=globber,
            exists=exists,
            realpath=lambda p: real.get(p, p),
        )
        self.assertEqual(ports, ["/dev/ttyUSB0"])
        self.assertNotIn("/dev/ttyUSB1", ports)

    def test_acm_before_usb(self):
        globber = _glob_map(
            {
                "/dev/ttyACM*": ["/dev/ttyACM0"],
                "/dev/ttyUSB*": ["/dev/ttyUSB0"],
                "lidar_by_id": [LIDAR_BY_ID],
                "roarm": [],
            }
        )
        real = {
            "/dev/ttyACM0": "/dev/ttyACM0",
            LIDAR_BY_ID: "/dev/ttyUSB0",
            "/dev/ttyUSB0": "/dev/ttyUSB0",
        }
        ports = base_ctrl.lidar_port_candidates(
            env=None,
            globber=globber,
            exists=lambda p: p in real or p in set(real.values()),
            realpath=lambda p: real.get(p, p),
        )
        self.assertEqual(ports[0], "/dev/ttyACM0")
        self.assertIn("/dev/ttyUSB0", ports)

    def test_env_override_wins(self):
        globber = _glob_map(
            {
                "/dev/ttyACM*": ["/dev/ttyACM0"],
                "/dev/ttyUSB*": ["/dev/ttyUSB0"],
                "lidar_by_id": [],
                "roarm": [],
            }
        )
        real = {
            "/dev/ttyACM0": "/dev/ttyACM0",
            "/dev/ttyUSB0": "/dev/ttyUSB0",
            "/dev/ttyUSB9": "/dev/ttyUSB9",
        }
        ports = base_ctrl.lidar_port_candidates(
            env="/dev/ttyUSB9",
            globber=globber,
            exists=lambda p: p in real,
            realpath=lambda p: real.get(p, p),
        )
        self.assertEqual(ports[0], "/dev/ttyUSB9")


class TestLd19Parse(unittest.TestCase):
    def test_parse_distances_and_start_angle(self):
        frame = bytearray(47)
        frame[0] = 0x54
        frame[1] = 0x2C
        # 90.00 deg = 9000 = 0x2328 little-endian
        frame[4] = 0x28
        frame[5] = 0x23
        # end 100.00 deg = 10000 = 0x2710
        frame[42] = 0x10
        frame[43] = 0x27
        for i in range(12):
            dist = 1000 + i
            frame[6 + i * 3] = dist & 0xFF
            frame[7 + i * 3] = (dist >> 8) & 0xFF
            frame[8 + i * 3] = 180
        parsed = base_ctrl.parse_ld19_frame(frame)
        self.assertIsNotNone(parsed)
        start, end, dists, intens = parsed
        self.assertAlmostEqual(start, 90.0, places=2)
        self.assertAlmostEqual(end, 100.0, places=2)
        self.assertEqual(dists[0], 1000)
        self.assertEqual(dists[-1], 1011)
        self.assertEqual(intens[0], 180)

    def test_reject_short_and_bad_header(self):
        self.assertIsNone(base_ctrl.parse_ld19_frame(b"\x54\x2c"))
        bad = bytearray(47)
        bad[0] = 0x00
        self.assertIsNone(base_ctrl.parse_ld19_frame(bad))


class TestAppLidarRoutePresent(unittest.TestCase):
    def test_api_lidar_route_exists(self):
        src = Path(ROOT, "app.py").read_text(encoding="utf-8")
        self.assertIn("@app.route('/api/lidar'", src)
        self.assertIn("base.set_use_lidar", src)
        self.assertIn("lidar_port_candidates", Path(ROOT, "base_ctrl.py").read_text(encoding="utf-8"))

    def test_lidar_public_exposes_detected(self):
        src = Path(ROOT, "app.py").read_text(encoding="utf-8")
        idx = src.find("def _lidar_public")
        self.assertGreater(idx, 0)
        body = src[idx : idx + 1600]
        self.assertIn("'detected'", body)
        self.assertIn("lidar_port_candidates", body)

    def test_twin_payload_includes_lidar(self):
        src = Path(ROOT, "app.py").read_text(encoding="utf-8")
        idx = src.find("def api_twin")
        self.assertGreater(idx, 0)
        body = src[idx : idx + 2200]
        self.assertIn("_lidar_public", body)
        self.assertIn("'lidar'", body)

    def test_ui_has_lidar_chip_and_hangar_toggle(self):
        index = Path(ROOT, "templates", "index.html").read_text(encoding="utf-8")
        self.assertIn("lidar-toggle-btn", index)
        self.assertIn("loadout-lidar-btn", index)
        self.assertIn("toggleLidar()", index)
        twin_html = Path(ROOT, "templates", "3d_twin.html").read_text(encoding="utf-8")
        self.assertIn("hud-lidar", twin_html)
        self.assertIn("lidar-radar", twin_html)
        twin_js = Path(ROOT, "templates", "twin.js").read_text(encoding="utf-8")
        self.assertIn("applyLidarScan", twin_js)
        self.assertIn("drawLidarRadar", twin_js)


if __name__ == "__main__":
    unittest.main()
