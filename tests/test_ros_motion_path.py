"""Unit tests for motion path selection and chassis-driver PID parsing."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ros_motion import (  # noqa: E402
    parse_chassis_driver_pids,
    parse_ugv_bringup_pids,
    preferred_motion_path,
)


class TestPreferredMotionPath(unittest.TestCase):
    def test_direct_always_serial(self):
        self.assertEqual(preferred_motion_path('direct', True), 'direct')
        self.assertEqual(preferred_motion_path('direct', False), 'direct')
        self.assertEqual(preferred_motion_path('serial', True), 'direct')

    def test_ros2_with_bridge_uses_ros2(self):
        self.assertEqual(preferred_motion_path('ros2', True), 'ros2')

    def test_ros2_without_bridge_falls_back_to_serial(self):
        # UART released + rosbridge down → must not stay on ros2-only
        self.assertEqual(preferred_motion_path('ros2', False), 'serial_fallback')

    def test_empty_mode_defaults_direct(self):
        self.assertEqual(preferred_motion_path('', False), 'direct')
        self.assertEqual(preferred_motion_path(None, True), 'direct')

    def test_autoheal_policy_matches_path_helper(self):
        """Down bridge ⇒ serial path until bridge returns."""
        self.assertEqual(preferred_motion_path('ros2', True), 'ros2')
        self.assertEqual(preferred_motion_path('ros2', False), 'serial_fallback')


class TestParseBringupPids(unittest.TestCase):
    def test_extracts_real_binary_skips_wrappers(self):
        ps = (
            "  11 bash -lc pgrep -af ugv_bringup\n"
            "  42 /opt/ros/humble/lib/ugv_bringup/ugv_bringup --ros-args -p serial_port:=/dev/ttyAMA0\n"
            "  99 grep -F ugv_bringup\n"
            " 101 docker exec ugv_ros2 bash -lc 'pkill -f ugv_bringup'\n"
        )
        self.assertEqual(parse_ugv_bringup_pids(ps), [42])

    def test_empty_and_none(self):
        self.assertEqual(parse_ugv_bringup_pids(''), [])
        self.assertEqual(parse_ugv_bringup_pids(None), [])

    def test_dedupes(self):
        ps = "7 ros2 run ugv_bringup ugv_bringup\n7 ros2 run ugv_bringup ugv_bringup\n"
        self.assertEqual(parse_ugv_bringup_pids(ps), [7])


class TestParseChassisDriverPids(unittest.TestCase):
    def test_ugv_driver_min_and_not_roarm(self):
        ps = (
            "  11 bash -lc pgrep -af ugv_driver_min\n"
            "  55 python3 /opt/ugv_ros2/ugv_driver_min.py\n"
            "  66 python3 /opt/ugv_ros2/roarm_driver_min.py\n"
            "  99 grep -E ugv_driver_min\n"
        )
        self.assertEqual(parse_chassis_driver_pids(ps), [55])

    def test_both_bringup_and_driver_min(self):
        ps = (
            "  42 /opt/ros/humble/lib/ugv_bringup/ugv_bringup --ros-args\n"
            "  55 python3 /opt/ugv_ros2/ugv_driver_min.py\n"
        )
        self.assertEqual(parse_chassis_driver_pids(ps), [42, 55])

    def test_empty(self):
        self.assertEqual(parse_chassis_driver_pids(''), [])
        self.assertEqual(parse_chassis_driver_pids(None), [])


class TestAppRos2ParitySource(unittest.TestCase):
    """Do not import app.py (opens UART). Check the ported lifecycle is present."""

    @classmethod
    def setUpClass(cls):
        cls.app_src = (ROOT / 'app.py').read_text(encoding='utf-8')
        cls.entry = (ROOT / 'ros2' / 'entrypoint.sh').read_text(encoding='utf-8')

    def test_helpers_exist(self):
        for name in (
            'def _rosbridge_is_up(',
            'def _ensure_rosbridge_running(',
            'def _ensure_ugv_bringup_running(',
            'def _stop_ugv_bringup(',
            'def _ros2_autoheal_tick(',
            'def _ensure_flask_serial(',
            'UGV_ROS_AUTOHEAL',
        ):
            self.assertIn(name, self.app_src, name)

    def test_prefers_ugv_driver_min(self):
        self.assertIn('ugv_driver_min', self.app_src)
        self.assertIn('def _probe_chassis_driver_kind(', self.app_src)
        # Must not require the full ugv_ws workspace image
        self.assertNotIn('ugv_ws-ugv_ros2', self.app_src)

    def test_does_not_start_roarm_from_chassis_helpers(self):
        # Chassis ensure/stop must not launch RoArm drivers
        start = self.app_src.find('def _ensure_ugv_bringup_running(')
        stop = self.app_src.find('def _stop_ugv_bringup(')
        sidecar = self.app_src.find('def _ensure_ros2_sidecar_stack(')
        self.assertGreater(start, 0)
        self.assertGreater(stop, start)
        self.assertGreater(sidecar, stop)
        chassis_block = self.app_src[start:sidecar]
        self.assertNotIn('roarm_driver', chassis_block)
        self.assertNotIn('start_roarm_driver', chassis_block)
        self.assertNotIn('ROARM_ENABLE_DRIVER=1', chassis_block)

    def test_direct_preload_comment(self):
        self.assertIn('startup Direct: rosbridge preload', self.app_src)
        self.assertIn('no chassis driver', self.app_src)

    def test_entrypoint_rosbridge_survives_driver_exit(self):
        # Liveness is rosbridge only — killing ugv_driver_min must leave :9090 up
        self.assertIn('while kill -0 "$RB_PID"', self.entry)
        self.assertNotIn('&& kill -0 "$DRV_PID"', self.entry)
        self.assertIn('Do not auto-restart ugv_driver_min', self.entry)


if __name__ == '__main__':
    unittest.main()
