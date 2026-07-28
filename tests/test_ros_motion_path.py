"""Unit tests for motion path selection (PTZ/chassis routing honesty)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ros_motion import preferred_motion_path  # noqa: E402


class TestPreferredMotionPath(unittest.TestCase):
    def test_direct_always_serial(self):
        self.assertEqual(preferred_motion_path('direct', True), 'direct')
        self.assertEqual(preferred_motion_path('direct', False), 'direct')
        self.assertEqual(preferred_motion_path('serial', True), 'direct')

    def test_ros2_with_bridge_uses_ros2(self):
        self.assertEqual(preferred_motion_path('ros2', True), 'ros2')

    def test_ros2_without_bridge_falls_back_to_serial(self):
        # This is the PTZ-dead case: UART released + rosbridge down → must not stay on ros2-only
        self.assertEqual(preferred_motion_path('ros2', False), 'serial_fallback')

    def test_empty_mode_defaults_direct(self):
        self.assertEqual(preferred_motion_path('', False), 'direct')
        self.assertEqual(preferred_motion_path(None, True), 'direct')

    def test_autoheal_policy_matches_path_helper(self):
        """Document the heal contract: down bridge ⇒ serial path until bridge returns."""
        # Healthy ROS path
        self.assertEqual(preferred_motion_path('ros2', True), 'ros2')
        # Autoheal failed / bridge still dead → UI must use serial_fallback (not drop)
        self.assertEqual(preferred_motion_path('ros2', False), 'serial_fallback')


if __name__ == '__main__':
    unittest.main()
