"""Unit tests for Seek nav-plan safety + battery gate (no Flask / camera)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from seek_nav import (  # noqa: E402
    interpret_base_voltage,
    reset_escape_cycle,
    seek_battery_block_reason,
    seek_nav_plan,
    seek_normalize_action,
    seek_normalize_distance,
    seek_normalize_obstacle_range,
    SEEK_REVERSE_MAX_MS,
    SEEK_REVERSE_MAX_LIN,
)


class TestNormalize(unittest.TestCase):
    def test_action_aliases(self):
        self.assertEqual(seek_normalize_action('ahead'), 'forward')
        self.assertEqual(seek_normalize_action('left'), 'turn_left')
        self.assertEqual(seek_normalize_action('RIGHT'), 'turn_right')
        self.assertEqual(seek_normalize_action('reverse'), 'backward')

    def test_distance_aliases(self):
        self.assertEqual(seek_normalize_distance('near'), 'short')
        self.assertEqual(seek_normalize_distance('mid'), 'medium')
        self.assertEqual(seek_normalize_distance('far'), 'long')
        self.assertEqual(seek_normalize_distance(None), 'medium')

    def test_obstacle_aliases_and_metres(self):
        self.assertEqual(seek_normalize_obstacle_range(True), 'near')
        self.assertEqual(seek_normalize_obstacle_range('clear'), 'none')
        self.assertEqual(seek_normalize_obstacle_range('0.4m'), 'near')
        self.assertEqual(seek_normalize_obstacle_range('2 m'), 'far')
        self.assertEqual(seek_normalize_obstacle_range('5m'), 'none')


class TestNavPlanSafety(unittest.TestCase):
    def setUp(self):
        reset_escape_cycle()

    def test_near_obstacle_never_forwards(self):
        plan = seek_nav_plan('forward', 'long', obstacle_range='near')
        self.assertNotEqual(plan['action'], 'forward')
        self.assertIn(plan['action'], ('turn_left', 'turn_right', 'backward'))
        self.assertEqual(plan['drive_distance'], 'short')
        self.assertIsNotNone(plan['safety_override'])

    def test_path_clear_false_treated_as_near(self):
        plan = seek_nav_plan('forward', 'long', path_clear_forward=False)
        self.assertNotEqual(plan['action'], 'forward')
        self.assertEqual(plan['obstacle_range'], 'near')

    def test_reverse_capped_to_short(self):
        plan = seek_nav_plan('backward', 'long', obstacle_range='none')
        self.assertEqual(plan['action'], 'backward')
        self.assertEqual(plan['drive_distance'], 'short')
        self.assertLessEqual(plan['duration_ms'], SEEK_REVERSE_MAX_MS)
        self.assertLessEqual(abs(plan['linear_x']), SEEK_REVERSE_MAX_LIN)
        self.assertLess(plan['linear_x'], 0)
        self.assertIn('capped', (plan['safety_override'] or '').lower())

    def test_no_double_reverse(self):
        plan = seek_nav_plan(
            'backward', 'short',
            last_action='backward',
            obstacle_range='near',
            prefer_turn='turn_right',
        )
        self.assertNotEqual(plan['action'], 'backward')
        self.assertEqual(plan['action'], 'turn_right')
        self.assertEqual(plan['drive_distance'], 'short')

    def test_stuck_prefers_turn_not_reverse(self):
        plan = seek_nav_plan('forward', 'medium', stuck=True, prefer_turn='turn_left')
        self.assertEqual(plan['action'], 'turn_left')
        self.assertEqual(plan['drive_distance'], 'short')

    def test_stuck_after_turn_allows_one_short_reverse(self):
        plan = seek_nav_plan(
            'forward', 'medium',
            stuck=True,
            last_action='turn_left',
        )
        self.assertEqual(plan['action'], 'backward')
        self.assertEqual(plan['drive_distance'], 'short')
        self.assertLessEqual(plan['duration_ms'], SEEK_REVERSE_MAX_MS)

    def test_open_path_upgrades_short_to_long(self):
        plan = seek_nav_plan('forward', 'short', obstacle_range='none')
        self.assertEqual(plan['action'], 'forward')
        self.assertEqual(plan['drive_distance'], 'long')

    def test_mid_range_caps_long_forward(self):
        plan = seek_nav_plan('forward', 'long', obstacle_range='medium')
        self.assertEqual(plan['action'], 'forward')
        self.assertEqual(plan['drive_distance'], 'medium')

    def test_clear_forward_unimpeded(self):
        plan = seek_nav_plan(
            'forward', 'medium',
            path_clear_forward=True,
            obstacle_range='none',
        )
        self.assertEqual(plan['action'], 'forward')
        self.assertGreater(plan['linear_x'], 0)
        self.assertGreater(plan['duration_ms'], 0)


class TestBatteryGate(unittest.TestCase):
    def test_unknown_voltage_does_not_block(self):
        self.assertIsNone(interpret_base_voltage(None))
        self.assertIsNone(interpret_base_voltage(48))  # ADC-ish
        self.assertIsNone(interpret_base_voltage(-1))
        self.assertIsNone(seek_battery_block_reason(None, low_v=9.5))

    def test_plausible_volts(self):
        self.assertEqual(interpret_base_voltage(11.2), 11.2)
        self.assertEqual(interpret_base_voltage('10.1'), 10.1)

    def test_low_voltage_blocks(self):
        reason = seek_battery_block_reason(9.1, low_v=9.5)
        self.assertIsNotNone(reason)
        self.assertIn('9.10', reason)
        self.assertIn('UGV_SEEK_BATTERY_GATE=0', reason)

    def test_healthy_voltage_allows(self):
        self.assertIsNone(seek_battery_block_reason(11.4, low_v=9.5))

    def test_gate_override(self):
        self.assertIsNone(
            seek_battery_block_reason(8.0, low_v=9.5, gate_enabled=False)
        )


class TestNoDuplicateNavPlan(unittest.TestCase):
    def test_app_imports_seek_nav_plan(self):
        text = (ROOT / 'app.py').read_text(encoding='utf-8')
        self.assertNotIn('def _seek_nav_plan(', text)
        self.assertIn('seek_nav_plan as _seek_nav_plan', text)
        self.assertIn('def _stop_ugv_bringup(', text)


if __name__ == '__main__':
    unittest.main()
