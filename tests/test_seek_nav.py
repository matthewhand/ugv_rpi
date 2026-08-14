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
    SEEK_DRIVE_LIN_BY_DIST,
    SEEK_TURN_MS_BY_DIST,
    seek_commit_through_opening,
    seek_prefer_away_from_wall,
    seek_may_reverse,
    seek_hop_from_forward_cm,
    seek_action_from_schema,
    set_seek_dry_run,
    seek_dry_run_active,
    seek_chassis_allowed,
    seek_drive_log_verb,
    seek_sweep_scorecard,
    seek_live_start_error,
    seek_views_are_rear_cruise,
    seek_found_confident,
    begin_seek_dry_run,
    end_seek_dry_run,
    chassis_serial_allowed,
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

    def test_chassis_forward_is_punchy_not_creep(self):
        """Cable runner / carpet: 0.12 hops stall; live punch was ~0.26 / 1.1s."""
        plan = seek_nav_plan('forward', 'medium', obstacle_range='none')
        self.assertGreaterEqual(plan['linear_x'], 0.24)
        self.assertLessEqual(plan['linear_x'], 0.35)
        self.assertGreaterEqual(plan['duration_ms'], 800)
        self.assertLessEqual(plan['duration_ms'], 1800)
        self.assertGreaterEqual(SEEK_DRIVE_LIN_BY_DIST['short'], 0.20)

    def test_chassis_turns_are_short_fast_spins(self):
        """UI-Fast T:1 — 2–7s at full speed over-rotates; live 700ms was a room turn."""
        short = seek_nav_plan('turn_left', 'short')
        medium = seek_nav_plan('turn_right', 'medium')
        long = seek_nav_plan('turn_left', 'long')
        self.assertEqual(SEEK_TURN_MS_BY_DIST['medium'], 700)
        self.assertGreaterEqual(short['duration_ms'], 250)
        self.assertLessEqual(short['duration_ms'], 450)
        self.assertGreaterEqual(medium['duration_ms'], 600)
        self.assertLessEqual(medium['duration_ms'], 850)
        self.assertLessEqual(long['duration_ms'], 1300)


class TestHouseNavRules(unittest.TestCase):
    def test_doorway_commit_after_forward_in_chute(self):
        c = seek_commit_through_opening(
            'forward', 'medium',
            obstacle_range='none',
            left_open=0.2, right_open=0.15, centre_open=True,
        )
        self.assertIsNotNone(c)
        self.assertEqual(c['action'], 'forward')
        self.assertEqual(c['drive_distance'], 'long')
        self.assertIn('jamb', c['reason'].lower())

    def test_no_commit_when_centre_blocked(self):
        self.assertIsNone(seek_commit_through_opening(
            'forward', 'medium',
            obstacle_range='near',
            left_open=0.1, right_open=0.1, centre_open=False,
        ))

    def test_no_commit_unless_last_was_forward(self):
        self.assertIsNone(seek_commit_through_opening(
            'turn_left', 'short',
            obstacle_range='none',
            left_open=0.1, right_open=0.1, centre_open=True,
        ))

    def test_turn_away_from_near_wall(self):
        self.assertEqual(
            seek_prefer_away_from_wall(
                'forward', obstacle_range='near',
                left_open=0.8, right_open=0.2,
            ),
            'turn_left',
        )
        self.assertIsNone(seek_prefer_away_from_wall(
            'forward', obstacle_range='none', left_open=0.2, right_open=0.2,
        ))

    def test_reverse_requires_both_rear_quarters(self):
        self.assertFalse(seek_may_reverse(
            can_forward=True, can_turn=False,
            rear_left_clear=True, rear_right_clear=True,
        ))
        self.assertFalse(seek_may_reverse(
            can_forward=False, can_turn=True,
            rear_left_clear=True, rear_right_clear=True,
        ))
        self.assertFalse(seek_may_reverse(
            can_forward=False, can_turn=False,
            rear_left_clear=True, rear_right_clear=False,
        ))
        self.assertFalse(seek_may_reverse(
            can_forward=False, can_turn=False,
            rear_left_clear=True, rear_right_clear=True,
            last_action='backward',
        ))
        self.assertTrue(seek_may_reverse(
            can_forward=False, can_turn=False,
            rear_left_clear=True, rear_right_clear=True,
            last_action='turn_left',
        ))


class TestSeekNavSchema(unittest.TestCase):
    def test_cm_buckets(self):
        self.assertEqual(seek_hop_from_forward_cm(0), 'none')
        self.assertEqual(seek_hop_from_forward_cm(15), 'short')
        self.assertEqual(seek_hop_from_forward_cm(30), 'short')
        self.assertEqual(seek_hop_from_forward_cm(60), 'medium')
        self.assertEqual(seek_hop_from_forward_cm(100), 'long')

    def test_schema_forward_long(self):
        m = seek_action_from_schema({
            'can_forward': True, 'forward_clear_cm': 200, 'forward_hop': 'long',
            'can_turn_left': True, 'can_turn_right': True,
            'rear_left_clear': True, 'rear_right_clear': True,
            'can_backward': False, 'backward_hop': 'none',
        })
        self.assertEqual(m['action'], 'forward')
        self.assertEqual(m['drive_distance'], 'long')

    def test_schema_turn_when_blocked(self):
        m = seek_action_from_schema({
            'can_forward': False, 'forward_clear_cm': 0, 'forward_hop': 'none',
            'can_turn_left': False, 'can_turn_right': True,
            'rear_left_clear': False, 'rear_right_clear': True,
            'can_backward': False, 'backward_hop': 'none',
        })
        self.assertEqual(m['action'], 'turn_right')

    def test_schema_reverse_needs_both_rear(self):
        blocked = {
            'can_forward': False, 'forward_clear_cm': 0, 'forward_hop': 'none',
            'can_turn_left': False, 'can_turn_right': False,
            'can_backward': True, 'backward_hop': 'short',
        }
        no = seek_action_from_schema({**blocked, 'rear_left_clear': True, 'rear_right_clear': False})
        self.assertNotEqual(no['action'], 'backward')
        yes = seek_action_from_schema({**blocked, 'rear_left_clear': True, 'rear_right_clear': True})
        self.assertEqual(yes['action'], 'backward')
        self.assertEqual(yes['drive_distance'], 'short')

    def test_forward_hop_none_is_not_a_drive(self):
        m = seek_action_from_schema({
            'can_forward': True, 'forward_clear_cm': 0, 'forward_hop': 'none',
            'can_turn_left': True, 'can_turn_right': False,
            'rear_left_clear': False, 'rear_right_clear': False,
            'can_backward': False, 'backward_hop': 'none',
        })
        self.assertEqual(m['action'], 'turn_left')


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


class TestDryRunAndSweep(unittest.TestCase):
    def tearDown(self):
        set_seek_dry_run(False)

    def test_latch_blocks_chassis(self):
        self.assertTrue(seek_chassis_allowed())
        set_seek_dry_run(True)
        self.assertTrue(seek_dry_run_active())
        self.assertFalse(seek_chassis_allowed())
        self.assertEqual(seek_drive_log_verb(), 'WOULD drive')
        set_seek_dry_run(False)
        self.assertTrue(seek_chassis_allowed())
        self.assertEqual(seek_drive_log_verb(), 'driving')

    def test_explicit_flag_overrides_latch(self):
        set_seek_dry_run(False)
        self.assertFalse(seek_chassis_allowed(dry_run=True))
        set_seek_dry_run(True)
        self.assertTrue(seek_chassis_allowed(dry_run=False))

    def test_scorecard_ok_and_missing(self):
        jpeg = b'\xff\xd8' + (b'x' * 1200)
        views = [
            {'name': 'left', 'jpeg': jpeg, 'bytes': len(jpeg), 'pan_settled': True, 'detected_labels': ['person']},
            {'name': 'straight', 'jpeg': jpeg, 'bytes': len(jpeg), 'pan_settled': True},
            {'name': 'right', 'jpeg': jpeg, 'bytes': len(jpeg), 'pan_settled': False},
        ]
        ok = seek_sweep_scorecard(views)
        self.assertTrue(ok['ok'])
        self.assertEqual(ok['n'], 3)
        self.assertEqual(ok['missing'], 0)
        weak = seek_sweep_scorecard([
            {'name': 'left', 'jpeg': b'', 'bytes': 0},
            {'name': 'straight', 'jpeg': jpeg, 'bytes': len(jpeg)},
            {'name': 'right', 'jpeg': None, 'bytes': 12},
        ])
        self.assertFalse(weak['ok'])
        self.assertEqual(weak['missing'], 2)
        self.assertIn('WEAK', weak['summary'])

    def test_live_start_requires_confirm(self):
        self.assertIsNone(seek_live_start_error(dry_run=True, confirm_live=False))
        self.assertIsNotNone(seek_live_start_error(dry_run=False, confirm_live=False))
        self.assertIsNone(seek_live_start_error(dry_run=False, confirm_live=True))

    def test_rear_cruise_vs_lookdown(self):
        cruise = [
            {'name': 'left', 'pan_deg': -135},
            {'name': 'straight', 'pan_deg': 0},
            {'name': 'right', 'pan_deg': 135},
        ]
        lookdown = [
            {'name': 'left', 'pan_deg': -55},
            {'name': 'straight', 'pan_deg': 0},
            {'name': 'right', 'pan_deg': 55},
        ]
        self.assertTrue(seek_views_are_rear_cruise(cruise))
        self.assertFalse(seek_views_are_rear_cruise(lookdown))

    def test_found_confident_thresholds(self):
        weak = {'found': True, 'best': {'confidence': 0.30}}
        self.assertFalse(seek_found_confident(weak, view_hits=1)['ok'])
        self.assertTrue(seek_found_confident(weak, view_hits=2, scan_conf=0.22)['ok'])
        strong = {'found': True, 'best': {'confidence': 0.70}}
        self.assertTrue(seek_found_confident(strong, view_hits=1)['ok'])
        self.assertFalse(seek_found_confident({'found': False}, view_hits=2)['ok'])

    def test_dry_run_generation_does_not_unlatch_newer(self):
        g1 = begin_seek_dry_run()
        g2 = begin_seek_dry_run()
        self.assertTrue(seek_dry_run_active())
        self.assertFalse(end_seek_dry_run(g1))
        self.assertTrue(seek_dry_run_active())
        self.assertTrue(end_seek_dry_run(g2))
        self.assertFalse(seek_dry_run_active())

    def test_uart_guard_blocks_nonzero_in_dry_run(self):
        set_seek_dry_run(True)
        try:
            self.assertFalse(chassis_serial_allowed({'T': 1, 'L': 1.0, 'R': -1.0}))
            self.assertFalse(chassis_serial_allowed({'T': 13, 'X': 0.2, 'Z': 0}))
            self.assertTrue(chassis_serial_allowed({'T': 1, 'L': 0, 'R': 0}))
            self.assertTrue(chassis_serial_allowed({'T': 133, 'X': 20, 'Y': 0}))
        finally:
            set_seek_dry_run(False)


if __name__ == '__main__':
    unittest.main()
