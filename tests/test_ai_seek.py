"""Unit tests for Seek helpers (goal parse, OpenCV oracle, controller bounds)."""
from __future__ import annotations

import sys
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ai_seek import (  # noqa: E402
    SeekController,
    evaluate_goal_detections,
    enrich_detections,
    parse_seek_goal,
    parse_llm_goal,
    parse_llm_found_payload,
    parse_seek_referee,
    parse_on_found,
    format_on_found_tts,
    detector_labels,
    normalize_seek_max_steps,
    normalize_seek_timeout_s,
    motion_lock_should_ignore_zero,
    DEFAULT_SEEK_MAX_STEPS,
    DEFAULT_SEEK_TIMEOUT_S,
    DEFAULT_SEEK_DRY_RUN,
    SEEK_MAX_STEPS_CAP,
    SEEK_TIMEOUT_S_MIN,
    SEEK_TIMEOUT_S_CAP,
    REFEREE_DETECTOR,
    REFEREE_LLM,
    ON_FOUND_NONE,
    ON_FOUND_TTS,
)


class TestParseSeekGoal(unittest.TestCase):
    def test_exact_label(self):
        lab, err = parse_seek_goal('dog')
        self.assertEqual(lab, 'dog')
        self.assertIsNone(err)

    def test_alias(self):
        lab, err = parse_seek_goal('puppy')
        self.assertEqual(lab, 'dog')
        self.assertIsNone(err)

    def test_phrase_with_token(self):
        lab, err = parse_seek_goal('find the person please')
        self.assertEqual(lab, 'person')
        self.assertIsNone(err)

    def test_empty(self):
        lab, err = parse_seek_goal('  ')
        self.assertIsNone(lab)
        self.assertIn('empty', err or '')

    def test_unknown(self):
        lab, err = parse_seek_goal('unicorn')
        self.assertIsNone(lab)
        self.assertIn('unknown', err or '')

    def test_background_rejected(self):
        lab, err = parse_seek_goal('background')
        self.assertIsNone(lab)
        self.assertIsNotNone(err)


class TestEvaluateGoal(unittest.TestCase):
    def test_found_when_label_and_conf_match(self):
        dets = [
            {'label': 'boat', 'confidence': 0.9, 'bbox_norm': [0.1, 0.1, 0.2, 0.2]},
            {'label': 'dog', 'confidence': 0.55, 'bbox_norm': [0.3, 0.3, 0.6, 0.7]},
        ]
        r = evaluate_goal_detections(dets, 'dog', conf_threshold=0.22)
        self.assertTrue(r['found'])
        self.assertEqual(r['match_count'], 1)
        self.assertEqual(r['best']['label'], 'dog')
        self.assertIn('dog', r['labels_found'])

    def test_not_found_below_conf(self):
        dets = [{'label': 'dog', 'confidence': 0.1, 'bbox_norm': [0, 0, 1, 1]}]
        r = evaluate_goal_detections(dets, 'dog', conf_threshold=0.5)
        self.assertFalse(r['found'])
        self.assertEqual(r['match_count'], 0)

    def test_wrong_label(self):
        dets = [{'label': 'cat', 'confidence': 0.9, 'bbox_norm': [0, 0, 1, 1]}]
        r = evaluate_goal_detections(dets, 'dog', conf_threshold=0.2)
        self.assertFalse(r['found'])

    def test_enrich_centers(self):
        dets = [{'label': 'person', 'confidence': 0.8, 'bbox_norm': [0.0, 0.0, 1.0, 1.0]}]
        out = enrich_detections(dets, filter_label='person')
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(out[0]['center_x'], 0.5)
        self.assertAlmostEqual(out[0]['offset_x'], 0.0)


class TestRefereeAndLlmParse(unittest.TestCase):
    def test_detector_labels_no_background(self):
        labs = detector_labels()
        self.assertIn('dog', labs)
        self.assertNotIn('background', labs)

    def test_parse_referee(self):
        self.assertEqual(parse_seek_referee('opencv'), REFEREE_DETECTOR)
        self.assertEqual(parse_seek_referee('llm'), REFEREE_LLM)
        self.assertEqual(parse_seek_referee('vision'), REFEREE_LLM)

    def test_on_found_parse_and_tts(self):
        self.assertEqual(parse_on_found('none'), ON_FOUND_NONE)
        self.assertEqual(parse_on_found('tts'), ON_FOUND_TTS)
        self.assertEqual(parse_on_found('announce'), ON_FOUND_TTS)
        self.assertEqual(
            format_on_found_tts('I have found the {goal}.', 'dog'),
            'I have found the dog.',
        )

    def test_llm_goal_free_text(self):
        lab, err = parse_llm_goal('red fire extinguisher')
        self.assertEqual(lab, 'red fire extinguisher')
        self.assertIsNone(err)
        lab, err = parse_llm_goal('  ')
        self.assertIsNone(lab)
        self.assertIsNotNone(err)

    def test_parse_found_json_object(self):
        r = parse_llm_found_payload('{"found": true, "reason": "dog in view"}')
        self.assertTrue(r['found'])
        self.assertTrue(r['parse_ok'])
        self.assertIn('dog', r['reason'])

    def test_parse_found_false_and_garbage(self):
        r = parse_llm_found_payload('{"found": false, "reason": "empty hallway"}')
        self.assertFalse(r['found'])
        g = parse_llm_found_payload('I think maybe yes there is something')
        self.assertFalse(g['found'])
        self.assertFalse(g['parse_ok'])

    def test_parse_found_dict(self):
        r = parse_llm_found_payload({'found': True, 'reason': 'ok'})
        self.assertTrue(r['found'])


class TestSeekController(unittest.TestCase):
    def test_bad_goal_rejected(self):
        ctrl = SeekController()
        r = ctrl.start('notaclass', loop_fn=lambda *a: None)
        self.assertFalse(r['success'])
        self.assertIn('unknown', r.get('error', ''))

    def test_llm_referee_accepts_free_text(self):
        ctrl = SeekController()

        def loop(c, label, conf, max_steps, timeout_s):
            c.finish('timeout', message='done', step=0)

        r = ctrl.start(
            'yellow sticky note on the wall',
            loop_fn=loop,
            referee=REFEREE_LLM,
            max_steps=1,
            timeout_s=5,
        )
        self.assertTrue(r['success'])
        self.assertEqual(r['status']['referee'], REFEREE_LLM)
        deadline = time.time() + 2.0
        while time.time() < deadline and ctrl.is_running():
            time.sleep(0.02)

    def test_stop_cancels_running(self):
        ctrl = SeekController()

        def loop(c, label, conf, max_steps, timeout_s):
            for i in range(50):
                if c.should_stop():
                    c.finish('stopped', message='stopped', step=i)
                    return
                c.update(step=i + 1, message=f'step {i}')
                time.sleep(0.05)
            c.finish('timeout', message='loop end', step=50)

        r = ctrl.start('dog', loop_fn=loop, max_steps=50, timeout_s=30)
        self.assertTrue(r['success'])
        self.assertEqual(r['status']['phase'], 'running')
        time.sleep(0.08)
        ctrl.stop()
        # Wait for thread to notice cancel
        deadline = time.time() + 3.0
        while time.time() < deadline:
            st = ctrl.status()
            if st['phase'] != 'running':
                break
            time.sleep(0.05)
        st = ctrl.status()
        self.assertEqual(st['phase'], 'stopped')

    def test_rejects_second_start(self):
        ctrl = SeekController()

        def loop(c, label, conf, max_steps, timeout_s):
            time.sleep(0.4)
            c.finish('timeout', message='done', step=1)

        r1 = ctrl.start('person', loop_fn=loop, max_steps=2, timeout_s=10)
        self.assertTrue(r1['success'])
        r2 = ctrl.start('dog', loop_fn=loop)
        self.assertFalse(r2['success'])
        self.assertIn('already', r2.get('error', ''))
        deadline = time.time() + 3.0
        while time.time() < deadline and ctrl.is_running():
            time.sleep(0.05)

    def test_found_only_when_opencv_matches(self):
        """found phase is set only after evaluate_goal_detections says found."""
        ctrl = SeekController()

        def loop(c, label, conf, max_steps, timeout_s):
            # Miss first
            miss = evaluate_goal_detections(
                [{'label': 'boat', 'confidence': 0.9, 'bbox_norm': [0, 0, 0.2, 0.2]}],
                label,
                conf,
            )
            c.update(step=1, last_detection=miss)
            self.assertFalse(miss['found'])
            # Hit
            hit = evaluate_goal_detections(
                [{'label': label, 'confidence': 0.8, 'bbox_norm': [0.2, 0.2, 0.6, 0.7]}],
                label,
                conf,
            )
            self.assertTrue(hit['found'])
            c.finish('found', message=f'Found {label}', step=2, last_detection=hit)

        r = ctrl.start('dog', loop_fn=loop, max_steps=4, timeout_s=10)
        self.assertTrue(r['success'])
        deadline = time.time() + 2.0
        while time.time() < deadline and ctrl.is_running():
            time.sleep(0.02)
        st = ctrl.status()
        self.assertEqual(st['phase'], 'found')
        self.assertTrue(st['last_detection']['found'])
        self.assertEqual(st['last_detection']['goal_label'], 'dog')


class TestSeekLimitDefaults(unittest.TestCase):
    """Finite pilot defaults + explicit unlimited override (ROI pack)."""

    def test_defaults_are_finite_and_nonzero(self):
        self.assertGreater(DEFAULT_SEEK_MAX_STEPS, 0)
        self.assertGreater(DEFAULT_SEEK_TIMEOUT_S, 0)
        self.assertTrue(DEFAULT_SEEK_DRY_RUN)

    def test_start_defaults_to_dry_run(self):
        ctrl = SeekController()

        def loop(c, label, conf, max_steps, timeout_s):
            c.finish('timeout', message='done', step=0)

        r = ctrl.start('person', loop_fn=loop, max_steps=1, timeout_s=5)
        self.assertTrue(r['success'])
        self.assertTrue((r.get('status') or {}).get('dry_run'))
        deadline = time.time() + 2.0
        while time.time() < deadline and ctrl.is_running():
            time.sleep(0.02)

    def test_missing_uses_finite_default(self):
        self.assertEqual(normalize_seek_max_steps(None), DEFAULT_SEEK_MAX_STEPS)
        self.assertEqual(normalize_seek_max_steps(''), DEFAULT_SEEK_MAX_STEPS)
        self.assertEqual(normalize_seek_timeout_s(None), DEFAULT_SEEK_TIMEOUT_S)
        self.assertEqual(normalize_seek_timeout_s(''), DEFAULT_SEEK_TIMEOUT_S)

    def test_explicit_zero_is_unlimited(self):
        self.assertEqual(normalize_seek_max_steps(0), 0)
        self.assertEqual(normalize_seek_max_steps('0'), 0)
        self.assertEqual(normalize_seek_max_steps('unlimited'), 0)
        self.assertEqual(normalize_seek_timeout_s(0), 0.0)
        self.assertEqual(normalize_seek_timeout_s('inf'), 0.0)

    def test_positive_capped(self):
        self.assertEqual(normalize_seek_max_steps(12), 12)
        self.assertEqual(normalize_seek_max_steps(99999), SEEK_MAX_STEPS_CAP)
        self.assertEqual(normalize_seek_timeout_s(1), SEEK_TIMEOUT_S_MIN)
        self.assertEqual(normalize_seek_timeout_s(999999), SEEK_TIMEOUT_S_CAP)

    def test_controller_start_adopts_normalized_defaults(self):
        ctrl = SeekController()

        def loop(c, label, conf, max_steps, timeout_s):
            # Capture what the loop receives and finish immediately
            c.update(step=0, message=f'ms={max_steps},to={timeout_s}')
            c.finish('stopped', message='done', step=0)

        r = ctrl.start(
            'dog',
            loop_fn=loop,
            max_steps=normalize_seek_max_steps(None),
            timeout_s=normalize_seek_timeout_s(None),
        )
        self.assertTrue(r['success'])
        deadline = time.time() + 2.0
        while time.time() < deadline and ctrl.is_running():
            time.sleep(0.02)
        st = ctrl.status()
        self.assertEqual(st['max_steps'], DEFAULT_SEEK_MAX_STEPS)
        self.assertEqual(st['timeout_s'], DEFAULT_SEEK_TIMEOUT_S)


class TestMotionLockZeroPolicy(unittest.TestCase):
    """AI lock drops heartbeat zeros; force_stop (emergency STOP) always delivers."""

    def test_heartbeat_zero_blocked_while_locked(self):
        self.assertTrue(
            motion_lock_should_ignore_zero(True, True, force_stop=False)
        )

    def test_force_stop_never_blocked(self):
        self.assertFalse(
            motion_lock_should_ignore_zero(True, True, force_stop=True)
        )
        self.assertFalse(
            motion_lock_should_ignore_zero(True, True, force_stop=True)
        )

    def test_nonzero_never_blocked(self):
        self.assertFalse(
            motion_lock_should_ignore_zero(True, False, force_stop=False)
        )

    def test_unlocked_zeros_pass(self):
        self.assertFalse(
            motion_lock_should_ignore_zero(False, True, force_stop=False)
        )

    def test_seek_stop_clears_running_phase(self):
        """Shipped SeekController.stop() path used by emergency/seek stop APIs."""
        ctrl = SeekController()
        started = threading.Event()
        release = threading.Event()

        def loop(c, label, conf, max_steps, timeout_s):
            started.set()
            # Block until test requests stop (or timeout)
            for _ in range(200):
                if c.should_stop() or release.is_set():
                    break
                time.sleep(0.01)
            c.finish('stopped', message='stopped by test', step=0)

        r = ctrl.start('dog', loop_fn=loop, max_steps=50, timeout_s=30)
        self.assertTrue(r['success'])
        self.assertTrue(started.wait(1.0))
        self.assertTrue(ctrl.is_running())
        st = ctrl.stop()
        release.set()
        self.assertIn(st.get('phase'), ('stopped', 'running', 'idle'))
        deadline = time.time() + 2.0
        while time.time() < deadline and ctrl.is_running():
            time.sleep(0.02)
        self.assertFalse(ctrl.is_running())
        self.assertNotEqual(ctrl.status().get('phase'), 'running')


if __name__ == '__main__':
    unittest.main()
