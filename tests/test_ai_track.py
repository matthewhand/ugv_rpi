"""Unit tests for PTZ Tracking goal routing and bbox → pan/tilt."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ai_track import (  # noqa: E402
    resolve_track_goal,
    bbox_offsets,
    ptz_delta_from_offsets,
    clamp_ptz,
)
from ai_seek import REFEREE_DETECTOR, REFEREE_LLM  # noqa: E402


class TestResolveTrackGoal(unittest.TestCase):
    def test_detector_for_voc(self):
        r = resolve_track_goal('dog')
        self.assertEqual(r['goal'], 'dog')
        self.assertEqual(r['referee'], REFEREE_DETECTOR)
        self.assertIsNone(r['error'])

    def test_alias_person(self):
        r = resolve_track_goal('human')
        self.assertEqual(r['goal'], 'person')
        self.assertEqual(r['referee'], REFEREE_DETECTOR)

    def test_llm_for_unique(self):
        r = resolve_track_goal('toilet')
        self.assertEqual(r['goal'], 'toilet')
        self.assertEqual(r['referee'], REFEREE_LLM)
        self.assertIsNone(r['error'])

    def test_empty(self):
        r = resolve_track_goal('  ')
        self.assertIsNotNone(r['error'])


class TestBboxToPtz(unittest.TestCase):
    def test_right_of_centre_pans_positive(self):
        ox, oy = bbox_offsets({'center_x': 0.75, 'center_y': 0.5})
        self.assertAlmostEqual(ox, 0.25)
        self.assertAlmostEqual(oy, 0.0)
        dpan, dtilt = ptz_delta_from_offsets(ox, oy)
        self.assertGreater(dpan, 0)
        self.assertAlmostEqual(dtilt, 0.0)

    def test_below_centre_tilts_down(self):
        dpan, dtilt = ptz_delta_from_offsets(0.0, 0.2)
        self.assertAlmostEqual(dpan, 0.0)
        self.assertLess(dtilt, 0)

    def test_clamp(self):
        p, t = clamp_ptz(200, 120)
        self.assertEqual(p, 180.0)
        self.assertEqual(t, 90.0)


if __name__ == '__main__':
    unittest.main()
