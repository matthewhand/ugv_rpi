"""PTZ Tracking mode: scan with the gimbal, lock and centre on a goal.

No chassis motion. Detector for MobileNet-SSD classes; LLM vision for free-text
goals that are not on the closed list.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional, Tuple

from ai_seek import (
    MOBILENET_SSD_LABELS,
    REFEREE_DETECTOR,
    REFEREE_LLM,
    parse_llm_goal,
    parse_seek_goal,
    parse_seek_referee,
)

DEFAULT_TRACK_MAX_STEPS = 40
DEFAULT_TRACK_TIMEOUT_S = 180.0
DEFAULT_TRACK_CONF = 0.22
# Horizontal / vertical FOV used to turn bbox offset into T:133 degrees
TRACK_HFOV_DEG = 72.0
TRACK_VFOV_DEG = 54.0
TRACK_CENTER_TOL = 0.10  # |offset| below this = locked on target


def resolve_track_goal(text: str) -> Dict[str, Optional[str]]:
    """Pick referee from the goal string.

    Known MobileNet-SSD / alias → detector. Anything else non-empty → LLM.
    """
    lab, err = parse_seek_goal(text)
    if lab:
        return {'goal': lab, 'referee': REFEREE_DETECTOR, 'error': None}
    lab2, err2 = parse_llm_goal(text)
    if lab2:
        return {'goal': lab2, 'referee': REFEREE_LLM, 'error': None}
    return {'goal': None, 'referee': None, 'error': err or err2 or 'goal is empty'}


def bbox_offsets(best: Optional[dict]) -> Tuple[Optional[float], Optional[float]]:
    """Return (offset_x, offset_y) in [-0.5, 0.5] from image centre, or (None, None)."""
    if not isinstance(best, dict):
        return None, None
    try:
        cx = float(best.get('center_x'))
        cy = float(best.get('center_y'))
    except (TypeError, ValueError):
        return None, None
    return round(cx - 0.5, 3), round(cy - 0.5, 3)


def ptz_delta_from_offsets(offset_x, offset_y, hfov=TRACK_HFOV_DEG, vfov=TRACK_VFOV_DEG):
    """Map image offsets to T:133 Δpan / Δtilt.

    +offset_x (object right of centre) → +pan (look right).
    +offset_y (object below centre) → −tilt (look down).
    """
    try:
        ox = float(offset_x)
    except (TypeError, ValueError):
        ox = 0.0
    try:
        oy = float(offset_y)
    except (TypeError, ValueError):
        oy = 0.0
    d_pan = ox * float(hfov)
    d_tilt = -oy * float(vfov)
    return round(d_pan, 2), round(d_tilt, 2)


def clamp_ptz(pan, tilt):
    p = max(-180.0, min(180.0, float(pan)))
    t = max(-30.0, min(90.0, float(tilt)))
    return p, t


class TrackController:
    """One PTZ tracking run at a time (independent of Seek)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._state: Dict[str, Any] = self._idle_state()

    @staticmethod
    def _idle_state() -> Dict[str, Any]:
        return {
            'phase': 'idle',
            'referee': REFEREE_DETECTOR,
            'goal_text': '',
            'goal_label': None,
            'step': 0,
            'max_steps': DEFAULT_TRACK_MAX_STEPS,
            'timeout_s': DEFAULT_TRACK_TIMEOUT_S,
            'conf_threshold': DEFAULT_TRACK_CONF,
            'started_at': None,
            'finished_at': None,
            'last_detection': None,
            'last_check_seq': 0,
            'error': None,
            'message': 'Idle',
            'event_log': [],
            'log_seq': 0,
            'locked': False,
            'lock_pan': None,
            'lock_tilt': None,
            'cam_aim': None,
        }

    def status(self) -> Dict[str, Any]:
        with self._lock:
            snap = dict(self._state)
            snap['cancel_requested'] = self._cancel.is_set()
            if snap.get('started_at') and snap['phase'] == 'running':
                snap['elapsed_s'] = round(time.time() - float(snap['started_at']), 2)
            return snap

    def should_stop(self) -> bool:
        return self._cancel.is_set()

    def update(self, **kwargs) -> None:
        with self._lock:
            self._state.update(kwargs)

    def append_log(self, kind: str, text: str, **extra) -> None:
        with self._lock:
            self._state['log_seq'] = int(self._state.get('log_seq') or 0) + 1
            entry = {
                'seq': self._state['log_seq'],
                't': time.time(),
                'kind': kind,
                'text': text,
            }
            entry.update(extra)
            log = list(self._state.get('event_log') or [])
            log.append(entry)
            self._state['event_log'] = log[-200:]

    def stop(self) -> Dict[str, Any]:
        self._cancel.set()
        with self._lock:
            if self._state['phase'] == 'running':
                self._state['message'] = 'Stop requested…'
            return dict(self._state)

    def is_running(self) -> bool:
        with self._lock:
            return self._state['phase'] == 'running'

    def finish(self, phase: str, message: str = '', **kwargs) -> None:
        with self._lock:
            self._state['phase'] = phase
            self._state['finished_at'] = time.time()
            if message:
                self._state['message'] = message
            self._state.update(kwargs)

    def start(self, goal_text: str, *, loop_fn, max_steps=DEFAULT_TRACK_MAX_STEPS,
              timeout_s=DEFAULT_TRACK_TIMEOUT_S, conf_threshold=DEFAULT_TRACK_CONF):
        resolved = resolve_track_goal(goal_text)
        if resolved.get('error') or not resolved.get('goal'):
            return {'success': False, 'error': resolved.get('error') or 'goal is empty'}
        label = resolved['goal']
        referee = parse_seek_referee(resolved['referee'])
        with self._lock:
            if self._state['phase'] == 'running':
                return {'success': False, 'error': 'track already running', 'status': dict(self._state)}
            self._cancel.clear()
            self._state = self._idle_state()
            self._state.update({
                'phase': 'running',
                'referee': referee,
                'goal_text': (goal_text or '').strip(),
                'goal_label': label,
                'max_steps': int(max_steps or 0),
                'timeout_s': float(timeout_s or 0),
                'conf_threshold': float(conf_threshold),
                'started_at': time.time(),
                'message': f'Tracking {label} ({referee})…',
            })
        self.append_log(
            'start',
            f'Track started · goal={label} · referee={referee} · PTZ-only (no drive)',
            goal=label, referee=referee,
        )
        t = threading.Thread(
            target=loop_fn,
            args=(self, label, float(conf_threshold), int(max_steps or 0), float(timeout_s or 0)),
            name='ugv-track',
            daemon=True,
        )
        self._thread = t
        t.start()
        return {'success': True, 'status': self.status()}


track_controller = TrackController()
