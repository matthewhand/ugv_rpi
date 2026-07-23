"""Seek mode helpers: goal parsing + OpenCV goal judgment (LLM is not the referee).

Pure pieces live here so unit tests can drive them without a full Flask boot.
"""
from __future__ import annotations

import re
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

# MobileNet-SSD VOC classes (must match cv_ctrl.OpencvFuncs.class_names)
MOBILENET_SSD_LABELS = frozenset({
    'background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car',
    'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person',
    'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor',
})

# Common aliases → detector label
_LABEL_ALIASES = {
    'puppy': 'dog',
    'pup': 'dog',
    'canine': 'dog',
    'kitty': 'cat',
    'kitten': 'cat',
    'human': 'person',
    'people': 'person',
    'man': 'person',
    'woman': 'person',
    'bike': 'bicycle',
    'motorbike': 'motorbike',
    'motorcycle': 'motorbike',
    'tv': 'tvmonitor',
    'television': 'tvmonitor',
    'plant': 'pottedplant',
    'potted plant': 'pottedplant',
    'table': 'diningtable',
    'airplane': 'aeroplane',
    'plane': 'aeroplane',
}

DEFAULT_SEEK_MAX_STEPS = 12
DEFAULT_SEEK_TIMEOUT_S = 180.0
DEFAULT_SEEK_CONF = 0.22
DEFAULT_SEEK_STEP_PAUSE_S = 0.35


def parse_seek_goal(text: str) -> Tuple[Optional[str], Optional[str]]:
    """Map free text to a MobileNet-SSD label.

    Returns (label, error). On success error is None; on failure label is None.
    """
    raw = (text or '').strip().lower()
    if not raw:
        return None, 'goal is empty'
    # Prefer multi-word aliases first
    if raw in _LABEL_ALIASES:
        lab = _LABEL_ALIASES[raw]
        if lab in MOBILENET_SSD_LABELS and lab != 'background':
            return lab, None
    # Exact class match
    if raw in MOBILENET_SSD_LABELS and raw != 'background':
        return raw, None
    # First token / word that matches
    tokens = re.findall(r'[a-z]+', raw)
    for t in tokens:
        if t in _LABEL_ALIASES:
            lab = _LABEL_ALIASES[t]
            if lab in MOBILENET_SSD_LABELS and lab != 'background':
                return lab, None
        if t in MOBILENET_SSD_LABELS and t != 'background':
            return t, None
    known = sorted(l for l in MOBILENET_SSD_LABELS if l != 'background')
    return None, f'unknown goal class {text!r}; supported: {", ".join(known)}'


def enrich_detections(dets: List[dict], filter_label: str = '') -> List[dict]:
    """Add center/offset fields; optional case-insensitive label filter."""
    fl = (filter_label or '').strip().lower()
    out: List[dict] = []
    for d in dets or []:
        if not isinstance(d, dict):
            continue
        lab = (d.get('label') or '').lower()
        if fl and lab != fl:
            continue
        bb = d.get('bbox_norm') or d.get('bbox') or [0, 0, 0, 0]
        try:
            x1, y1, x2, y2 = [float(v) for v in bb[:4]]
        except (TypeError, ValueError):
            x1 = y1 = x2 = y2 = 0.0
        item = dict(d)
        item['center_x'] = round((x1 + x2) / 2.0, 3)
        item['center_y'] = round((y1 + y2) / 2.0, 3)
        item['offset_x'] = round(item['center_x'] - 0.5, 3)
        out.append(item)
    return out


def evaluate_goal_detections(
    dets: List[dict],
    goal_label: str,
    conf_threshold: float = DEFAULT_SEEK_CONF,
) -> Dict[str, Any]:
    """OpenCV-side goal judgment. LLM text is never consulted here.

    found is True only when at least one detection has matching label and
    confidence >= conf_threshold.
    """
    goal = (goal_label or '').strip().lower()
    thr = max(0.05, min(0.95, float(conf_threshold)))
    enriched = enrich_detections(dets, filter_label=goal)
    matches = []
    for d in enriched:
        try:
            c = float(d.get('confidence', 0) or 0)
        except (TypeError, ValueError):
            c = 0.0
        if c >= thr:
            matches.append(d)
    labels = sorted({(d.get('label') or '') for d in (dets or []) if isinstance(d, dict) and d.get('label')})
    best = matches[0] if matches else None
    return {
        'found': bool(matches),
        'goal_label': goal,
        'conf_threshold': thr,
        'match_count': len(matches),
        'matches': matches,
        'best': best,
        'labels_found': labels,
        'all_count': len(dets or []),
    }


class SeekController:
    """Process-wide Seek run state (one seek at a time)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._state: Dict[str, Any] = self._idle_state()

    @staticmethod
    def _idle_state() -> Dict[str, Any]:
        return {
            'phase': 'idle',  # idle|running|found|stopped|failed|timeout
            'goal_text': '',
            'goal_label': None,
            'step': 0,
            'max_steps': DEFAULT_SEEK_MAX_STEPS,
            'timeout_s': DEFAULT_SEEK_TIMEOUT_S,
            'conf_threshold': DEFAULT_SEEK_CONF,
            'started_at': None,
            'finished_at': None,
            'last_detection': None,
            'last_llm_reply': None,
            'last_tools': [],
            'error': None,
            'message': 'Idle',
            'history': [],
        }

    def status(self) -> Dict[str, Any]:
        with self._lock:
            snap = dict(self._state)
            snap['cancel_requested'] = self._cancel.is_set()
            if snap.get('started_at') and snap['phase'] == 'running':
                snap['elapsed_s'] = round(time.time() - float(snap['started_at']), 2)
            return snap

    def stop(self) -> Dict[str, Any]:
        self._cancel.set()
        with self._lock:
            if self._state['phase'] == 'running':
                self._state['message'] = 'Stop requested…'
            return dict(self._state)

    def is_running(self) -> bool:
        with self._lock:
            return self._state['phase'] == 'running'

    def start(
        self,
        goal_text: str,
        *,
        loop_fn,
        max_steps: int = DEFAULT_SEEK_MAX_STEPS,
        timeout_s: float = DEFAULT_SEEK_TIMEOUT_S,
        conf_threshold: float = DEFAULT_SEEK_CONF,
    ) -> Dict[str, Any]:
        """Start seek. loop_fn(controller, label, conf, max_steps, timeout_s) runs in a thread."""
        label, err = parse_seek_goal(goal_text)
        if err:
            return {'success': False, 'error': err}
        with self._lock:
            if self._state['phase'] == 'running':
                return {'success': False, 'error': 'seek already running', 'status': dict(self._state)}
            self._cancel.clear()
            self._state = self._idle_state()
            self._state.update({
                'phase': 'running',
                'goal_text': (goal_text or '').strip(),
                'goal_label': label,
                'max_steps': int(max_steps),
                'timeout_s': float(timeout_s),
                'conf_threshold': float(conf_threshold),
                'started_at': time.time(),
                'message': f'Seeking {label}…',
            })
        t = threading.Thread(
            target=loop_fn,
            args=(self, label, float(conf_threshold), int(max_steps), float(timeout_s)),
            daemon=True,
            name='ai-seek-loop',
        )
        self._thread = t
        t.start()
        return {'success': True, 'status': self.status()}

    def update(self, **kwargs) -> None:
        with self._lock:
            self._state.update(kwargs)

    def finish(self, phase: str, message: str = '', **kwargs) -> None:
        with self._lock:
            self._state['phase'] = phase
            self._state['finished_at'] = time.time()
            if message:
                self._state['message'] = message
            self._state.update(kwargs)

    def should_stop(self) -> bool:
        return self._cancel.is_set()


# Singleton used by Flask app
seek_controller = SeekController()
