"""Seek mode helpers: goal parsing + referee judgment (detector or LLM JSON).

Pure pieces live here so unit tests can drive them without a full Flask boot.
"""
from __future__ import annotations

import json
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
    'guy': 'person',
    'boy': 'person',
    'girl': 'person',
    'individual': 'person',
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

# Referee modes: closed-set on-device detector vs free-text LLM vision judge
REFEREE_DETECTOR = 'detector'  # OpenCV / MobileNet-SSD / future YOLO class list
REFEREE_LLM = 'llm'            # vision model answers found: true|false via JSON
VALID_REFEREES = frozenset({REFEREE_DETECTOR, REFEREE_LLM})

# Finite pilot defaults (0 still means unlimited when explicitly requested)
DEFAULT_SEEK_MAX_STEPS = 30
# Wheels stay still unless the operator explicitly turns dry-run off.
DEFAULT_SEEK_DRY_RUN = True
DEFAULT_SEEK_TIMEOUT_S = 300.0  # 5 minutes
DEFAULT_SEEK_CONF = 0.85  # Raised from 0.22 - empty corridors were showing false sofa/chair
# Halt-as-found bar. Scan still uses DEFAULT_SEEK_CONF so weak hits are logged.
DEFAULT_SEEK_FOUND_CONF = 0.45
DEFAULT_SEEK_STEP_PAUSE_S = 0.35
# Caps applied when a positive limit is requested
SEEK_MAX_STEPS_CAP = 500
SEEK_TIMEOUT_S_MIN = 5.0
SEEK_TIMEOUT_S_CAP = 7200.0

# Per-class detector minimum confidence thresholds
# Classes not in this map use the general conf_threshold parameter
# Matthew: sofa and chair need 99% to avoid false positives (carpet, floor patterns)
DETECTOR_CLASS_MIN_CONF = {
    'sofa': 0.99,
    'chair': 0.99,
}


def normalize_seek_max_steps(raw, default: int = DEFAULT_SEEK_MAX_STEPS) -> int:
    """Normalize max_steps from a request body / UI.

    Missing/empty → *default* (finite pilot default).
    Explicit 0 / inf / unlimited / none → 0 (unlimited).
    Positive values capped at SEEK_MAX_STEPS_CAP.
    """
    if raw is None or raw == '':
        try:
            return max(0, int(default))
        except (TypeError, ValueError):
            return int(DEFAULT_SEEK_MAX_STEPS)
    if str(raw).lower() in ('inf', 'infinite', 'unlimited', 'none'):
        return 0
    try:
        ms = int(raw)
    except (TypeError, ValueError):
        try:
            return max(0, int(default))
        except (TypeError, ValueError):
            return int(DEFAULT_SEEK_MAX_STEPS)
    if ms < 0:
        return 0
    if ms > 0:
        return min(int(SEEK_MAX_STEPS_CAP), ms)
    return 0  # explicit 0 = unlimited


def normalize_seek_timeout_s(raw, default: float = DEFAULT_SEEK_TIMEOUT_S) -> float:
    """Normalize timeout_s from a request body / UI.

    Missing/empty → *default* (finite pilot default).
    Explicit 0 / inf / unlimited / none → 0.0 (no time limit).
    Positive values clamped to [SEEK_TIMEOUT_S_MIN, SEEK_TIMEOUT_S_CAP].
    """
    if raw is None or raw == '':
        try:
            return max(0.0, float(default))
        except (TypeError, ValueError):
            return float(DEFAULT_SEEK_TIMEOUT_S)
    if str(raw).lower() in ('inf', 'infinite', 'unlimited', 'none'):
        return 0.0
    try:
        ts = float(raw)
    except (TypeError, ValueError):
        try:
            return max(0.0, float(default))
        except (TypeError, ValueError):
            return float(DEFAULT_SEEK_TIMEOUT_S)
    if ts < 0:
        return 0.0
    if ts > 0:
        return max(float(SEEK_TIMEOUT_S_MIN), min(float(SEEK_TIMEOUT_S_CAP), ts))
    return 0.0  # explicit 0 = no time limit


def motion_lock_should_ignore_zero(
    lock_active: bool,
    is_zero_cmd: bool,
    *,
    force_stop: bool = False,
) -> bool:
    """Whether a zero chassis cmd should be dropped while AI motion lock is armed.

    force_stop=True always delivers the zero (emergency STOP / stop_motors /
    seek stop). Heartbeat idle zeros remain blocked during timed AI drives.
    """
    if force_stop:
        return False
    return bool(lock_active) and bool(is_zero_cmd)

# What to do when the referee reports found
ON_FOUND_NONE = 'none'
ON_FOUND_TTS = 'tts'
VALID_ON_FOUND = frozenset({ON_FOUND_NONE, ON_FOUND_TTS})
DEFAULT_ON_FOUND = ON_FOUND_TTS
DEFAULT_ON_FOUND_TTS = 'I have found the {goal}.'
DEFAULT_LLM_SCENE_NAV = True
DEFAULT_LLM_NAV_INTERVAL = 10


def parse_on_found(value: str) -> str:
    raw = (value or '').strip().lower()
    if raw in ('', 'nothing', 'no', 'off', 'idle'):
        return ON_FOUND_NONE
    if raw in ('tts', 'speak', 'announce', 'say'):
        return ON_FOUND_TTS
    if raw in VALID_ON_FOUND:
        return raw
    return ON_FOUND_NONE


def format_on_found_tts(template: str, goal: str) -> str:
    """Fill {goal} / {label} in the TTS template."""
    g = (goal or 'target').strip() or 'target'
    tmpl = (template or DEFAULT_ON_FOUND_TTS).strip() or DEFAULT_ON_FOUND_TTS
    try:
        return tmpl.format(goal=g, label=g, target=g)
    except Exception:
        return f'I have found the {g}.'


def detector_labels() -> List[str]:
    """Sorted MobileNet-SSD class names available for dropdown (no background)."""
    return sorted(l for l in MOBILENET_SSD_LABELS if l != 'background')


def parse_seek_referee(value: str) -> str:
    raw = (value or '').strip().lower()
    if raw in ('opencv', 'cv', 'mobilenet', 'yolo', 'on_device', 'on-device'):
        return REFEREE_DETECTOR
    if raw in ('vision', 'vlm', 'gpt', 'model'):
        return REFEREE_LLM
    if raw in VALID_REFEREES:
        return raw
    return REFEREE_DETECTOR


def parse_seek_goal(text: str) -> Tuple[Optional[str], Optional[str]]:
    """Map free text to a MobileNet-SSD label (detector referee only).

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
    known = detector_labels()
    return None, f'unknown goal class {text!r}; choose from detector list: {", ".join(known)}'


def parse_llm_goal(text: str) -> Tuple[Optional[str], Optional[str]]:
    """Free-text goal for LLM referee (any non-empty description)."""
    raw = (text or '').strip()
    if not raw:
        return None, 'goal is empty'
    if len(raw) > 500:
        return None, 'goal too long (max 500 chars)'
    return raw, None


def parse_llm_found_payload(raw: Any) -> Dict[str, Any]:
    """Parse structured LLM judge output into {found: bool, reason, raw}.

    Accepts dict, JSON string, or messy prose with a JSON object / true|false.
    On unparseable content, found is False (do not end Seek on garbage).
    """
    reason = ''
    found = False
    parsed: Any = None

    if isinstance(raw, dict):
        parsed = raw
    elif isinstance(raw, bool):
        return {'found': bool(raw), 'reason': '', 'raw': raw, 'parse_ok': True}
    elif isinstance(raw, str):
        text = raw.strip()
        # Strip markdown fences
        if text.startswith('```'):
            text = re.sub(r'^```(?:json)?\s*', '', text)
            text = re.sub(r'\s*```$', '', text)
        try:
            parsed = json.loads(text)
        except Exception:
            # Find first {...} blob
            m = re.search(r'\{[^{}]*\}', text, re.S)
            if m:
                try:
                    parsed = json.loads(m.group(0))
                except Exception:
                    parsed = None
            if parsed is None:
                # Last resort: explicit true/false tokens
                low = text.lower()
                if re.search(r'\bfound\b["\s:]*true\b', low) or re.search(r'^\s*true\s*$', low):
                    found = True
                    reason = text[:200]
                    return {'found': True, 'reason': reason, 'raw': raw, 'parse_ok': True}
                if re.search(r'\bfound\b["\s:]*false\b', low) or re.search(r'^\s*false\s*$', low):
                    return {'found': False, 'reason': text[:200], 'raw': raw, 'parse_ok': True}
                return {
                    'found': False,
                    'reason': 'unparseable judge output',
                    'raw': (text[:300] if text else raw),
                    'parse_ok': False,
                }
    else:
        return {'found': False, 'reason': 'empty judge output', 'raw': raw, 'parse_ok': False}

    if isinstance(parsed, dict):
        for key in ('found', 'is_found', 'success', 'done', 'visible', 'present'):
            if key in parsed:
                val = parsed[key]
                if isinstance(val, str):
                    found = val.strip().lower() in ('1', 'true', 'yes', 'y')
                else:
                    found = bool(val)
                break
        reason = str(parsed.get('reason') or parsed.get('explanation') or parsed.get('note') or '')[:300]
        return {'found': found, 'reason': reason, 'raw': parsed, 'parse_ok': True}

    return {'found': False, 'reason': 'unexpected JSON type', 'raw': parsed, 'parse_ok': False}


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
    """Detector-side goal judgment. LLM free-text is never consulted here.

    found is True only when at least one detection has matching label and
    confidence >= conf_threshold (or per-class minimum, whichever is higher).
    
    Per-class minimums (DETECTOR_CLASS_MIN_CONF):
    - sofa: 99% (0.99) - avoid false positives on textured carpet/floor
    - chair: 99% (0.99) - avoid false positives on floor patterns
    - Other classes use conf_threshold parameter
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
        
        # Use per-class minimum if defined, otherwise use general threshold
        label = (d.get('label') or '').strip().lower()
        min_conf = DETECTOR_CLASS_MIN_CONF.get(label, thr)
        effective_thr = max(thr, min_conf)
        
        if c >= effective_thr:
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
        'referee': REFEREE_DETECTOR,
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
            'referee': REFEREE_DETECTOR,
            'goal_text': '',
            'goal_label': None,
            'step': 0,
            'max_steps': DEFAULT_SEEK_MAX_STEPS,
            'timeout_s': DEFAULT_SEEK_TIMEOUT_S,
            'conf_threshold': DEFAULT_SEEK_CONF,
            'started_at': None,
            'finished_at': None,
            'last_detection': None,
            'last_check_at': None,  # unix time of last referee fire (for UX pulse)
            'last_check_seq': 0,    # increments each detector/judge call
            'last_llm_reply': None,
            'last_nav': None,
            'last_tools': [],
            'error': None,
            'message': 'Idle',
            'history': [],
            # Ring buffer of human-readable seek log lines for the UI
            'event_log': [],
            'log_seq': 0,
            'on_found': ON_FOUND_NONE,
            'on_found_tts': DEFAULT_ON_FOUND_TTS,
            'on_found_done': False,
            'on_found_phrase': None,
            'on_found_error': None,
            'llm_scene_nav': DEFAULT_LLM_SCENE_NAV,
            'llm_nav_interval': DEFAULT_LLM_NAV_INTERVAL,
            'dry_run': DEFAULT_SEEK_DRY_RUN,
            'dry_run_gen': 0,
            'last_views': [],
            'last_sweep': None,
            'panorama_data_url': None,
            # Single SoT for live pan overlay (written only by app._seek_publish_cam_aim)
            'cam_aim': None,
        }

    def status(self) -> Dict[str, Any]:
        with self._lock:
            snap = dict(self._state)
            # status() must not retain huge binary blobs if present on views
            if isinstance(snap.get('last_views'), list):
                slim_views = []
                for v in snap['last_views']:
                    if not isinstance(v, dict):
                        continue
                    slim_views.append({
                        k: v.get(k) for k in (
                            'name', 'pan_deg', 'bytes', 'data_url', 'has_target',
                            'detected_labels', 'raw_detections', 'pan_settled',
                        ) if k in v or k in ('name', 'pan_deg', 'has_target', 'detected_labels')
                    })
                snap['last_views'] = slim_views
            snap['cancel_requested'] = self._cancel.is_set()
            if snap.get('started_at') and snap['phase'] == 'running':
                snap['elapsed_s'] = round(time.time() - float(snap['started_at']), 2)
            return snap

    def append_log(self, kind: str, text: str, **fields) -> None:
        """Append a UI-visible seek log event (ring buffer, newest last)."""
        kind = (kind or 'info').strip().lower()[:32]
        text = (text or '').strip()
        if not text:
            return
        if len(text) > 500:
            text = text[:497] + '…'
        with self._lock:
            seq = int(self._state.get('log_seq') or 0) + 1
            self._state['log_seq'] = seq
            entry = {
                'seq': seq,
                't': round(time.time(), 3),
                'kind': kind,
                'text': text,
            }
            for k, v in fields.items():
                if v is None:
                    continue
                if isinstance(v, (str, int, float, bool)):
                    entry[k] = v
                else:
                    entry[k] = str(v)[:200]
            log = list(self._state.get('event_log') or [])
            log.append(entry)
            # Keep last 200 events so a page refresh can rebuild a long seek log
            if len(log) > 200:
                log = log[-200:]
            self._state['event_log'] = log

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
        referee: str = REFEREE_DETECTOR,
        on_found: str = ON_FOUND_NONE,
        on_found_tts: str = DEFAULT_ON_FOUND_TTS,
        llm_scene_nav: bool = DEFAULT_LLM_SCENE_NAV,
        llm_nav_interval: int = DEFAULT_LLM_NAV_INTERVAL,
        seek_multi_image: bool = False,
        dry_run: bool = DEFAULT_SEEK_DRY_RUN,
    ) -> Dict[str, Any]:
        """Start seek. loop_fn(controller, goal_key, conf, max_steps, timeout_s) runs in a thread."""
        referee = parse_seek_referee(referee)
        if referee == REFEREE_LLM:
            label, err = parse_llm_goal(goal_text)
        else:
            label, err = parse_seek_goal(goal_text)
        if err:
            return {'success': False, 'error': err}
        on_found = parse_on_found(on_found)
        tts_tmpl = (on_found_tts or DEFAULT_ON_FOUND_TTS).strip() or DEFAULT_ON_FOUND_TTS
        if len(tts_tmpl) > 200:
            tts_tmpl = tts_tmpl[:200]
        try:
            interval_val = max(1, min(100, int(llm_nav_interval)))
        except (ValueError, TypeError):
            interval_val = DEFAULT_LLM_NAV_INTERVAL
        with self._lock:
            if self._state['phase'] == 'running':
                return {'success': False, 'error': 'seek already running', 'status': dict(self._state)}
            self._cancel.clear()
            self._state = self._idle_state()
            ms = int(max_steps)
            if ms < 0:
                ms = 0
            ts = float(timeout_s)
            if ts < 0:
                ts = 0.0
            self._state.update({
                'phase': 'running',
                'referee': referee,
                'goal_text': (goal_text or '').strip(),
                'goal_label': label,
                'max_steps': ms,  # 0 = unlimited
                'timeout_s': ts,  # 0 = no time limit
                'conf_threshold': float(conf_threshold),
                'on_found': on_found,
                'on_found_tts': tts_tmpl,
                'on_found_done': False,
                'llm_scene_nav': bool(llm_scene_nav),
                'llm_nav_interval': interval_val,
                'seek_multi_image': bool(seek_multi_image),
                'dry_run': bool(dry_run),
                'dry_run_gen': 0,
                'started_at': time.time(),
                'message': (
                    f'Seeking {label} ({referee})'
                    + (' · DRY-RUN no drive' if dry_run else ' · LIVE DRIVE')
                    + '…'
                ),
                'event_log': [],
                'log_seq': 0,
            })
            start_ms, start_ts, start_conf = ms, ts, float(conf_threshold)
        # Latch BEFORE the thread exists so sticks/CLI cannot sneak a hop.
        if dry_run:
            try:
                from seek_nav import begin_seek_dry_run
                gen = begin_seek_dry_run()
                with self._lock:
                    self._state['dry_run_gen'] = int(gen)
            except Exception:
                pass
        self.append_log(
            'start',
            f'Seek started · goal={label} · referee={referee} · on_found={on_found}'
            f' · scene_nav={bool(llm_scene_nav)} · nav_interval={interval_val}'
            f' · {"DRY-RUN no drive" if dry_run else "LIVE DRIVE"}',
            goal=label, referee=referee, on_found=on_found,
        )
        t = threading.Thread(
            target=loop_fn,
            args=(self, label, start_conf, start_ms, start_ts),
            daemon=True,
            name='ai-seek-loop',
        )
        self._thread = t
        t.start()
        return {'success': True, 'status': self.status()}

    def on_found_action(self) -> str:
        with self._lock:
            return self._state.get('on_found') or ON_FOUND_NONE

    def on_found_tts_template(self) -> str:
        with self._lock:
            return self._state.get('on_found_tts') or DEFAULT_ON_FOUND_TTS

    def update(self, **kwargs) -> None:
        with self._lock:
            # Bump detector UX counters when a new referee result is attached
            if 'last_detection' in kwargs and kwargs['last_detection'] is not None:
                kwargs = dict(kwargs)
                kwargs.setdefault('last_check_at', time.time())
                kwargs['last_check_seq'] = int(self._state.get('last_check_seq') or 0) + 1
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

    def referee(self) -> str:
        with self._lock:
            return self._state.get('referee') or REFEREE_DETECTOR


# Singleton used by Flask app
seek_controller = SeekController()
