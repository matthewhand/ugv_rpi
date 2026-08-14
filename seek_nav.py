"""Pure Seek nav-plan helpers (no Flask / camera).

Extracted so unit tests can prove safety overrides without booting app.py.
"""
from __future__ import annotations

import re
import threading
from typing import Any, Dict, Optional

# Chassis-calibrated 2026-08-13 (this Waveshare rover, carpet + floor cable runner).
# Soft T:13 angular ~0.4–0.7 does NOT yaw — Seek turns must be T:1 UI-Fast
# (config max_speed 1.3). Soft linear 0.12–0.16 stalls on the cable ramp;
# punch ~0.22–0.28. Fast tank 700ms ≈ a solid room turn; multi-second Fast
# spins over-rotate.
SEEK_DRIVE_MS_BY_DIST = {
    'short': 850,
    'medium': 1100,
    'long': 1600,
}
SEEK_DRIVE_LIN_BY_DIST = {
    'short': 0.22,
    'medium': 0.26,
    'long': 0.28,
}
SEEK_ESCAPE_REVERSE_MS = 850
SEEK_ESCAPE_REVERSE_LIN = 0.20
SEEK_REVERSE_MAX_MS = 900
SEEK_REVERSE_MAX_LIN = 0.22
SEEK_TURN_REPEATS_BY_DIST = {
    'short': 1,
    'medium': 1,
    'long': 1,
}
# Approximate yaw at UI-Fast T:1 on this carpet (not encoder-closed-loop).
SEEK_TURN_DEG_BY_DIST = {
    'short': 35,
    'medium': 80,
    'long': 130,
}
SEEK_TURN_MS_BY_DIST = {
    'short': 350,
    'medium': 700,
    'long': 1100,
}
SEEK_VALID_DISTANCES = frozenset({'short', 'medium', 'long'})
SEEK_DIST_ALIASES = {
    'short': 'short', 'near': 'short', 'slow': 'short', 'crawl': 'short',
    'close': 'short', 'small': 'short', 's': 'short', '1': 'short',
    'medium': 'medium', 'mid': 'medium', 'normal': 'medium', 'default': 'medium',
    'm': 'medium', '2': 'medium',
    'long': 'long', 'far': 'long', 'fast': 'long', 'open': 'long',
    'large': 'long', 'big': 'long', 'l': 'long', '3': 'long',
}
SEEK_ESCAPE_SEQ = ('turn_left', 'turn_right', 'turn_left', 'backward', 'turn_right')
SEEK_OBSTACLE_RANGES = frozenset({'none', 'far', 'medium', 'near', 'unknown'})
SEEK_OBSTACLE_ALIASES = {
    'none': 'none', 'clear': 'none', 'open': 'none', 'empty': 'none',
    'no': 'none', 'nothing': 'none', 'n/a': 'none', 'na': 'none',
    'far': 'far', 'distant': 'far', 'long': 'far', 'away': 'far',
    'medium': 'medium', 'mid': 'medium', 'middle': 'medium', 'moderate': 'medium',
    'near': 'near', 'close': 'near', 'immediate': 'near', 'very_close': 'near',
    'very close': 'near', 'blocking': 'near', 'blocked': 'near', 'imminent': 'near',
    'unknown': 'unknown', '?': 'unknown',
}

_escape_idx = 0


def reset_escape_cycle() -> None:
    global _escape_idx
    _escape_idx = 0


def seek_normalize_distance(raw, default: str = 'medium') -> str:
    if raw is None:
        return default if default in SEEK_VALID_DISTANCES else 'medium'
    s = str(raw).strip().lower().replace('-', ' ').replace('_', ' ')
    if s in SEEK_DIST_ALIASES:
        return SEEK_DIST_ALIASES[s]
    for tok in re.findall(r'[a-z0-9]+', s):
        if tok in SEEK_DIST_ALIASES:
            return SEEK_DIST_ALIASES[tok]
    return default if default in SEEK_VALID_DISTANCES else 'medium'


def seek_normalize_action(raw, default: str = 'forward') -> str:
    a = str(raw or default).strip().lower().replace(' ', '_').replace('-', '_')
    if a in ('forward', 'go_forward', 'straight', 'ahead'):
        return 'forward'
    if a in ('turn_left', 'left', 'l'):
        return 'turn_left'
    if a in ('turn_right', 'right', 'r'):
        return 'turn_right'
    if a in ('backward', 'back', 'drive_backward', 'reverse', 'retreat'):
        return 'backward'
    return default if default in ('forward', 'turn_left', 'turn_right', 'backward') else 'forward'


def seek_normalize_obstacle_range(raw, default: str = 'unknown') -> str:
    if raw is None or raw is False:
        return default if default in SEEK_OBSTACLE_RANGES else 'unknown'
    if raw is True:
        return 'near'
    s = str(raw).strip().lower().replace('-', ' ').replace('_', ' ')
    if s in SEEK_OBSTACLE_ALIASES:
        return SEEK_OBSTACLE_ALIASES[s]
    for tok in re.findall(r'[a-z?]+', s):
        if tok in SEEK_OBSTACLE_ALIASES:
            return SEEK_OBSTACLE_ALIASES[tok]
    m = re.search(r'(\d+(?:\.\d+)?)\s*m', s)
    if m:
        metres = float(m.group(1))
        if metres < 0.6:
            return 'near'
        if metres < 1.5:
            return 'medium'
        if metres < 4.0:
            return 'far'
        return 'none'
    return default if default in SEEK_OBSTACLE_RANGES else 'unknown'


def seek_next_escape_action(prefer: Optional[str] = None) -> str:
    global _escape_idx
    if prefer in ('turn_left', 'turn_right', 'backward', 'left', 'right'):
        pref = seek_normalize_action(prefer)
        _escape_idx += 1
        return pref
    action = SEEK_ESCAPE_SEQ[_escape_idx % len(SEEK_ESCAPE_SEQ)]
    _escape_idx += 1
    return action


def seek_nav_plan(
    action,
    drive_distance: str = 'medium',
    *,
    path_clear_forward=None,
    stuck: bool = False,
    obstacle_range=None,
    last_action=None,
    prefer_turn=None,
) -> Dict[str, Any]:
    """Normalize action+distance and apply safety overrides (no I/O)."""
    action = seek_normalize_action(action)
    dist = seek_normalize_distance(drive_distance, default='medium')
    obs = seek_normalize_obstacle_range(obstacle_range, default='unknown')
    last = seek_normalize_action(last_action) if last_action else None
    prefer_turn = seek_normalize_action(prefer_turn) if prefer_turn else None
    if prefer_turn not in ('turn_left', 'turn_right'):
        prefer_turn = None
    safety_override = None
    requested = action

    if obs == 'unknown':
        if path_clear_forward is True:
            obs = 'none'
        elif path_clear_forward is False:
            obs = 'near'

    if last == 'backward' and action == 'backward' and obs in ('near', 'medium', 'unknown'):
        action = prefer_turn or 'turn_left'
        dist = 'short'
        safety_override = f'after reverse: turn to open ({action}/{dist}) instead of reverse again'

    if action == 'backward' and dist in ('medium', 'long') and not safety_override:
        dist = 'short'
        safety_override = 'reverse capped to short (avoid rear wall)'

    if stuck and action in ('forward', 'backward') and not safety_override:
        if last == 'backward':
            action = prefer_turn or 'turn_left'
            dist = 'short'
            safety_override = f'stuck after reverse: turn to open → {action}/{dist}'
        elif last in ('turn_left', 'turn_right'):
            action = 'backward'
            dist = 'short'
            safety_override = 'stuck after turn: reverse short nudge only'
        else:
            action = prefer_turn or 'turn_left'
            dist = 'short'
            safety_override = f'stuck: turn to open → {action}/{dist} (prefer over reverse)'
    elif action == 'forward' and not safety_override:
        if obs == 'near':
            if last == 'backward':
                action = prefer_turn or 'turn_left'
                dist = 'short'
                safety_override = f'near after reverse: turn open → {action}/{dist}'
            elif last in ('turn_left', 'turn_right'):
                action = 'backward'
                dist = 'short'
                safety_override = 'near after turn: reverse short nudge only'
            else:
                action = prefer_turn or seek_next_escape_action(prefer=prefer_turn or 'turn_left')
                if action == 'backward':
                    action = prefer_turn or 'turn_left'
                dist = 'short'
                safety_override = f'near obstacle: turn open → {action}/{dist} (not reverse)'
        elif obs == 'medium':
            if dist == 'long':
                dist = 'medium'
                safety_override = 'mid-range obstacle: forward capped to medium'
            elif dist == 'short':
                dist = 'medium'
                safety_override = 'mid-range: upgrade short→medium corridor hop'
        elif obs == 'far':
            if dist == 'short':
                dist = 'medium'
                safety_override = 'far obstacle: upgrade short→medium'
        elif obs == 'none':
            if dist == 'short':
                dist = 'long'
                safety_override = 'open path: upgrade short→long into empty space'
        elif obs == 'unknown' and path_clear_forward is False:
            if last == 'backward':
                action = prefer_turn or 'turn_left'
                dist = 'short'
                safety_override = f'blocked after reverse: turn open → {action}/{dist}'
            else:
                action = prefer_turn or 'turn_left'
                dist = 'short'
                safety_override = f'blocked/unknown: turn open → {action}/{dist} (not reverse)'

    effective_clear = obs in ('none', 'far') and not stuck
    if path_clear_forward is False and obs == 'near':
        effective_clear = False

    is_turn = action in ('turn_left', 'turn_right')
    is_linear = action in ('forward', 'backward')
    repeats = int(SEEK_TURN_REPEATS_BY_DIST.get(dist, 2)) if is_turn else 1
    turn_deg = int(SEEK_TURN_DEG_BY_DIST.get(dist, 90)) if is_turn else 0
    duration_ms = int(SEEK_DRIVE_MS_BY_DIST.get(dist, 1800)) if is_linear else int(
        SEEK_TURN_MS_BY_DIST.get(dist, 900)
    )
    linear_x = float(SEEK_DRIVE_LIN_BY_DIST.get(dist, 0.15)) if is_linear else 0.12
    if action == 'backward':
        dist = 'short'
        duration_ms = min(
            int(SEEK_DRIVE_MS_BY_DIST.get('short', 900)),
            int(SEEK_ESCAPE_REVERSE_MS),
            int(SEEK_REVERSE_MAX_MS),
        )
        linear_x = -abs(min(
            float(SEEK_DRIVE_LIN_BY_DIST.get('short', 0.12)),
            float(SEEK_ESCAPE_REVERSE_LIN),
            float(SEEK_REVERSE_MAX_LIN),
        ))

    if is_turn:
        side = 'left' if action == 'turn_left' else 'right'
        magnitude = f'~{turn_deg}° {side} fast-spin {duration_ms}ms'
        summary = f'{action}/{dist} {magnitude}'
    elif action == 'backward':
        magnitude = f'~{duration_ms / 1000.0:.1f}s reverse @ |v|={abs(linear_x):.2f}'
        summary = f'backward/{dist} {magnitude}'
    else:
        magnitude = f'~{duration_ms / 1000.0:.1f}s forward @ v={linear_x:.2f}'
        summary = f'forward/{dist} {magnitude}'

    return {
        'action': action,
        'drive_distance': dist,
        'is_turn': is_turn,
        'is_linear': is_linear,
        'repeats': repeats,
        'turn_deg': turn_deg,
        'duration_ms': duration_ms,
        'linear_x': linear_x,
        'magnitude': magnitude,
        'summary': summary,
        'path_clear_forward': effective_clear if path_clear_forward is None else bool(path_clear_forward),
        'obstacle_range': obs,
        'stuck': bool(stuck),
        'safety_override': safety_override,
        'requested_action': requested,
        'last_action': last,
        'prefer_turn': prefer_turn,
    }


def interpret_base_voltage(raw) -> Optional[float]:
    """ESP32 `v` field → pack volts, or None if missing / untrustworthy.

    Waveshare sometimes reports raw ADC-ish values (>30). Those are not volts.
    """
    if raw is None or raw is False:
        return None
    try:
        voltage = float(raw)
    except (TypeError, ValueError):
        return None
    if voltage <= 0 or voltage > 30:
        return None
    return voltage


def seek_commit_through_opening(
    last_action,
    last_dist='medium',
    *,
    obstacle_range='unknown',
    left_open=None,
    right_open=None,
    centre_open=None,
) -> Optional[Dict[str, Any]]:
    """If we just drove into a doorway/hall chute, keep going past the jambs.

    Live lesson: stopping in the frame wedges the chassis. Travel at least as
    far *past* the threshold as you drove *into* it (one more hop, same or longer).
    Walls on both sides + centre not near = chute.
    """
    if seek_normalize_action(last_action) != 'forward':
        return None
    obs = seek_normalize_obstacle_range(obstacle_range, default='unknown')
    if obs == 'near':
        return None
    if centre_open is False:
        return None
    try:
        lo = None if left_open is None else float(left_open)
        ro = None if right_open is None else float(right_open)
    except (TypeError, ValueError):
        return None
    if lo is None or ro is None:
        return None
    if lo >= 0.45 or ro >= 0.45:
        return None
    last = seek_normalize_distance(last_dist, default='medium')
    nxt = 'medium' if last == 'short' else last
    if last == 'medium':
        nxt = 'long'
    return {
        'action': 'forward',
        'drive_distance': nxt,
        'reason': (
            'doorway commit: clear the frame — drive as far past the jambs '
            f'as you entered ({last}→{nxt})'
        ),
    }


def seek_may_reverse(
    *,
    can_forward: bool,
    can_turn: bool,
    rear_left_clear: bool,
    rear_right_clear: bool,
    last_action=None,
) -> bool:
    """Reverse only as last resort: no forward, no turn, and BOTH rear quarters clear.

    Rear quarters: left half of the −135° still and right half of the +135° still.
    Never reverse twice in a row.
    """
    if can_forward:
        return False
    if can_turn:
        return False
    if seek_normalize_action(last_action) == 'backward':
        return False
    return bool(rear_left_clear) and bool(rear_right_clear)


SEEK_FORWARD_CM_ENUM = (0, 15, 30, 60, 100, 200)
SEEK_NAV_TOOL_NAME = 'seek_nav_answer'
SEEK_NAV_TOOL = {
    'type': 'function',
    'function': {
        'name': SEEK_NAV_TOOL_NAME,
        'description': (
            'Answer each navigation question with the smallest allowed token. '
            'Do not write prose. CENTRE=front path; LEFT photo is −135° rear-left; '
            'RIGHT photo is +135° rear-right.'
        ),
        'parameters': {
            'type': 'object',
            'additionalProperties': False,
            'properties': {
                'forward_clear_cm': {
                    'type': 'integer',
                    'enum': list(SEEK_FORWARD_CM_ENUM),
                    'description': (
                        'CENTRE panel only: estimated centimetres until a collision '
                        'if we drive straight. 0 = blocked now.'
                    ),
                },
                'can_forward': {
                    'type': 'boolean',
                    'description': 'Can we move forward at all from the CENTRE panel?',
                },
                'forward_hop': {
                    'type': 'string',
                    'enum': ['none', 'short', 'medium', 'long'],
                    'description': 'If can_forward, how far to hop. none if we cannot.',
                },
                'can_turn_left': {
                    'type': 'boolean',
                    'description': 'Is a left in-place turn useful (left side not a wall)?',
                },
                'can_turn_right': {
                    'type': 'boolean',
                    'description': 'Is a right in-place turn useful (right side not a wall)?',
                },
                'rear_left_clear': {
                    'type': 'boolean',
                    'description': (
                        'LEFT HALF of the −135° (rear-left) photo is clear behind us.'
                    ),
                },
                'rear_right_clear': {
                    'type': 'boolean',
                    'description': (
                        'RIGHT HALF of the +135° (rear-right) photo is clear behind us.'
                    ),
                },
                'can_backward': {
                    'type': 'boolean',
                    'description': (
                        'True only if we cannot go forward, cannot usefully turn, '
                        'AND both rear halves are clear.'
                    ),
                },
                'backward_hop': {
                    'type': 'string',
                    'enum': ['none', 'short'],
                    'description': 'short only when can_backward; else none.',
                },
                'goal_found': {
                    'type': 'boolean',
                    'description': 'Target object is clearly visible in a panel.',
                },
                'goal_view': {
                    'type': 'string',
                    'enum': ['none', 'left', 'straight', 'right'],
                },
            },
            'required': [
                'forward_clear_cm', 'can_forward', 'forward_hop',
                'can_turn_left', 'can_turn_right',
                'rear_left_clear', 'rear_right_clear',
                'can_backward', 'backward_hop',
                'goal_found', 'goal_view',
            ],
        },
    },
}


def _truthy(v) -> bool:
    if isinstance(v, bool):
        return v
    if v is None:
        return False
    s = str(v).strip().lower()
    return s in ('1', 'true', 'yes', 'y', 'on')


def seek_hop_from_forward_cm(cm) -> str:
    """Map estimated clear centimetres to a hop bucket."""
    try:
        n = int(float(cm))
    except (TypeError, ValueError):
        return 'medium'
    if n <= 0:
        return 'none'
    if n <= 30:
        return 'short'
    if n <= 70:
        return 'medium'
    return 'long'


def seek_action_from_schema(data: Optional[Dict[str, Any]], last_action=None) -> Dict[str, Any]:
    """Turn function-call tokens into action + drive_distance (no I/O)."""
    d = data if isinstance(data, dict) else {}
    can_fwd = _truthy(d.get('can_forward'))
    hop = str(d.get('forward_hop') or '').strip().lower()
    if hop not in ('none', 'short', 'medium', 'long'):
        hop = seek_hop_from_forward_cm(d.get('forward_clear_cm'))
    if hop == 'none':
        can_fwd = False
    if can_fwd and hop not in ('short', 'medium', 'long'):
        hop = 'medium'
    can_l = _truthy(d.get('can_turn_left'))
    can_r = _truthy(d.get('can_turn_right'))
    rear_l = _truthy(d.get('rear_left_clear'))
    rear_r = _truthy(d.get('rear_right_clear'))
    can_back = _truthy(d.get('can_backward'))
    back_hop = str(d.get('backward_hop') or '').strip().lower()
    try:
        cm = int(float(d.get('forward_clear_cm')))
    except (TypeError, ValueError):
        cm = None

    if can_fwd:
        return {
            'action': 'forward',
            'drive_distance': hop,
            'forward_clear_cm': cm,
            'can_forward': True,
            'can_turn_left': can_l,
            'can_turn_right': can_r,
            'can_backward': False,
        }

    if can_l and not can_r:
        side, turn = 'left', 'turn_left'
    elif can_r and not can_l:
        side, turn = 'right', 'turn_right'
    elif can_l and can_r:
        side, turn = 'left', 'turn_left'
    else:
        side, turn = None, None

    if turn:
        return {
            'action': turn,
            'drive_distance': 'short',
            'forward_clear_cm': cm,
            'can_forward': False,
            'can_turn_left': can_l,
            'can_turn_right': can_r,
            'can_backward': False,
            'open_side': side,
        }

    if seek_may_reverse(
        can_forward=False,
        can_turn=False,
        rear_left_clear=rear_l,
        rear_right_clear=rear_r,
        last_action=last_action,
    ) and (can_back or back_hop == 'short' or (rear_l and rear_r)):
        return {
            'action': 'backward',
            'drive_distance': 'short',
            'forward_clear_cm': cm,
            'can_forward': False,
            'can_turn_left': False,
            'can_turn_right': False,
            'can_backward': True,
        }

    return {
        'action': 'turn_left',
        'drive_distance': 'short',
        'forward_clear_cm': cm,
        'can_forward': False,
        'can_turn_left': can_l,
        'can_turn_right': can_r,
        'can_backward': False,
        'open_side': 'left',
    }


def seek_prefer_away_from_wall(
    action,
    *,
    obstacle_range='unknown',
    left_open=None,
    right_open=None,
    prefer_turn=None,
) -> Optional[str]:
    """If CENTRE is a wall, turn toward the more open side — never inch into it."""
    obs = seek_normalize_obstacle_range(obstacle_range, default='unknown')
    act = seek_normalize_action(action)
    if obs != 'near' or act != 'forward':
        return None
    pref = seek_normalize_action(prefer_turn) if prefer_turn else None
    try:
        lo = None if left_open is None else float(left_open)
        ro = None if right_open is None else float(right_open)
    except (TypeError, ValueError):
        lo = ro = None
    if lo is not None and ro is not None:
        if ro > lo + 0.04:
            return 'turn_right'
        if lo > ro + 0.04:
            return 'turn_left'
    if pref in ('turn_left', 'turn_right'):
        return pref
    return 'turn_left'


def seek_battery_block_reason(
    voltage,
    *,
    low_v: float = 9.5,
    gate_enabled: bool = True,
) -> Optional[str]:
    """If Seek should refuse to drive, return an operator-facing reason.

    Unknown voltage does **not** block (we cannot read it). Override with
    gate_enabled=False (`UGV_SEEK_BATTERY_GATE=0`).
    """
    if not gate_enabled:
        return None
    try:
        threshold = float(low_v)
    except (TypeError, ValueError):
        threshold = 9.5
    if voltage is None:
        return None
    try:
        v = float(voltage)
    except (TypeError, ValueError):
        return None
    if v <= threshold:
        return (
            f'Battery low ({v:.2f} V ≤ {threshold:.1f} V). '
            'Charge or set UGV_SEEK_BATTERY_GATE=0 to override.'
        )
    return None


# --- Dry-run (no chassis). Process-wide latch so every motor path can refuse. ---
_DRY_RUN_LOCK = threading.Lock()
_dry_run_active = False
SEEK_SWEEP_MIN_JPEG_BYTES = 800


def set_seek_dry_run(active: bool) -> bool:
    """Latch Seek dry-run. True = chassis commands must no-op."""
    global _dry_run_active
    with _DRY_RUN_LOCK:
        _dry_run_active = bool(active)
        return _dry_run_active


def seek_dry_run_active() -> bool:
    with _DRY_RUN_LOCK:
        return bool(_dry_run_active)


def seek_chassis_allowed(*, dry_run=None) -> bool:
    """False when Seek dry-run is on (explicit flag or process latch)."""
    if dry_run is None:
        dry_run = seek_dry_run_active()
    return not bool(dry_run)


def seek_drive_log_verb(dry_run=None) -> str:
    if dry_run is None:
        dry_run = seek_dry_run_active()
    return 'WOULD drive' if dry_run else 'driving'


def seek_live_start_error(*, dry_run: bool, confirm_live: bool) -> Optional[str]:
    """Live chassis Seek must be explicitly confirmed. Dry-run never needs this."""
    if dry_run:
        return None
    if not confirm_live:
        return (
            'Live drive refused: set dry_run=true (default) or pass confirm_live=true. '
            'Uncheck Dry run in the UI only when you mean to move the chassis.'
        )
    return None


def seek_views_are_rear_cruise(views) -> bool:
    """True when LEFT/RIGHT panels are rear ±135° (not look-down ±55°)."""
    pans = []
    for v in views or []:
        if not isinstance(v, dict):
            continue
        if v.get('name') not in ('left', 'right'):
            continue
        try:
            pans.append(abs(float(v.get('pan_deg'))))
        except (TypeError, ValueError):
            continue
    if not pans:
        return False
    return all(p >= 90.0 for p in pans)


def seek_found_confident(
    check,
    *,
    min_conf: float = 0.45,
    view_hits: int = 1,
    scan_conf: float = 0.22,
) -> Dict[str, Any]:
    """Whether to HALT Seek as found. Weak single-view hits stay candidates.

    Scan threshold (0.22) is for logging. Halt needs >= min_conf on one view
    or >=2 views at the scan threshold (look-up / second panel confirmed).
    """
    if not isinstance(check, dict) or not check.get('found'):
        return {'ok': False, 'reason': 'not found'}
    best = check.get('best') if isinstance(check.get('best'), dict) else {}
    raw_c = (
        best.get('confidence')
        if best.get('confidence') is not None
        else check.get('confidence')
    )
    try:
        conf = float(raw_c or 0.0)
    except (TypeError, ValueError):
        conf = 0.0
    try:
        hits = int(view_hits or check.get('view_hits') or 1)
    except (TypeError, ValueError):
        hits = 1
    if hits >= 2 and conf >= float(scan_conf):
        return {
            'ok': True,
            'reason': f'{hits} views conf={conf:.2f}',
            'confidence': conf,
            'view_hits': hits,
        }
    if conf >= float(min_conf):
        return {
            'ok': True,
            'reason': f'conf={conf:.2f}',
            'confidence': conf,
            'view_hits': hits,
        }
    return {
        'ok': False,
        'reason': (
            f'weak hit conf={conf:.2f} views={hits} '
            f'(need ≥{float(min_conf):.2f} or 2 views)'
        ),
        'confidence': conf,
        'view_hits': hits,
    }


def seek_sweep_scorecard(views, *, min_bytes: int = SEEK_SWEEP_MIN_JPEG_BYTES) -> Dict[str, Any]:
    """Per-view sweep QA: bytes, settle, labels. No OpenCV.

    A view is ok when it has a real JPEG (not a missing-camera tile).
    """
    cards = []
    missing = 0
    for v in views or []:
        if not isinstance(v, dict):
            continue
        jpeg = v.get('jpeg') or b''
        try:
            nbytes = int(v.get('bytes') or (len(jpeg) if jpeg else 0))
        except (TypeError, ValueError):
            nbytes = len(jpeg) if jpeg else 0
        ok = nbytes >= int(min_bytes)
        if not ok:
            missing += 1
        labels = v.get('detected_labels') or []
        if not isinstance(labels, list):
            labels = [str(labels)]
        cards.append({
            'name': v.get('name'),
            'pan_deg': v.get('pan_deg'),
            'bytes': nbytes,
            'settled': bool(v.get('pan_settled')),
            'labels': [str(x) for x in labels[:8]],
            'ok': ok,
            'wait_reason': v.get('pan_wait_reason'),
        })
    n = len(cards)
    all_ok = missing == 0 and n >= 3
    names = ', '.join(
        f"{c.get('name') or '?'}={'ok' if c.get('ok') else 'MISS'} {c.get('bytes') or 0}B"
        for c in cards
    ) or 'no views'
    return {
        'views': cards,
        'n': n,
        'missing': missing,
        'ok': all_ok,
        'summary': (
            f'sweep {"OK" if all_ok else "WEAK"} {n} views, {missing} missing — {names}'
        ),
    }
