"""
In-app operational log for the UGV Flask control app.

Ring buffer of structured events (not stick heartbeats / per-frame noise).
Also mirrors to stdout so run_dev.sh / ugv.log stay aligned with the UI pane.
"""

from __future__ import annotations

import collections
import json
import threading
import time
from datetime import datetime, timezone
from typing import Any, Callable, Deque, Dict, List, Optional


_LEVELS = {
    'debug': 10,
    'info': 20,
    'warn': 30,
    'warning': 30,
    'error': 40,
}


def _norm_level(level: str) -> str:
    level = (level or 'info').strip().lower()
    if level == 'warning':
        return 'warn'
    if level not in _LEVELS:
        return 'info'
    return level


class AppLog:
    def __init__(self, maxlen: int = 500):
        self._buf: Deque[Dict[str, Any]] = collections.deque(maxlen=max(50, int(maxlen)))
        self._lock = threading.Lock()
        self._seq = 0
        self._listeners: List[Callable[[Dict[str, Any]], None]] = []
        # throttle key -> last emit monotonic time
        self._throttle: Dict[str, float] = {}

    def add_listener(self, fn: Callable[[Dict[str, Any]], None]) -> None:
        with self._lock:
            self._listeners.append(fn)

    def log(
        self,
        level: str,
        event: str,
        msg: Optional[str] = None,
        *,
        throttle_s: float = 0.0,
        throttle_key: Optional[str] = None,
        **fields: Any,
    ) -> Optional[Dict[str, Any]]:
        """Append an event. Returns entry dict, or None if throttled."""
        level = _norm_level(level)
        event = (event or 'event').strip() or 'event'
        now = time.time()

        if throttle_s and throttle_s > 0:
            key = throttle_key or f'{level}:{event}:{msg or ""}'
            with self._lock:
                last = self._throttle.get(key, 0.0)
                if now - last < throttle_s:
                    return None
                self._throttle[key] = now

        # Drop oversized / sensitive-ish blobs
        clean: Dict[str, Any] = {}
        for k, v in fields.items():
            if v is None:
                continue
            if isinstance(v, (str, int, float, bool)):
                if isinstance(v, str) and len(v) > 500:
                    clean[k] = v[:500] + '…'
                else:
                    clean[k] = v
            elif isinstance(v, dict):
                try:
                    s = json.dumps(v, default=str)
                    clean[k] = json.loads(s) if len(s) < 800 else s[:800] + '…'
                except Exception:
                    clean[k] = str(v)[:400]
            else:
                clean[k] = str(v)[:400]

        with self._lock:
            self._seq += 1
            entry = {
                'id': self._seq,
                'ts': now,
                'iso': datetime.fromtimestamp(now, tz=timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z',
                'level': level,
                'event': event,
                'msg': msg or event,
                'fields': clean,
            }
            self._buf.append(entry)
            listeners = list(self._listeners)

        line = self.format_line(entry)
        print(line, flush=True)

        for fn in listeners:
            try:
                fn(entry)
            except Exception:
                pass
        return entry

    def info(self, event: str, msg: Optional[str] = None, **fields: Any) -> Optional[Dict[str, Any]]:
        throttle_s = float(fields.pop('throttle_s', 0) or 0)
        throttle_key = fields.pop('throttle_key', None)
        return self.log('info', event, msg, throttle_s=throttle_s, throttle_key=throttle_key, **fields)

    def warn(self, event: str, msg: Optional[str] = None, **fields: Any) -> Optional[Dict[str, Any]]:
        throttle_s = float(fields.pop('throttle_s', 0) or 0)
        throttle_key = fields.pop('throttle_key', None)
        return self.log('warn', event, msg, throttle_s=throttle_s, throttle_key=throttle_key, **fields)

    def error(self, event: str, msg: Optional[str] = None, **fields: Any) -> Optional[Dict[str, Any]]:
        throttle_s = float(fields.pop('throttle_s', 0) or 0)
        throttle_key = fields.pop('throttle_key', None)
        return self.log('error', event, msg, throttle_s=throttle_s, throttle_key=throttle_key, **fields)

    def debug(self, event: str, msg: Optional[str] = None, **fields: Any) -> Optional[Dict[str, Any]]:
        throttle_s = float(fields.pop('throttle_s', 0) or 0)
        throttle_key = fields.pop('throttle_key', None)
        return self.log('debug', event, msg, throttle_s=throttle_s, throttle_key=throttle_key, **fields)

    def get(
        self,
        *,
        since_id: int = 0,
        limit: int = 200,
        min_level: str = 'debug',
        event: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        min_n = _LEVELS.get(_norm_level(min_level), 10)
        limit = max(1, min(int(limit or 200), 1000))
        since_id = int(since_id or 0)
        with self._lock:
            items = list(self._buf)
        out = []
        for e in items:
            if e['id'] <= since_id:
                continue
            if _LEVELS.get(e['level'], 0) < min_n:
                continue
            if event and e.get('event') != event:
                continue
            out.append(e)
        if len(out) > limit:
            out = out[-limit:]
        return out

    def clear(self) -> int:
        with self._lock:
            n = len(self._buf)
            self._buf.clear()
            return n

    def stats(self) -> Dict[str, Any]:
        with self._lock:
            return {
                'count': len(self._buf),
                'maxlen': self._buf.maxlen,
                'last_id': self._seq,
            }

    @staticmethod
    def format_line(entry: Dict[str, Any]) -> str:
        lvl = (entry.get('level') or 'info').upper()
        ts = entry.get('iso') or ''
        # local-ish short time for stdout: use iso UTC as-is
        event = entry.get('event') or ''
        msg = entry.get('msg') or ''
        fields = entry.get('fields') or {}
        extra = ''
        if fields:
            try:
                parts = [f'{k}={v!r}' if not isinstance(v, (int, float, bool)) else f'{k}={v}'
                         for k, v in list(fields.items())[:12]]
                extra = ' | ' + ' '.join(parts)
            except Exception:
                extra = ''
        if msg == event:
            return f'[ops {ts}] {lvl:5} {event}{extra}'
        return f'[ops {ts}] {lvl:5} {event}: {msg}{extra}'


# Process-wide singleton
app_log = AppLog(maxlen=int(__import__('os').environ.get('UGV_APP_LOG_MAX', '500') or 500))
