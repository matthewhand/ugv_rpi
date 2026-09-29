"""MQTT topic sketch for Hailo optional features (no broker secrets).

Wire a real client later. Keep credentials in .env / gitignored config only.
"""
from __future__ import annotations

# Prefix is configurable via env UGV_HAILO_MQTT_PREFIX (default below).
DEFAULT_PREFIX = "ugv/hailo"

TOPICS = {
    "status": "{prefix}/status",
    "pose": "{prefix}/pose",
    "hand": "{prefix}/hand",
    "gesture": "{prefix}/gesture",
    "face": "{prefix}/face",
}


def topic(name: str, prefix: str | None = None) -> str:
    pref = prefix or DEFAULT_PREFIX
    tmpl = TOPICS.get(name)
    if not tmpl:
        raise KeyError(f"unknown hailo mqtt topic: {name}")
    return tmpl.format(prefix=pref.rstrip("/"))


def topic_map(prefix: str | None = None) -> dict[str, str]:
    return {k: topic(k, prefix) for k in TOPICS}
