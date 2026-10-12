"""Hailo-8L optional vision features (pose / hand / face).

Separate from Frigate's detector. Enable via loadout.hailo or /dev/hailo0
auto-detect. Stubs no-op until HEFs are downloaded outside git.
"""
from __future__ import annotations

from .detect import device_present, features_enabled, status_payload
from .runner import tick

__all__ = [
    "device_present",
    "features_enabled",
    "status_payload",
    "tick",
]
