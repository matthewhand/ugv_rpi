"""Device + loadout gate for Hailo optional features."""
from __future__ import annotations

import os
from typing import Any, Mapping, Optional

import loadout as loadout_mod

DEFAULT_DEVICE = "/dev/hailo0"


def device_present(device: Optional[str] = None) -> bool:
    path = device or DEFAULT_DEVICE
    try:
        return bool(os.path.exists(path))
    except OSError:
        return False


def features_enabled(
    lo: Optional[Mapping[str, Any]] = None,
    *,
    device_exists: Optional[bool] = None,
) -> bool:
    """True when loadout.hailo.enabled is true, or auto and device is present."""
    return loadout_mod.hailo_wanted(lo, device_exists=device_exists)


def status_payload(lo: Optional[Mapping[str, Any]] = None) -> dict[str, Any]:
    normalized = loadout_mod.normalize_loadout(lo)
    hailo = normalized["hailo"]
    present = device_present(hailo.get("device") or DEFAULT_DEVICE)
    active = features_enabled(normalized, device_exists=present)
    return {
        "device": hailo.get("device") or DEFAULT_DEVICE,
        "device_present": present,
        "enabled_mode": hailo.get("enabled"),
        "active": active,
        "pose": bool(hailo.get("pose")),
        "hand": bool(hailo.get("hand")),
        "face": bool(hailo.get("face")),
        "hef_dir": hailo.get("hef_dir"),
        "note": (
            "Separate app from Frigate; share /dev/hailo0 via HailoRT SHARED "
            "group if both run. HEFs are not in git — see docs/HAILO.md."
        ),
    }
