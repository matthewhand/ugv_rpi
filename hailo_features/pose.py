"""Pose estimation stub (yolov8s_pose HEF — not bundled)."""
from __future__ import annotations

from typing import Any, Optional


def run_once(frame: Any = None, *, hef_path: Optional[str] = None) -> dict[str, Any]:
    """No-op until HEF + HailoRT pipeline is wired.

    Community pattern: hailo-rpi5-examples pose_estimation (yolov8s_pose).
    """
    _ = frame
    return {
        "ok": False,
        "skipped": True,
        "reason": "scaffold_no_hef" if not hef_path else "scaffold_not_wired",
        "poses": [],
        "hef": hef_path,
    }
