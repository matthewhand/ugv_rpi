"""One-shot tick for optional Hailo features (stubs when gated off / no HEF)."""
from __future__ import annotations

import os
from typing import Any, Mapping, Optional

from . import face, hand, pose
from .detect import features_enabled, status_payload
from .mqtt_topics import topic_map


def _hef(hef_dir: str, name: str) -> Optional[str]:
    path = os.path.join(hef_dir, name)
    return path if os.path.isfile(path) else None


def tick(
    lo: Optional[Mapping[str, Any]] = None,
    frame: Any = None,
    *,
    mqtt_prefix: Optional[str] = None,
) -> dict[str, Any]:
    """Run enabled stubs. Always safe: never loads HEFs that are missing."""
    status = status_payload(lo)
    out: dict[str, Any] = {
        "status": status,
        "mqtt_topics": topic_map(mqtt_prefix or os.environ.get("UGV_HAILO_MQTT_PREFIX")),
        "results": {},
    }
    if not status["active"]:
        out["skipped"] = True
        out["reason"] = "disabled_or_no_device"
        return out

    hef_dir = status.get("hef_dir") or "models/hailo"
    results = out["results"]
    if status.get("pose"):
        results["pose"] = pose.run_once(
            frame, hef_path=_hef(hef_dir, "yolov8s_pose.hef")
        )
    if status.get("hand"):
        results["hand"] = hand.run_once(
            frame, hef_path=_hef(hef_dir, "hand_landmark.hef")
        )
    if status.get("face"):
        results["face"] = face.run_once(
            frame, hef_path=_hef(hef_dir, "retinaface_mobilenet_v1.hef")
        )
    out["skipped"] = False
    return out
