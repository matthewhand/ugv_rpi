"""Pose estimation (yolov8s_pose HEF — not bundled in git)."""
from __future__ import annotations

import os
import shutil
import subprocess
from typing import Any, Optional


def _parse_hef_meta(hef_path: str) -> dict[str, Any]:
    """Soft probe: hailortcli parse-hef when CLI matches the driver."""
    cli = shutil.which("hailortcli")
    if not cli:
        return {"parsed": False, "reason": "no_hailortcli"}
    try:
        proc = subprocess.run(
            [cli, "parse-hef", hef_path],
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"parsed": False, "reason": f"parse_failed:{exc}"}
    out = (proc.stdout or "") + (proc.stderr or "")
    if proc.returncode != 0:
        # Common on host when library/driver mismatch; Frigate container is source of truth.
        return {
            "parsed": False,
            "reason": "parse_rc_%s" % proc.returncode,
            "detail": out.strip()[:400],
        }
    arch = ""
    for line in out.splitlines():
        if "Architecture HEF was compiled for" in line:
            arch = line.split(":", 1)[-1].strip()
            break
    return {
        "parsed": True,
        "architecture": arch or None,
        "detail": out.strip()[:400],
    }


def run_once(frame: Any = None, *, hef_path: Optional[str] = None) -> dict[str, Any]:
    """Probe HEF readiness. Full GST/HailoRT infer + SHARED with Frigate is next.

    Community pattern: hailo-rpi5-examples pose_estimation (yolov8s_pose).
    """
    _ = frame
    if not hef_path or not os.path.isfile(hef_path):
        return {
            "ok": False,
            "skipped": True,
            "reason": "scaffold_no_hef",
            "poses": [],
            "hef": hef_path,
        }
    meta = _parse_hef_meta(os.path.realpath(hef_path))
    if meta.get("parsed"):
        return {
            "ok": True,
            "skipped": False,
            "reason": "hef_ready_infer_not_wired",
            "poses": [],
            "hef": hef_path,
            "hef_meta": meta,
        }
    return {
        "ok": False,
        "skipped": True,
        "reason": "hef_present_parse_unavailable",
        "poses": [],
        "hef": hef_path,
        "hef_meta": meta,
    }
