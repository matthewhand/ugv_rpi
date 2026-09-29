"""Face-dimension / person-cue stub (retinaface-style pipelines — not bundled)."""
from __future__ import annotations

from typing import Any, Optional


def run_once(frame: Any = None, *, hef_path: Optional[str] = None) -> dict[str, Any]:
    """No-op until HEF + pipeline is wired.

    Intended outputs later: bbox + coarse face metrics useful as person cues
    (not identity / recognition storage).
    """
    _ = frame
    return {
        "ok": False,
        "skipped": True,
        "reason": "scaffold_no_hef" if not hef_path else "scaffold_not_wired",
        "faces": [],
        "hef": hef_path,
    }
