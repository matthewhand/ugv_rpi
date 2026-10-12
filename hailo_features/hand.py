"""Hand / gesture stub (community hand GST / HEF pipelines — not bundled)."""
from __future__ import annotations

from typing import Any, Optional


def run_once(frame: Any = None, *, hef_path: Optional[str] = None) -> dict[str, Any]:
    """No-op until HEF + pipeline is wired."""
    _ = frame
    return {
        "ok": False,
        "skipped": True,
        "reason": "scaffold_no_hef" if not hef_path else "scaffold_not_wired",
        "hands": [],
        "gestures": [],
        "hef": hef_path,
    }
