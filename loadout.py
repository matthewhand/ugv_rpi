"""Chassis + attachment loadout (software profile).

Runtime state lives in a gitignored `.loadout.json`. Machine-specific
drive signs stay in `config.yaml` and are never written by this module.

Selecting `attachment=roarm2` marks the USB RoArm path as wanted. Flask
starts/stops `roarm_ctrl` only when that attachment is active; rover+ptz
keeps `roarm_started=false`.
"""

from __future__ import annotations

import json
import os
import threading
from typing import Any, Mapping, MutableMapping, Optional

LOADOUT_FILENAME = ".loadout.json"

# Waveshare `base_config.main_type` / `module_type` (see `s 22` / T:900).
BASES: dict[str, dict[str, Any]] = {
    "rover": {
        "id": "rover",
        "label": "Wheeled rover",
        "main_type": 2,
        "drive": "wheels",
        "robot_name": "UGV Rover",
    },
    "beast": {
        "id": "beast",
        "label": "Tracked beast",
        "main_type": 3,
        "drive": "tracks",
        "robot_name": "UGV Beast",
    },
}

ATTACHMENTS: dict[str, dict[str, Any]] = {
    "none": {
        "id": "none",
        "label": "None",
        "module_type": 0,
        "wired": True,
    },
    "ptz": {
        "id": "ptz",
        "label": "PTZ camera turret",
        "module_type": 2,
        "wired": True,
    },
    "roarm2": {
        "id": "roarm2",
        "label": "RoArm-M2",
        "module_type": 1,
        # Software path is wired; hardware open is gated by hangar selection.
        "wired": True,
    },
}

CAMERA_PREFERS = ("auto", "csi", "usb")

DEFAULT_LOADOUT: dict[str, Any] = {
    "base": "rover",
    "attachment": "ptz",
    "use_lidar": False,
    "camera_prefer": "auto",
}

ARM_OFFLINE_MESSAGE = "arm offline — USB RoArm not started"
ARM_READY_MESSAGE = "RoArm USB path active when hangar attachment=roarm2"

# Keys that belong on a machine and must never be copied from a loadout.
_DRIVE_SIGN_KEYS = ("drive_linear_sign", "drive_angular_sign")


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off", ""):
        return False
    return default


def base_from_main_type(main_type: Any) -> str:
    try:
        n = int(main_type)
    except (TypeError, ValueError):
        return DEFAULT_LOADOUT["base"]
    for key, spec in BASES.items():
        if spec["main_type"] == n:
            return key
    return DEFAULT_LOADOUT["base"]


def attachment_from_module_type(module_type: Any) -> str:
    try:
        n = int(module_type)
    except (TypeError, ValueError):
        return DEFAULT_LOADOUT["attachment"]
    for key, spec in ATTACHMENTS.items():
        if spec["module_type"] == n:
            return key
    return DEFAULT_LOADOUT["attachment"]


def normalize_loadout(data: Optional[Mapping[str, Any]] = None) -> dict[str, Any]:
    """Return a validated loadout. Unknown keys dropped; bad values → default."""
    src = dict(data or {})
    base = str(src.get("base") or "").strip().lower()
    if base not in BASES:
        base = DEFAULT_LOADOUT["base"]
    attachment = str(src.get("attachment") or "").strip().lower()
    if attachment not in ATTACHMENTS:
        attachment = DEFAULT_LOADOUT["attachment"]
    camera_prefer = str(src.get("camera_prefer") or DEFAULT_LOADOUT["camera_prefer"]).strip().lower()
    if camera_prefer not in CAMERA_PREFERS:
        camera_prefer = DEFAULT_LOADOUT["camera_prefer"]
    return {
        "base": base,
        "attachment": attachment,
        "use_lidar": _as_bool(src.get("use_lidar"), DEFAULT_LOADOUT["use_lidar"]),
        "camera_prefer": camera_prefer,
    }


def wants_roarm(loadout: Optional[Mapping[str, Any]] = None) -> bool:
    """True when hangar attachment asks for USB RoArm drivers."""
    lo = normalize_loadout(loadout)
    return lo["attachment"] == "roarm2"


def loadout_from_config(cfg: Optional[Mapping[str, Any]]) -> dict[str, Any]:
    """Derive a loadout from `config.yaml` `base_config` (no drive-sign copy)."""
    base_cfg = (cfg or {}).get("base_config") or {}
    prefer = str(base_cfg.get("camera_prefer") or DEFAULT_LOADOUT["camera_prefer"]).strip().lower()
    return normalize_loadout(
        {
            "base": base_from_main_type(base_cfg.get("main_type")),
            "attachment": attachment_from_module_type(base_cfg.get("module_type")),
            "use_lidar": base_cfg.get("use_lidar", DEFAULT_LOADOUT["use_lidar"]),
            "camera_prefer": prefer,
        }
    )


def effective_types(loadout: Mapping[str, Any]) -> dict[str, Any]:
    lo = normalize_loadout(loadout)
    base = BASES[lo["base"]]
    att = ATTACHMENTS[lo["attachment"]]
    arm_wired = bool(att.get("wired", True))
    arm_wanted = wants_roarm(lo)
    return {
        "main_type": int(base["main_type"]),
        "module_type": int(att["module_type"]),
        "use_lidar": bool(lo["use_lidar"]),
        "camera_prefer": lo["camera_prefer"],
        "robot_name": base["robot_name"],
        "drive": base["drive"],
        "arm_wired": arm_wired,
        "arm_wanted": arm_wanted,
        "arm_offline": False,
        "arm_message": ARM_READY_MESSAGE if arm_wanted else None,
    }


def apply_loadout_to_config(
    cfg: MutableMapping[str, Any],
    loadout: Mapping[str, Any],
) -> MutableMapping[str, Any]:
    """Overlay type / lidar / name onto `cfg['base_config']`.

    Never writes `drive_linear_sign`, `drive_angular_sign`, or persists yaml.
    In-memory `arm_config.transport` mirrors hangar attachment for USB gating.
    """
    if "base_config" not in cfg or not isinstance(cfg.get("base_config"), dict):
        cfg["base_config"] = {}
    # Snapshot drive signs so a sloppy caller cannot clobber them via update().
    preserved = {k: cfg["base_config"][k] for k in _DRIVE_SIGN_KEYS if k in cfg["base_config"]}
    types = effective_types(loadout)
    cfg["base_config"]["main_type"] = types["main_type"]
    cfg["base_config"]["module_type"] = types["module_type"]
    cfg["base_config"]["use_lidar"] = types["use_lidar"]
    cfg["base_config"]["camera_prefer"] = types["camera_prefer"]
    cfg["base_config"]["robot_name"] = types["robot_name"]
    cfg["base_config"].update(preserved)

    # Session-only arm transport mirror (never written to config.yaml by this module).
    if wants_roarm(loadout):
        arm = cfg.get("arm_config")
        if not isinstance(arm, dict):
            arm = {}
            cfg["arm_config"] = arm
        arm.setdefault("transport", "usb_serial")
        arm.setdefault("default_pose", "travel_tuck")
        arm.setdefault("ui_aim_default", "roarm")
    else:
        # Drop session overlay so PTZ robots do not look USB-armed.
        arm = cfg.get("arm_config")
        if isinstance(arm, dict) and arm.get("_loadout_session"):
            cfg.pop("arm_config", None)
        elif isinstance(arm, dict) and arm.get("transport") == "usb_serial" and not arm.get("persist"):
            # Keep machine-authored arm_config if present; hangar none/ptz still
            # gates start via wants_roarm(). Clear only ephemeral marker.
            arm.pop("_loadout_session", None)
    return cfg


def camera_strategy(loadout: Mapping[str, Any]) -> str:
    """Which camera path to try first: `usb` or `csi`.

    Rover CSI stays the default on `auto`. USB rediscovery is for Beast UVC
    (or an explicit `camera_prefer: usb`).
    """
    lo = normalize_loadout(loadout)
    prefer = lo["camera_prefer"]
    if prefer == "usb":
        return "usb"
    if prefer == "csi":
        return "csi"
    # auto: beast tracks → USB first; wheeled rover → CSI first
    return "usb" if lo["base"] == "beast" else "csi"


def catalog_payload() -> dict[str, Any]:
    return {
        "bases": [
            {k: spec[k] for k in ("id", "label", "main_type", "drive")}
            for spec in BASES.values()
        ],
        "attachments": [
            {k: spec[k] for k in ("id", "label", "module_type", "wired")}
            for spec in ATTACHMENTS.values()
        ],
        "camera_prefers": list(CAMERA_PREFERS),
    }


def drive_signs_from_config(cfg: Optional[Mapping[str, Any]]) -> dict[str, Any]:
    base_cfg = (cfg or {}).get("base_config") or {}
    out: dict[str, Any] = {}
    for key in _DRIVE_SIGN_KEYS:
        if key in base_cfg:
            try:
                out[key] = -1.0 if float(base_cfg[key]) < 0 else 1.0
            except (TypeError, ValueError):
                out[key] = base_cfg[key]
    return out


def public_payload(
    loadout: Mapping[str, Any],
    cfg: Optional[Mapping[str, Any]] = None,
    *,
    persisted_path: str = LOADOUT_FILENAME,
    roarm_started: bool = False,
    arm_status: Optional[str] = None,
    arm_message: Optional[str] = None,
) -> dict[str, Any]:
    lo = normalize_loadout(loadout)
    types = effective_types(lo)
    started = bool(roarm_started) and wants_roarm(lo)
    if arm_status is None:
        if lo["attachment"] == "none":
            arm_status = "n/a"
        elif not wants_roarm(lo):
            arm_status = "ready" if lo["attachment"] == "ptz" else "n/a"
        else:
            arm_status = "started" if started else "wanted"
    if arm_message is None:
        if wants_roarm(lo) and not started:
            arm_message = ARM_OFFLINE_MESSAGE
        elif wants_roarm(lo):
            arm_message = ARM_READY_MESSAGE
        else:
            arm_message = None
    payload: dict[str, Any] = {
        "ok": True,
        "loadout": lo,
        "effective": types,
        "persisted_path": persisted_path,
        "roarm_started": started,
        "note": (
            "drive signs stay in config.yaml; RoArm drivers start only when "
            "hangar attachment=roarm2"
        ),
        **catalog_payload(),
        **drive_signs_from_config(cfg),
        "arm_status": arm_status,
        "arm_message": arm_message,
    }
    return payload


class LoadoutStore:
    """Thread-safe `.loadout.json` store. Hardware side effects live in app.py."""

    def __init__(self, root_dir: str, path: Optional[str] = None):
        self.root_dir = root_dir
        self.path = path or os.path.join(root_dir, LOADOUT_FILENAME)
        self._lock = threading.Lock()
        self._data = dict(DEFAULT_LOADOUT)

    def load(self, fallback_from_config: Optional[Mapping[str, Any]] = None) -> dict[str, Any]:
        with self._lock:
            loaded = None
            try:
                if os.path.isfile(self.path):
                    with open(self.path, "r", encoding="utf-8") as fh:
                        loaded = json.load(fh)
            except Exception:
                loaded = None
            if isinstance(loaded, dict):
                self._data = normalize_loadout(loaded)
            elif fallback_from_config is not None:
                self._data = loadout_from_config(fallback_from_config)
            else:
                self._data = dict(DEFAULT_LOADOUT)
            return dict(self._data)

    def get(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._data)

    def save(self, data: Mapping[str, Any]) -> dict[str, Any]:
        normalized = normalize_loadout(data)
        parent = os.path.dirname(self.path) or "."
        os.makedirs(parent, exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(normalized, fh, indent=2, sort_keys=True)
            fh.write("\n")
        os.replace(tmp, self.path)
        with self._lock:
            self._data = normalized
        return dict(normalized)

    def set(self, patch: Optional[Mapping[str, Any]] = None) -> dict[str, Any]:
        """Merge a partial patch, persist, return the new loadout."""
        current = self.get()
        if patch:
            merged = {**current, **dict(patch)}
        else:
            merged = current
        return self.save(merged)

    def public(
        self,
        cfg: Optional[Mapping[str, Any]] = None,
        *,
        roarm_started: bool = False,
        arm_status: Optional[str] = None,
        arm_message: Optional[str] = None,
    ) -> dict[str, Any]:
        return public_payload(
            self.get(),
            cfg,
            persisted_path=os.path.basename(self.path) or LOADOUT_FILENAME,
            roarm_started=roarm_started,
            arm_status=arm_status,
            arm_message=arm_message,
        )


def load_effective(root_dir: str, cfg: Optional[Mapping[str, Any]] = None) -> dict[str, Any]:
    """Read `.loadout.json` (or derive from cfg) without mutating callers."""
    store = LoadoutStore(root_dir)
    return store.load(fallback_from_config=cfg)
