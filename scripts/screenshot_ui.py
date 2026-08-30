#!/usr/bin/env python3
"""Capture UGV dashboard screenshots for visual QA + operator guide alignment.

Usage:
  .tools/playwright-env/bin/python scripts/screenshot_ui.py --base URL --out DIR
  .tools/playwright-env/bin/python scripts/screenshot_ui.py --catalog   # every guide_id

--catalog walks features/toggles listed in docs/operator-guide.md (guide_id column).
Destructive actions (ESP32 WiFi confirm, Seek start, ROS restart) are NOT confirmed.
"""
from __future__ import annotations

import argparse
import json
import sys
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

from playwright.sync_api import sync_playwright


VIEWPORTS = {
    "desktop": {"width": 1440, "height": 900},
    "phone_landscape": {"width": 844, "height": 390},
}


def shot(page, path: Path, full_page: bool = True, clip=None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    kwargs = {"path": str(path), "full_page": full_page}
    if clip:
        kwargs["full_page"] = False
        kwargs["clip"] = clip
    page.screenshot(**kwargs)
    print(f"  wrote {path.name} ({path.stat().st_size} bytes)")


def safe_click(page, selector: str, timeout: int = 4000) -> bool:
    try:
        loc = page.locator(selector).first
        if loc.count() == 0:
            return False
        loc.click(timeout=timeout)
        return True
    except Exception as e:
        print(f"  click fail {selector}: {e}")
        return False


def dismiss_dialogs(page) -> None:
    """Auto-dismiss native confirm/alert so destructive toggles don't hang."""
    page.on("dialog", lambda d: d.dismiss())


def record(results, out, vp_name, page, name, guide_id, note="") -> None:
    f = out / f"{vp_name}__{name}.png"
    try:
        shot(page, f)
        results.append(
            {
                "name": name,
                "guide_id": guide_id,
                "viewport": vp_name,
                "file": f.name,
                "url": page.url,
                "title": page.title(),
                "note": note,
                "ok": True,
            }
        )
    except Exception as e:
        results.append(
            {
                "name": name,
                "guide_id": guide_id,
                "viewport": vp_name,
                "file": None,
                "url": page.url,
                "error": str(e),
                "note": note,
                "ok": False,
            }
        )
        print(f"  FAIL {name}: {e}")


def go(page, base: str, path: str, wait_ms: int = 700, timeout_ms: int = 45000) -> bool:
    """Navigate and settle. Never raises: a slow/dead page must not kill the catalog."""
    try:
        page.goto(
            base.rstrip("/") + path,
            wait_until="domcontentloaded",
            timeout=timeout_ms,
        )
        page.wait_for_timeout(wait_ms)
        return True
    except Exception as e:
        print(f"  goto fail {path}: {e}")
        try:
            page.wait_for_timeout(wait_ms)
        except Exception:
            pass
        return False


def set_mode(page, mode: str) -> None:
    page.evaluate(
        """(m) => { try { localStorage.setItem('ugv_app_mode', m); } catch (e) {} }""",
        mode,
    )
    safe_click(page, f'button[data-mode="{mode}"], #mode-tab-{mode}')
    page.wait_for_timeout(500)


# ---------------------------------------------------------------------------
# Catalog: one step per operator-guide guide_id (desktop + phone for shell)
# ---------------------------------------------------------------------------

def run_catalog(base: str, out: Path, viewports: list[str]) -> list[dict]:
    results: list[dict] = []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        for vp_name in viewports:
            vp = VIEWPORTS[vp_name]
            print(f"\n=== CATALOG {vp_name} {vp} ===")
            context = browser.new_context(viewport=vp, ignore_https_errors=True)
            page = context.new_page()
            page.set_default_timeout(12000)
            dismiss_dialogs(page)

            # --- Shell: Raw ---
            go(page, base, "/")
            page.evaluate("""() => {
              try {
                localStorage.setItem('ugv_app_mode', 'raw');
                localStorage.setItem('ugv_chassis_heartbeat', '1');
              } catch (e) {}
            }""")
            go(page, base, "/", wait_ms=1000)
            set_mode(page, "raw")
            record(results, out, vp_name, page, "shell_raw", "shell.raw", "Raw mode default")
            record(results, out, vp_name, page, "raw_overview", "raw.overview", "Raw full panel")

            # Navbar crop
            try:
                f = out / f"{vp_name}__shell_navbar.png"
                shot(
                    page,
                    f,
                    full_page=False,
                    clip={"x": 0, "y": 0, "width": vp["width"], "height": min(120, vp["height"])},
                )
                results.append(
                    {
                        "name": "shell_navbar",
                        "guide_id": "shell.navbar_phone" if "phone" in vp_name else "shell.stop",
                        "viewport": vp_name,
                        "file": f.name,
                        "ok": True,
                        "note": "Navbar chrome crop",
                    }
                )
            except Exception as e:
                print(f"  navbar crop fail: {e}")

            # Idle heartbeat OFF then ON (localStorage + button)
            if safe_click(page, "#chassis-heartbeat-btn"):
                page.wait_for_timeout(300)
                record(results, out, vp_name, page, "shell_hb_off", "shell.hb_off", "HB toggled")
                safe_click(page, "#chassis-heartbeat-btn")
                page.wait_for_timeout(200)
                record(results, out, vp_name, page, "shell_hb_on", "shell.hb_on", "HB restored")

            # RTSP toggle (safe)
            if safe_click(page, "#rtsp-toggle-btn"):
                page.wait_for_timeout(400)
                record(results, out, vp_name, page, "shell_rtsp_toggled", "shell.rtsp_on", "RTSP toggled once")
                safe_click(page, "#rtsp-toggle-btn")
                page.wait_for_timeout(200)
                record(results, out, vp_name, page, "shell_rtsp_restored", "shell.rtsp_off", "RTSP restored")

            # Control path chip (may show restart banner — do not restart)
            path_label = page.locator("#motor-toggle-btn").inner_text() if page.locator("#motor-toggle-btn").count() else ""
            record(
                results,
                out,
                vp_name,
                page,
                "shell_path_current",
                "shell.path_direct" if "Direct" in path_label else "shell.path_ros2",
                f"path chip text={path_label!r} (not flipped — avoids UART thrash)",
            )

            # WiFi chip only (dismiss confirm via dialog handler)
            record(results, out, vp_name, page, "shell_wifi_chip", "shell.wifi_chip", "WiFi chip idle")

            # Lidar chip (do not toggle — USB exclusive lock)
            lidar_visible = page.locator("#lidar-toggle-btn").count() and page.locator("#lidar-toggle-btn").is_visible()
            record(
                results,
                out,
                vp_name,
                page,
                "shell_lidar",
                "shell.lidar",
                "Lidar chip visible" if lidar_visible else "Lidar chip hidden (no USB / hangar off)",
            )

            # Ops log
            if safe_click(page, "#ops-log-btn"):
                page.wait_for_timeout(600)
                record(results, out, vp_name, page, "shell_ops_log", "shell.ops_log", "App log open")
                safe_click(page, "#ops-log-close, #ops-log-btn")

            # Twin drawer
            if safe_click(page, "#twin-btn"):
                page.wait_for_timeout(1500)
                record(results, out, vp_name, page, "shell_twin_drawer", "shell.twin_drawer", "Twin drawer")
                record(results, out, vp_name, page, "twin_drawer", "twin.drawer", "Same as shell twin drawer")
                safe_click(page, "#twin-close, #twin-btn")

            # --- Raw feature toggles (desktop only to limit explosion; phone gets overview) ---
            if vp_name == "desktop":
                set_mode(page, "raw")
                # Speed
                if safe_click(page, "button:has-text('Middle')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_speed_mid", "raw.speed_mid")
                if safe_click(page, "button:has-text('Fast')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_speed_fast", "raw.speed_fast")
                if safe_click(page, "button:has-text('Slow')"):
                    page.wait_for_timeout(150)

                # Steady ON — scope to PT Steady row (avoid matching unrelated ON buttons)
                if safe_click(page, "text=PT Steady/Ahead >> xpath=.. >> button:has-text('ON')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_steady_on", "raw.steady_on")
                elif safe_click(page, ".ctl9 button.ctl_btn:has-text('ON')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_steady_on", "raw.steady_on")

                # CV detection — use ctl_btn_mov / faces if present
                if safe_click(page, ".ctl_btn_mov, button:has-text('Movtion')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_cv_motion", "raw.cv_motion")
                if safe_click(page, ".ctl_btn_faces, button:has-text('Faces')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_cv_faces", "raw.cv_faces")
                if safe_click(page, "button:has-text('None')"):
                    page.wait_for_timeout(100)

                if safe_click(page, ".ctl_btn_caputure, button:has-text('Capture')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_reaction_capture", "raw.reaction_capture")

                if safe_click(page, "button:has-text('AUTODRIVE')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_autodrive", "raw.autodrive")
                if safe_click(page, "button:has-text('OBJECTS')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_objects", "raw.objects")
                if safe_click(page, "button:has-text('MP FACE')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_mp_face", "raw.mp_face")

                # Head / base lights — stock ctl_btn labels
                if safe_click(page, "text=Head Light Ctrl >> xpath=.. >> button:has-text('ON')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_headlight_on", "raw.headlight_on")
                if safe_click(page, "button:has-text('BASE ON')"):
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "raw_base_light", "raw.base_light")

                # Lights — scroll to PTZ block
                page.locator("#ptz-selftest-btn, button:has-text('Run PTZ')").first.scroll_into_view_if_needed()
                page.wait_for_timeout(200)
                record(results, out, vp_name, page, "raw_ptz_selftest", "raw.ptz_selftest")

            # --- Chat ---
            set_mode(page, "chat")
            page.wait_for_timeout(700)
            record(results, out, vp_name, page, "shell_chat", "shell.chat")
            record(results, out, vp_name, page, "chat_overview", "chat.overview")
            record(results, out, vp_name, page, "chat_still_empty", "chat.still_empty")
            # attach off
            if page.locator("#chat-attach").count():
                page.locator("#chat-attach").uncheck()
                page.wait_for_timeout(150)
                record(results, out, vp_name, page, "chat_attach_off", "chat.attach_off")
                page.locator("#chat-attach").check()
            # grab still
            if safe_click(page, "#chat-snap-btn"):
                page.wait_for_timeout(1200)
                record(results, out, vp_name, page, "chat_still_grabbed", "chat.still_grabbed")
            record(results, out, vp_name, page, "chat_link_ai", "chat.link_ai")

            # --- Seek ---
            set_mode(page, "seek")
            page.wait_for_timeout(800)
            record(results, out, vp_name, page, "shell_seek", "shell.seek")
            record(results, out, vp_name, page, "seek_overview", "seek.overview")
            record(results, out, vp_name, page, "seek_pano_empty", "seek.pano_empty")
            record(results, out, vp_name, page, "seek_actions", "seek.actions")
            record(results, out, vp_name, page, "seek_limits", "seek.limits")

            if safe_click(page, "#seek-mode-detector"):
                page.wait_for_timeout(250)
                record(results, out, vp_name, page, "seek_mode_a", "seek.mode_a")
            if safe_click(page, "#seek-mode-detector-llm"):
                page.wait_for_timeout(250)
                record(results, out, vp_name, page, "seek_mode_b", "seek.mode_b")
            if safe_click(page, "#seek-mode-llm-vision"):
                page.wait_for_timeout(250)
                record(results, out, vp_name, page, "seek_mode_c", "seek.mode_c")
            # restore default b
            safe_click(page, "#seek-mode-detector-llm")
            page.wait_for_timeout(150)
            record(results, out, vp_name, page, "seek_goal_select", "seek.goal_select")

            if page.locator("#seek-on-found").count():
                page.locator("#seek-on-found").select_option("none")
                page.wait_for_timeout(200)
                record(results, out, vp_name, page, "seek_onfound_none", "seek.onfound_none")
                page.locator("#seek-on-found").select_option("tts")
                page.wait_for_timeout(200)
                record(results, out, vp_name, page, "seek_onfound_tts", "seek.onfound_tts")

            # scene nav off (mode b)
            scene = page.locator("#seek-llm-scene-nav")
            try:
                if scene.count() and scene.is_enabled():
                    if scene.is_checked():
                        scene.uncheck()
                    page.wait_for_timeout(200)
                    record(results, out, vp_name, page, "seek_scenenav_off", "seek.scenenav_off")
                    try:
                        scene.check()
                    except Exception as e:
                        print(f"  scene.check restore fail: {e}")
            except Exception as e:
                print(f"  scenenav toggle fail: {e}")

            # dry-run default ON (guide row)
            record(results, out, vp_name, page, "seek_dry_run", "seek.dry_run", "Dry run checkbox default ON")

            # Simulated running chrome (UI only — does not start Seek / motors)
            # Uses real lock path: config disabled + is-locked + pill + body class
            try:
                page.evaluate(
                    "() => {"
                    " if (typeof window.ugvSetSeekControlsRunning === 'function')"
                    "   window.ugvSetSeekControlsRunning(true);"
                    " else document.body.classList.add('seek-running');"
                    " if (typeof window.ugvSetSeekRunningIndicator === 'function')"
                    "   window.ugvSetSeekRunningIndicator(true, { step: 3, max_steps: 30 });"
                    " else {"
                    "   var p = document.getElementById('seek-running-pill');"
                    "   if (p) { p.removeAttribute('hidden'); p.textContent = 'Seek running · 3/30'; }"
                    " }"
                    "}"
                )
                page.wait_for_timeout(350)
                record(
                    results,
                    out,
                    vp_name,
                    page,
                    "seek_running_chrome",
                    "seek.running_chrome",
                    "Simulated running chrome: lock config + pill (no real seek / motors)",
                )
                page.evaluate(
                    "() => {"
                    " if (typeof window.ugvSetSeekControlsRunning === 'function')"
                    "   window.ugvSetSeekControlsRunning(false);"
                    " else document.body.classList.remove('seek-running');"
                    " if (typeof window.ugvSetSeekRunningIndicator === 'function')"
                    "   window.ugvSetSeekRunningIndicator(false);"
                    " else {"
                    "   var p = document.getElementById('seek-running-pill');"
                    "   if (p) { p.setAttribute('hidden', ''); }"
                    " }"
                    "}"
                )
            except Exception as e:
                print(f"  seek.running_chrome fail: {e}")

            # --- Loadout mode ---
            set_mode(page, "loadout")
            page.wait_for_timeout(600)
            record(results, out, vp_name, page, "shell_loadout", "shell.loadout", "Loadout tab: hangar bays")

            # --- AI page ---
            go(page, base, "/ai", wait_ms=1200)
            record(results, out, vp_name, page, "ai_overview", "ai.overview")
            record(results, out, vp_name, page, "ai_still_empty", "ai.still_empty")
            record(results, out, vp_name, page, "ai_config_panel", "ai.config_panel")
            record(results, out, vp_name, page, "ai_tools_tree", "ai.tools_tree")
            if "phone" in vp_name:
                record(results, out, vp_name, page, "ai_landscape", "ai.landscape")

            # --- Twin full page (local three.js + twin.js; keep a budget for WebGL) ---
            go(page, base, "/3d", wait_ms=1800, timeout_ms=45000)
            record(results, out, vp_name, page, "twin_page", "twin.page")

            # --- Settings / media ---
            go(page, base, "/settings.html", wait_ms=500)
            record(results, out, vp_name, page, "settings_overview", "settings.overview")
            go(page, base, "/photo.html", wait_ms=500)
            record(results, out, vp_name, page, "photo_overview", "photo.overview")
            go(page, base, "/video.html", wait_ms=500)
            record(results, out, vp_name, page, "video_overview", "video.overview")

            context.close()
        browser.close()
    return results


def run_simple(base: str, out: Path, viewports: list[str]) -> list[dict]:
    """Legacy shorter suite (pages only)."""
    results = []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        for vp_name in viewports:
            vp = VIEWPORTS[vp_name]
            print(f"\n=== SIMPLE {vp_name} ===")
            context = browser.new_context(viewport=vp, ignore_https_errors=True)
            page = context.new_page()
            dismiss_dialogs(page)
            go(page, base, "/")
            set_mode(page, "raw")
            record(results, out, vp_name, page, "01_index_raw", "shell.raw")
            set_mode(page, "chat")
            record(results, out, vp_name, page, "02_index_chat", "shell.chat")
            set_mode(page, "seek")
            record(results, out, vp_name, page, "03_index_seek", "shell.seek")
            if safe_click(page, "#ops-log-btn"):
                page.wait_for_timeout(400)
                record(results, out, vp_name, page, "04_ops_log", "shell.ops_log")
            if safe_click(page, "#twin-btn"):
                page.wait_for_timeout(1200)
                record(results, out, vp_name, page, "05_twin", "shell.twin_drawer")
            go(page, base, "/ai")
            record(results, out, vp_name, page, "06_ai", "ai.overview")
            go(page, base, "/3d")
            record(results, out, vp_name, page, "07_3d", "twin.page")
            go(page, base, "/settings.html")
            record(results, out, vp_name, page, "08_settings", "settings.overview")
            context.close()
        browser.close()
    return results


def parse_guide_ids(guide_path: Path) -> list[str]:
    ids = []
    if not guide_path.is_file():
        return ids
    for line in guide_path.read_text().splitlines():
        line = line.strip()
        if not line.startswith("|"):
            continue
        parts = [p.strip() for p in line.split("|")]
        # | guide_id | Feature | ...
        if len(parts) >= 3 and parts[1] and parts[1] not in ("guide_id", "----------", "---"):
            gid = parts[1].strip().strip("`").strip()
            if "." in gid and " " not in gid and not gid.startswith("-"):
                ids.append(gid)
    return ids


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://127.0.0.1:5000")
    ap.add_argument("--out", default="ui_shots")
    ap.add_argument("--viewports", default="desktop,phone_landscape")
    ap.add_argument(
        "--catalog",
        action="store_true",
        help="Walk every operator-guide feature/toggle (recommended for QA)",
    )
    ap.add_argument(
        "--guide",
        default="docs/operator-guide.md",
        help="Operator guide path for coverage report",
    )
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    try:
        with urllib.request.urlopen(args.base.rstrip("/") + "/", timeout=5) as r:
            print(f"base {args.base} -> HTTP {r.status}")
    except Exception as e:
        print(f"ERROR: app not reachable at {args.base}: {e}", file=sys.stderr)
        return 2

    vps = [v.strip() for v in args.viewports.split(",") if v.strip() and v.strip() in VIEWPORTS]
    if args.catalog:
        results = run_catalog(args.base, out, vps)
    else:
        results = run_simple(args.base, out, vps)

    guide_ids = parse_guide_ids(Path(args.guide))
    captured_ids = sorted({r.get("guide_id") for r in results if r.get("guide_id")})
    missing = [g for g in guide_ids if g not in captured_ids]
    extra = [g for g in captured_ids if g not in guide_ids]

    manifest = {
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "base": args.base,
        "mode": "catalog" if args.catalog else "simple",
        "guide": args.guide,
        "guide_ids_total": len(guide_ids),
        "guide_ids_captured": len([g for g in guide_ids if g in captured_ids]),
        "missing_guide_ids": missing,
        "extra_guide_ids": extra,
        "ok_count": sum(1 for r in results if r.get("ok")),
        "fail_count": sum(1 for r in results if not r.get("ok")),
        "results": results,
    }
    man = out / "manifest.json"
    man.write_text(json.dumps(manifest, indent=2))
    print(
        f"\nmanifest {man} mode={manifest['mode']} "
        f"ok={manifest['ok_count']} fail={manifest['fail_count']} "
        f"guide_coverage={manifest['guide_ids_captured']}/{manifest['guide_ids_total']}"
    )
    if missing:
        print("missing guide_ids:", ", ".join(missing))
    return 0 if manifest["fail_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
