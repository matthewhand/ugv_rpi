#!/usr/bin/env python3
"""Capture full-page screenshots of every UGV dashboard surface via Playwright.

Usage:
  .tools/playwright-env/bin/python scripts/screenshot_ui.py [--base URL] [--out DIR]

Outputs PNGs + a manifest.json under the out dir (default: ui_shots/).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from playwright.sync_api import sync_playwright


# Viewport presets: desktop teleop + phone landscape (real teleop form factor)
VIEWPORTS = {
    "desktop": {"width": 1440, "height": 900},
    "phone_landscape": {"width": 844, "height": 390},
}


def shot(page, path: Path, full_page: bool = True) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    page.screenshot(path=str(path), full_page=full_page)
    print(f"  wrote {path} ({path.stat().st_size} bytes)")


def safe_click(page, selector: str, timeout: int = 3000) -> bool:
    try:
        loc = page.locator(selector).first
        if loc.count() == 0:
            return False
        loc.click(timeout=timeout)
        return True
    except Exception as e:
        print(f"  click fail {selector}: {e}")
        return False


def capture_suite(base: str, out: Path, vp_name: str, vp: dict) -> list[dict]:
    results = []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        context = browser.new_context(
            viewport=vp,
            device_scale_factor=1,
            ignore_https_errors=True,
        )
        page = context.new_page()
        page.set_default_timeout(15000)

        def go(path: str, wait_ms: int = 800) -> None:
            url = base.rstrip("/") + path
            page.goto(url, wait_until="domcontentloaded")
            page.wait_for_timeout(wait_ms)

        def record(name: str, note: str = "") -> None:
            f = out / f"{vp_name}__{name}.png"
            try:
                shot(page, f)
                results.append(
                    {
                        "name": name,
                        "viewport": vp_name,
                        "file": str(f.name),
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
                        "viewport": vp_name,
                        "file": None,
                        "url": page.url,
                        "error": str(e),
                        "note": note,
                        "ok": False,
                    }
                )
                print(f"  FAIL {name}: {e}")

        # ---- Primary pages ----
        print(f"[{vp_name}] / (Raw default)")
        go("/")
        # Clear any leftover localStorage mode so we start from raw
        page.evaluate(
            """() => {
              try { localStorage.setItem('ugv_app_mode', 'raw'); } catch (e) {}
            }"""
        )
        go("/", wait_ms=1200)
        record("01_index_raw", "Main dashboard Raw mode")

        print(f"[{vp_name}] Chat mode tab")
        if safe_click(page, '#mode-tab-chat, button[data-mode="chat"]'):
            page.wait_for_timeout(700)
            record("02_index_chat", "Chat mode panel")
        else:
            record("02_index_chat", "Chat tab click failed")

        print(f"[{vp_name}] Seek mode tab")
        if safe_click(page, '#mode-tab-seek, button[data-mode="seek"]'):
            page.wait_for_timeout(900)
            record("03_index_seek", "Seek mode panel")
        else:
            record("03_index_seek", "Seek tab click failed")

        print(f"[{vp_name}] App log drawer open")
        if safe_click(page, "#ops-log-btn"):
            page.wait_for_timeout(500)
            record("04_ops_log_open", "Ops log drawer")
            # close via same toggle if present
            safe_click(page, "#ops-log-btn")

        print(f"[{vp_name}] 3D Twin drawer open")
        if safe_click(page, "#twin-btn"):
            page.wait_for_timeout(1500)
            record("05_twin_drawer", "3D Twin popup drawer")
            safe_click(page, "#twin-btn")

        # ---- Standalone pages ----
        print(f"[{vp_name}] /ai")
        go("/ai", wait_ms=1200)
        record("06_ai_agent", "Full AI agent page")

        print(f"[{vp_name}] /3d")
        go("/3d", wait_ms=2000)
        record("07_3d_twin_full", "Full-page 3D twin")

        print(f"[{vp_name}] /settings.html")
        go("/settings.html", wait_ms=600)
        record("08_settings", "Settings page")

        print(f"[{vp_name}] /photo.html")
        go("/photo.html", wait_ms=600)
        record("09_photo", "Photo gallery page")

        print(f"[{vp_name}] /video.html")
        go("/video.html", wait_ms=600)
        record("10_video", "Video gallery page")

        # ---- Narrow: STOP + seek pill emphasis (seek mode again) ----
        print(f"[{vp_name}] navbar close-up (Seek)")
        go("/")
        page.evaluate(
            """() => {
              try { localStorage.setItem('ugv_app_mode', 'seek'); } catch (e) {}
            }"""
        )
        go("/", wait_ms=800)
        safe_click(page, '#mode-tab-seek, button[data-mode="seek"]')
        page.wait_for_timeout(500)
        # viewport-only shot of top chrome
        try:
            f = out / f"{vp_name}__11_navbar_chrome.png"
            page.screenshot(path=str(f), full_page=False, clip={"x": 0, "y": 0, "width": vp["width"], "height": min(140, vp["height"])})
            print(f"  wrote {f}")
            results.append(
                {
                    "name": "11_navbar_chrome",
                    "viewport": vp_name,
                    "file": f.name,
                    "url": page.url,
                    "note": "Navbar chrome crop",
                    "ok": True,
                }
            )
        except Exception as e:
            print(f"  navbar crop fail: {e}")

        context.close()
        browser.close()
    return results


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://127.0.0.1:5000")
    ap.add_argument("--out", default="ui_shots")
    ap.add_argument("--viewports", default="desktop,phone_landscape")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    # Health check
    import urllib.request

    try:
        with urllib.request.urlopen(args.base.rstrip("/") + "/", timeout=5) as r:
            print(f"base {args.base} -> HTTP {r.status}")
    except Exception as e:
        print(f"ERROR: app not reachable at {args.base}: {e}", file=sys.stderr)
        return 2

    all_results = []
    for name in [v.strip() for v in args.viewports.split(",") if v.strip()]:
        if name not in VIEWPORTS:
            print(f"unknown viewport {name}, skip")
            continue
        print(f"\n=== viewport {name} {VIEWPORTS[name]} ===")
        all_results.extend(capture_suite(args.base, out, name, VIEWPORTS[name]))

    manifest = {
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "base": args.base,
        "results": all_results,
        "ok_count": sum(1 for r in all_results if r.get("ok")),
        "fail_count": sum(1 for r in all_results if not r.get("ok")),
    }
    man_path = out / "manifest.json"
    man_path.write_text(json.dumps(manifest, indent=2))
    print(f"\nmanifest {man_path} ok={manifest['ok_count']} fail={manifest['fail_count']}")
    return 0 if manifest["fail_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
