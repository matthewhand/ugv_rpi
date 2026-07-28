# Plan: Merge PTZ-robot updates with Beast RoArm work

**Date:** 2026-07-23  
**Goal:** Bring `origin/main` (other robot / PTZ + Seek + UI) into this tree **without** losing or breaking Beast **USB RoArm** work, and keep the **other robot on PTZ-only** (no RoArm).

---

## 1. Situation summary

| Item | State |
|------|--------|
| Local `main` (this Pi) | `453acd6` + **uncommitted** RoArm/Beast changes |
| `origin/main` (other agent) | **~38 commits ahead** — Seek, PTZ, UI modes, 3D twin, drive-sign cleanup |
| Divergence | Local **0 ahead / 38 behind** origin; RoArm work not committed |
| Hardware split | **Other bot:** PTZ gimbal camera. **This Beast:** USB RoArm (no PTZ head) |

### Overlap (conflict-risk code)

- `app.py` (largest risk — both sides huge)
- `config.yaml`
- `ros_motion.py`
- `templates/control.js`, `templates/index.html`
- `README.md`

### Ours-only (easy to keep)

- `roarm_ctrl.py` (new)
- `scripts/` (safe nav, corridor, survey)
- `ros2/roarm_*`, compose/entrypoint arm hooks
- `tests/test_roarm_*.py` (and related)
- `cv_ctrl.py` multi-index USB rediscovery (may need re-apply if they didn’t touch it)
- Beast runtime files: `.ui_aim_mode.json` (gitignored ideally)

### Theirs-only (take wholesale)

- `ai_seek.py`
- `templates/modes.js`
- Seek UI / panorama / navbar modes in `index.html` + `style.css`
- `tests/test_ai_seek.py`
- 3D twin meshes under `static/meshes/`
- Most Seek/PTZ-specific commits

---

## 2. Product rules (non-negotiable)

1. **PT robot** (`module_type: 2`, no `arm_config.transport: usb_serial`):  
   - Full Seek + PTZ pan/tilt behaviour as on `origin/main`.  
   - No USB RoArm open, no `travel_tuck`, no Aim:RoArm unless explicitly forced.

2. **Beast** (`module_type: 1`, `arm_config.transport: usb_serial`):  
   - Chassis + AI + shared UI shell from their branch.  
   - Arm = USB RoArm (`roarm_ctrl`), default **`travel_tuck`** (low CG).  
   - “Look around” / survey = **base yaw** and/or **limited arm peek**, **not** PTZ `T:133` pan sweeps.  
   - Camera = USB UVC with **index rediscovery** (not hard-coded `video0`).

3. **Shared chassis / control_mode** (direct vs ros2) stays one implementation; arm path is independent of UART ownership when transport is USB.

---

## 3. Git workflow (code update)

### Phase 0 — Freeze and branch (do first)

```bash
cd ~/ugv_rpi
# Optional: discard junk
# rm -f .play_sounds.py.swp

# Save Beast work
git checkout -b beast/roarm-preserve
git add -A  # or carefully select files; exclude *.wav/*.swp if undesired
# Prefer selective add:
git add roarm_ctrl.py scripts/ ros2/ tests/ \
  app.py base_ctrl.py config.yaml cv_ctrl.py ros_motion.py \
  templates/control.js templates/index.html README.md
git status
git commit -m "wip(beast): USB RoArm path, travel_tuck, Aim mode, camera rediscovery"
```

Also snapshot docs outside the repo if needed:

```bash
# already under ~/beast-image/ — leave as-is; update in Phase 5
ls ~/beast-image/*.md
```

### Phase 1 — Integrate their main

```bash
git fetch origin
git checkout -b integrate/ptz-main-plus-roarm origin/main
git merge beast/roarm-preserve
# OR: rebase beast/roarm-preserve onto origin/main, then merge to integrate branch
```

**Preference:** merge (clearer conflict markers for huge `app.py`).  
Do **not** force-push `main` until verified on both configs.

### Phase 2 — Conflict resolution order

Resolve in this order (dependencies first):

| Order | File / area | Rule |
|------|-------------|------|
| 1 | `config.yaml` | Add **both** `arm_config` (Beast) and any Seek keys they added. Document two example profiles in comments. |
| 2 | `roarm_ctrl.py` | Keep ours (no conflict if untracked → just remains). |
| 3 | `ros_motion.py` | Keep their drive-sign cleanup; **re-add** RoArm publish helpers (`publish_roarm_joints`, command topic). Drop obsolete invert if they removed it. |
| 4 | `cv_ctrl.py` | Keep multi-index `_open_usb_camera` rediscovery. |
| 5 | `base_ctrl.py` | Keep serial claim/release + lights safety if still needed. |
| 6 | `app.py` | **Hardest** — see §4. |
| 7 | `templates/*` | Prefer their modes/navbar shell; re-insert Aim:RoArm button + `useRoArmOverlay()` gating. |
| 8 | `README.md` | Combine §6. |

### Phase 3 — Finish RoArm against their APIs

After a clean merge:

1. **Boot pose:** USB RoArm → `travel_tuck` (not T:100 inverted L).  
2. **Aim mode:** `ui_aim_mode` roarm|pt; wire to their Raw/Chat/Seek if modes exist.  
3. **Seek adapter (Beast):**  
   - If Seek requests “pan look-around”, on Beast call **base yaw survey** or skip PTZ with a clear log: `seek_look: roarm_base_yaw`.  
   - Do not send large `T:133` sequences when `arm_usb_enabled()`.  
4. **Distance model:** Align short/medium/long hop times with their Seek distance enums where names match.  
5. **Tests:**  
   - `tests/test_ai_seek.py` (their)  
   - `tests/test_roarm_*.py` / control tests (ours)  
   - Smoke: PT config boot, Beast config boot.

### Phase 4 — Land on main

```bash
# After dual-config smoke tests
git checkout main
git merge integrate/ptz-main-plus-roarm
git push origin main   # only when ready; no force
```

Optional: keep `beast/roarm-preserve` as backup tag:  
`git tag beast-roarm-pre-merge-<date>`

---

## 4. `app.py` merge strategy (detail)

Their tree added a large Seek surface; ours added RoArm routing + aim mode + boot pose.

**Approach:**

1. Start from **their** `app.py` (Seek intact).  
2. Re-port our blocks as discrete sections (copy from `beast/roarm-preserve`):

   - Arm config helpers: `arm_transport()`, `arm_usb_enabled()`, `get_ui_aim_mode()`, `set_ui_aim_mode()`  
   - `_roarm` init / `get_roarm()`  
   - `_route_arm_ui_cmd`, `_route_roarm_raw`  
   - `_route_json_command` branches for T:144 + USB T:100/102/105/114/121/210  
   - `/api/ui_aim_mode`, status fields: `roarm`, `arm_transport`, `ui_aim_mode`  
   - Boot: `travel_tuck` when USB arm  
   - `cmdline_ctrl` → `_route_json_command` (not raw `base_json_ctrl` only)

3. **Seek hooks:**  
   Search their code for pan/PTZ look-around. Wrap:

   ```python
   if arm_usb_enabled():
       # base-yaw survey or no-op + log; do not PTZ
   else:
       # their existing PTZ pan path
   ```

4. Avoid duplicating drive-sign logic; **one** linear-sign convention (theirs after `f5f61fa` / seek wall fixes).

---

## 5. Config dual-profile (code + docs)

In `config.yaml` (and optionally `config.beast.yaml` / `config.ptz.yaml` examples):

```yaml
# Beast (this robot)
# base_config.module_type: 1
# arm_config.transport: usb_serial
# arm_config.default_pose: travel_tuck

# PTZ robot (other)
# base_config.module_type: 2
# arm_config.transport: base_uart   # or omit arm_config
# # no ROARM_SERIAL
```

Do **not** require two codebases—**one tree, two configs**.

---

## 6. Markdown / docs update plan

### 6.1 In-repo (`~/ugv_rpi`)

| File | Action |
|------|--------|
| `README.md` | **Merge structure:** keep their “fork status / Seek / PTZ” sections; add **Beast / USB RoArm** section (transport, postures, Aim mode, camera rediscovery, dual-robot note). Link to detailed docs. |
| `docs/MERGE_PTZ_AND_ROARM_PLAN.md` | This plan (living doc until merge done). |
| `docs/ROARM.md` (**new**, short) | Operator-facing: postures, Aim mode, env `ROARM_SERIAL`, what Seek does on Beast vs PTZ. |
| `docs/CONFIG_PROFILES.md` (**new**, short) | Side-by-side Beast vs PTZ `config.yaml` snippets. |
| Any Seek docs they added in README | Leave; add “PTZ robot only” callouts. |

### 6.2 Beast image docs (`~/beast-image/`)

| File | Action |
|------|--------|
| `CUSTOM_BUILD.md` | Refresh dual-ESP32 + USB RoArm truth; note camera re-enumerate; default `travel_tuck`; link to merge status. |
| `ROARM_CONTROL_PLAN.md` | Mark Phase 2 (Flask USB) mostly done; Phase 3 ROS hybrid; add “coexistence with Seek/PTZ tree” subsection. |
| `CHANGES.md` | Append 2026-07-23+ entries: Aim mode, travel_tuck, camera fix, survey, merge plan. |
| `ARM_AXIS_PROOF.md` | Keep disclaimer (USB vs base UART); point to `tests/` + stills dirs. |
| **New** `MERGE_NOTES.md` (optional) | Post-merge checklist results (what was verified on which robot). |

### 6.3 Doc content principles

- **One sentence of hardware truth** at top of each Beast doc: RoArm ≠ base UART.  
- **PTZ robot** never documented as having RoArm USB.  
- Prefer links over duplicating long Seek docs in Beast files.  
- Record **invalid evidence** (T:1005 on base, IO4/5) still invalid for arm proof.

### 6.4 Doc update order (after code merge)

1. `docs/CONFIG_PROFILES.md` (so configs are unambiguous)  
2. `docs/ROARM.md`  
3. `README.md` combined  
4. `~/beast-image/CUSTOM_BUILD.md` + `ROARM_CONTROL_PLAN.md` + `CHANGES.md`  
5. Close this plan with a “Done” section + date

---

## 7. Verification checklist

### PTZ / other robot config

- [ ] Boot, video, stick PTZ (`T:133`)  
- [ ] Seek mode: pan look-around, triple view, drive  
- [ ] No open of CP2102 / no RoArm errors if port absent  
- [ ] `tests/test_ai_seek.py` (or offline subset)

### Beast config

- [ ] Boot → arm **travel_tuck**  
- [ ] Aim: RoArm moves USB arm; Aim: PT does not break if unused  
- [ ] Camera survives index change (unplug/replug or restart)  
- [ ] Control mode ros2: wheels via ROS; arm still USB hybrid  
- [ ] Seek either disabled, or look-around uses base yaw / safe arm peek only  
- [ ] Short hop + wall refuse still works  
- [ ] `python -m py_compile` on touched modules; roarm tests if hardware present

### Regression

- [ ] Single `app.py` process  
- [ ] No force-push  
- [ ] Secrets/`.env` not committed  

---

## 8. Risk register

| Risk | Mitigation |
|------|------------|
| `app.py` conflict unreadable | Port RoArm as discrete functions from branch; don’t hand-merge Seek |
| Seek assumes PTZ HW | Feature-flag / `arm_usb_enabled()` branch |
| Config drift between robots | Example profiles + `CONFIG_PROFILES.md` |
| Losing uncommitted RoArm | Phase 0 branch commit first |
| Camera index | Already fixed; re-test after merge |
| Double app processes | Document start script; kill extras |

---

## 9. Execution timeline (suggested)

| Step | Owner action | Outcome |
|------|----------------|---------|
| A | Commit `beast/roarm-preserve` | RoArm frozen |
| B | Branch from `origin/main`, merge preserve | Conflicts listed |
| C | Resolve config → ros_motion → cv → app → templates | Integrable tree |
| D | Seek Beast adapter | No PTZ pan on RoArm |
| E | Dual-config smoke | Green checklist |
| F | Docs pass (§6) | README + beast-image MD updated |
| G | Merge to `main`, push | Shared line of development |

---

## 10. Answer to “yes?”

**Yes:** update the local codebase with their significant PTZ/Seek/UI work **and** preserve/finish RoArm by:

1. Branching our work first,  
2. Merging onto their main,  
3. Config-gating PTZ vs USB RoArm,  
4. Re-porting RoArm into their app/UI shell,  
5. Updating in-repo + `~/beast-image` markdown so both robots stay documented correctly.

## Done (2026-07-23)

- Branch `beast/roarm-preserve` @ `c0382cf`
- Integration branch `integrate/ptz-main-plus-roarm` (merge + docs/tests commits)
- Seek look-around gated via `arm_usb_enabled()` in `_seek_look_deg`
- Docs: `docs/CONFIG_PROFILES.md`, `docs/ROARM.md`, README dual-robot section, beast-image MD updates
- Offline tests: `test_dual_robot_gating`, `test_ai_seek`, `test_roarm_ros2`
