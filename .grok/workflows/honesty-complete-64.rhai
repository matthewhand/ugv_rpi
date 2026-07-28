// Honesty / guide-alignment completion workflow — maximize completion odds via 64+ agents.
//
// Stages: FIND (parallel guide_id + honesty slots) → IMPLEMENT (parallel small fixes)
//         → VERIFY (parallel adversarial skeptics) → SYNTHESIZE (report).
//
// Budget math (default args.slots=64):
//   finders:    64  (one per work-list slot)
//   implement:   8  (fixed honesty priorities; plan-only if dry_run)
//   verify:     16  (2 skeptics × 8 implement results)
//   synthesize:  1
//   TOTAL:    89  → set agent_budget ≥ 96 (recommend 128)
//
// Args:
//   slots     (int, default 64, min 8, max 64) — finder parallel size
//   dry_run   (bool) — implementers propose only (read-only capability)
//   repo      (string) — default /home/ws/ugv_rpi
//
// Usage:
//   workflow tool: script_path=.../honesty-complete-64.rhai
//     args={slots:64, dry_run:false} agent_budget=128

let meta = #{
    name: "honesty-complete-64",
    description: "64+ agent fan-out to complete UI honesty/guide-alignment backlog (find, implement, verify, report)",
    when_to_use: "Chat live feed blank, Seek mode copy lies, catalog/guide gaps, residual ui-qa-report defects",
    phases: [
        #{ title: "Find", detail: "parallel honesty checks per guide_id / defect class" },
        #{ title: "Implement", detail: "parallel small disjoint honesty fixes" },
        #{ title: "Verify", detail: "adversarial evidence-gated skeptics" },
        #{ title: "Synthesize", detail: "completion report" },
    ],
};

// ---- schemas (loose enough for validate_only canned hosts) ----
let find_schema = #{
    "type": "object",
    "required": ["slot", "ok"],
    "properties": #{
        "slot": #{ "type": "string" },
        "ok": #{ "type": "boolean" },
        "issue": #{ "type": "string" },
        "file": #{ "type": "string" },
        "severity": #{ "type": "string" },
    },
};

let impl_schema = #{
    "type": "object",
    "required": ["task_id", "done"],
    "properties": #{
        "task_id": #{ "type": "string" },
        "done": #{ "type": "boolean" },
        "summary": #{ "type": "string" },
        "files_changed": #{ "type": "array", "items": #{ "type": "string" } },
    },
};

let verdict_schema = #{
    "type": "object",
    "required": ["task_id", "real"],
    "properties": #{
        "task_id": #{ "type": "string" },
        "real": #{ "type": "boolean" },
        "reason": #{ "type": "string" },
        "evidence": #{ "type": "string" },
    },
};

let synth_schema = #{
    "type": "object",
    "required": ["landed", "remaining", "agent_note"],
    "properties": #{
        "landed": #{ "type": "array", "items": #{ "type": "string" } },
        "remaining": #{ "type": "array", "items": #{ "type": "string" } },
        "agent_note": #{ "type": "string" },
    },
};

// ---- args ----
let slots = 64;
let dry_run = false;
let repo = "/home/ws/ugv_rpi";
if args != () {
    if args.slots != () {
        slots = args.slots;
        if slots < 8 { slots = 8; }
        if slots > 64 { slots = 64; }
    }
    if args.dry_run == true { dry_run = true; }
    if args.repo != () { repo = args.repo; }
}

// Fixed work-list: operator-guide guide_ids (54) + honesty defect pads (10) = 64
let work_list = [
    "shell.raw", "shell.chat", "shell.seek", "shell.stop",
    "shell.rtsp_off", "shell.rtsp_on", "shell.path_direct", "shell.path_ros2",
    "shell.hb_on", "shell.hb_off", "shell.wifi_chip", "shell.ops_log",
    "shell.twin_drawer", "shell.navbar_phone",
    "raw.overview", "raw.speed_mid", "raw.speed_fast", "raw.steady_on",
    "raw.cv_motion", "raw.cv_faces", "raw.reaction_capture", "raw.autodrive",
    "raw.objects", "raw.mp_face", "raw.headlight_on", "raw.base_light",
    "raw.ptz_selftest",
    "chat.overview", "chat.still_empty", "chat.still_grabbed", "chat.attach_off", "chat.link_ai",
    "seek.overview", "seek.mode_a", "seek.mode_b", "seek.mode_c",
    "seek.goal_select", "seek.onfound_none", "seek.onfound_tts", "seek.scenenav_off",
    "seek.limits", "seek.pano_empty", "seek.actions", "seek.running_chrome",
    "ai.overview", "ai.still_empty", "ai.config_panel", "ai.tools_tree", "ai.landscape",
    "twin.page", "twin.drawer",
    "settings.overview", "photo.overview", "video.overview",
    "honesty.chat_live_mjpeg", "honesty.seek_mode_hint", "honesty.catalog_path_manual",
    "honesty.radio_checked_css", "honesty.running_chrome_label", "honesty.freq_stop_rename_residue",
    "honesty.seek_config_lock", "honesty.navbar_chip_labels", "honesty.empty_still_placeholder",
    "honesty.guide_qa_report_sync",
];

// Fixed implement tasks (disjoint honesty priorities)
let impl_tasks = [
    "chat_live_mjpeg",
    "seek_mode_hint",
    "catalog_path_manual",
    "radio_checked_css",
    "running_chrome_label",
    "empty_still_placeholder",
    "seek_config_lock_docs",
    "guide_qa_sync",
];

log("honesty-complete-64 slots=" + slots.to_string() + " dry_run=" + dry_run.to_string());
let b0 = budget();
log("budget total=" + b0.total.to_string() + " spent=" + b0.spent.to_string() + " remaining=" + b0.remaining.to_string());

// ========== FIND ==========
phase("Find");
let find_jobs = [];
let fi = 0;
while fi < slots {
    let slot_name = work_list[fi];
    let fp = "";
    fp += "You are a FINDER agent for UGV UI honesty/guide alignment.\n";
    fp += "Repo root: " + repo + "\n";
    fp += "Your slot: " + slot_name + "\n";
    fp += "Read docs/operator-guide.md for this guide_id (or honesty.* defect class).\n";
    fp += "Use read_file and grep on templates/* and docs/* — do NOT answer from memory.\n";
    fp += "Check whether UI code matches the guide Expected UI for this slot.\n";
    fp += "Also skim docs/ui-qa-report.md if relevant.\n";
    fp += "Set ok=true if aligned; ok=false if misaligned/missing with issue + file path.\n";
    fp += "severity: P0|P1|P2|P3 or empty if ok.\n";
    fp += "Empty issue string only when ok=true after reading code.\n";
    fp += "Unsafe: do not start motors, Seek, WiFi stop, or ROS path thrash.\n";
    find_jobs.push(#{
        prompt: fp,
        label: "find-" + fi.to_string(),
        capability_mode: "read-only",
        output_schema: find_schema,
    });
    fi += 1;
}
log("launching " + find_jobs.len().to_string() + " finders");
let find_results = parallel(find_jobs);

let issues = [];
let fr_i = 0;
while fr_i < find_results.len() {
    let r = find_results[fr_i];
    if r != () && r.success && r.output != () {
        if r.output.ok == false {
            let iss = "";
            if r.output.issue != () { iss = r.output.issue; }
            let fil = "";
            if r.output.file != () { fil = r.output.file; }
            let sl = work_list[fr_i];
            if r.output.slot != () { sl = r.output.slot; }
            issues.push(#{ slot: sl, issue: iss, file: fil });
        }
    }
    fr_i += 1;
}
log("finders reported issues=" + issues.len().to_string());

// ========== IMPLEMENT ==========
phase("Implement");
let impl_jobs = [];
let ii = 0;
while ii < impl_tasks.len() {
    let tid = impl_tasks[ii];
    let ip = "";
    ip += "You are an IMPLEMENTER for UGV UI honesty.\n";
    ip += "Repo: " + repo + "\n";
    ip += "task_id: " + tid + "\n";
    ip += "dry_run: " + dry_run.to_string() + "\n";
    ip += "Context: docs/operator-guide.md, docs/ui-qa-report.md, docs/ui-backlog.md.\n";
    if tid == "chat_live_mjpeg" {
        ip += "TASK: Fix Chat mode blank live camera (#chat-live-preview). Ensure MJPEG /video_feed shows or honest fail state. Wire retry like Seek live. Read templates/index.html modes.js style.css first.\n";
    }
    if tid == "seek_mode_hint" {
        ip += "TASK: Make Seek camera hint text mode-specific (a=detector classic, b=detector+LLM nav, c=LLM vision). Dynamic from radio selection. templates/index.html + modes.js.\n";
    }
    if tid == "catalog_path_manual" {
        ip += "TASK: Document in docs/operator-guide.md and docs/ui-qa-report.md that shell.path_direct/ros2 catalog capture is manual/env-dependent — no UART thrash. Small doc-only edit.\n";
    }
    if tid == "radio_checked_css" {
        ip += "TASK: Confirm seek radio highlight uses :has(input:checked) not static --default only. If already fixed, set done=true with evidence. Else fix style.css.\n";
    }
    if tid == "running_chrome_label" {
        ip += "TASK: Ensure seek.running_chrome in guide/catalog notes is labeled simulated when phase idle. Docs or scripts/screenshot_ui.py comments only if needed.\n";
    }
    if tid == "empty_still_placeholder" {
        ip += "TASK: Verify Chat/AI empty still placeholders (no broken black boxes). Fix if broken. templates.\n";
    }
    if tid == "seek_config_lock_docs" {
        ip += "TASK: Confirm config locks while seek-running; document in operator-guide if missing. modes.js + docs.\n";
    }
    if tid == "guide_qa_sync" {
        ip += "TASK: Sync docs/ui-backlog.md open items with honesty priorities; mark done what is already shipped.\n";
    }
    if dry_run {
        ip += "DRY RUN: Do NOT edit files. Read code and set done=false with summary of proposed change.\n";
        impl_jobs.push(#{
            prompt: ip,
            label: "impl-" + tid,
            capability_mode: "read-only",
            output_schema: impl_schema,
        });
    } else {
        ip += "Apply the smallest fix. Use read_file before search_replace. Bump CSS/JS ?v= cache bust if needed. Do not commit.\n";
        impl_jobs.push(#{
            prompt: ip,
            label: "impl-" + tid,
            capability_mode: "read-write",
            output_schema: impl_schema,
        });
    }
    ii += 1;
}
log("launching " + impl_jobs.len().to_string() + " implementers");
let impl_results = parallel(impl_jobs);

// ========== VERIFY ==========
phase("Verify");
// 2 skeptics per implement task (16) — fail closed without evidence
let vjobs = [];
let vi = 0;
while vi < impl_tasks.len() {
    let tid = impl_tasks[vi];
    let summary = "";
    let done_flag = false;
    if impl_results[vi] != () && impl_results[vi].success && impl_results[vi].output != () {
        if impl_results[vi].output.summary != () { summary = impl_results[vi].output.summary; }
        if impl_results[vi].output.done == true { done_flag = true; }
    }
    let sk = 0;
    while sk < 2 {
        let vp = "";
        vp += "You are an ADVERSARIAL VERIFIER. Fail closed.\n";
        vp += "Repo: " + repo + "\n";
        vp += "task_id: " + tid + "\n";
        vp += "Implementer claimed done=" + done_flag.to_string() + " summary=" + summary + "\n";
        vp += "Independently read_file/grep the shipped templates/docs.\n";
        vp += "Set real=true ONLY if you find concrete evidence the honesty fix is present in source.\n";
        vp += "If dry_run only proposed a fix, real=false unless code already had the fix.\n";
        vp += "evidence must quote file path and short snippet or line observation.\n";
        vp += "Default real=false when unsure.\n";
        vjobs.push(#{
            prompt: vp,
            label: "verify-" + tid + "-s" + sk.to_string(),
            capability_mode: "read-only",
            output_schema: verdict_schema,
        });
        sk += 1;
    }
    vi += 1;
}
log("launching " + vjobs.len().to_string() + " verifiers");
let verdicts = parallel(vjobs);

// Tally: task confirmed if ≥1 skeptic real=true with non-empty evidence
let confirmed = [];
let ti = 0;
while ti < impl_tasks.len() {
    let tid = impl_tasks[ti];
    let any_real = false;
    let ev = "";
    let base = ti * 2;
    let s = 0;
    while s < 2 {
        let idx = base + s;
        if idx < verdicts.len() {
            let v = verdicts[idx];
            if v != () && v.success && v.output != () {
                if v.output.real == true {
                    if v.output.evidence != () {
                        let e = v.output.evidence;
                        if e != "" {
                            any_real = true;
                            ev = e;
                        }
                    }
                }
            }
        }
        s += 1;
    }
    if any_real {
        confirmed.push(#{ task_id: tid, evidence: ev });
    }
    ti += 1;
}
log("verified confirmed=" + confirmed.len().to_string() + "/" + impl_tasks.len().to_string());

// ========== SYNTHESIZE ==========
phase("Synthesize");
let b1 = budget();
let sp = "";
sp += "Synthesize honesty-complete-64 run results.\n";
sp += "Repo: " + repo + "\n";
sp += "dry_run=" + dry_run.to_string() + " slots=" + slots.to_string() + "\n";
sp += "budget spent≈" + b1.spent.to_string() + " total=" + b1.total.to_string() + "\n";
sp += "Finders launched: " + slots.to_string() + " issues_flagged=" + issues.len().to_string() + "\n";
sp += "Implement tasks: " + impl_tasks.len().to_string() + "\n";
sp += "Confirmed by verifiers (count): " + confirmed.len().to_string() + "\n";
sp += "Write landed[] (task_ids confirmed or already fixed) and remaining[] honesty gaps.\n";
sp += "agent_note: one paragraph on completion odds and what operators should re-check.\n";
sp += "Read docs/ui-qa-report.md once if helpful. No code edits.\n";
let syn = agent(sp, #{
    label: "synthesize",
    capability_mode: "read-only",
    output_schema: synth_schema,
});

let landed = [];
let remaining = [];
let note = "";
if syn != () && syn.success && syn.output != () {
    if syn.output.landed != () { landed = syn.output.landed; }
    if syn.output.remaining != () { remaining = syn.output.remaining; }
    if syn.output.agent_note != () { note = syn.output.agent_note; }
}

// Host-side report (durable)
let report = "# honesty-complete-64 completion report\n\n";
report += "slots=" + slots.to_string() + " dry_run=" + dry_run.to_string() + "\n";
report += "budget_total=" + b1.total.to_string() + " budget_spent=" + b1.spent.to_string() + "\n";
report += "finders=" + slots.to_string() + " implementers=" + impl_tasks.len().to_string();
report += " verifiers=" + vjobs.len().to_string() + " synthesize=1\n";
let planned = slots + impl_tasks.len() + vjobs.len() + 1;
report += "planned_logical_agents=" + planned.to_string() + "\n\n";
report += "## Issues flagged by finders\n";
report += "count=" + issues.len().to_string() + "\n";
report += "## Confirmed implementations\n";
report += "count=" + confirmed.len().to_string() + "\n";
let ci = 0;
while ci < confirmed.len() {
    report += "- " + confirmed[ci].task_id + "\n";
    ci += 1;
}
report += "\n## Synthesis landed\n";
let li = 0;
while li < landed.len() {
    report += "- " + landed[li] + "\n";
    li += 1;
}
report += "\n## Synthesis remaining\n";
let ri = 0;
while ri < remaining.len() {
    report += "- " + remaining[ri] + "\n";
    ri += 1;
}
report += "\n## Note\n" + note + "\n";

let path = write_scratch_file("completion-report.md", report);
log("wrote " + path + " planned_agents=" + planned.to_string() + " spent=" + b1.spent.to_string());

complete(#{
    path: path,
    planned_logical_agents: planned,
    budget_spent: b1.spent,
    budget_total: b1.total,
    finders: slots,
    implementers: impl_tasks.len(),
    verifiers: vjobs.len(),
    issues_flagged: issues.len(),
    confirmed: confirmed.len(),
    dry_run: dry_run,
});
